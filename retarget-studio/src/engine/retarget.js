// Retargeting engine.
//
// The mapping is EXPLICIT (target joint -> source joint), never name based.
// Two retarget policies are provided:
//
//  "compensate" (world-space delta): every target joint takes the change in
//     world orientation of its mapped source joint relative to the source rest.
//     A target joint whose source ancestor vanished (e.g. missing neck) still
//     inherits the surviving twist through the chain, so motion is visibly
//     compensated rather than frozen.
//
//  "skip" (local delta): the textbook local-rotation transfer
//     qT = restT * F * (restS^-1 * animS) * F^-1. Target joints with no source
//     mapping stay exactly at their own rest pose, making the skip visible.
//
// Root translation and joint rotation are handled separately. Target bone
// lengths always come from the target model: only the root position is scaled,
// every other offset remains the target's authored (canonical) rest offset.

import { loadSkeleton, canonicalRestPose, fk } from './skeleton.js';
import { toCanonicalQuat, toCanonicalVec } from './frame.js';
import { applyConstraints } from './constraints.js';
import {
  qmul, qconj, qrot, qcanonical, qidentity, vadd, vscale,
  vsub, qdist, vdist, vlen, slerp, deg,
} from './vecquat.js';

// ---- mapping analysis -------------------------------------------------------

export function analyzeMapping(srcRaw, tgtRaw, mapping) {
  const src = loadSkeleton(srcRaw);
  const tgt = loadSkeleton(tgtRaw);
  const srcRest = canonicalRestPose(src);
  const tgtRest = canonicalRestPose(tgt);

  const mappedTargets = [];
  const unmappedTargets = [];
  const usedSources = new Set();
  for (const name of tgt.ordered) {
    const s = mapping[name];
    if (s && src.byName.has(s)) {
      mappedTargets.push({ target: name, source: s });
      usedSources.add(s);
    } else {
      unmappedTargets.push(name);
    }
  }
  const skippedSources = src.ordered.filter((n) => !usedSources.has(n));

  // Rest-pose deviation between corresponding joints in canonical space.
  const rows = mappedTargets.map(({ target, source }) => {
    const sw = srcRest.worldQuat.get(source);
    const tw = tgtRest.worldQuat.get(target);
    // World orientation difference of the joints themselves.
    const worldAngle = qdist(sw, tw);
    // Difference of the direction in which each joint's child bone points.
    const srcChild = src.ordered.find((n) => src.byName.get(n).parent === source);
    const tgtChild = tgt.ordered.find((n) => tgt.byName.get(n).parent === target);
    let directionAngle = null;
    if (srcChild && tgtChild) {
      const sd = qrot(sw, srcRest.offset.get(srcChild));
      const td = qrot(tw, tgtRest.offset.get(tgtChild));
      const sn = vscale(sd, 1 / (vlen(sd) || 1));
      const tn = vscale(td, 1 / (vlen(td) || 1));
      const cos = Math.max(-1, Math.min(1, sn[0] * tn[0] + sn[1] * tn[1] + sn[2] * tn[2]));
      directionAngle = Math.acos(cos);
    }
    return {
      target,
      source,
      worldRestDeltaDeg: deg(worldAngle),
      directionRestDeltaDeg: directionAngle === null ? null : deg(directionAngle),
      targetBoneLen: tgtRest.boneLength.get(target),
      sourceBoneLen: srcRest.boneLength.get(source),
      lengthRatio: tgtRest.boneLength.get(target) / (srcRest.boneLength.get(source) || 1),
    };
  });

  return {
    src, tgt, srcRest, tgtRest,
    mappedTargets, unmappedTargets, skippedSources, usedSources,
    rows,
  };
}

// ---- clip normalization -----------------------------------------------------

// Convert an authored source clip into canonical space and evaluate its FK.
function normalizeClip(src, srcRest, clip) {
  return clip.frames.map((f) => {
    const localQuats = new Map();
    const rootName = src.ordered[0];
    for (const [name, q] of Object.entries(f.locals)) {
      if (name === rootName) continue; // root orientation is a separate input
      localQuats.set(name, toCanonicalQuat(src.frame, q));
    }
    const rootPos = toCanonicalVec(src.frame, f.rootPos);
    const rootQuat = toCanonicalQuat(src.frame, f.rootRot);
    const pose = fk(src, srcRest, localQuats, rootPos, rootQuat);
    return { time: f.time, rootPos, rootQuat, localQuats, pose };
  });
}

// ---- per-frame retarget -----------------------------------------------------

function retargetFrame(info, frame, policy, rootGroundOffset) {
  const { tgt, tgtRest, srcRest, mappedTargets } = info;
  const localOut = new Map();
  const worldOut = new Map();
  const srcWorld = frame.pose.worldQuat;
  const mappedByTarget = new Map(mappedTargets.map((m) => [m.target, m.source]));

  for (const name of tgt.ordered) {
    const j = tgt.byName.get(name);
    const parent = j.parent;
    const parentWorld = parent === null ? null : worldOut.get(parent);
    const sName = mappedByTarget.get(name);

    if (!sName) {
      // Unmapped target joint. Both policies keep its LOCAL rest rotation, so
      // the target's own pose/bone lengths are preserved. In "compensate" the
      // world orientation is still propagated, so mapped descendants inherit
      // upstream source motion; in "skip" the joint simply stays at rest.
      const local = tgtRest.localQuat.get(name);
      localOut.set(name, local);
      worldOut.set(name, parent === null ? local : qmul(parentWorld, local));
      continue;
    }

    const sRestW = srcRest.worldQuat.get(sName);
    const sAnimW = srcWorld.get(sName);
    const tRestW = tgtRest.worldQuat.get(name);

    if (parent === null) {
      // Root rotation: source root world delta applied from the target's rest.
      const deltaW = qmul(sAnimW, qconj(sRestW));
      const tw = qmul(deltaW, tRestW);
      localOut.set(name, tw);
      worldOut.set(name, tw);
      continue;
    }

    if (policy === 'compensate') {
      // World-space delta: the source change in world orientation replayed on
      // the target, independent of rest-pose or chain-length differences.
      const deltaW = qmul(sAnimW, qconj(sRestW));
      const tw = qmul(deltaW, tRestW);
      localOut.set(name, qmul(qconj(parentWorld), tw));
      worldOut.set(name, tw);
    } else {
      // Local-space delta: transfer only the source joint's own local change.
      const sParent = info.src.byName.get(sName).parent;
      const animLocal = sParent
        ? qmul(qconj(frame.pose.worldQuat.get(sParent)), sAnimW)
        : sAnimW;
      const localDelta = qmul(animLocal, qconj(srcRest.localQuat.get(sName)));
      const tLocal = qmul(tgtRest.localQuat.get(name), localDelta);
      localOut.set(name, tLocal);
      worldOut.set(name, qmul(parentWorld, tLocal));
    }
  }

  // Root translation is handled separately from joint rotation: scale the
  // source root displacement by the height ratio and add it to the target's
  // own rest root position. Every other offset stays target-authored, which is
  // exactly what preserves the target's bone lengths.
  const srcRoot = info.src.ordered[0];
  const tgtRoot = tgt.ordered[0];
  const restRootPos = tgtRest.worldPos.get(tgtRoot);
  const srcRestRoot = srcRest.worldPos.get(srcRoot);
  const displacement = vsub(frame.rootPos, srcRestRoot);
  let rootPos = vadd(restRootPos, vscale(displacement, info.rootScale));
  if (rootGroundOffset) rootPos = vadd(rootPos, [0, rootGroundOffset, 0]);

  return { time: frame.time, rootPos, localQuats: localOut };
}

// Enforce shortest-path sign continuity across the whole key sequence and
// across the loop seam. Operates per joint on the canonical local quaternions.
function stitchSigns(tgt, frames) {
  const seamReports = [];
  for (const name of tgt.ordered) {
    let prev = null;
    let flips = 0;
    for (let i = 0; i < frames.length; i++) {
      let qv = frames[i].localQuats.get(name);
      if (prev) {
        const c = qcanonical(prev, qv);
        if (c[3] * qv[3] < 0) flips++;
        qv = c;
      }
      frames[i].localQuats.set(name, qv);
      prev = qv;
    }
    // Loop seam: last key must already equal the first numerically; ensure the
    // sign also matches so playback never snaps at the wrap.
    const first = frames[0].localQuats.get(name);
    const last = frames[frames.length - 1].localQuats.get(name);
    const seamDot = first[0] * last[0] + first[1] * last[1] + first[2] * last[2] + first[3] * last[3];
    if (seamDot < 0) {
      frames[frames.length - 1].localQuats.set(name, [-last[0], -last[1], -last[2], -last[3]]);
      flips++;
    }
    seamReports.push({ joint: name, signFlips: flips, seamDot });
  }
  return seamReports;
}

// ---- public API -------------------------------------------------------------

export function retargetClip(srcRaw, tgtRaw, clip, mapping, options = {}) {
  const policy = options.policy || 'compensate';
  const rootGroundOffset = options.rootGroundOffset ?? 0;
  const info = analyzeMapping(srcRaw, tgtRaw, mapping);

  // Uniform root scale = ratio of pelvis heights so proportions differ but
  // the travelled distance is proportional to the character size.
  const srcRootHeight = info.srcRest.worldPos.get(info.src.ordered[0])[1];
  const tgtRootHeight = info.tgtRest.worldPos.get(info.tgt.ordered[0])[1];
  info.rootScale = tgtRootHeight / (srcRootHeight || 1);

  const norm = normalizeClip(info.src, info.srcRest, clip);
  // retargetFrame walks tgt.ordered, which is already topologically sorted, so
  // parent world orientations are always resolved before their children.
  const outFrames = norm.map((frame) =>
    retargetFrame(info, frame, policy, rootGroundOffset));

  // Optional constraint pass (ROM limits + contact correction). When
  // options.constraints is null/undefined the output is exactly the initial
  // unconstrained retarget. The source clip is never touched: correction
  // rewrites only the freshly produced target keyframes.
  let constraintReport = null;
  if (options.constraints) {
    constraintReport = applyConstraints(info.tgt, info.tgtRest, outFrames,
      options.constraints, { loop: clip.loop });
  }

  const seamReports = stitchSigns(info.tgt, outFrames);

  // Evaluate target FK for every output frame (canonical world space).
  const solved = outFrames.map((f) => {
    const rootQuat = f.localQuats.get(info.tgt.ordered[0]);
    const pose = fk(info.tgt, info.tgtRest, f.localQuats, f.rootPos, rootQuat);
    return { ...f, rootQuat, pose };
  });

  return {
    policy,
    rootScale: info.rootScale,
    mapping,
    unmappedTargets: info.unmappedTargets,
    skippedSources: info.skippedSources,
    mappingRows: info.rows,
    frames: solved,
    seamReports,
    constraintReport,
    duration: clip.duration,
    loop: clip.loop,
    _info: info,
  };
}

// Sample a retargeted clip at time t (seconds) with sign-safe interpolation.
export function sampleClip(retarget, t) {
  const { frames, duration, loop, _info } = retarget;
  let time = t;
  if (loop) time = ((t % duration) + duration) % duration;
  else time = Math.min(t, duration);

  let i = 0;
  while (i < frames.length - 2 && frames[i + 1].time <= time) i++;
  const a = frames[i];
  const b = frames[Math.min(i + 1, frames.length - 1)];
  const span = b.time - a.time || 1;
  const alpha = Math.max(0, Math.min(1, (time - a.time) / span));

  const localQuats = new Map();
  for (const name of _info.tgt.ordered) {
    localQuats.set(name, slerp(a.localQuats.get(name), b.localQuats.get(name), alpha));
  }
  const rootPos = [
    a.rootPos[0] + (b.rootPos[0] - a.rootPos[0]) * alpha,
    a.rootPos[1] + (b.rootPos[1] - a.rootPos[1]) * alpha,
    a.rootPos[2] + (b.rootPos[2] - a.rootPos[2]) * alpha,
  ];
  const rootQuat = localQuats.get(_info.tgt.ordered[0]);
  const pose = fk(_info.tgt, _info.tgtRest, localQuats, rootPos, rootQuat);
  return { time, rootPos, rootQuat, localQuats, pose };
}

// Re-export helpers used by validation/UI.
export { qdist, vdist };
