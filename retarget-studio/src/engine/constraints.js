// Joint range-of-motion (ROM) limits and contact-point correction.
//
// Design contract (mirrors the task requirements):
//
//  * Explicit local reference frames. A limit is an ordered list of
//    { axis, min, max } entries; each axis is a unit vector in the joint's
//    OWN REST-LOCAL frame. The animated delta d = rest^-1 * anim is
//    decomposed swing/twist-style along those axes in order, each twist is
//    clamped, and the rotation is recomposed. Nothing is evaluated in a
//    hidden global frame.
//
//  * Periodic boundary safety. A twist angle is only defined modulo 2pi, so
//    clamping the principal value would snap continuous motion that crosses
//    +/-180deg. Instead angles are UNWRAPPED against the previous frame
//    (choose the 2pi-equivalent closest to the running value) and clamped in
//    unwrapped space. Quaternions are 2pi-periodic, so loop seams stay exact.
//
//  * Priorities and residuals. Contacts carry a priority; they are solved
//    low-priority first so higher priorities are disturbed last. Whatever
//    cannot be satisfied (unreachable targets) is reported per frame as a
//    residual together with the priority — never silently dropped.
//
//  * No bone stretching, bounded cost. Correction only ever rewrites local
//    ROTATIONS; offsets always come from the target model, so bone lengths
//    are invariant by construction. The IK solver is FABRIK (length-
//    preserving position passes realized with pure rotations) with a hard
//    iteration cap, tolerance/minimum-improvement stopping, bounded analytic
//    re-bends out of degenerate straight poses, and best-pose tracking: when
//    a target is unreachable the closest pose seen is kept. A per-frame
//    rotation rate cap spreads forced jumps over several frames instead of
//    teleporting a joint; the un-executed remainder shows up as residual.
//    The cap runs on EVERY frame of a contact chain, so the frame a clamp
//    disengages also bleeds the correction off smoothly instead of snapping
//    back to the retargeted pose.

import { fk } from './skeleton.js';
import {
  qmul, qconj, qnorm, qdist, axisAngle, slerp,
  vadd, vsub, vscale, vlen, vnorm, vdot, vcross, vdist, rad,
} from './vecquat.js';

const TWO_PI = Math.PI * 2;

export const DEFAULT_IK = {
  maxIterations: 16, // hard cap on solver sweeps (finite cost)
  tolerance: 1e-3,   // meters; converged below this residual
  minImprovement: 1e-6, // meters/iteration; below this we are stalled
  // Temporal rate cap: a contact chain may rotate at most this much per frame
  // (rad). Larger required changes are approached over several frames and the
  // shortfall shows up as residual — smoothing is never bought with hidden
  // teleportation. 0 disables the cap.
  maxFrameStep: 0.2,
};

// ---- angle extraction ------------------------------------------------------

// Signed twist angle of q about unit axis, principal value in (-pi, pi].
export function twistAngle(qq, axis) {
  const proj = qq[0] * axis[0] + qq[1] * axis[1] + qq[2] * axis[2];
  let a = 2 * Math.atan2(proj, qq[3]);
  if (a > Math.PI) a -= TWO_PI;
  else if (a <= -Math.PI) a += TWO_PI;
  return a;
}

// Swing/twist decomposition about axis: q = twist * swing, twist returned.
export function twistAbout(qq, axis) {
  const proj = qq[0] * axis[0] + qq[1] * axis[1] + qq[2] * axis[2];
  return qnorm([axis[0] * proj, axis[1] * proj, axis[2] * proj, qq[3]]);
}

// The 2pi-equivalent of `angle` closest to `reference`.
export function unwrapNear(angle, reference) {
  return angle + TWO_PI * Math.round((reference - angle) / TWO_PI);
}

// ---- settings normalization -------------------------------------------------

// Raw settings come from JSON (degrees). Normalized settings use radians and
// fill defaults. Limits: { joint: [{ axis:[x,y,z], min, max, minDeg, maxDeg }] }.
export function normalizeConstraintSettings(raw = {}) {
  const limits = {};
  for (const [joint, axes] of Object.entries(raw.limits || {})) {
    if (!Array.isArray(axes)) continue;
    const specs = [];
    for (const a of axes) {
      if (!Array.isArray(a?.axis) || a.axis.length !== 3) continue;
      let min = Number(a.min);
      let max = Number(a.max);
      if (!Number.isFinite(min) || !Number.isFinite(max)) continue;
      if (min > max) [min, max] = [max, min];
      specs.push({
        axis: vnorm(a.axis.map(Number)),
        min: rad(min), max: rad(max),
        minDeg: min, maxDeg: max,
      });
    }
    if (specs.length) limits[joint] = specs;
  }
  const contacts = (raw.contacts || []).map((c, i) => ({
    id: String(c.id ?? `contact-${i}`),
    effector: c.effector,
    // Empty/absent chain means "derive from the effector's ancestors".
    chain: Array.isArray(c.chain) && c.chain.length ? c.chain.slice() : null,
    priority: Number(c.priority) || 0,
    mode: c.mode === 'point' ? 'point' : 'ground',
    target: Array.isArray(c.target) ? c.target.map(Number) : null,
    groundY: Number(c.groundY) || 0,
    enter: Number.isFinite(Number(c.enter)) ? Number(c.enter) : 0.03,
    exit: Number.isFinite(Number(c.exit)) ? Number(c.exit) : 0.06,
    pin: c.pin !== false, // ground contacts pin by default (no sliding)
    enabled: c.enabled !== false,
  }));
  const ik = {
    maxIterations: Number(raw.ik?.maxIterations) || DEFAULT_IK.maxIterations,
    tolerance: Number(raw.ik?.tolerance) || DEFAULT_IK.tolerance,
    minImprovement: Number(raw.ik?.minImprovement) || DEFAULT_IK.minImprovement,
    maxFrameStep: raw.ik?.maxFrameStepDeg != null
      ? rad(Number(raw.ik.maxFrameStepDeg))
      : DEFAULT_IK.maxFrameStep,
  };
  return { limits, contacts, ik };
}

// Default IK chain for an effector: up to `count` ancestors, root excluded.
export function defaultChain(tgt, effector, count = 3) {
  const chain = [];
  let cur = effector;
  while (cur && chain.length < count) {
    const j = tgt.byName.get(cur);
    if (!j) break;
    if (j.parent !== null) chain.unshift(cur);
    cur = j.parent;
  }
  return chain;
}

// ---- ROM limit clamp ---------------------------------------------------------

// Clamp `local` (canonical local rotation of one joint) against ordered axis
// limits, all expressed in the joint's rest-local frame. `stateAngles` is the
// per-axis unwrapped-angle state persisted across frames (mutated); pass an
// array of nulls for the first frame. `events`, when given, receives
// { axisIndex, excessRad, valueRad, min, max } for every axis that bound.
//
// Clamping one axis perturbs the others (non-commuting rotations), so the
// decompose-clamp-recompose cycle repeats until every axis extracts inside
// its range (max 3 passes — cross-axis coupling is second-order, so this
// converges immediately in practice and the cap keeps the cost bounded).
export function clampJointLocal(restLocal, local, axisSpecs, stateAngles, events) {
  let current = local;
  for (let pass = 0; pass < 3; pass++) {
    // Only the first pass reports: later passes clean up cross-axis residue.
    const passEvents = pass === 0 ? (events ?? []) : [];
    current = clampOnce(restLocal, current, axisSpecs, stateAngles, passEvents);
    if (passEvents.every((e) => Math.abs(e.excessRad) < 1e-9)) return current;
  }
  return current;
}

function clampOnce(restLocal, local, axisSpecs, stateAngles, events) {
  let remaining = qmul(qconj(restLocal), local);
  const twists = [];
  for (let i = 0; i < axisSpecs.length; i++) {
    const spec = axisSpecs[i];
    const twist = twistAbout(remaining, spec.axis);
    const principal = twistAngle(twist, spec.axis);
    const prev = stateAngles[i];
    const unwrapped = prev === null || prev === undefined
      ? principal
      : unwrapNear(principal, prev);
    const clamped = Math.min(spec.max, Math.max(spec.min, unwrapped));
    stateAngles[i] = clamped;
    if (events && Math.abs(clamped - unwrapped) > 1e-9) {
      events.push({
        axisIndex: i,
        excessRad: unwrapped - clamped,
        valueRad: unwrapped,
        min: spec.min, max: spec.max,
      });
    }
    twists.push(axisAngle(spec.axis, clamped));
    remaining = qmul(qconj(twist), remaining);
  }
  // Recompose d' = T1' * T2' * ... * residual, then back to local space.
  let d = remaining;
  for (let i = twists.length - 1; i >= 0; i--) d = qmul(twists[i], d);
  return qmul(restLocal, d);
}

// ---- IK: FABRIK positions + rotation extraction (rotations only) ------------

function perpendicular(n) {
  const ax = Math.abs(n[0]) < Math.abs(n[1])
    ? (Math.abs(n[0]) < Math.abs(n[2]) ? [1, 0, 0] : [0, 0, 1])
    : (Math.abs(n[1]) < Math.abs(n[2]) ? [0, 1, 0] : [0, 0, 1]);
  return vnorm(vcross(n, ax));
}

// Analytic two-bone re-bend of the last two segments into the bend plane.
// Used to escape degenerate (near-collinear) configurations that iterative
// position solvers crawl out of only linearly. Lengths are preserved exactly.
function reBendLastTwo(P, L, targetPos, bendNormal) {
  const m = L.length;
  const B = P[m - 2];
  const toT = vsub(targetPos, B);
  const d = vlen(toT) || 1e-12;
  const L0 = L[m - 2];
  const L1 = L[m - 1];
  const dir = vscale(toT, 1 / d);
  const cosA = Math.max(-1, Math.min(1,
    (L0 * L0 + d * d - L1 * L1) / (2 * L0 * d || 1)));
  const sinA = Math.sqrt(Math.max(0, 1 - cosA * cosA));
  let perp = bendNormal ? vcross(dir, bendNormal) : null;
  if (!perp || vlen(perp) < 1e-9) perp = perpendicular(dir);
  perp = vnorm(perp);
  P[m - 1] = vadd(B, vadd(vscale(dir, L0 * cosA), vscale(perp, L0 * sinA)));
  P[m] = targetPos.slice();
}

// Drive `effector` toward `targetPos` by rotating the joints in `chain`
// (base..tip order). Mutates localQuats. Bone lengths are never touched:
// the solver moves joint POSITIONS with FABRIK (length-preserving by
// construction) and realizes them with pure local rotations.
//
// Bounded cost: at most ik.maxIterations sweeps, each stopped early by the
// tolerance; a sweep that improves less than half the remaining error is
// "stalled" — a reachable target then triggers an analytic re-bend into the
// chain's bend plane (at most 2), an unreachable one stops. The best pose
// seen is restored at the end, so an unreachable target keeps the nearest
// feasible configuration instead of a half-iterated one.
//
// Branch stability: a 2-bone chain has two mirror solutions (flexion vs
// extension side), and iterative solvers pick one chaotically near full
// extension — producing 30deg frame-to-frame jumps. Two countermeasures:
// the bend-plane normal is kept with cross-frame hysteresis (sign flips
// suppressed), and for 2-bone chains EVERY solve is seeded with an analytic
// re-bend into that plane. Re-bends always try both sides of the plane and
// keep the lower error, ties broken by proximity to the solve-start pose,
// so the chosen branch is continuous across frames and never asks the ROM
// clamp to truncate a bend it could have taken on the other side.
//
// When limitCtx = { limits, state } is given, chain joints with ROM limits
// are re-clamped after every update using the shared unwrapped-angle state,
// so IK results respect the same limits and stay continuous across frames.
// `bendState` (optional, persisted across frames per contact) carries the
// bend-plane normal with hysteresis: a nearly straight chain cannot pick a
// stable bend direction from its own geometry, and an unflipped normal keeps
// the knee/elbow folding the same way frame after frame.
// Returns { error, iterations, converged }.
export function solveChainIK(tgt, rest, localQuats, rootPos, rootQuat,
  effector, chain, targetPos, ik, limitCtx, bendState) {
  const usable = chain.filter((j) =>
    tgt.byName.has(j) && tgt.byName.get(j).parent !== null);
  const posJoints = usable[usable.length - 1] === effector
    ? usable.slice()
    : [...usable, effector];
  const m = posJoints.length - 1; // number of bones in the chain
  const L = [];
  for (let i = 0; i < m; i++) L.push(rest.boneLength.get(posJoints[i + 1]));
  const totalLen = L.reduce((a, b) => a + b, 0);
  const startLocals = new Map(localQuats);

  // Bend plane normal from the pose at solve start, with cross-frame
  // hysteresis: near-straight chains produce noisy normals that can flip sign
  // frame to frame, so the running normal is kept unless the chain bends
  // clearly, and flips are suppressed.
  const pose0 = fk(tgt, rest, localQuats, rootPos, rootQuat);
  let rawNormal = null;
  for (let i = 1; i < m && !rawNormal; i++) {
    const a = vsub(pose0.worldPos.get(posJoints[i]), pose0.worldPos.get(posJoints[i - 1]));
    const b = vsub(pose0.worldPos.get(posJoints[i + 1]), pose0.worldPos.get(posJoints[i]));
    const n = vcross(a, b);
    if (vlen(n) > 1e-6) rawNormal = vnorm(n);
  }
  let bendNormal = bendState?.normal ?? null;
  if (rawNormal) {
    if (bendNormal && vdot(rawNormal, bendNormal) < 0) {
      rawNormal = vscale(rawNormal, -1);
    }
    bendNormal = rawNormal;
  }
  if (bendState) bendState.normal = bendNormal;

  // Realize target positions Q with pure rotations, base joint to tip.
  const applyPositions = (Q) => {
    for (let i = 0; i < usable.length; i++) {
      const joint = usable[i];
      const now = fk(tgt, rest, localQuats, rootPos, rootQuat);
      const jPos = now.worldPos.get(joint);
      const cur = vsub(now.worldPos.get(posJoints[i + 1]), jPos);
      const want = vsub(Q[i + 1], jPos);
      const l1 = vlen(cur);
      const l2 = vlen(want);
      if (l1 < 1e-9 || l2 < 1e-9) continue;
      const n1 = vscale(cur, 1 / l1);
      const n2 = vscale(want, 1 / l2);
      const cos = Math.max(-1, Math.min(1, vdot(n1, n2)));
      let axis = vcross(n1, n2);
      const sin = vlen(axis);
      let ang;
      if (sin < 1e-9) {
        if (cos > 0) continue;
        ang = Math.PI;
        axis = perpendicular(n1);
      } else {
        axis = vscale(axis, 1 / sin);
        ang = Math.atan2(sin, cos);
      }
      const dq = axisAngle(axis, ang);
      const jw = now.worldQuat.get(joint);
      const parent = tgt.byName.get(joint).parent;
      const pw = now.worldQuat.get(parent);
      let newLocal = qmul(qconj(pw), qmul(dq, jw));
      const specs = limitCtx?.limits?.[joint];
      if (specs) {
        newLocal = clampJointLocal(
          rest.localQuat.get(joint), newLocal, specs, limitCtx.state.get(joint), null);
      }
      localQuats.set(joint, newLocal);
    }
  };

  const errOf = () => {
    const pose = fk(tgt, rest, localQuats, rootPos, rootQuat);
    return vdist(pose.worldPos.get(effector), targetPos);
  };

  // How far a candidate pose stays inside the ROM limits, measured as the
  // minimum per-axis headroom (rad) over the chain's limited joints. Larger
  // is better; -Infinity when no limited axes exist on the chain.
  const limitHeadroom = (locals) => {
    let worst = Infinity;
    for (const joint of usable) {
      const specs = limitCtx?.limits?.[joint];
      if (!specs) continue;
      let remaining = qmul(qconj(rest.localQuat.get(joint)), locals.get(joint));
      for (const spec of specs) {
        const twist = twistAbout(remaining, spec.axis);
        const angle = twistAngle(twist, spec.axis);
        worst = Math.min(worst, angle - spec.min, spec.max - angle);
        remaining = qmul(qconj(twist), remaining);
      }
    }
    return worst;
  };

  // Re-bend tries are evaluated without committing: localQuats and the shared
  // clamp-unwrap state are restored before the winner is applied.
  const snapshotCtx = () => ({
    locals: new Map(localQuats),
    clamp: limitCtx?.state
      ? usable.map((j) => [j, limitCtx.state.get(j)?.slice()])
      : null,
  });
  const restoreCtx = (snap) => {
    for (const [k, v] of snap.locals) localQuats.set(k, v);
    if (snap.clamp) {
      for (const [j, s] of snap.clamp) {
        if (s && limitCtx.state.has(j)) limitCtx.state.set(j, s);
      }
    }
  };
  const tryRebend = (normal) => {
    const snap = snapshotCtx();
    const pose = fk(tgt, rest, localQuats, rootPos, rootQuat);
    const P = posJoints.map((j) => pose.worldPos.get(j));
    reBendLastTwo(P, L, targetPos, normal);
    applyPositions(P);
    const err = errOf();
    const locals = new Map(localQuats);
    restoreCtx(snap);
    return { err, locals };
  };
  // Both sides of the bend plane are always tried. Lower error wins; near
  // ties prefer more limit headroom (anatomically sustainable bends), then
  // continuity with the previous frame's solution — so the chosen branch is
  // stable across frames and never asks the ROM clamp to truncate a bend it
  // could have taken on the other side.
  const pickRebend = (normal) => {
    const a = tryRebend(normal);
    const b = tryRebend(normal ? vscale(normal, -1) : null);
    const slack = Math.max(ik.tolerance, 1e-9);
    if (a.err < b.err - slack) return { ...a, flipped: false };
    if (b.err < a.err - slack) return { ...b, flipped: true };
    const ha = limitHeadroom(a.locals);
    const hb = limitHeadroom(b.locals);
    if (ha > hb + 1e-3) return { ...a, flipped: false };
    if (hb > ha + 1e-3) return { ...b, flipped: true };
    const reference = bendState?.prevLocals ?? startLocals;
    const dist = (cand) => usable.reduce((s, j) =>
      s + qdist(cand.locals.get(j) ?? reference.get(j), reference.get(j)), 0);
    return dist(a) <= dist(b)
      ? { ...a, flipped: false }
      : { ...b, flipped: true };
  };

  // Seed 2-bone chains with the analytic solution in the hysteresis plane:
  // exact up to the swivel, and it anchors the branch before FABRIK polishes.
  if (bendState && m === 2) {
    const win = pickRebend(bendNormal);
    for (const [k, v] of win.locals) localQuats.set(k, v);
    if (win.flipped && bendNormal) bendNormal = vscale(bendNormal, -1);
    bendState.normal = bendNormal;
  }

  let best = { err: Infinity, locals: null };
  let prevErr = Infinity;
  let iterations = 0;
  let converged = false;
  let rebends = 0;

  for (let iter = 0; iter < ik.maxIterations; iter++) {
    iterations = iter + 1;
    const pose = fk(tgt, rest, localQuats, rootPos, rootQuat);
    const err = vdist(pose.worldPos.get(effector), targetPos);
    if (err < best.err) best = { err, locals: new Map(localQuats) };
    if (err <= ik.tolerance) { converged = true; break; }

    const P = posJoints.map((j) => pose.worldPos.get(j));
    const reachable = vdist(P[0], targetPos) < totalLen - 1e-12;

    if (iter > 0 && prevErr - err < Math.max(ik.minImprovement, 0.5 * err)) {
      // Stalled: crawling or stuck. Reachable -> analytic re-bend (bounded);
      // unreachable -> the extension pose is already the nearest feasible.
      if (!reachable || rebends >= 2) break;
      rebends++;
      const win = pickRebend(bendNormal);
      for (const [k, v] of win.locals) localQuats.set(k, v);
      if (win.flipped && bendNormal) bendNormal = vscale(bendNormal, -1);
      if (bendState) bendState.normal = bendNormal;
      prevErr = Infinity;
      continue;
    }
    prevErr = err;

    // FABRIK position pass (length-preserving in both branches).
    const Q = P.map((p) => p.slice());
    if (!reachable) {
      for (let i = 0; i < m; i++) {
        Q[i + 1] = vadd(Q[i], vscale(vnorm(vsub(targetPos, Q[i])), L[i]));
      }
    } else {
      Q[m] = targetPos.slice();
      for (let i = m - 1; i >= 0; i--) {
        const d = vsub(P[i], Q[i + 1]);
        const dir = vlen(d) < 1e-9 ? vsub(P[i], P[i + 1]) : d;
        Q[i] = vadd(Q[i + 1], vscale(vnorm(dir), L[i]));
      }
      Q[0] = P[0].slice();
      for (let i = 0; i < m; i++) {
        const d = vsub(Q[i + 1], Q[i]);
        const dir = vlen(d) < 1e-9 ? vsub(P[i + 1], P[i]) : d;
        Q[i + 1] = vadd(Q[i], vscale(vnorm(dir), L[i]));
      }
    }
    applyPositions(Q);
  }

  // Keep the closest pose seen: an unreachable target leaves the chain at its
  // nearest feasible configuration instead of a half-iterated one.
  const finalPose = fk(tgt, rest, localQuats, rootPos, rootQuat);
  const finalErr = vdist(finalPose.worldPos.get(effector), targetPos);
  let out;
  if (best.locals && best.err < finalErr) {
    for (const [k, v] of best.locals) localQuats.set(k, v);
    out = { error: best.err, iterations, converged: best.err <= ik.tolerance };
  } else {
    out = { error: finalErr, iterations, converged: converged || finalErr <= ik.tolerance };
  }
  // Remember the solved branch for next frame's continuity tie-break.
  if (bendState) {
    bendState.prevLocals = new Map();
    for (const j of usable) {
      const q = localQuats.get(j);
      if (q) bendState.prevLocals.set(j, q.slice());
    }
  }
  return out;
}

// ---- contact state machine ----------------------------------------------------

// Ground contacts use hysteresis (enter/exit thresholds) so contact state
// cannot flicker frame to frame; while active the target is the pinned
// landing spot (pin:true) or the floor projection of the current foot
// position (pin:false, a pure floor clamp).
function updateContact(spec, state, effPos) {
  if (spec.mode === 'point') {
    state.active = true;
    state.target = spec.target;
    return;
  }
  const h = effPos[1] - spec.groundY;
  if (!state.active && h <= spec.enter) {
    state.active = true;
    state.pin = [effPos[0], spec.groundY, effPos[2]];
    state.activations = (state.activations || 0) + 1;
  } else if (state.active && h > spec.exit) {
    state.active = false;
  }
  if (state.active) {
    state.target = spec.pin ? state.pin : [effPos[0], spec.groundY, effPos[2]];
  }
}

// ---- per-frame pipeline -------------------------------------------------------

// Limit each chain joint's rotation change vs the contact's previous frame to
// maxStep rad (slerp toward the solved pose). The contact's prev map is
// updated to the FINAL locals either way, so an engaging contact ramps from
// the pose the retarget actually produced. Returns true when anything capped.
// Also runs on skipped/inactive frames: without it, the frame a floor clamp
// disengages snaps from the corrected pose back to the retargeted pose in a
// single frame (near full extension a few mm of ankle correction is a ~20deg
// knee swing). Capping every frame bleeds the correction off over several
// frames instead; normal motion is far under the cap and passes through
// untouched.
function capChainStep(contact, f, maxStep) {
  const next = new Map();
  let capped = false;
  for (const joint of contact.spec.chain) {
    const solved = f.localQuats.get(joint);
    if (!solved) continue;
    let out = solved;
    const p = contact.prev?.get(joint);
    if (p && maxStep > 0) {
      const step = qdist(p, solved);
      if (step > maxStep) {
        out = slerp(p, solved, maxStep / step);
        capped = true;
      }
    }
    f.localQuats.set(joint, out);
    next.set(joint, out.slice());
  }
  contact.prev = next;
  return capped;
}

// Apply ROM limits and contacts to a retargeted keyframe sequence, in place.
// frames: [{ time, rootPos, localQuats: Map }] in canonical space.
// Returns a report: { perFrame, stats, settings, warnings }.
export function applyConstraints(tgt, rest, frames, rawSettings, { loop = false } = {}) {
  const settings = normalizeConstraintSettings(rawSettings);
  const warnings = [];

  const limitState = new Map();
  for (const joint of Object.keys(settings.limits)) {
    if (!tgt.byName.has(joint)) {
      warnings.push(`限制引用了未知关节 ${joint}，已忽略`);
      delete settings.limits[joint];
      continue;
    }
    limitState.set(joint, settings.limits[joint].map(() => null));
  }

  const contacts = [];
  for (const spec of settings.contacts) {
    if (!spec.enabled) continue;
    if (!tgt.byName.has(spec.effector)) {
      warnings.push(`接触点 ${spec.id} 引用了未知关节 ${spec.effector}，已忽略`);
      continue;
    }
    const chain = spec.chain ?? defaultChain(tgt, spec.effector, 3);
    contacts.push({
      spec: { ...spec, chain },
      state: { active: false, pin: null, target: null, activations: 0 },
      bend: { normal: null },
      prev: null, // last frame's final locals for this chain (rate cap)
    });
  }
  // Solve low priority first: higher priorities run later and are disturbed
  // less, so they win conflicts. Residuals are reported regardless.
  contacts.sort((a, b) => a.spec.priority - b.spec.priority);

  const rootName = tgt.ordered[0];
  const perFrame = [];
  let clampCount = 0;
  let maxIterations = 0;
  let rateLimited = 0;

  for (const f of frames) {
    const rootQuat = f.localQuats.get(rootName);
    const frameRep = { time: f.time, limits: [], contacts: [] };

    // 1. ROM limits, in skeleton order for determinism.
    for (const joint of tgt.ordered) {
      const specs = settings.limits[joint];
      if (!specs) continue;
      const before = f.localQuats.get(joint);
      if (!before) continue;
      const events = [];
      const after = clampJointLocal(
        rest.localQuat.get(joint), before, specs, limitState.get(joint), events);
      if (events.length) {
        f.localQuats.set(joint, after);
        clampCount += events.length;
        for (const e of events) frameRep.limits.push({ joint, ...e });
      }
    }

    // 2. Contacts in priority order.
    for (const contact of contacts) {
      const { spec, state, bend } = contact;
      const pose = fk(tgt, rest, f.localQuats, f.rootPos, rootQuat);
      const effPos = pose.worldPos.get(spec.effector);
      updateContact(spec, state, effPos);
      if (!state.active) {
        if (capChainStep(contact, f, settings.ik.maxFrameStep)) {
          rateLimited++;
          frameRep.contacts.push({
            id: spec.id, active: false, priority: spec.priority, rateLimited: true,
          });
        } else {
          frameRep.contacts.push({
            id: spec.id, active: false, priority: spec.priority,
          });
        }
        continue;
      }
      // Floor clamp (pin:false): only penetration is corrected. A foot above
      // the floor inside the hysteresis band keeps its retargeted pose.
      if (spec.mode === 'ground' && !spec.pin && effPos[1] >= spec.groundY) {
        const capped = capChainStep(contact, f, settings.ik.maxFrameStep);
        if (capped) rateLimited++;
        frameRep.contacts.push({
          id: spec.id, active: true, priority: spec.priority,
          target: state.target.slice(), residual: 0, iterations: 0,
          converged: !capped, rateLimited: capped || undefined,
        });
        continue;
      }
      const solved = solveChainIK(tgt, rest, f.localQuats, f.rootPos, rootQuat,
        spec.effector, spec.chain, state.target, settings.ik,
        { limits: settings.limits, state: limitState }, bend);
      maxIterations = Math.max(maxIterations, solved.iterations);
      // Temporal rate cap: spread geometrically forced jumps (e.g. the knee
      // snapping straight when the root crosses the reach boundary) over
      // several frames. The un-capped remainder becomes honest residual.
      const capped = capChainStep(contact, f, settings.ik.maxFrameStep);
      let residual = solved.error;
      if (capped) {
        rateLimited++;
        const after = fk(tgt, rest, f.localQuats, f.rootPos, rootQuat);
        residual = vdist(after.worldPos.get(spec.effector), state.target);
      }
      frameRep.contacts.push({
        id: spec.id,
        active: true,
        priority: spec.priority,
        target: state.target.slice(),
        residual,
        iterations: solved.iterations,
        converged: solved.converged && !capped,
        rateLimited: capped || undefined,
      });
    }
    perFrame.push(frameRep);
  }

  // Loop seam: the last key is authored identical to the first, so it must
  // stay identical after correction even if unwrapped angles wound by 2pi*k.
  if (loop && frames.length > 1) {
    const first = frames[0];
    const last = frames[frames.length - 1];
    for (const [name, qq] of first.localQuats) last.localQuats.set(name, qq.slice());
    last.rootPos = first.rootPos.slice();
  }

  return {
    perFrame,
    settings,
    warnings,
    stats: {
      clampCount,
      maxIterations,
      rateLimited,
      contactActivations: contacts.map((c) => ({
        id: c.spec.id, activations: c.state.activations || 0,
      })),
    },
  };
}

// ---- reporting / metrics ------------------------------------------------------

// Merge per-frame triggers into contiguous time ranges for the UI event log.
export function aggregateEvents(report, residualTolerance = 1e-3) {
  const series = new Map();
  report.perFrame.forEach((fr, fi) => {
    for (const l of fr.limits) {
      const key = `limit|${l.joint}|${l.axisIndex}`;
      if (!series.has(key)) series.set(key, []);
      series.get(key).push({
        fi, time: fr.time, value: Math.abs(l.excessRad),
        type: 'limit', joint: l.joint, axisIndex: l.axisIndex,
      });
    }
    for (const c of fr.contacts) {
      if (!c.active || c.residual <= residualTolerance) continue;
      const key = `contact|${c.id}`;
      if (!series.has(key)) series.set(key, []);
      series.get(key).push({
        fi, time: fr.time, value: c.residual,
        type: 'contact', id: c.id, priority: c.priority,
      });
    }
  });
  const events = [];
  for (const list of series.values()) {
    let start = 0;
    for (let i = 1; i <= list.length; i++) {
      if (i < list.length && list[i].fi === list[i - 1].fi + 1) continue;
      const group = list.slice(start, i);
      const peak = group.reduce((a, b) => (b.value > a.value ? b : a));
      events.push({
        ...peak,
        t0: group[0].time,
        t1: group[group.length - 1].time,
        tPeak: peak.time,
        maxValue: peak.value,
        frames: group.length,
      });
      start = i;
    }
  }
  return events.sort((a, b) => a.t0 - b.t0 || b.maxValue - a.maxValue);
}

// Temporal smoothness of a set of joints: adjacent-frame rotation steps.
export function temporalSmoothness(frames, joints) {
  let maxStep = 0;
  let total = 0;
  for (const name of joints) {
    for (let i = 1; i < frames.length; i++) {
      const d = qdist(frames[i - 1].localQuats.get(name), frames[i].localQuats.get(name));
      total += d;
      if (d > maxStep) maxStep = d;
    }
  }
  return { maxStepRad: maxStep, totalVariationRad: total };
}

// World positions of an effector across a frame sequence.
export function effectorPositions(tgt, rest, frames, effector) {
  const rootName = tgt.ordered[0];
  return frames.map((f) => {
    const pose = fk(tgt, rest, f.localQuats, f.rootPos, f.localQuats.get(rootName));
    return pose.worldPos.get(effector);
  });
}
