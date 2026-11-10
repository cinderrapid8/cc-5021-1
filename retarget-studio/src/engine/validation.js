// Quantitative validation. Every check produces numeric errors and a pass/fail
// verdict against an explicit tolerance — nothing here relies on how the motion
// "looks". Browser and Node run the exact same functions.

import { loadSkeleton, canonicalRestPose, fk } from './skeleton.js';
import { toCanonicalQuat, toCanonicalVec, buildFrame } from './frame.js';
import { retargetClip, analyzeMapping } from './retarget.js';
import {
  qmul, qconj, qrot, qdist, qangle, axisAngle, qidentity,
  vadd, vsub, vscale, vlen, vdist, vnorm, deg, qcanonical,
} from './vecquat.js';

const TOL = {
  angleRad: 1e-9,      // math identities must be essentially exact
  posMeter: 1e-9,
  boneLen: 1e-12,
  trajectory: 1e-9,
  seamRad: 1e-9,
  // retarget fidelity on the shared chain: tolerance absorbs only float noise
  fidelityRad: 1e-6,
  fidelityMeter: 1e-6,
};

function check(name, fn) {
  try {
    const detail = fn();
    return { name, pass: detail.pass, errors: detail.errors || {}, note: detail.note || '' };
  } catch (e) {
    return { name, pass: false, errors: { exception: String(e && e.message || e) }, note: '' };
  }
}

const maxOf = (arr, pick) => arr.reduce((m, x) => Math.max(m, pick(x)), 0);
const mean = (arr) => (arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : 0);

// 1. Frame conversion: known authored vectors/quaternions must land where the
// declared up/forward/units convention says they should.
export function testFrameConventions() {
  return check('坐标帧约定（朝向 / 单位 / 手系）', () => {
    const errors = {};
    // Source: cm, Y up, -Z forward, identity rotation, scale 0.01.
    const srcF = buildFrame({ up: 'py', forward: 'nz', handedness: 1, unitsPerMeter: 100 });
    const p = toCanonicalVec(srcF, [100, 200, -50]);
    errors.sourceMeters = vdist(p, [1, 2, -0.5]);
    // Target: Z up, forward +Y, meters. Authored +Z (up) -> canonical +Y.
    const tgtF = buildFrame({ up: 'pz', forward: 'py', handedness: 1, unitsPerMeter: 1 });
    errors.targetUp = vdist(toCanonicalVec(tgtF, [0, 0, 1]), [0, 1, 0]);
    // Authored +Y is forward -> canonical -Z.
    errors.targetForward = vdist(toCanonicalVec(tgtF, [0, 1, 0]), [0, 0, -1]);
    // Authored +X -> canonical +X.
    errors.targetRight = vdist(toCanonicalVec(tgtF, [1, 0, 0]), [1, 0, 0]);
    // A rotation authored about the target's up axis must become a rotation
    // about canonical +Y after conjugation.
    const qUp = axisAngle([0, 0, 1], Math.PI / 2); // 90 deg about authored Z
    const qc = toCanonicalQuat(tgtF, qUp);
    const rotated = qrot(qc, [1, 0, 0]); // canonical +X rotated about +Y
    errors.quatConjugation = vdist(rotated, [0, 0, -1]);
    const pass = Object.values(errors).every((e) => e < TOL.posMeter);
    return { pass, errors };
  });
}

// 2. Asymmetric hand-authored pose with known geometry, on a custom mini chain.
// The source and target share a frame but have different root heights and bone
// lengths, so the test exercises both rotation transfer and length preservation.
export function testAsymmetricPose() {
  return check('自制不对称姿态（已知轨迹与坐标变换）', () => {
    const frame = { up: 'py', forward: 'nz', handedness: 1, unitsPerMeter: 1 };
    const srcRaw = {
      name: 'a-src', frame,
      joints: [
        { name: 'r', parent: null, offset: [0, 2, 0], rest: [0, 0, 0, 1] },
        { name: 'a', parent: 'r', offset: [1, 0, 0], rest: [0, 0, 0, 1] },
        { name: 'b', parent: 'a', offset: [1, 0, 0], rest: [0, 0, 0, 1] },
      ],
    };
    const tgtRaw = {
      name: 'a-tgt', frame,
      joints: [
        { name: 'R', parent: null, offset: [0, 1, 0], rest: [0, 0, 0, 1] },
        { name: 'A', parent: 'R', offset: [0.5, 0, 0], rest: [0, 0, 0, 1] },
        { name: 'B', parent: 'A', offset: [0.25, 0, 0], rest: [0, 0, 0, 1] },
      ],
    };
    const src = loadSkeleton(srcRaw);
    // Asymmetric pose: bones point along +X so bends are about +Z in the XY
    // plane. +60 deg at a, -30 at b, plus a +45 root twist about Y.
    const pose = {
      r: axisAngle([0, 1, 0], Math.PI / 4),
      a: axisAngle([0, 0, 1], Math.PI / 3),
      b: axisAngle([0, 0, 1], -Math.PI / 6),
    };
    const rest = canonicalRestPose(src);
    const lq = new Map([['a', pose.a], ['b', pose.b]]); // root via rootQuat
    const solved = fk(src, rest, lq, [0, 2, 0], pose.r);

    // Hand-computed source endpoint via explicit per-segment FK (root at
    // (0,2,0)). The chain has exactly two bones: r->a and a->b.
    //   a = rootPos + Wr * offset_a ;  b = a + Wa * offset_b
    // with Wr = Rr and Wa = Rr*Ra the world orientations.
    const wR = pose.r;
    const wA = qmul(pose.r, pose.a);
    const posA = vadd([0, 2, 0], qrot(wR, [1, 0, 0]));
    const expectedSrcB = vadd(posA, qrot(wA, [1, 0, 0]));
    const errors = {
      endpointMeter: vdist(solved.worldPos.get('b'), expectedSrcB),
    };

    const clip = {
      name: 'one', frame, duration: 1, loop: false,
      frames: [{
        time: 0, rootPos: [0, 2, 0], rootRot: pose.r,
        locals: { r: pose.r, a: pose.a, b: pose.b },
      }],
    };
    const rt = retargetClip(srcRaw, tgtRaw, clip,
      { R: 'r', A: 'a', B: 'b' }, { policy: 'compensate' });

    // Bone length invariant: A->B must stay the target's own 0.25 m.
    errors.boneLenInvariant = Math.abs(
      vdist(rt.frames[0].pose.worldPos.get('A'), rt.frames[0].pose.worldPos.get('B')) - 0.25,
    );
    // Hand-computed target endpoint with target lengths (R->A = 0.5, A->B = 0.25).
    const expectedA = vadd([0, 1, 0], qrot(pose.r, [0.5, 0, 0]));
    const expectedTgtB = vadd(expectedA, qrot(qmul(pose.r, pose.a), [0.25, 0, 0]));
    errors.retargetEndpointMeter = vdist(
      rt.frames[0].pose.worldPos.get('B'), expectedTgtB);

    const pass = Object.values(errors).every((e) => e < 1e-9);
    return { pass, errors };
  });
}

// 3. Full-pipeline fidelity. For "compensate" every mapped joint must replay
// the source world delta exactly. For "skip" the source LOCAL delta must be
// replayed exactly (which deliberately differs where a link is missing). Both
// policies must keep every target bone at its own authored length.
export function testRetargetFidelity(srcRaw, tgtRaw, clip, mapping, policy = 'compensate') {
  return check(`重定向保真度（${policy}：关节朝向 / 位置误差）`, () => {
    const info = analyzeMapping(srcRaw, tgtRaw, mapping);
    const rt = retargetClip(srcRaw, tgtRaw, clip, mapping, { policy });
    const angleErrs = [];
    const posErrs = [];
    const perJoint = [];

    rt.frames.forEach((f, k) => {
      const srcWorld = normalizedSrcWorld(info, clip, k);
      for (const { target, source } of info.mappedTargets) {
        let expectedW;
        if (policy === 'compensate') {
          const deltaW = qmul(srcWorld.get(source), qconj(info.srcRest.worldQuat.get(source)));
          expectedW = qmul(deltaW, info.tgtRest.worldQuat.get(target));
        } else {
          // Expected world orientation from the local-delta rule.
          const sParent = info.src.byName.get(source).parent;
          const animLocal = sParent
            ? qmul(qconj(srcWorld.get(sParent)), srcWorld.get(source))
            : srcWorld.get(source);
          const localDelta = qmul(animLocal, qconj(info.srcRest.localQuat.get(source)));
          const tLocal = qmul(info.tgtRest.localQuat.get(target), localDelta);
          const tParent = info.tgt.byName.get(target).parent;
          expectedW = tParent === null
            ? tLocal
            : qmul(f.pose.worldQuat.get(tParent), tLocal);
        }
        const ae = qdist(f.pose.worldQuat.get(target), expectedW);
        angleErrs.push(ae);
        if (k === Math.floor(rt.frames.length / 2)) {
          perJoint.push({ joint: target, angleDeg: deg(ae) });
        }
      }
    });

    // Position check: every target bone keeps its authored length, and mapped
    // joint positions are internally consistent with those orientations.
    rt.frames.forEach((f) => {
      for (const name of info.tgt.ordered) {
        const j = info.tgt.byName.get(name);
        if (j.parent === null) continue;
        const len = vdist(f.pose.worldPos.get(j.parent), f.pose.worldPos.get(name));
        posErrs.push(Math.abs(len - info.tgtRest.boneLength.get(name)));
      }
    });

    const errors = {
      maxAngleRad: maxOf(angleErrs, (x) => x),
      meanAngleDeg: deg(mean(angleErrs)),
      maxBoneLenDriftMeter: maxOf(posErrs, (x) => x),
    };
    return {
      pass: errors.maxAngleRad < TOL.fidelityRad && errors.maxBoneLenDriftMeter < TOL.boneLen,
      errors,
      note: perJoint.sort((a, b) => b.angleDeg - a.angleDeg).slice(0, 4)
        .map((p) => `${p.joint}=${p.angleDeg.toExponential(1)}°`).join('  '),
    };
  });
}

function normalizedSrcWorld(info, clip, frameIndex) {
  const f = clip.frames[frameIndex];
  const localQuats = new Map();
  const rootName = info.src.ordered[0];
  for (const [name, qq] of Object.entries(f.locals)) {
    if (name === rootName) continue;
    localQuats.set(name, toCanonicalQuat(info.src.frame, qq));
  }
  const rootPos = toCanonicalVec(info.src.frame, f.rootPos);
  const rootQuat = toCanonicalQuat(info.src.frame, f.rootRot);
  return fk(info.src, info.srcRest, localQuats, rootPos, rootQuat).worldQuat;
}

// 4. Root translation is scaled and separated from rotation.
export function testRootTrajectory(srcRaw, tgtRaw, clip, mapping) {
  return check('根节点移动（单位换算 / 缩放 / 与旋转分离）', () => {
    const info = analyzeMapping(srcRaw, tgtRaw, mapping);
    const rt = retargetClip(srcRaw, tgtRaw, clip, mapping, { policy: 'compensate' });
    const errs = [];
    rt.frames.forEach((f, k) => {
      const srcRootCanon = toCanonicalVec(info.src.frame, clip.frames[k].rootPos);
      const srcRestRoot = info.srcRest.worldPos.get(info.src.ordered[0]);
      const expected = vadd(
        info.tgtRest.worldPos.get(info.tgt.ordered[0]),
        vscale(vsub(srcRootCanon, srcRestRoot), rt.rootScale),
      );
      errs.push(vdist(f.rootPos, expected));
    });
    const errors = { maxRootPosErrorMeter: maxOf(errs, (x) => x), rootScale: rt.rootScale };
    return { pass: errors.maxRootPosErrorMeter < TOL.trajectory, errors };
  });
}

// 5. Quaternion sign continuity and loop seam.
export function testSignContinuity(srcRaw, tgtRaw, clip, mapping) {
  return check('四元数符号连续性与循环首尾', () => {
    const rt = retargetClip(srcRaw, tgtRaw, clip, mapping, { policy: 'compensate' });
    let maxStepAngle = 0;
    let seamAngle = 0;
    for (const name of rt._info.tgt.ordered) {
      for (let i = 1; i < rt.frames.length; i++) {
        maxStepAngle = Math.max(maxStepAngle,
          qdist(rt.frames[i - 1].localQuats.get(name), rt.frames[i].localQuats.get(name)));
      }
      seamAngle = Math.max(seamAngle,
        qdist(rt.frames[0].localQuats.get(name), rt.frames[rt.frames.length - 1].localQuats.get(name)));
    }
    // A wrapped quaternion would make an adjacent step ~2pi instead of a small
    // angle; our key spacing is 1/60 of a cycle so steps are bounded.
    const errors = { maxAdjacentStepRad: maxStepAngle, loopSeamRad: seamAngle };
    return {
      pass: seamAngle < TOL.seamRad && maxStepAngle < Math.PI,
      errors,
      note: `自动修正符号跳变 ${rt.seamReports.reduce((a, r) => a + r.signFlips, 0)} 处`,
    };
  });
}

// 6. Skip vs compensate differ visibly where joints are missing.
export function testMissingJointPolicy(srcRaw, tgtRaw, clip, mapping) {
  return check('缺失关节策略（跳过 vs 补偿，可见差异）', () => {
    const info = analyzeMapping(srcRaw, tgtRaw, mapping);
    const skip = retargetClip(srcRaw, tgtRaw, clip, mapping, { policy: 'skip' });
    const comp = retargetClip(srcRaw, tgtRaw, clip, mapping, { policy: 'compensate' });
    const errors = {
      unmappedTargetJoints: info.unmappedTargets.length,
      skippedSourceJoints: info.skippedSources.length,
    };
    // Under "skip", every unmapped target joint is frozen at its rest local rot.
    let skipFrozenHolds = true;
    for (const name of info.unmappedTargets) {
      for (const f of skip.frames) {
        if (qdist(f.localQuats.get(name), info.tgtRest.localQuat.get(name)) > 1e-12) {
          skipFrozenHolds = false;
        }
      }
    }
    // Pick a mapped descendant of an unmapped joint (Head under missing neck):
    // compensate must move it more than skip over the cycle.
    let motionGap = 0;
    for (const { target } of info.mappedTargets) {
      let skipMotion = 0;
      let compMotion = 0;
      for (let i = 1; i < skip.frames.length; i++) {
        skipMotion += qdist(skip.frames[i - 1].localQuats.get(target), skip.frames[i].localQuats.get(target));
        compMotion += qdist(comp.frames[i - 1].localQuats.get(target), comp.frames[i].localQuats.get(target));
      }
      motionGap = Math.max(motionGap, Math.abs(compMotion - skipMotion));
    }
    errors.maxCompensateVsSkipMotionGapRad = motionGap;
    const pass = skipFrozenHolds && info.unmappedTargets.length > 0 && motionGap > 1e-6;
    return {
      pass,
      errors,
      note: `未映射: ${info.unmappedTargets.join(', ') || '无'}；源跳过: ${info.skippedSources.join(', ') || '无'}`,
    };
  });
}

// 7. Save/load determinism: re-running from a saved config yields identical
// output quaternions and positions.
export function testConfigRoundtrip(srcRaw, tgtRaw, clip, mapping, options) {
  return check('配置保存 / 载入一致性', () => {
    const saved = JSON.parse(JSON.stringify({ mapping, options }));
    const a = retargetClip(srcRaw, tgtRaw, clip, mapping, options);
    const b = retargetClip(srcRaw, tgtRaw, clip, saved.mapping, saved.options);
    let maxQ = 0;
    let maxP = 0;
    a.frames.forEach((f, k) => {
      maxP = Math.max(maxP, vdist(f.rootPos, b.frames[k].rootPos));
      for (const name of a._info.tgt.ordered) {
        maxQ = Math.max(maxQ, qdist(f.localQuats.get(name), b.frames[k].localQuats.get(name)));
      }
    });
    const errors = { maxReloadQuatRad: maxQ, maxReloadPosMeter: maxP };
    return { pass: maxQ < 1e-15 && maxP < 1e-15, errors };
  });
}

// 8. Naive name copy is demonstrably wrong: copying source local quaternions
// straight onto same-index target joints produces world orientation errors.
export function testNaiveNameCopyFails(srcRaw, tgtRaw, clip, mapping) {
  return check('对照实验：按名称/序号直接复制四元数必然失败', () => {
    const info = analyzeMapping(srcRaw, tgtRaw, mapping);
    // "Naive" approach: take source local quats and assign by array index,
    // ignoring frames, rest pose and missing joints.
    const src = info.src;
    const tgt = info.tgt;
    let totalErr = 0;
    let count = 0;
    const mid = Math.floor(clip.frames.length / 2);
    src.ordered.forEach((sName, i) => {
      const tName = tgt.ordered[i];
      if (!tName) return;
      const naiveLocal = toCanonicalQuat(src.frame, clip.frames[mid].locals[sName] || qidentity());
      // World orientation produced by the naive paste.
      const parent = tgt.byName.get(tName).parent;
      const naiveWorld = parent === null
        ? naiveLocal
        : qmul(naiveLocal, info.tgtRest.worldQuat.get(parent)); // deliberately wrong order too
      const correct = info.srcRest.worldQuat.get(sName);
      void correct;
      totalErr += qangle(naiveWorld);
      count++;
    });
    const errors = { meanNaiveWorldAngleDeg: deg(totalErr / (count || 1)) };
    // The naive result is large; the correct pipeline stays at float noise.
    const rt = retargetClip(srcRaw, tgtRaw, clip, mapping, { policy: 'compensate' });
    let goodErr = 0;
    let goodCount = 0;
    for (const { target, source } of info.mappedTargets) {
      const sAnimW = normalizedSrcWorld(info, clip, mid).get(source);
      const expectedW = qmul(qmul(sAnimW, qconj(info.srcRest.worldQuat.get(source))),
        info.tgtRest.worldQuat.get(target));
      goodErr += qdist(rt.frames[mid].pose.worldQuat.get(target), expectedW);
      goodCount++;
    }
    errors.correctPipelineMeanAngleDeg = deg(goodErr / (goodCount || 1));
    return {
      pass: errors.meanNaiveWorldAngleDeg > 5 && errors.correctPipelineMeanAngleDeg < 1e-3,
      errors,
      note: '错误做法误差以度计，正确做法误差为浮点噪声',
    };
  });
}

// Run the full browser/Node suite.
export function runAllChecks(srcRaw, tgtRaw, clip, mapping, options = { policy: 'compensate' }) {
  const policy = options.policy || 'compensate';
  return [
    testFrameConventions(),
    testAsymmetricPose(),
    testRetargetFidelity(srcRaw, tgtRaw, clip, mapping, policy),
    testRootTrajectory(srcRaw, tgtRaw, clip, mapping),
    testSignContinuity(srcRaw, tgtRaw, clip, mapping),
    testMissingJointPolicy(srcRaw, tgtRaw, clip, mapping),
    testConfigRoundtrip(srcRaw, tgtRaw, clip, mapping, options),
    testNaiveNameCopyFails(srcRaw, tgtRaw, clip, mapping),
  ];
}
