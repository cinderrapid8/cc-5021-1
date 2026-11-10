import test from 'node:test';
import assert from 'node:assert/strict';
import { sourceSkeleton, targetSkeleton, defaultMapping } from '../src/data/skeletons.js';
import { buildSourceClip, expectedPose, CLIP_PERIOD } from '../src/data/clip.js';
import { runAllChecks } from '../src/engine/validation.js';
import { retargetClip, sampleClip, analyzeMapping } from '../src/engine/retarget.js';
import { loadSkeleton, canonicalRestPose, fk } from '../src/engine/skeleton.js';
import { toCanonicalVec, toCanonicalQuat } from '../src/engine/frame.js';
import { qdist, vdist } from '../src/engine/vecquat.js';

const clip = buildSourceClip(60);

test('validation suite: all quantitative checks pass', () => {
  const results = runAllChecks(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate' });
  for (const r of results) {
    assert.ok(r.pass, `${r.name} failed: ${JSON.stringify(r.errors)} ${r.note}`);
  }
});

test('skip policy also satisfies fidelity on mapped joints', () => {
  const results = runAllChecks(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'skip' });
  const fidelity = results.find((r) => r.name.includes('保真度'));
  const seam = results.find((r) => r.name.includes('符号连续性'));
  assert.ok(fidelity.pass, JSON.stringify(fidelity.errors));
  assert.ok(seam.pass, JSON.stringify(seam.errors));
});

test('target bone lengths are always the model’s own', () => {
  const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);
  for (const policy of ['compensate', 'skip']) {
    const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping, { policy });
    for (const f of rt.frames) {
      for (const name of info.tgt.ordered) {
        const j = info.tgt.byName.get(name);
        if (j.parent === null) continue;
        const len = vdist(f.pose.worldPos.get(j.parent), f.pose.worldPos.get(name));
        assert.ok(Math.abs(len - info.tgtRest.boneLength.get(name)) < 1e-12,
          `${policy} ${name} bone length drifted to ${len}`);
      }
    }
  }
});

test('known source trajectory: analytic FK matches expectedPose exactly', () => {
  const src = loadSkeleton(sourceSkeleton);
  const rest = canonicalRestPose(src);
  const u = 0.25; // sin=1, cos=0 -> every value known by hand
  const pose = expectedPose(u);
  const canon = new Map();
  for (const [k, v] of Object.entries(pose.locals)) {
    if (k === src.ordered[0]) continue;
    canon.set(k, toCanonicalQuat(src.frame, v));
  }
  const rootPos = toCanonicalVec(src.frame, pose.rootPos);
  const rootQuat = toCanonicalQuat(src.frame, pose.rootRot);
  const { worldPos } = fk(src, rest, canon, rootPos, rootQuat);
  // Root authored X displacement at u=.25: 30 cm -> 0.3 m canonical +X.
  assert.ok(Math.abs(worldPos.get('root')[0] - 0.3) < 1e-12);
  // Root up = 100 cm = 1 m plus bob sin(pi)=0 at this phase.
  assert.ok(Math.abs(worldPos.get('root')[1] - 1.0) < 1e-12);
  // Root Z: 25*(1-cos(pi/2)) = 25 cm = 0.25 m canonical +Z.
  assert.ok(Math.abs(worldPos.get('root')[2] - 0.25) < 1e-12);
});

test('loop seam: first and last target poses are identical', () => {
  const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping);
  const first = rt.frames[0];
  const last = rt.frames[rt.frames.length - 1];
  for (const name of rt._info.tgt.ordered) {
    assert.ok(qdist(first.localQuats.get(name), last.localQuats.get(name)) < 1e-12,
      `seam mismatch on ${name}`);
  }
  assert.ok(vdist(first.rootPos, last.rootPos) < 1e-12);
});

test('sampling at t=0 and t=period gives the same pose, no sign snap', () => {
  const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping);
  const a = sampleClip(rt, 0);
  const b = sampleClip(rt, CLIP_PERIOD);
  for (const name of rt._info.tgt.ordered) {
    assert.ok(qdist(a.localQuats.get(name), b.localQuats.get(name)) < 1e-12);
  }
});

test('mapping analysis reports every missing and extra joint', () => {
  const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);
  assert.deepEqual(info.unmappedTargets.sort(), ['Belt_Twist', 'Skull']);
  assert.deepEqual(info.skippedSources.sort(), ['L_toe', 'R_toe', 'neck']);
});

test('rest-pose deviation table exposes nonzero differences', () => {
  const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);
  const armRow = info.rows.find((r) => r.target === 'arm_L');
  assert.ok(armRow.worldRestDeltaDeg > 0, 'A-pose arm must differ from T-pose rest');
  assert.ok(armRow.lengthRatio > 0 && armRow.lengthRatio !== 1);
});

test('target-only joints stay on their own rest pose under skip', () => {
  const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);
  const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping, { policy: 'skip' });
  for (const name of ['Belt_Twist', 'Skull']) {
    for (const f of rt.frames) {
      assert.ok(qdist(f.localQuats.get(name), info.tgtRest.localQuat.get(name)) < 1e-12);
    }
  }
});

test('compensate vs skip differ at Head; truly unmapped joints are frozen under skip', () => {
  const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);
  const skip = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping, { policy: 'skip' });
  const comp = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping, { policy: 'compensate' });
  const motion = (rt, joint) => {
    let total = 0;
    for (let i = 1; i < rt.frames.length; i++) {
      total += qdist(rt.frames[i - 1].localQuats.get(joint), rt.frames[i].localQuats.get(joint));
    }
    return total;
  };
  // Belt_Twist has no source mapping: frozen at target rest under both policies.
  for (const rt of [skip, comp]) {
    for (const f of rt.frames) {
      assert.ok(qdist(f.localQuats.get('Belt_Twist'), info.tgtRest.localQuat.get('Belt_Twist')) < 1e-12);
    }
  }
  // Head maps to source head: skip gives only head's own local delta, while
  // compensate also absorbs the skipped neck twist -> visibly different motion.
  assert.ok(motion(comp, 'Head') > 0.01);
  assert.ok(Math.abs(motion(comp, 'Head') - motion(skip, 'Head')) > 1e-6);
});

test('root scale equals pelvis-height ratio and uses meters on both sides', () => {
  const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping);
  const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);
  const expected = info.tgtRest.worldPos.get('Pelvis')[1] /
    info.srcRest.worldPos.get('root')[1];
  assert.ok(Math.abs(rt.rootScale - expected) < 1e-12);
  assert.ok(Math.abs(rt.rootScale - 0.95) < 1e-12); // target pelvis 0.95 m, source 1.0 m
});
