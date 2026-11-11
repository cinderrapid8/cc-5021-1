import test from 'node:test';
import assert from 'node:assert/strict';
import { sourceSkeleton, targetSkeleton, defaultMapping } from '../src/data/skeletons.js';
import { buildSourceClip } from '../src/data/clip.js';
import { defaultConstraintsFor } from '../src/data/constraints.js';
import { retargetClip, analyzeMapping } from '../src/engine/retarget.js';
import {
  normalizeConstraintSettings, twistAbout, twistAngle, unwrapNear,
  temporalSmoothness, aggregateEvents,
} from '../src/engine/constraints.js';
import { defaultConfig, serializeConfig, parseConfig } from '../src/engine/config.js';
import { toCanonicalQuat } from '../src/engine/frame.js';
import { exportJSON } from '../src/three/exporter.js';
import { qmul, qconj, qdist, vdist, vnorm, vsub, vdot, deg, rad } from '../src/engine/vecquat.js';

const clip = buildSourceClip(60);
const info = analyzeMapping(sourceSkeleton, targetSkeleton, defaultMapping);

// Worst ROM-limit excess (rad) of a frame sequence, measured with the same
// ordered rest-local decomposition the clamp uses.
function worstLimitExcess(frames, settings) {
  const norm = normalizeConstraintSettings(settings);
  let worst = 0;
  for (const f of frames) {
    for (const [joint, specs] of Object.entries(norm.limits)) {
      const restLocal = info.tgtRest.localQuat.get(joint);
      const local = f.localQuats.get(joint);
      if (!restLocal || !local) continue;
      let remaining = qmul(qconj(restLocal), local);
      for (const spec of specs) {
        const twist = twistAbout(remaining, spec.axis);
        const principal = twistAngle(twist, spec.axis);
        const mid = (spec.min + spec.max) / 2;
        const unwrapped = unwrapNear(principal, mid);
        worst = Math.max(worst, spec.min - unwrapped, unwrapped - spec.max, 0);
        remaining = qmul(qconj(twist), remaining);
      }
    }
  }
  return worst;
}

function maxBoneDrift(rt) {
  let drift = 0;
  for (const f of rt.frames) {
    for (const name of info.tgt.ordered) {
      const j = info.tgt.byName.get(name);
      if (j.parent === null) continue;
      const len = vdist(f.pose.worldPos.get(j.parent), f.pose.worldPos.get(name));
      drift = Math.max(drift, Math.abs(len - info.tgtRest.boneLength.get(name)));
    }
  }
  return drift;
}

test('scenario: short-armed target reaches for a high point', () => {
  const still = {
    name: 'still', frame: sourceSkeleton.frame, duration: 1, loop: false,
    frames: [0, 0.5, 1].map((t) => ({
      time: t, rootPos: [0, 100, 0], rootRot: [0, 0, 0, 1], locals: {},
    })),
  };
  const shoulder = info.tgtRest.worldPos.get('arm_L');
  const reach = info.tgtRest.boneLength.get('elbow_L')
    + info.tgtRest.boneLength.get('wrist_L')
    + info.tgtRest.boneLength.get('hand_L');
  const highPoint = [shoulder[0], shoulder[1] + reach + 0.25, shoulder[2]];

  const rt = retargetClip(sourceSkeleton, targetSkeleton, still, defaultMapping, {
    constraints: {
      contacts: [{
        id: 'hand-goal', effector: 'hand_L', chain: ['arm_L', 'elbow_L', 'wrist_L'],
        priority: 5, mode: 'point', target: highPoint,
      }],
      ik: { maxIterations: 16, tolerance: 0.001, maxFrameStepDeg: 0 },
    },
  });
  const rep = rt.constraintReport.perFrame[2].contacts[0];
  // Unreachable by 0.25 m: nearest feasible pose = full extension toward the
  // point, residual honestly reported with the contact's priority.
  assert.ok(Math.abs(rep.residual - 0.25) < 5e-3, `residual ${rep.residual}`);
  assert.equal(rep.converged, false);
  assert.equal(rep.priority, 5);
  const hand = rt.frames[2].pose.worldPos.get('hand_L');
  const align = vdot(vnorm(vsub(hand, shoulder)), vnorm(vsub(highPoint, shoulder)));
  assert.ok(deg(Math.acos(Math.max(-1, Math.min(1, align)))) < 3, 'arm aims at target');
  assert.ok(maxBoneDrift(rt) < 1e-12, 'bones never stretched');
});

test('scenario: shoulder limits bind on the demo clip, bones intact', () => {
  const settings = defaultConstraintsFor('Stocky (target)');
  const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate', constraints: settings });
  const rep = rt.constraintReport;
  assert.ok(rep.stats.clampCount > 0, 'limits actually trigger on the demo motion');
  assert.ok(worstLimitExcess(rt.frames, settings) < 1e-12, 'no frame exceeds a limit');
  assert.ok(maxBoneDrift(rt) < 1e-12);
  // Shoulder events are reported per frame so the UI can locate them.
  const shoulderEvents = rep.perFrame.flatMap((f, i) =>
    f.limits.filter((l) => l.joint.startsWith('arm_')).map((l) => ({ ...l, i })));
  assert.ok(shoulderEvents.length > 0, 'shoulder clamp events reported');
  assert.ok(shoulderEvents.every((e) => Number.isFinite(e.excessRad) && e.axisIndex >= 0));
});

test('scenario: continuous multi-frame contact stays pinned and smooth', () => {
  const N = 40;
  const frames = [];
  for (let i = 0; i <= N; i++) {
    const u = i / N;
    frames.push({
      time: (i / N) * 2,
      rootPos: [0, 100 + 8 * Math.sin(2 * Math.PI * u), 0],
      rootRot: [0, 0, 0, 1],
      locals: {},
    });
  }
  const bob = { name: 'bob', frame: sourceSkeleton.frame, duration: 2, loop: false, frames };
  const settings = {
    contacts: [{
      id: 'foot-pin', effector: 'ankle_L', chain: ['thigh_L', 'knee_L'],
      priority: 10, mode: 'ground', groundY: 0.17, enter: 0.03, exit: 0.06, pin: true,
    }],
    ik: { maxIterations: 16, tolerance: 0.001, maxFrameStepDeg: 12 },
  };
  const rtOff = retargetClip(sourceSkeleton, targetSkeleton, bob, defaultMapping, {});
  const rtOn = retargetClip(sourceSkeleton, targetSkeleton, bob, defaultMapping,
    { constraints: settings });
  const rep = rtOn.constraintReport;

  // End-effector error before vs after, over active frames.
  let errBefore = 0;
  let errAfter = 0;
  let active = 0;
  rep.perFrame.forEach((fr, i) => {
    const c = fr.contacts[0];
    if (!c.active) return;
    active++;
    errBefore += vdist(rtOff.frames[i].pose.worldPos.get('ankle_L'), c.target);
    errAfter += c.residual;
    // Pinned frames: the ankle truly sits at the pin.
    if (c.residual <= 2e-3) {
      assert.ok(vdist(rtOn.frames[i].pose.worldPos.get('ankle_L'), c.target) <= 2e-3);
    }
  });
  assert.ok(errAfter / active < errBefore / active, 'correction reduces contact error');

  // Temporal smoothness: the rate cap bounds every per-frame joint step, so
  // forced jumps (knee snapping straight at the reach boundary) are spread
  // over frames instead of teleporting.
  const smooth = temporalSmoothness(rtOn.frames, ['thigh_L', 'knee_L']);
  assert.ok(smooth.maxStepRad <= rad(12) + 1e-9,
    `max frame step ${deg(smooth.maxStepRad).toFixed(2)}° exceeds the 12° cap`);
  assert.ok(rep.stats.rateLimited > 0, 'rate cap actually engages in this scenario');
  assert.ok(maxBoneDrift(rtOn) < 1e-12);
});

test('scenario: angle limit across the ±180° boundary does not truncate motion', () => {
  const frame = { up: 'py', forward: 'nz', handedness: 1, unitsPerMeter: 1 };
  const skel = {
    name: 'unwrap-chain', frame,
    joints: [
      { name: 'r', parent: null, offset: [0, 1, 0], rest: [0, 0, 0, 1] },
      { name: 'j', parent: 'r', offset: [1, 0, 0], rest: [0, 0, 0, 1] },
      { name: 'e', parent: 'j', offset: [1, 0, 0], rest: [0, 0, 0, 1] },
    ],
  };
  const N = 40;
  const frames = [];
  for (let i = 0; i <= N; i++) {
    frames.push({
      time: (i / N) * 2, rootPos: [0, 1, 0], rootRot: [0, 0, 0, 1],
      locals: { j: axisAngleZ(150 + (60 * i) / N) },
    });
  }
  const spin = { name: 'spin', frame, duration: 2, loop: false, frames };
  const mapping = { r: 'r', j: 'j', e: 'e' };
  const rt = retargetClip(skel, skel, spin, mapping,
    { constraints: { limits: { j: [{ axis: [0, 0, 1], min: -170, max: 190 }] } } });

  // Unwrapped angle series must rise continuously and saturate at +190° —
  // never snap to the -170° branch when crossing ±180°.
  let prev = rad(150);
  let maxStep = 0;
  const angles = rt.frames.map((f) => {
    const a = unwrapNear(twistAngle(f.localQuats.get('j'), [0, 0, 1]), prev);
    maxStep = Math.max(maxStep, Math.abs(a - prev));
    prev = a;
    return a;
  });
  assert.ok(deg(maxStep) < 5, `continuous, max step ${deg(maxStep).toFixed(2)}°`);
  assert.ok(Math.abs(angles[angles.length - 1] - rad(190)) < 1e-6, 'saturates at +190°');
  assert.ok(Math.max(...angles) <= rad(190) + 1e-9);
});

function axisAngleZ(degAngle) {
  const half = rad(degAngle) / 2;
  return [0, 0, Math.sin(half), Math.cos(half)];
}

test('unconstrained retarget remains selectable and bit-identical', () => {
  const base = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate' });
  const off = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate', constraints: null });
  for (let k = 0; k < base.frames.length; k++) {
    for (const name of info.tgt.ordered) {
      assert.ok(qdist(base.frames[k].localQuats.get(name),
        off.frames[k].localQuats.get(name)) < 1e-15);
    }
  }
  assert.equal(off.constraintReport, null);
});

test('constraint settings are stored per target and survive save/load', () => {
  const cfg = {
    ...defaultConfig(),
    mapping: { ...defaultMapping },
    constraints: {
      enabled: true,
      targets: { 'Stocky (target)': defaultConstraintsFor('Stocky (target)') },
    },
  };
  const round = parseConfig(JSON.parse(JSON.stringify(serializeConfig(cfg))));
  assert.equal(round.constraints.enabled, true);
  assert.deepEqual(round.constraints.targets['Stocky (target)'],
    cfg.constraints.targets['Stocky (target)']);
  // A config written before constraints existed parses with them disabled.
  const legacy = parseConfig(JSON.parse(JSON.stringify({
    ...serializeConfig(cfg), constraints: undefined,
  })));
  assert.deepEqual(legacy.constraints, { enabled: false, targets: {} });
});

test('JSON export carries the actually corrected pose curves', () => {
  const settings = { limits: { arm_L: [{ axis: [0, 1, 0], min: -10, max: 10 }] } };
  const rtOn = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate', constraints: settings });
  const rtOff = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate' });
  const json = exportJSON(info.tgt, rtOn);

  // Find a frame where the limit changed arm_L, then confirm the exported
  // curve matches the CORRECTED frame, not the unconstrained one.
  const k = rtOn.frames.findIndex((f, i) =>
    qdist(f.localQuats.get('arm_L'), rtOff.frames[i].localQuats.get('arm_L')) > 1e-6);
  assert.ok(k > 0, 'limit must visibly change the curve');
  const exported = toCanonicalQuat(info.tgt.frame, json.frames[k].locals.arm_L);
  assert.ok(qdist(exported, rtOn.frames[k].localQuats.get('arm_L')) < 1e-12,
    'export equals corrected pose');
  assert.ok(qdist(exported, rtOff.frames[k].localQuats.get('arm_L')) > 1e-6,
    'export is not the unconstrained pose');
  // Constraint metadata travels with the export for downstream auditing.
  assert.ok(json.constraints?.stats?.clampCount > 0);
  assert.ok(Array.isArray(json.constraints?.events));
});

test('aggregateEvents locates every constraint trigger in time', () => {
  const settings = defaultConstraintsFor('Stocky (target)');
  const rt = retargetClip(sourceSkeleton, targetSkeleton, clip, defaultMapping,
    { policy: 'compensate', constraints: settings });
  const events = aggregateEvents(rt.constraintReport);
  assert.ok(events.length > 0, 'demo clip triggers limit events');
  for (const e of events) {
    assert.ok(e.t0 <= e.tPeak && e.tPeak <= e.t1, 'event has a locateable time range');
    assert.ok(e.maxValue > 0);
    assert.ok(e.frames >= 1);
  }
  const limitEvents = events.filter((e) => e.type === 'limit');
  assert.ok(limitEvents.some((e) => e.joint.startsWith('arm_')),
    'shoulder triggers are individually locateable');
});
