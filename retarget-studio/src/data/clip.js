// Analytically authored source animation (no captured data, so every expected
// value is known exactly). It combines the three requested movements:
//   - arm swing (shoulders/elbows)
//   - body twist (root yaw + counter-twisting spine/chest)
//   - stepping (opposed hip/knee bends) plus a curved root translation.
//
// Keyframes are authored in the SOURCE skeleton's own frame (cm, Y up, -Z fwd).
// The retarget pipeline converts them; tests compare against hand-computed
// canonical expectations, so frame handling is actually exercised.

import { axisAngle, qmul, qidentity } from '../engine/vecquat.js';

const AX = (rad) => axisAngle([1, 0, 0], rad);
const AY = (rad) => axisAngle([0, 1, 0], rad);
const AZ = (rad) => axisAngle([0, 0, 1], rad);

const PERIOD = 2.0; // seconds, loops once
const TWO_PI = Math.PI * 2;

// Local source joint rotations at normalized phase u in [0, 1).
function poseAt(u) {
  const s = Math.sin(TWO_PI * u);
  const c = Math.cos(TWO_PI * u);

  const rootRot = AY(0.5 * s); // body twist about up axis
  // Root walks a small curved path (a loop), plus vertical bob.
  const rootPos = [
    30 * Math.sin(TWO_PI * u),          // authored X, cm
    100 + 4 * Math.sin(TWO_PI * 2 * u), // up, cm (2 bobs per cycle)
    25 * (1 - Math.cos(TWO_PI * u)),    // authored Z, cm
  ];

  const locals = new Map([
    // Counter-twisting spine so the twist is visibly distributed.
    ['spine', AY(-0.25 * s)],
    ['chest', AY(0.35 * s)],
    ['neck', AY(0.1 * s)],
    ['head', qmul(AY(0.15 * s), AX(0.1 * c))],

    // Left arm: swing forward/back + asymmetric raise + elbow bend.
    ['L_shoulder', qmul(AY(0.9 * s), AZ(0.25 * c))],
    ['L_elbow', AX(0.6 * (0.5 + 0.5 * s))],
    ['L_wrist', qidentity()],
    ['L_hand', qidentity()],

    // Right arm opposite phase.
    ['R_shoulder', qmul(AY(-0.9 * s), AZ(-0.25 * c))],
    ['R_elbow', AX(0.6 * (0.5 - 0.5 * s))],
    ['R_wrist', qidentity()],
    ['R_hand', qidentity()],

    // Stepping: opposed hip swing, knees flex at the lift phase. Knee flexion
    // is a NEGATIVE X rotation: +X would swing the ankle forward (-Z), bending
    // the knee backward past its anatomical range.
    ['L_hip', AX(-0.5 * s)],
    ['L_knee', AX(-0.8 * Math.max(0, s) * Math.max(0, s))],
    ['L_ankle', AX(0.2 * s)],
    ['L_toe', qidentity()],

    ['R_hip', AX(0.5 * s)],
    ['R_knee', AX(-0.8 * Math.max(0, -s) * Math.max(0, -s))],
    ['R_ankle', AX(-0.2 * s)],
    ['R_toe', qidentity()],
  ]);

  return { rootPos, rootRot, locals };
}

// Build a keyframed clip. sampleCount+1 keys are emitted so the loop seam is
// explicit; the final key is numerically identical to the first (same pose),
// which the sign-continuity logic must preserve rather than flip.
export function buildSourceClip(sampleCount = 60) {
  const frames = [];
  for (let i = 0; i <= sampleCount; i++) {
    const u = (i % sampleCount) / sampleCount; // last index wraps to phase 0
    const t = (i / sampleCount) * PERIOD;
    const pose = poseAt(u);
    frames.push({
      time: Number(t.toFixed(6)),
      rootPos: pose.rootPos,
      rootRot: pose.rootRot,
      locals: Object.fromEntries(pose.locals),
    });
  }
  return {
    name: 'swing-twist-step',
    frame: { up: 'py', forward: 'nz', handedness: 1, unitsPerMeter: 100 },
    duration: PERIOD,
    loop: true,
    frames,
  };
}

// Exposed for tests: exact expected pose at a given normalized phase.
export const expectedPose = poseAt;
export const CLIP_PERIOD = PERIOD;
