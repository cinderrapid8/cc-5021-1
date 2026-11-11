// Default constraint settings, keyed by TARGET skeleton name. These belong to
// the target character (its anatomy), travel with saved configs, and never
// touch the source animation.
//
// Limit axes are unit vectors in each joint's REST-LOCAL frame (for the
// Stocky target most rest locals are identity, so the axes are the canonical
// X/Y/Z; arm_L/arm_R and the knees have non-identity rest rotations and their
// axes are still evaluated in the joint's own local frame). Ranges are in
// degrees and were chosen against the measured motion envelope of the demo
// clip so correction is visible at the extremes without gutting the motion:
//   shoulder twist Y measured ±43.3° -> limited to ±35°
//   shoulder X ±26.9° -> ±25°;  shoulder Z ±14.8° -> ±15° (contained)
//   elbow X (forearm roll, the bones run along local X) 0..28.4° -> -5..20°;
//   elbow Y ±18.9° -> ±15°
//   thigh X ±28.6° -> ±25°; ankle X ±11.5° -> ±10°
//   knees: the shank offsets run along local -Z, so local X is the hinge and
//   anatomical flexion is a NEGATIVE X twist (the demo flexes 0..-45.8°).
//   The authored rests are anatomically asymmetric — knee_L rest is 8°
//   hyperextended, knee_R rest is 8° flexed — so full extension is twist -8°
//   for the left and +8° for the right knee. Both knees get the same
//   anatomical allowance: 0..40° of total flexion (~25% trim of the 53.8°
//   peak) plus a small hyperextension margin, and each range contains the
//   rest pose (twist 0) so a neutral frame never clamps:
//   knee_L [-48, +2]  (straight at -8, 40° flexion at -48, rest 0 inside)
//   knee_R [-32, +10] (straight at +8, 40° flexion at -32, rest 0 inside)

export const defaultConstraintSettings = {
  'Stocky (target)': {
    limits: {
      arm_L: [
        { axis: [0, 1, 0], min: -35, max: 35 },
        { axis: [1, 0, 0], min: -25, max: 25 },
        { axis: [0, 0, 1], min: -15, max: 15 },
      ],
      arm_R: [
        { axis: [0, 1, 0], min: -35, max: 35 },
        { axis: [1, 0, 0], min: -25, max: 25 },
        { axis: [0, 0, 1], min: -15, max: 15 },
      ],
      elbow_L: [
        { axis: [1, 0, 0], min: -5, max: 20 },
        { axis: [0, 1, 0], min: -15, max: 15 },
      ],
      elbow_R: [
        { axis: [1, 0, 0], min: -5, max: 20 },
        { axis: [0, 1, 0], min: -15, max: 15 },
      ],
      thigh_L: [{ axis: [1, 0, 0], min: -25, max: 25 }],
      thigh_R: [{ axis: [1, 0, 0], min: -25, max: 25 }],
      knee_L: [{ axis: [1, 0, 0], min: -48, max: 2 }],
      knee_R: [{ axis: [1, 0, 0], min: -32, max: 10 }],
      ankle_L: [{ axis: [1, 0, 0], min: -10, max: 10 }],
      ankle_R: [{ axis: [1, 0, 0], min: -10, max: 10 }],
    },
    contacts: [
      {
        id: 'foot-L',
        effector: 'ankle_L',
        chain: ['thigh_L', 'knee_L'],
        priority: 10,
        mode: 'ground',
        groundY: 0.15,
        enter: 0.03,
        exit: 0.07,
        pin: false, // floor clamp: correct penetration only, no horizontal pin
        enabled: true,
      },
      {
        id: 'foot-R',
        effector: 'ankle_R',
        chain: ['thigh_R', 'knee_R'],
        priority: 10,
        mode: 'ground',
        groundY: 0.15,
        enter: 0.03,
        exit: 0.07,
        pin: false,
        enabled: true,
      },
      {
        // Demo of a specified hand target point; disabled by default.
        id: 'hand-goal',
        effector: 'hand_R',
        chain: ['arm_R', 'elbow_R', 'wrist_R'],
        priority: 20,
        mode: 'point',
        target: [-0.3, 1.5, -0.2],
        enabled: false,
      },
    ],
    ik: {
      maxIterations: 16,
      tolerance: 0.001,
      minImprovement: 1e-6,
      // Per-frame rotation cap for contact chains (deg). Forced jumps (e.g.
      // the leg snapping straight at the reach boundary) are spread over
      // several frames; the shortfall is reported as residual. 0 disables.
      maxFrameStepDeg: 12,
    },
  },
};

// Deep-cloned defaults for a target skeleton name (null when unknown).
export function defaultConstraintsFor(targetName) {
  const s = defaultConstraintSettings[targetName];
  return s ? JSON.parse(JSON.stringify(s)) : null;
}
