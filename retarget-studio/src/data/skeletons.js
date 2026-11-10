// Two deliberately different authored skeletons.
//
// Source ("Slim"): Y-up, forward -Z, right-handed, centimeters, clean T-pose,
//                  19 joints incl. neck and toes.
// Target ("Stocky"): Z-up, forward +Y, right-handed, meters, A-pose with bent
//                    knees and splayed arms, shorter limbs, NO neck and NO toes,
//                    plus two extra detail joints (belt twist, skull) that the
//                    source cannot drive. Joint names are intentionally
//                    different so matching by name alone is impossible.
//
// All quaternions: [x, y, z, w], authored in each skeleton's own frame.

export const sourceSkeleton = {
  name: 'Slim (source)',
  frame: { up: 'py', forward: 'nz', handedness: 1, unitsPerMeter: 100 },
  joints: [
    { name: 'root', parent: null, offset: [0, 100, 0], rest: [0, 0, 0, 1] },
    { name: 'spine', parent: 'root', offset: [0, 15, 0], rest: [0, 0, 0, 1] },
    { name: 'chest', parent: 'spine', offset: [0, 15, 0], rest: [0, 0, 0, 1] },
    { name: 'neck', parent: 'chest', offset: [0, 15, 0], rest: [0, 0, 0, 1] },
    { name: 'head', parent: 'neck', offset: [0, 12, 0], rest: [0, 0, 0, 1] },

    { name: 'L_shoulder', parent: 'chest', offset: [8, 8, 0], rest: [0, 0, 0, 1] },
    { name: 'L_elbow', parent: 'L_shoulder', offset: [28, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'L_wrist', parent: 'L_elbow', offset: [26, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'L_hand', parent: 'L_wrist', offset: [8, 0, 0], rest: [0, 0, 0, 1] },

    { name: 'R_shoulder', parent: 'chest', offset: [-8, 8, 0], rest: [0, 0, 0, 1] },
    { name: 'R_elbow', parent: 'R_shoulder', offset: [-28, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'R_wrist', parent: 'R_elbow', offset: [-26, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'R_hand', parent: 'R_wrist', offset: [-8, 0, 0], rest: [0, 0, 0, 1] },

    { name: 'L_hip', parent: 'root', offset: [10, -5, 0], rest: [0, 0, 0, 1] },
    { name: 'L_knee', parent: 'L_hip', offset: [0, -42, 0], rest: [0, 0, 0, 1] },
    { name: 'L_ankle', parent: 'L_knee', offset: [0, -42, 0], rest: [0, 0, 0, 1] },
    { name: 'L_toe', parent: 'L_ankle', offset: [0, -3, 8], rest: [0, 0, 0, 1] },

    { name: 'R_hip', parent: 'root', offset: [-10, -5, 0], rest: [0, 0, 0, 1] },
    { name: 'R_knee', parent: 'R_hip', offset: [0, -42, 0], rest: [0, 0, 0, 1] },
    { name: 'R_ankle', parent: 'R_knee', offset: [0, -42, 0], rest: [0, 0, 0, 1] },
    { name: 'R_toe', parent: 'R_ankle', offset: [0, -3, 8], rest: [0, 0, 0, 1] },
  ],
};

// Helper rotations in the target's authored Z-up frame.
const ry = (deg) => {
  const r = (deg * Math.PI) / 180;
  return [0, Math.sin(r / 2), 0, Math.cos(r / 2)];
};
const rx = (deg) => {
  const r = (deg * Math.PI) / 180;
  return [Math.sin(r / 2), 0, 0, Math.cos(r / 2)];
};

export const targetSkeleton = {
  name: 'Stocky (target)',
  // Z-up, forward +Y, meters. Different axes AND units from the source.
  frame: { up: 'pz', forward: 'py', handedness: 1, unitsPerMeter: 1 },
  joints: [
    { name: 'Pelvis', parent: null, offset: [0, 0, 0.95], rest: [0, 0, 0, 1] },
    // Extra detail joint with no source counterpart.
    { name: 'Belt_Twist', parent: 'Pelvis', offset: [0, 0, 0.03], rest: [0, 0, 0, 1] },
    { name: 'Spine', parent: 'Pelvis', offset: [0, 0, 0.12], rest: [0, 0, 0, 1] },
    { name: 'Chest', parent: 'Spine', offset: [0, 0, 0.12], rest: [0, 0, 0, 1] },
    // No neck: source "neck" has nowhere to go.
    { name: 'Head', parent: 'Chest', offset: [0, 0, 0.22], rest: [0, 0, 0, 1] },
    { name: 'Skull', parent: 'Head', offset: [0, 0, 0.1], rest: [0, 0, 0, 1] },

    // A-pose: arms splayed 35 degrees downward about the authored Y axis.
    { name: 'arm_L', parent: 'Chest', offset: [0.14, 0, 0.09], rest: ry(35) },
    { name: 'elbow_L', parent: 'arm_L', offset: [0.22, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'wrist_L', parent: 'elbow_L', offset: [0.2, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'hand_L', parent: 'wrist_L', offset: [0.06, 0, 0], rest: [0, 0, 0, 1] },

    { name: 'arm_R', parent: 'Chest', offset: [-0.14, 0, 0.09], rest: ry(-35) },
    { name: 'elbow_R', parent: 'arm_R', offset: [-0.22, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'wrist_R', parent: 'elbow_R', offset: [-0.2, 0, 0], rest: [0, 0, 0, 1] },
    { name: 'hand_R', parent: 'wrist_R', offset: [-0.06, 0, 0], rest: [0, 0, 0, 1] },

    // Slightly pre-bent knees (8 degrees about authored X).
    { name: 'thigh_L', parent: 'Pelvis', offset: [0.09, 0, -0.04], rest: [0, 0, 0, 1] },
    { name: 'knee_L', parent: 'thigh_L', offset: [0, 0, -0.38], rest: rx(8) },
    { name: 'ankle_L', parent: 'knee_L', offset: [0, 0, -0.37], rest: [0, 0, 0, 1] },
    // No toes.

    { name: 'thigh_R', parent: 'Pelvis', offset: [-0.09, 0, -0.04], rest: [0, 0, 0, 1] },
    { name: 'knee_R', parent: 'thigh_R', offset: [0, 0, -0.38], rest: rx(-8) },
    { name: 'ankle_R', parent: 'knee_R', offset: [0, 0, -0.37], rest: [0, 0, 0, 1] },
  ],
};

// Explicit target -> source correspondence. A target joint with null source is
// visibly unmapped; a source joint absent from the values is skipped/compensated.
export const defaultMapping = {
  Pelvis: 'root',
  // Belt_Twist intentionally unmapped (target-only joint).
  Spine: 'spine',
  Chest: 'chest',
  Head: 'head', // source neck skipped: head attaches one level higher
  // Skull intentionally unmapped (target-only joint).
  arm_L: 'L_shoulder',
  elbow_L: 'L_elbow',
  wrist_L: 'L_wrist',
  hand_L: 'L_hand',
  arm_R: 'R_shoulder',
  elbow_R: 'R_elbow',
  wrist_R: 'R_wrist',
  hand_R: 'R_hand',
  thigh_L: 'L_hip',
  knee_L: 'L_knee',
  ankle_L: 'L_ankle',
  // source L_toe skipped (target has no toe)
  thigh_R: 'R_hip',
  knee_R: 'R_knee',
  ankle_R: 'R_ankle',
  // source R_toe skipped
};
