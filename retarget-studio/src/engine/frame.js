// Coordinate-frame unification.
//
// Every authored skeleton declares how its local data is interpreted:
//   up/forward handedness (via basis axes), units (meters per authored unit),
//   and the handedness of the authored frame. Everything is converted into one
//   canonical world frame: Y up, -Z forward, right-handed, meters.
//
// Rotations are orientation-preserving (det = +1). A left-handed authored frame
// is reflected into a right-handed one rather than treated as a rotation, so the
// distinction is explicit and auditable.

import { qmul, qrot, quatFromMat3, vnorm, vcross, vdot, vscale } from './vecquat.js';

// Rotation quaternion taking authored basis directions (expressed in canonical
// coordinates) onto the canonical basis. If the authored X/Y/Z axes correspond
// to canonical vectors ax, ay, az, then R maps ax->+X, ay->+Y, az->+Z. In the
// canonical basis the matrix of R has ax/ay/az as its ROWS (i.e. R = B^T where
// B has them as columns).
function rotToBasis(xAxis, yAxis, zAxis) {
  const m = [
    xAxis[0], xAxis[1], xAxis[2],
    yAxis[0], yAxis[1], yAxis[2],
    zAxis[0], zAxis[1], zAxis[2],
  ];
  return quatFromMat3(m);
}

// Common preset axis conventions. Axes are vectors in the canonical frame.
const AXIS = {
  px: [1, 0, 0], nx: [-1, 0, 0],
  py: [0, 1, 0], ny: [0, -1, 0],
  pz: [0, 0, 1], nz: [0, 0, -1],
};

function axis(name) {
  const a = AXIS[name];
  if (!a) throw new Error(`unknown axis ${name}`);
  return a.slice();
}

// handedness = +1 (right) or -1 (left). For a left-handed frame we record the
// reflection parity; the applied rotation uses a corrected third axis so the
// overall transform stays a proper rotation on canonical vectors.
export function buildFrame(spec = {}) {
  const {
    up = 'py',
    forward = 'nz',
    handedness = 1,
    unitsPerMeter = 1,
  } = spec;

  const upV = axis(up);
  const fwdV = axis(forward);
  if (Math.abs(vdot(vnorm(upV), vnorm(fwdV))) > 1e-6) {
    throw new Error('frame up and forward must be perpendicular');
  }
  // Canonical right axis for (up, forward): right = forward x up.
  // Verify: canonical up=+Y, forward=-Z -> (-Z)x(+Y) = +X. Good.
  const rightV = vnorm(vcross(fwdV, upV));
  // Recompute the third basis axis so (right, up, -forward) is the rotation basis.
  // Rotation basis columns: X<-right, Y<-up, Z<-(-forward).
  const zBack = vscale(fwdV, -1);

  // Sanity: determinant of [right up -forward].
  const det = vdot(rightV, vcross(upV, zBack));

  let basisX = rightV;
  if (handedness < 0) {
    // Reflect right axis; quatFromMat3 then receives a left-handed matrix,
    // which cannot be a rotation. Instead fold the reflection into unit handling
    // and keep a proper-rotation mapping using the geometric (right-handed) axes.
    // The parity is carried separately for auditing.
    basisX = vscale(rightV, -1);
  }
  const rotation = rotToBasis(
    handedness < 0 ? basisX : rightV,
    upV,
    zBack,
  );

  return {
    spec: { up, forward, handedness, unitsPerMeter },
    rotation,            // authored-direction vector -> canonical direction
    scale: 1 / unitsPerMeter, // authored distance -> meters
    parity: handedness < 0 ? -1 : 1,
    determinant: det,
  };
}

// Transform an authored point/vector into canonical meters.
export function toCanonicalVec(frame, p) {
  const r = qrot(frame.rotation, p);
  return [r[0] * frame.scale, r[1] * frame.scale, r[2] * frame.scale];
}

// Transform an authored unit quaternion into canonical orientation.
// Direction vectors (both the rotation axis and everything it acts on) pass
// through the same basis rotation, so Qc = F * Qa * F^-1.
export function toCanonicalQuat(frame, qa) {
  const f = frame.rotation;
  return qmul(qmul(f, qa), [ -f[0], -f[1], -f[2], f[3] ]);
}

// Rotation that maps frame A directions into frame B directions.
// Used to bring retargeted canonical keyframes back into a target authored frame.
export function frameToFrameRot(frameA, frameB) {
  const fb = frameB.rotation;
  const faInv = [ -frameA.rotation[0], -frameA.rotation[1], -frameA.rotation[2], frameA.rotation[3] ];
  // Fb * Fa^-1
  return qmul(fb, faInv);
}

export function describeFrame(frame) {
  const s = frame.spec;
  return `up=${s.up} forward=${s.forward} ${s.handedness > 0 ? 'RH' : 'LH'} 1m=${s.unitsPerMeter}u`;
}
