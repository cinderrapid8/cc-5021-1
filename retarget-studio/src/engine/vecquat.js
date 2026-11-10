// Minimal dependency-free vector / quaternion math.
// Vectors: [x, y, z]. Quaternions: [x, y, z, w] (w scalar last).

export const v = (x = 0, y = 0, z = 0) => [x, y, z];
export const q = (x = 0, y = 0, z = 0, w = 1) => [x, y, z, w];

export const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vcross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const vlen = (a) => Math.hypot(a[0], a[1], a[2]);
export const vnorm = (a) => {
  const l = vlen(a) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
export const vdist = (a, b) => vlen(vsub(a, b));

// Convert an axis+angle (radians) into a unit quaternion.
export function axisAngle(axis, angle) {
  const n = vnorm(axis);
  const h = angle * 0.5;
  const s = Math.sin(h);
  return [n[0] * s, n[1] * s, n[2] * s, Math.cos(h)];
}

export function qnorm(a) {
  const l = Math.hypot(a[0], a[1], a[2], a[3]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l, a[3] / l];
}

// Hamilton product a * b (apply b first, then a).
export function qmul(a, b) {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}

export const qconj = (a) => [-a[0], -a[1], -a[2], a[3]];
export const qidentity = () => [0, 0, 0, 1];

// Rotate vector p by unit quaternion a.
export function qrot(a, p) {
  // a * (p,0) * a^-1
  const u = [a[0], a[1], a[2]];
  const w = a[3];
  const t = vscale(vcross(u, p), 2);
  return vadd(vadd(p, vscale(t, w)), vcross(u, t));
}

// Relative rotation from a to b: b * a^-1.
export const qdelta = (a, b) => qmul(b, qconj(a));

// Smallest signed angle (radians) of the rotation represented by a.
export function qangle(a) {
  const n = qnorm(a);
  return 2 * Math.acos(Math.min(1, Math.abs(n[3])));
}

// Geodesic distance between two unit quaternions, radians in [0, pi].
export function qdist(a, b) {
  return qangle(qmul(qconj(a), b));
}

// Flip sign of b when needed so that dot(a,b) >= 0 (same hemisphere -> shortest path).
export function qcanonical(reference, b) {
  const d = reference[0] * b[0] + reference[1] * b[1] + reference[2] * b[2] + reference[3] * b[3];
  return d < 0 ? [-b[0], -b[1], -b[2], -b[3]] : b;
}

// Spherical linear interpolation with shortest-path sign handling.
export function slerp(a, b, t) {
  let bb = qcanonical(a, b);
  let cos = a[0] * bb[0] + a[1] * bb[1] + a[2] * bb[2] + a[3] * bb[3];
  if (cos > 0.9995) {
    // Near-parallel: linear + normalize is numerically safer.
    return qnorm([
      a[0] + t * (bb[0] - a[0]),
      a[1] + t * (bb[1] - a[1]),
      a[2] + t * (bb[2] - a[2]),
      a[3] + t * (bb[3] - a[3]),
    ]);
  }
  cos = Math.max(-1, Math.min(1, cos));
  const theta = Math.acos(cos);
  const sin = Math.sin(theta);
  const wa = Math.sin((1 - t) * theta) / sin;
  const wb = Math.sin(t * theta) / sin;
  return qnorm([
    a[0] * wa + bb[0] * wb,
    a[1] * wa + bb[1] * wb,
    a[2] * wa + bb[2] * wb,
    a[3] * wa + bb[3] * wb,
  ]);
}

// Decompose a rotation quaternion into three basis columns (the rotated X/Y/Z axes).
export function basisFromQuat(qq) {
  return {
    xAxis: vnorm(qrot(qq, [1, 0, 0])),
    yAxis: vnorm(qrot(qq, [0, 1, 0])),
    zAxis: vnorm(qrot(qq, [0, 0, 1])),
  };
}

// Build a quaternion that maps the supplied basis axes (vectors in canonical
// coordinates) onto the canonical +X/+Y/+Z. The axis vectors are the rows of
// the rotation, so they are laid out contiguously per axis.
export function quatFromAxes(xAxis, yAxis, zAxis) {
  const m = [
    xAxis[0], xAxis[1], xAxis[2],
    yAxis[0], yAxis[1], yAxis[2],
    zAxis[0], zAxis[1], zAxis[2],
  ];
  return quatFromMat3(m);
}

// Row-major rotation matrix -> quaternion, standard Shepperd method.
// m[r*3+c] = component r of basis axis c.
export function quatFromMat3(m) {
  const m00 = m[0], m01 = m[1], m02 = m[2];
  const m10 = m[3], m11 = m[4], m12 = m[5];
  const m20 = m[6], m21 = m[7], m22 = m[8];
  const tr = m00 + m11 + m22;
  let x, y, z, w;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2; // s = 4w
    w = 0.25 * s;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2; // s = 4x
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2; // s = 4y
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2; // s = 4z
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }
  return qnorm([x, y, z, w]);
}

// Row-major 3x3 matrix multiply.
export function mat3Mul(a, b) {
  const out = new Array(9).fill(0);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      out[r * 3 + c] =
        a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
    }
  }
  return out;
}

export const deg = (rad) => (rad * 180) / Math.PI;
export const rad = (d) => (d * Math.PI) / 180;
