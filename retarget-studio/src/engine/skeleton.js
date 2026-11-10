// Skeleton data model and forward kinematics.
//
// Authored skeletons are plain JSON:
//   { name, frame: {up, forward, handedness, unitsPerMeter},
//     joints: [{ name, parent: string|null, offset:[x,y,z], rest:[x,y,z,w] }] }
// offsets and rest quaternions are authored in that skeleton's own frame.
// All runtime math happens in the canonical frame (Y up, -Z forward, meters).

import { buildFrame, toCanonicalVec, toCanonicalQuat } from './frame.js';
import { qmul, qrot, vadd, vlen, qidentity } from './vecquat.js';

export function loadSkeleton(raw) {
  if (!raw || !Array.isArray(raw.joints) || raw.joints.length === 0) {
    throw new Error('skeleton needs a non-empty joints array');
  }
  const frame = buildFrame(raw.frame || {});
  const byName = new Map();
  for (const j of raw.joints) {
    if (byName.has(j.name)) throw new Error(`duplicate joint ${j.name}`);
    if (!Array.isArray(j.offset) || j.offset.length !== 3) {
      throw new Error(`joint ${j.name} needs offset [x,y,z]`);
    }
    const rest = j.rest && j.rest.length === 4 ? j.rest.slice() : qidentity();
    byName.set(j.name, {
      name: j.name,
      parent: j.parent ?? null,
      offset: j.offset.slice(),
      rest,
      side: j.side || null,
    });
  }
  for (const j of byName.values()) {
    if (j.parent !== null && !byName.has(j.parent)) {
      throw new Error(`joint ${j.name} references missing parent ${j.parent}`);
    }
  }
  // Topological order (parents before children).
  const ordered = [];
  const visit = (name) => {
    const j = byName.get(name);
    if (j.parent) visit(j.parent);
    if (!ordered.includes(name)) ordered.push(name);
  };
  for (const name of byName.keys()) visit(name);

  return { name: raw.name || 'skeleton', frame, byName, ordered };
}

// Per-joint authored data converted into canonical meters / canonical rotation.
export function canonicalRestPose(skel) {
  const localQuat = new Map();
  const offset = new Map();
  const worldQuat = new Map();
  const worldPos = new Map();
  const boneLength = new Map();

  for (const name of skel.ordered) {
    const j = skel.byName.get(name);
    const off = toCanonicalVec(skel.frame, j.offset);
    const lq = toCanonicalQuat(skel.frame, j.rest);
    offset.set(name, off);
    localQuat.set(name, lq);
    boneLength.set(name, vlen(off));

    if (j.parent === null) {
      worldQuat.set(name, lq);
      worldPos.set(name, off.slice());
    } else {
      const pw = worldQuat.get(j.parent);
      worldQuat.set(name, qmul(pw, lq));
      worldPos.set(name, vadd(worldPos.get(j.parent), qrot(pw, off)));
    }
  }
  return { localQuat, offset, worldQuat, worldPos, boneLength };
}

// Forward kinematics given canonical local rotations and a canonical root pose.
// localQuats: Map<name, canonical quaternion> for NON-ROOT joints (the root's
// world orientation is supplied separately as rootQuat). Missing entries fall
// back to rest. Root translation and orientation are therefore independent
// inputs, never baked into child joint rotations.
export function fk(skel, rest, localQuats, rootPos, rootQuat) {
  const worldQuat = new Map();
  const worldPos = new Map();
  for (const name of skel.ordered) {
    const j = skel.byName.get(name);
    if (j.parent === null) {
      worldQuat.set(name, rootQuat || rest.localQuat.get(name));
      worldPos.set(name, (rootPos || rest.offset.get(name)).slice());
    } else {
      const lq = localQuats.get(name) || rest.localQuat.get(name);
      const pw = worldQuat.get(j.parent);
      const wq = qmul(pw, lq);
      worldQuat.set(name, wq);
      worldPos.set(name, vadd(worldPos.get(j.parent), qrot(pw, rest.offset.get(name))));
    }
  }
  return { worldQuat, worldPos };
}

// Chain of joint names from root to the given joint.
export function chainTo(skel, name) {
  const out = [];
  let cur = name;
  while (cur) {
    out.unshift(cur);
    cur = skel.byName.get(cur).parent;
  }
  return out;
}

// Lowest common ancestor of two joints.
export function commonAncestor(skel, a, b) {
  const ca = chainTo(skel, a);
  let cur = b;
  while (cur && !ca.includes(cur)) cur = skel.byName.get(cur).parent;
  return cur;
}
