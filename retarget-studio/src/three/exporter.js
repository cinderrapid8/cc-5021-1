// Export the retargeted result as a genuinely playable asset rather than a
// screenshot. Two outputs are produced:
//   1. GLB via GLTFExporter: a named bone hierarchy + AnimationClip (quaternion
//      and vector keyframe tracks), expressed in the TARGET's own authored
//      frame so external tools see its native Z-up/meters convention.
//   2. A self-describing JSON clip the workbench can reload and play directly.

import * as THREE from 'three';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { qmul, qconj, qrot } from '../engine/vecquat.js';

// Rotate a canonical quaternion back into the target authored frame:
// Qa = F^-1 * Qc * F.
function canonicalToAuthoredQuat(frame, qc) {
  const fi = qconj(frame.rotation);
  return qmul(qmul(fi, qc), frame.rotation);
}

function canonicalToAuthoredVec(frame, p) {
  const fi = qconj(frame.rotation);
  const r = qrot(fi, p);
  // authored = canonical(meters) rotated back, then divided by meter scale.
  return [r[0] / frame.scale, r[1] / frame.scale, r[2] / frame.scale];
}

// Build a THREE.Object3D hierarchy from the target skeleton's authored data.
function buildObjectHierarchy(tgtSkel) {
  const nodes = new Map();
  for (const name of tgtSkel.ordered) {
    const j = tgtSkel.byName.get(name);
    const node = new THREE.Object3D();
    node.name = name;
    const off = j.offset;
    node.position.fromArray(off);
    node.quaternion.fromArray(j.rest);
    nodes.set(name, node);
  }
  let root = null;
  for (const name of tgtSkel.ordered) {
    const j = tgtSkel.byName.get(name);
    if (j.parent === null) root = nodes.get(name);
    else nodes.get(j.parent).add(nodes.get(name));
  }
  return { nodes, root };
}

// Convert retargeted canonical frames into authored-frame keyframe tracks.
function buildTracks(tgtSkel, frame, retarget) {
  const tracks = [];
  const times = retarget.frames.map((f) => f.time);
  const rootName = tgtSkel.ordered[0];

  for (const name of tgtSkel.ordered) {
    const qValues = [];
    const pValues = [];
    for (const f of retarget.frames) {
      const qa = canonicalToAuthoredQuat(frame, f.localQuats.get(name));
      qValues.push(qa[0], qa[1], qa[2], qa[3]);
      if (name === rootName) {
        const pa = canonicalToAuthoredVec(frame, f.rootPos);
        pValues.push(pa[0], pa[1], pa[2]);
      }
    }
    tracks.push(new THREE.QuaternionKeyframeTrack(
      `${name}.quaternion`, Float32Array.from(times), Float32Array.from(qValues)));
    if (name === rootName) {
      tracks.push(new THREE.VectorKeyframeTrack(
        `${name}.position`, Float32Array.from(times), Float32Array.from(pValues)));
    }
  }
  return tracks;
}

// Produce a THREE.AnimationClip plus the object hierarchy it drives.
export function buildPlayableClip(tgtSkel, retarget) {
  const { nodes, root } = buildObjectHierarchy(tgtSkel);
  const tracks = buildTracks(tgtSkel, tgtSkel.frame, retarget);
  const clip = new THREE.AnimationClip('retargeted', retarget.duration, tracks);
  return { root, nodes, clip };
}

// Export GLB. Returns a promise resolving to an ArrayBuffer.
export function exportGLB(tgtSkel, retarget) {
  const { root, clip } = buildPlayableClip(tgtSkel, retarget);
  const exporter = new GLTFExporter();
  return new Promise((resolve, reject) => {
    exporter.parse(
      root,
      (result) => resolve(result),
      (err) => reject(err),
      { binary: true, animations: [clip], embedImages: false },
    );
  });
}

// Self-describing JSON the workbench can reload without Three.js.
export function exportJSON(tgtSkel, retarget) {
  const frame = tgtSkel.frame;
  return {
    format: 'retarget-studio/clip',
    version: 1,
    targetSkeleton: tgtSkel.name,
    authoredFrame: frame.spec,
    duration: retarget.duration,
    loop: retarget.loop,
    policy: retarget.policy,
    mapping: retarget.mapping,
    rootScale: retarget.rootScale,
    frames: retarget.frames.map((f) => ({
      time: f.time,
      rootPos: canonicalToAuthoredVec(frame, f.rootPos),
      locals: Object.fromEntries([...f.localQuats.entries()].map(
        ([name, qc]) => [name, canonicalToAuthoredQuat(frame, qc)])),
    })),
  };
}

export function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
