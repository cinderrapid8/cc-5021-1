// Three.js skeleton viewer. Renders one skeleton in canonical world space
// (Y up, meters) as a stick figure: tapered bones, joint spheres, and a ground
// grid. Joints are color coded by mapping status so skipped / unmapped joints
// are visible rather than silently dropped.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COLORS = {
  mapped: 0x35c48a,
  unmapped: 0xe0b13e,   // target-only joint, never driven by the source
  skipped: 0x8a93a6,    // source joint with no target counterpart
  root: 0x5aa9ff,
  bone: 0xb9c2d4,
  ghost: 0xff7a5c,
};

function toThreeQ(q) {
  return new THREE.Quaternion(q[0], q[1], q[2], q[3]);
}
function toThreeV(p) {
  return new THREE.Vector3(p[0], p[1], p[2]);
}

// A single reusable viewer with its own renderer/camera/controls.
export class SkeletonViewer {
  constructor(container, { showGrid = true } = {}) {
    this.container = container;
    const w = Math.max(1, container.clientWidth);
    const h = Math.max(1, container.clientHeight);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14181f);

    this.camera = new THREE.PerspectiveCamera(45, w / h, 0.01, 100);
    this.camera.position.set(1.6, 1.4, 2.4);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(w, h);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 1, 0);

    const hemi = new THREE.HemisphereLight(0xdfe8ff, 0x2a2f3a, 1.1);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.4);
    key.position.set(2, 4, 3);
    this.scene.add(key);

    if (showGrid) {
      const grid = new THREE.GridHelper(4, 40, 0x3a4356, 0x232a38);
      this.scene.add(grid);
      const axes = new THREE.AxesHelper(0.35);
      this.scene.add(axes);
    }

    this.rig = new THREE.Group();
    this.scene.add(this.rig);
    this.boneMeshes = [];
    this.jointMeshes = new Map();
    this.labelSprites = new Map();
    this._ghost = null;
    this._markerGeo = new THREE.OctahedronGeometry(0.032);
    this._markerPool = [];
    this._ro = new ResizeObserver(() => this.resize());
    this._ro.observe(container);
  }

  clearRig() {
    while (this.rig.children.length) {
      const c = this.rig.children.pop();
      c.traverse?.((o) => {
        if (o.geometry) o.geometry.dispose();
        if (o.material) o.material.dispose();
      });
    }
    this.boneMeshes = [];
    this.jointMeshes.clear();
    this.labelSprites.clear();
  }

  // Build the static rig geometry from a loaded skeleton + canonical rest pose.
  // statusOf: Map<jointName, 'mapped'|'unmapped'|'skipped'|'root'>
  buildRig(skel, rest, statusOf) {
    this.clearRig();
    this.skel = skel;
    this.rest = rest;

    const jointGeo = new THREE.SphereGeometry(0.022, 16, 12);
    for (const name of skel.ordered) {
      const j = skel.byName.get(name);
      const status = statusOf?.get(name) || 'mapped';
      const mat = new THREE.MeshStandardMaterial({
        color: COLORS[status] ?? COLORS.mapped,
        roughness: 0.5, metalness: 0.1,
      });
      const sphere = new THREE.Mesh(jointGeo, mat);
      sphere.userData.joint = name;
      this.rig.add(sphere);
      this.jointMeshes.set(name, sphere);

      if (j.parent !== null) {
        const len = rest.boneLength.get(name);
        const geo = new THREE.CylinderGeometry(0.012, 0.012, Math.max(len, 1e-4), 10);
        geo.translate(0, len / 2, 0); // pivot at parent end
        const bmat = new THREE.MeshStandardMaterial({
          color: COLORS.bone, roughness: 0.6,
        });
        const bone = new THREE.Mesh(geo, bmat);
        bone.userData.joint = name;
        this.rig.add(bone);
        this.boneMeshes.push(bone);
      }
    }
  }

  // Update transforms from a solved FK pose (canonical world positions/quats).
  update(pose) {
    const { worldPos, worldQuat } = pose;
    for (const name of this.skel.ordered) {
      const j = this.skel.byName.get(name);
      const sphere = this.jointMeshes.get(name);
      sphere.position.copy(toThreeV(worldPos.get(name)));
      if (j.parent === null) {
        sphere.quaternion.copy(toThreeQ(worldQuat.get(name)));
      }
    }
    for (const bone of this.boneMeshes) {
      const name = bone.userData.joint;
      const parent = this.skel.byName.get(name).parent;
      const p = worldPos.get(parent);
      bone.position.copy(toThreeV(p));
      // Orient the bone's local +Y toward the child joint.
      const child = worldPos.get(name);
      const dir = toThreeV(child).sub(toThreeV(p)).normalize();
      bone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    }
  }

  // Show a translucent reference skeleton (e.g. the source inside the target
  // view) without it being mistaken for the retarget result. Clearly ghosted.
  setGhost(skel, rest, pose) {
    if (this._ghost) {
      this.rig.remove(this._ghost);
      this._ghost.traverse((o) => {
        if (o.geometry) o.geometry.dispose();
        if (o.material) o.material.dispose();
      });
    }
    const g = new THREE.Group();
    const mat = new THREE.MeshBasicMaterial({
      color: COLORS.ghost, transparent: true, opacity: 0.28, wireframe: true,
    });
    const geo = new THREE.SphereGeometry(0.018, 8, 6);
    for (const name of skel.ordered) {
      const s = new THREE.Mesh(geo, mat);
      s.position.copy(toThreeV(pose.worldPos.get(name)));
      g.add(s);
      const j = skel.byName.get(name);
      if (j.parent !== null) {
        const a = toThreeV(pose.worldPos.get(j.parent));
        const b = toThreeV(pose.worldPos.get(name));
        const line = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([a, b]),
          new THREE.LineBasicMaterial({ color: COLORS.ghost, transparent: true, opacity: 0.35 }),
        );
        g.add(line);
      }
    }
    this._ghost = g;
    this.rig.add(g);
  }

  clearGhost() {
    if (this._ghost) {
      this.rig.remove(this._ghost);
      this._ghost = null;
    }
  }

  // Emissive highlight for joints with a constraint trigger on the current
  // frame. emphasis: Map<jointName, hexColor>; joints not listed are cleared.
  // This makes the offending joint itself visible — nothing is hidden behind
  // limbs or camera moves.
  setJointEmphasis(emphasis) {
    for (const [name, mesh] of this.jointMeshes) {
      const hex = emphasis?.get(name) || 0;
      if ((mesh.userData.emphHex || 0) === hex) continue;
      mesh.userData.emphHex = hex;
      mesh.material.emissive.setHex(hex);
      mesh.material.emissiveIntensity = hex ? 0.85 : 0;
    }
  }

  // Contact target markers. markers: [{ position:[x,y,z], residual }]; the
  // marker turns red while the effector cannot reach it (residual > 1 mm).
  setMarkers(markers) {
    while (this._markerPool.length < markers.length) {
      const m = new THREE.Mesh(this._markerGeo, new THREE.MeshBasicMaterial({
        color: 0xffd166, transparent: true, opacity: 0.9, depthTest: false,
      }));
      m.renderOrder = 10;
      this.scene.add(m);
      this._markerPool.push(m);
    }
    for (let i = 0; i < this._markerPool.length; i++) {
      const m = this._markerPool[i];
      const spec = markers[i];
      if (!spec) { m.visible = false; continue; }
      m.visible = true;
      m.position.copy(toThreeV(spec.position));
      m.material.color.setHex(spec.residual > 1e-3 ? 0xff6b6b : 0xffd166);
    }
  }

  frameCamera(center = [0, 1, 0], radius = 1.4) {
    this.controls.target.copy(toThreeV(center));
    this.camera.position.set(center[0] + radius, center[1] + radius * 0.8, center[2] + radius * 1.4);
    this.controls.update();
  }

  resize() {
    const w = Math.max(1, this.container.clientWidth);
    const h = Math.max(1, this.container.clientHeight);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    this._ro.disconnect();
    this.clearRig();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}

export { COLORS };
