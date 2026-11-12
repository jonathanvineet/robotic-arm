// twin.js — Three.js digital twin. Solid arm = actual position reported by
// the device; translucent "ghost" = commanded target. Units are millimetres.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { toJoint, gripOpen, maxReach, fk } from './kinematics.js';
import { loadCad, buildCadArm } from './cad.js';

const D2R = Math.PI / 180;
export const HELPER_LAYER = 1; // drawn in the 3D view only, never seen by the simulated camera
export function toHelperLayer(obj) {
  obj.traverse((o) => o.layers.set(HELPER_LAYER));
  return obj;
}

// Styled after Final_assembly.SLDASM: blue base plate, red turret, white upper
// arm, blue forearm with MG996R elbow servo, orange gear-driven claw gripper.
function buildArm(L, mats) {
  const { plate, turret, upper, fore, servo, claw, joint } = mats;
  const shadow = (m) => {
    m.castShadow = true;
    m.receiveShadow = true;
    return m;
  };
  const box = (w, h, d, mat, x = 0, y = h / 2, z = 0) => {
    const m = shadow(new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat));
    m.position.set(x, y, z);
    return m;
  };
  const cyl = (r, h, mat, axis = 'y', seg = 40) => {
    const m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), mat));
    if (axis === 'z') m.rotation.x = Math.PI / 2;
    if (axis === 'x') m.rotation.z = Math.PI / 2;
    return m;
  };
  // box spanning two points in the XY plane of a group
  const bar = (p, q, t, d, mat) => {
    const dx = q[0] - p[0], dy = q[1] - p[1];
    const m = box(t, Math.hypot(dx, dy), d, mat, (p[0] + q[0]) / 2, (p[1] + q[1]) / 2);
    m.rotation.z = -Math.atan2(dx, dy);
    return m;
  };
  const servoBox = (w, h, d, y = 0) => {
    const g = new THREE.Group();
    g.add(box(w, h, d, servo, 0, y));
    g.add(box(w + 10, 2.5, d, servo, 0, y + h * 0.3)); // mounting ears
    return g;
  };

  const root = new THREE.Group();
  root.add(box(150, 6, 150, plate, 0, 3)); // base plate (fixed)

  const yaw = new THREE.Group();
  yaw.position.y = 6;
  root.add(yaw);
  const th = Math.max(20, L.base - 50);
  const tur = cyl(52, th, turret);
  tur.position.y = th / 2;
  yaw.add(tur);
  const ring = cyl(46, 4, plate);
  ring.position.y = th + 2;
  yaw.add(ring);
  // shoulder brackets + servo
  const sy = L.base - 6;
  yaw.add(box(8, sy - th, 46, turret, -26, th + (sy - th) / 2, 0));
  yaw.add(box(8, sy - th, 46, turret, 26, th + (sy - th) / 2, 0).translateX(0));
  const ss = servoBox(40, 20, 40);
  ss.position.set(0, th + 12, 0);
  yaw.add(ss);

  const shoulder = new THREE.Group();
  shoulder.position.set(L.fwd || 0, sy, -(L.side || 0));
  yaw.add(shoulder);
  shoulder.add(cyl(16, 62, joint, 'z'));
  // upper arm: twin white side plates with a slight bow (like arm_1)
  for (const z of [-24, 24]) {
    shoulder.add(bar([0, 0], [8, L.upper * 0.5], 10, 5, upper).translateZ(z));
    shoulder.add(bar([8, L.upper * 0.5], [0, L.upper], 10, 5, upper).translateZ(z));
  }
  shoulder.add(bar([0, L.upper * 0.15], [5, L.upper * 0.85], 14, 40, upper));

  const elbow = new THREE.Group();
  elbow.position.y = L.upper;
  shoulder.add(elbow);
  const es = servoBox(40, 20, 42); // MG996R on the elbow
  es.rotation.x = Math.PI / 2;
  elbow.add(es);
  elbow.add(box(28, L.fore, 26, fore));
  elbow.add(box(4, L.fore * 0.8, 28, turret, 15, L.fore * 0.45)); // red side strip

  const wrist = new THREE.Group();
  wrist.position.y = L.fore;
  elbow.add(wrist);
  const ws = servoBox(32, 16, 30);
  ws.rotation.x = Math.PI / 2;
  wrist.add(ws);
  wrist.add(box(22, L.wrist * 0.45, 20, fore));

  const roll = new THREE.Group();
  roll.position.y = L.wrist * 0.45;
  wrist.add(roll);
  roll.add(box(23, L.wrist * 0.4, 12, servo)); // MG90S
  roll.add(box(46, 6, 18, claw, 0, L.wrist * 0.55 - 3)); // gripper base
  const gears = [-10, 10].map((x) => {
    const g = cyl(9, 5, claw, 'z', 16);
    g.position.set(x, L.wrist * 0.55 + 2, 10);
    return g;
  });
  roll.add(...gears);

  const fingers = [-1, 1].map((side) => {
    const f = new THREE.Group();
    f.position.set(side * 10, L.wrist * 0.55, 0);
    const T = L.tool;
    const knee = [side * 9, T * 0.6];
    const tip = [-side * 7, T];
    f.add(bar([0, 0], knee, 7, 12, claw));
    f.add(bar(knee, tip, 6, 12, claw));
    f.userData.side = side;
    roll.add(f);
    return f;
  });

  return { root, yaw, shoulder, elbow, wrist, roll, fingers };
}

function poseArm(arm, s, J) {
  arm.yaw.rotation.y = toJoint(s[0], J[0]) * D2R;
  arm.shoulder.rotation.z = -toJoint(s[1], J[1]) * D2R;
  arm.elbow.rotation.z = -toJoint(s[2], J[2]) * D2R;
  arm.wrist.rotation.z = -toJoint(s[3], J[3]) * D2R;
  arm.roll.rotation.y = toJoint(s[4], J[4]) * D2R;
  const open = gripOpen(s[5], J[5]);
  if (arm.cad) for (const f of arm.fingers) f.rotation.z = f.userData.sign * (1 - open) * f.userData.close;
  else for (const f of arm.fingers) f.rotation.z = -f.userData.side * open * 0.5;
}

export class Twin {
  constructor(container) {
    this.container = container;
    this.links = null;
    this.opts = { ghost: true, trail: true, envelope: true };
    this.pose = null;
    this.ghostPose = null;
    this.J = null;

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);
    this.renderer = renderer;

    const scene = new THREE.Scene();
    this.scene = scene;
    const camera = new THREE.PerspectiveCamera(40, 1, 5, 8000);
    camera.layers.enableAll();
    this.camera = camera;
    this.frameHooks = [];
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(110, 120, 0);
    this.controls = controls;
    this.view('iso');

    scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1c22, 1.4));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(300, 700, 400);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -600, right: 600, top: 600, bottom: -600, near: 10, far: 2000 });
    scene.add(sun);

    // the table itself is added by world.js; these are view-only guides
    const grid = new THREE.GridHelper(1200, 24, 0x4a5060, 0x2a2e38);
    grid.material.transparent = true;
    grid.material.opacity = 0.35;
    grid.position.y = 0.4;
    scene.add(toHelperLayer(grid));
    const axes = new THREE.AxesHelper(80);
    axes.position.y = 1;
    scene.add(toHelperLayer(axes));

    const std = (color, roughness = 0.5, metalness = 0.1) => new THREE.MeshStandardMaterial({ color, roughness, metalness });
    this.mats = {
      plate: std(0x1f4fd1),
      turret: std(0xc8102e, 0.45),
      upper: std(0xe9ecef, 0.4),
      fore: std(0x2457d6),
      servo: std(0x2b2d33, 0.7),
      claw: std(0xf28c28, 0.45),
      joint: std(0x9aa3b2, 0.3, 0.8),
    };
    const ghostMat = new THREE.MeshBasicMaterial({
      color: 0x3fd0c9,
      transparent: true,
      opacity: 0.16,
      depthWrite: false,
    });
    this.ghostMats = Object.fromEntries(Object.keys(this.mats).map((k) => [k, ghostMat]));

    // TCP trail
    this.trailMax = 800;
    this.trailPos = new Float32Array(this.trailMax * 3);
    this.trailCount = 0;
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3));
    tg.setDrawRange(0, 0);
    this.trail = new THREE.Line(tg, new THREE.LineBasicMaterial({ color: 0x3fd0c9, transparent: true, opacity: 0.8 }));
    this.trail.frustumCulled = false;
    scene.add(toHelperLayer(this.trail));

    // target marker for IK
    this.marker = new THREE.Mesh(
      new THREE.SphereGeometry(6, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xffd166 }),
    );
    this.marker.visible = false;
    scene.add(toHelperLayer(this.marker));

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    let last = performance.now();
    const tick = () => {
      const now = performance.now();
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      for (const h of this.frameHooks) h(dt, now);
      controls.update();
      renderer.render(scene, camera);
      requestAnimationFrame(tick);
    };
    tick();
  }

  resize() {
    const w = this.container.clientWidth || 1;
    const h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  setLinks(L) {
    this.links = { ...L };
    for (const k of ['arm', 'ghost']) {
      if (this[k]) {
        this.scene.remove(this[k].root);
        this[k].root.traverse((o) => o.geometry?.dispose());
      }
    }
    if (this.cad && this.opts.cad !== false) {
      this.arm = buildCadArm(this.cad, (c) => this.cadMat(c));
      this.ghost = buildCadArm(this.cad, () => this.ghostMats.plate);
    } else {
      this.arm = buildArm(L, this.mats);
      this.ghost = buildArm(L, this.ghostMats);
    }
    this.ghost.root.traverse((o) => (o.castShadow = o.receiveShadow = false));
    toHelperLayer(this.ghost.root);
    this.scene.add(this.arm.root, this.ghost.root);
    this.onRebuild?.();

    if (this.ring) this.scene.remove(this.ring);
    const R = maxReach(L);
    this.ring = new THREE.Mesh(
      new THREE.RingGeometry(R - 1.5, R + 1.5, 128),
      new THREE.MeshBasicMaterial({ color: 0xf28c28, transparent: true, opacity: 0.35, side: THREE.DoubleSide }),
    );
    this.ring.rotation.x = -Math.PI / 2;
    this.ring.position.y = 0.6;
    this.scene.add(toHelperLayer(this.ring));
    this.clearTrail();
    this.refresh();
  }

  /** Load the real arm (from the STEP export); falls back to the drawn arm on failure. */
  async loadCad(url) {
    this.cad = await loadCad(url);
    if (this.links) this.setLinks(this.links);
    return this.cad;
  }

  cadMat(color) {
    this.cadMats ||= {};
    return (this.cadMats[color] ||= new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.1 }));
  }

  setOptions(o) {
    Object.assign(this.opts, o);
    this.refresh();
  }

  setPose(servos, J) {
    this.J = J;
    const prev = this.pose;
    this.pose = [...servos];
    if (this.opts.trail && prev && prev.some((v, i) => Math.abs(v - servos[i]) > 0.05)) {
      const p = fk(servos, J, this.links).tcp;
      this.pushTrail(p);
    }
    this.refresh();
  }

  setGhost(servos) {
    this.ghostPose = [...servos];
    this.refresh();
  }

  showMarker(p) {
    if (!p) {
      this.marker.visible = false;
      return;
    }
    this.marker.position.set(p.x, p.z, -p.y);
    this.marker.visible = true;
  }

  refresh() {
    if (!this.arm || !this.J) return;
    if (this.pose) poseArm(this.arm, this.pose, this.J);
    const g = this.ghostPose || this.pose;
    const diff = this.pose && g && g.some((v, i) => Math.abs(v - this.pose[i]) > 0.3);
    this.ghost.root.visible = this.opts.ghost && !!diff;
    if (g) poseArm(this.ghost, g, this.J);
    this.trail.visible = this.opts.trail;
    if (this.ring) this.ring.visible = this.opts.envelope;
  }

  pushTrail(p) {
    if (this.trailCount >= this.trailMax) {
      this.trailPos.copyWithin(0, 3);
      this.trailCount--;
    }
    const k = this.trailCount * 3;
    this.trailPos[k] = p.x;
    this.trailPos[k + 1] = p.z;
    this.trailPos[k + 2] = -p.y;
    this.trailCount++;
    this.trail.geometry.setDrawRange(0, this.trailCount);
    this.trail.geometry.attributes.position.needsUpdate = true;
  }

  clearTrail() {
    this.trailCount = 0;
    this.trail.geometry.setDrawRange(0, 0);
  }

  view(name) {
    const c = this.camera;
    const t = this.controls.target;
    t.set(110, 120, 0);
    const pos = {
      iso: [620, 460, 560],
      front: [880, 200, 0],
      side: [110, 200, 820],
      top: [110.01, 950, 0],
    }[name] || [620, 460, 560];
    c.position.set(...pos);
    c.lookAt(t);
    this.controls.update();
  }
}
