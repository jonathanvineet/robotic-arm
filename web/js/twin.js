// twin.js — Three.js digital twin. Solid arm = actual position reported by
// the device; translucent "ghost" = commanded target. Units are millimetres.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { toJoint, gripOpen, maxReach, fk } from './kinematics.js';

const D2R = Math.PI / 180;

function buildArm(L, mats) {
  const { link, joint, dark, accent } = mats;
  const shadow = (m) => {
    m.castShadow = true;
    m.receiveShadow = true;
    return m;
  };
  const box = (w, h, d, mat, y = h / 2) => {
    const m = shadow(new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat));
    m.position.y = y;
    return m;
  };
  const cyl = (r, h, mat, axis = 'y') => {
    const m = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 32), mat));
    if (axis === 'z') m.rotation.x = Math.PI / 2;
    if (axis === 'x') m.rotation.z = Math.PI / 2;
    return m;
  };

  const root = new THREE.Group();
  const plate = cyl(62, 10, dark);
  plate.position.y = 5;
  root.add(plate);

  const yaw = new THREE.Group();
  yaw.position.y = 10;
  root.add(yaw);
  const turret = cyl(44, L.base - 30, link);
  turret.position.y = (L.base - 30) / 2;
  yaw.add(turret);
  const yoke = box(56, 34, 62, dark, L.base - 30 + 10);
  yaw.add(yoke);

  const shoulder = new THREE.Group();
  shoulder.position.y = L.base - 10;
  yaw.add(shoulder);
  const sj = cyl(22, 70, joint, 'z');
  shoulder.add(sj);
  shoulder.add(box(30, L.upper, 22, link));
  shoulder.add(box(38, 36, 40, dark, 18)); // servo body

  const elbow = new THREE.Group();
  elbow.position.y = L.upper;
  shoulder.add(elbow);
  elbow.add(cyl(18, 52, joint, 'z'));
  elbow.add(box(26, L.fore, 20, link));

  const wrist = new THREE.Group();
  wrist.position.y = L.fore;
  elbow.add(wrist);
  wrist.add(cyl(15, 44, joint, 'z'));
  wrist.add(box(22, L.wrist * 0.45, 22, link));

  const roll = new THREE.Group();
  roll.position.y = L.wrist * 0.45;
  wrist.add(roll);
  roll.add(cyl(14, 8, accent));
  roll.add(box(20, L.wrist * 0.55, 18, dark));
  const palm = box(56, 8, 22, link, L.wrist * 0.55);
  roll.add(palm);

  const fingers = [-1, 1].map((side) => {
    const f = new THREE.Group();
    f.position.y = L.wrist * 0.55;
    const pad = box(6, L.tool, 16, accent);
    f.add(pad);
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
  for (const f of arm.fingers) f.position.x = f.userData.side * (4 + open * 20);
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
    this.camera = camera;
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 150, 0);
    this.controls = controls;
    this.view('iso');

    scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1c22, 1.4));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(300, 700, 400);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -600, right: 600, top: 600, bottom: -600, near: 10, far: 2000 });
    scene.add(sun);

    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(900, 64),
      new THREE.ShadowMaterial({ opacity: 0.35 }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    scene.add(floor);
    const grid = new THREE.GridHelper(1200, 24, 0x4a5060, 0x2a2e38);
    grid.material.transparent = true;
    grid.material.opacity = 0.6;
    scene.add(grid);
    const axes = new THREE.AxesHelper(80);
    axes.position.y = 0.5;
    scene.add(axes);

    this.mats = {
      link: new THREE.MeshStandardMaterial({ color: 0xf28c28, roughness: 0.45, metalness: 0.15 }),
      joint: new THREE.MeshStandardMaterial({ color: 0x9aa3b2, roughness: 0.3, metalness: 0.8 }),
      dark: new THREE.MeshStandardMaterial({ color: 0x23262e, roughness: 0.7, metalness: 0.2 }),
      accent: new THREE.MeshStandardMaterial({ color: 0x3fd0c9, roughness: 0.4, metalness: 0.3 }),
    };
    const ghostMat = new THREE.MeshBasicMaterial({
      color: 0x3fd0c9,
      transparent: true,
      opacity: 0.16,
      depthWrite: false,
    });
    this.ghostMats = { link: ghostMat, joint: ghostMat, dark: ghostMat, accent: ghostMat };

    // TCP trail
    this.trailMax = 800;
    this.trailPos = new Float32Array(this.trailMax * 3);
    this.trailCount = 0;
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3));
    tg.setDrawRange(0, 0);
    this.trail = new THREE.Line(tg, new THREE.LineBasicMaterial({ color: 0x3fd0c9, transparent: true, opacity: 0.8 }));
    this.trail.frustumCulled = false;
    scene.add(this.trail);

    // target marker for IK
    this.marker = new THREE.Mesh(
      new THREE.SphereGeometry(6, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xffd166 }),
    );
    this.marker.visible = false;
    scene.add(this.marker);

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const tick = () => {
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
    this.arm = buildArm(L, this.mats);
    this.ghost = buildArm(L, this.ghostMats);
    this.ghost.root.traverse((o) => (o.castShadow = o.receiveShadow = false));
    this.scene.add(this.arm.root, this.ghost.root);

    if (this.ring) this.scene.remove(this.ring);
    const R = maxReach(L);
    this.ring = new THREE.Mesh(
      new THREE.RingGeometry(R - 1.5, R + 1.5, 128),
      new THREE.MeshBasicMaterial({ color: 0xf28c28, transparent: true, opacity: 0.35, side: THREE.DoubleSide }),
    );
    this.ring.rotation.x = -Math.PI / 2;
    this.ring.position.y = 0.6;
    this.scene.add(this.ring);
    this.clearTrail();
    this.refresh();
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
    t.set(0, 150, 0);
    const pos = {
      iso: [520, 420, 560],
      front: [760, 180, 0],
      side: [0, 180, 760],
      top: [0.01, 900, 0],
    }[name] || [520, 420, 560];
    c.position.set(...pos);
    c.lookAt(t);
    this.controls.update();
  }
}
