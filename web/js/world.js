// world.js — the physical work cell in the digital twin:
//   table, 4 ArUco corner markers around the robot, sorting bin, overhead
//   camera on a stand, plastic objects with simple grasp/drop physics, and a
//   simulated camera that renders a real image AND produces YOLO-style detections.
// Everything is in robot-frame millimetres; three.js uses (x, z, -y).

import * as THREE from 'three';
import { makeCamera, cornerMarkers, LAYOUT_DEFAULT } from './vision-math.js';
import { fk, gripOpen } from './kinematics.js';
import { toHelperLayer } from './twin.js';

const T = (x, y, z) => new THREE.Vector3(x, z, -y); // robot -> three
const R = (v) => [v.x, -v.z, v.y]; // three -> robot

// DICT_4X4_50 ids 0..3 (inner 4x4 bits, row-major, 1 = white) — from OpenCV
const ARUCO = [
  ['1011', '0101', '0011', '0010'],
  ['0000', '1111', '1001', '1010'],
  ['0011', '0011', '0010', '1101'],
  ['1001', '1001', '0100', '0110'],
];

/** One printed marker (with a white border) as a texture. Image top = +x (robot forward). */
function markerTexture(id) {
  const px = 32;
  const c = document.createElement('canvas');
  c.width = c.height = px * 8; // 6 cells + 1-cell white border each side
  const g = c.getContext('2d');
  g.fillStyle = '#fff';
  g.fillRect(0, 0, c.width, c.height);
  g.fillStyle = '#000';
  g.fillRect(px, px, px * 6, px * 6);
  const bits = ARUCO[id];
  g.fillStyle = '#fff';
  if (bits) bits.forEach((row, r) => [...row].forEach((b, k) => b === '1' && g.fillRect(px * (k + 2), px * (r + 2), px, px)));
  else {
    g.font = `${px * 2}px sans-serif`;
    g.textAlign = 'center';
    g.fillText(String(id), c.width / 2, c.height / 2 + px * 0.7);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.magFilter = THREE.NearestFilter;
  return tex;
}

const KINDS = {
  bottle: { color: 0x4aa3ff, opacity: 0.8, half: 14, r: 14, len: 70 },
  cup: { color: 0xf5f5f5, opacity: 1, half: 18, r: 18 },
  lid: { color: 0xe63946, opacity: 1, half: 10, r: 17 },
};

function makeObject(kind) {
  const k = KINDS[kind];
  const mat = new THREE.MeshStandardMaterial({ color: k.color, roughness: 0.35, transparent: k.opacity < 1, opacity: k.opacity });
  const g = new THREE.Group();
  if (kind === 'bottle') {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(k.r, k.r, k.len, 24), mat);
    body.rotation.z = Math.PI / 2; // lying along the local x axis
    const cap = new THREE.Mesh(new THREE.CylinderGeometry(6, 6, 10, 16), new THREE.MeshStandardMaterial({ color: 0xffffff }));
    cap.rotation.z = Math.PI / 2;
    cap.position.x = k.len / 2 + 5;
    g.add(body, cap);
  } else if (kind === 'cup') {
    g.add(new THREE.Mesh(new THREE.CylinderGeometry(k.r, k.r * 0.8, k.half * 2, 24), mat));
  } else {
    g.add(new THREE.Mesh(new THREE.BoxGeometry(34, k.half * 2, 34), mat));
  }
  g.traverse((o) => {
    o.castShadow = true;
    o.receiveShadow = true;
  });
  g.userData = { kind, half: k.half, r: k.r, state: 'table', vy: 0 };
  return g;
}

export class World {
  constructor(twin) {
    this.twin = twin;
    this.scene = twin.scene;
    this.objects = [];
    this.sorted = 0;
    this.simVisible = true;
    this.settings = {
      layout: { ...LAYOUT_DEFAULT },
      bin: { x: -120, y: 210, z: 150 },
      binSize: 120,
      // straight above the base axis, looking straight down over the whole marker square (README §7)
      cam: { x: 0, y: 0, z: 1150, lookX: 0, lookY: 0, hfov: 70, w: 1920, h: 1080, roll: 0 },
      miss: 0.08,
      noise: 1.5,
    };

    // table (robot in the middle)
    const table = new THREE.Mesh(
      new THREE.BoxGeometry(1200, 24, 1200),
      new THREE.MeshStandardMaterial({ color: 0x6b6f78, roughness: 0.9 }),
    );
    table.position.copy(T(0, 0, -12));
    table.receiveShadow = true;
    this.scene.add(table);

    this.markers = new THREE.Group();
    this.scene.add(this.markers);
    this.guides = toHelperLayer(new THREE.Group());
    this.scene.add(this.guides);

    this.bin = new THREE.Group();
    this.scene.add(this.bin);

    this.rig = new THREE.Group();
    this.scene.add(this.rig);
    this.vcam = new THREE.PerspectiveCamera(45, 4 / 3, 5, 3000); // sees layer 0 only

    this.detGroup = toHelperLayer(new THREE.Group());
    this.scene.add(this.detGroup);

    this.rt = null;
    this.applySettings();
    twin.frameHooks.push((dt) => this.update(dt));
  }

  // ---------------------------------------------------------------- layout
  applySettings(patch = {}) {
    const s = Object.assign(this.settings, patch);
    // corner markers (printed squares taped to the table) + the square between them
    this.markers.clear();
    this.guides.clear();
    const lay = { ...LAYOUT_DEFAULT, ...s.layout };
    const pts = cornerMarkers(lay);
    for (const m of pts) {
      const side = (lay.size * 8) / 6; // marker + white border
      const mesh = new THREE.Mesh(new THREE.PlaneGeometry(side, side), new THREE.MeshStandardMaterial({ map: markerTexture(m.id), roughness: 0.9 }));
      // texture up (+v) -> robot +x, texture right (+u) -> robot -y
      mesh.rotation.set(-Math.PI / 2, 0, -Math.PI / 2);
      mesh.position.copy(T(m.x, m.y, 0.6));
      mesh.receiveShadow = true;
      this.markers.add(mesh);
    }
    const sq = [...pts, pts[0]].map((m) => T(m.x, m.y, 0.8));
    this.guides.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(sq), new THREE.LineBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.5 })));
    toHelperLayer(this.guides);
    // bin
    this.bin.clear();
    const bm = new THREE.MeshStandardMaterial({ color: 0x2f9e44, roughness: 0.6, side: THREE.DoubleSide });
    const B = s.binSize;
    const H = 80;
    const wall = (w, d, x, y) => {
      const m = new THREE.Mesh(new THREE.BoxGeometry(w, H, d), bm);
      m.position.copy(T(x, y, H / 2));
      m.castShadow = m.receiveShadow = true;
      this.bin.add(m);
    };
    wall(B, 4, s.bin.x, s.bin.y + B / 2);
    wall(B, 4, s.bin.x, s.bin.y - B / 2);
    wall(4, B, s.bin.x + B / 2, s.bin.y);
    wall(4, B, s.bin.x - B / 2, s.bin.y);
    const floor = new THREE.Mesh(new THREE.BoxGeometry(B, 4, B), bm);
    floor.position.copy(T(s.bin.x, s.bin.y, 2));
    this.bin.add(floor);
    // camera
    this.cam = makeCamera(s.cam);
    this.buildRig();
    // the preview is rendered at <= 960 px wide (detections use the full resolution maths)
    const k = Math.min(1, 960 / s.cam.w);
    const rw = Math.round(s.cam.w * k), rh = Math.round(s.cam.h * k);
    if (!this.rt || this.rt.width !== rw || this.rt.height !== rh) {
      this.rt?.dispose();
      this.rt = new THREE.WebGLRenderTarget(rw, rh, { samples: 0 });
      this.rt.texture.colorSpace = THREE.SRGBColorSpace;
      this.pixels = new Uint8Array(rw * rh * 4);
      this.off = document.createElement('canvas');
      this.off.width = rw;
      this.off.height = rh;
      this.img = new ImageData(rw, rh);
    }
  }

  buildRig() {
    const c = this.cam;
    const { cam } = this.settings;
    this.rig.clear();
    // virtual camera pose = same maths as makeCamera()
    this.vcam.position.copy(T(...c.pos));
    this.vcam.up.copy(T(...c.u));
    this.vcam.lookAt(T(...c.look));
    this.vcam.fov = c.vfov;
    this.vcam.aspect = c.w / c.h;
    this.vcam.updateProjectionMatrix();
    this.vcam.updateMatrixWorld();

    const dark = new THREE.MeshStandardMaterial({ color: 0x1c1d22, roughness: 0.6, metalness: 0.4 });
    // stand: pole BEHIND the camera (away from the work area), outside the arm's reach,
    // with a horizontal boom out to the camera — nothing crosses over the mat
    let dx = c.pos[0] - cam.lookX, dy = c.pos[1] - cam.lookY;
    if (Math.hypot(dx, dy) < 1) [dx, dy] = [-1, 0];
    const dl = Math.hypot(dx, dy);
    [dx, dy] = [dx / dl, dy / dl];
    let k = 0; // walk back from the camera until we're 460 mm from the base
    while (Math.hypot(c.pos[0] + dx * k, c.pos[1] + dy * k) < 460 && k < 2000) k += 5;
    const px = c.pos[0] + dx * k, py = c.pos[1] + dy * k;
    const top = c.pos[2] + 30;
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(9, 9, top, 16), dark);
    pole.position.copy(T(px, py, top / 2));
    const boom = new THREE.Mesh(new THREE.BoxGeometry(16, 16, 1), dark);
    boom.position.copy(T((px + c.pos[0]) / 2, (py + c.pos[1]) / 2, c.pos[2] + 22));
    boom.scale.z = Math.max(1, k);
    boom.lookAt(T(c.pos[0], c.pos[1], c.pos[2] + 22));
    const foot = new THREE.Mesh(new THREE.CylinderGeometry(60, 60, 8, 24), dark);
    foot.position.copy(T(px, py, 4));
    // camera body follows the virtual camera's orientation
    const body = new THREE.Group();
    body.position.copy(this.vcam.position);
    body.quaternion.copy(this.vcam.quaternion);
    const shell = new THREE.Mesh(new THREE.BoxGeometry(95, 30, 34), dark);
    shell.position.z = 15;
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(9, 9, 8, 20), new THREE.MeshStandardMaterial({ color: 0x0a2a4a, metalness: 0.8 }));
    lens.rotation.x = Math.PI / 2;
    lens.position.z = -2;
    body.add(shell, lens);
    for (const m of [pole, boom, foot, shell, lens]) m.castShadow = true;
    this.rig.add(pole, boom, foot, body);
    // plumb line: shows exactly where the camera hangs (on the base axis by default)
    const plumb = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([T(c.pos[0], c.pos[1], c.pos[2]), T(c.pos[0], c.pos[1], 1)]),
      new THREE.LineDashedMaterial({ color: 0x3fd0c9, dashSize: 12, gapSize: 8 }),
    );
    plumb.computeLineDistances();
    this.rig.add(toHelperLayer(plumb));
    // field of view on the table (view-only helper)
    const corners = [[0, 0], [c.w, 0], [c.w, c.h], [0, c.h]].map(([px, py]) => this.pixelToTable(px, py));
    const pts = [];
    for (const p of corners) pts.push(T(...c.pos), T(p[0], p[1], 0.8));
    for (let i = 0; i < 4; i++) pts.push(T(...corners[i], 0.8), T(...corners[(i + 1) % 4], 0.8));
    const fr = new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.55 }),
    );
    this.rig.add(toHelperLayer(fr));
  }

  /** Ray from the simulated camera through a pixel, intersected with the table. */
  pixelToTable(px, py) {
    const c = this.cam;
    const dx = (px - c.w / 2) / c.fx;
    const dy = -(py - c.h / 2) / c.fx;
    const dir = [0, 1, 2].map((k) => c.f[k] + dx * c.r[k] + dy * c.u[k]);
    const t = -c.pos[2] / dir[2];
    return [c.pos[0] + t * dir[0], c.pos[1] + t * dir[1]];
  }

  // ---------------------------------------------------------------- objects
  /** Drop plastics anywhere in the ring the arm can reach (all the way round). */
  spawn(n = 3, ring = { r0: 190, r1: 290 }) {
    const kinds = Object.keys(KINDS);
    const s = this.settings;
    for (let k = 0; k < n; k++) {
      for (let tries = 0; tries < 60; tries++) {
        const th = Math.random() * Math.PI * 2;
        const r = ring.r0 + Math.random() * (ring.r1 - ring.r0);
        const x = r * Math.cos(th);
        const y = r * Math.sin(th);
        if (Math.abs(x - s.bin.x) < s.binSize / 2 + 40 && Math.abs(y - s.bin.y) < s.binSize / 2 + 40) continue;
        if (Math.abs(th - Math.PI / 2) < 0.12) continue; // thin wedge on the exact left the base can't turn to
        if (this.objects.some((o) => o.userData.state === 'table' && Math.hypot(o.userData.x - x, o.userData.y - y) < 60)) continue;
        const o = makeObject(kinds[Math.floor(Math.random() * kinds.length)]);
        Object.assign(o.userData, { x, y });
        o.position.copy(T(x, y, o.userData.half));
        o.rotation.y = Math.random() * Math.PI;
        o.visible = this.simVisible;
        this.scene.add(o);
        this.objects.push(o);
        break;
      }
    }
  }

  clearObjects() {
    for (const o of this.objects) o.removeFromParent();
    this.objects = [];
    this.sorted = 0;
  }

  setSimVisible(v) {
    this.simVisible = v;
    for (const o of this.objects) o.visible = v;
  }

  /** Grasp / carry / drop, driven by the arm's actual pose in the twin. */
  update(dt) {
    const tw = this.twin;
    if (!tw.arm || !tw.pose || !tw.J) return;
    const open = gripOpen(tw.pose[5], tw.J[5]);
    const tcp = fk(tw.pose, tw.J, tw.links).tcp;
    const held = this.objects.find((o) => o.userData.state === 'held');

    if (held && !held.parent?.parent) {
      // arm was rebuilt while holding: drop it where the gripper is
      this.scene.attach(held);
      held.position.copy(T(tcp.x, tcp.y, Math.max(tcp.z, held.userData.half)));
      held.userData.state = 'falling';
    } else if (held && open > 0.6) {
      this.scene.updateMatrixWorld();
      this.scene.attach(held);
      held.userData.state = 'falling';
      held.userData.vy = 0;
    } else if (!held && open < 0.45 && this.wasOpen) {
      if (/debug/.test(location.search)) console.log('[world] close at tcp', tcp.x.toFixed(1), tcp.y.toFixed(1), tcp.z.toFixed(1), 'objs', this.objects.map((o) => `${o.userData.state}:${o.userData.x.toFixed(0)},${o.userData.y.toFixed(0)}`).join(' '));
      // closing: grab the object between the fingers, if any
      const cand = this.objects
        .filter((o) => o.userData.state === 'table')
        .map((o) => ({ o, d: Math.hypot(o.userData.x - tcp.x, o.userData.y - tcp.y) }))
        .filter(({ o, d }) => d < Math.max(20, o.userData.r) && tcp.z < o.userData.half * 2 + 12)
        .sort((a, b) => a.d - b.d)[0];
      if (cand) {
        this.scene.updateMatrixWorld();
        (tw.arm.hand || tw.arm.roll).attach(cand.o);
        cand.o.userData.state = 'held';
      }
    }
    this.wasOpen = open >= 0.45;

    for (const o of this.objects) {
      const u = o.userData;
      if (u.state !== 'falling') continue;
      u.vy -= 9810 * dt;
      o.position.y += u.vy * dt;
      const [x, y] = R(o.position);
      const s = this.settings;
      const overBin = Math.abs(x - s.bin.x) < s.binSize / 2 - 4 && Math.abs(y - s.bin.y) < s.binSize / 2 - 4;
      const floor = (overBin ? 4 : 0) + u.half;
      if (o.position.y <= floor) {
        o.position.y = floor;
        const yaw = new THREE.Euler().setFromQuaternion(o.quaternion, 'YXZ').y;
        o.rotation.set(0, yaw, 0);
        Object.assign(u, { x, y, vy: 0, state: overBin ? 'bin' : 'table' });
        if (overBin) this.sorted++;
      }
    }
  }

  // ---------------------------------------------------------------- simulated camera
  /** YOLO-like output + ArUco marker centres, same format as the Python bridge. */
  detect() {
    const c = this.cam;
    const { miss, noise } = this.settings;
    const n = () => (Math.random() - 0.5) * 2 * noise;
    const boxes = [];
    const box3 = new THREE.Box3();
    for (const o of this.objects) {
      if (o.userData.state !== 'table' || Math.random() < miss) continue;
      box3.setFromObject(o);
      let x1 = Infinity, y1 = Infinity, x2 = -Infinity, y2 = -Infinity, ok = true;
      for (const cx of [box3.min.x, box3.max.x])
        for (const cy of [box3.min.y, box3.max.y])
          for (const cz of [box3.min.z, box3.max.z]) {
            const p = c.project(R(new THREE.Vector3(cx, cy, cz)));
            if (p.depth <= 0) ok = false;
            x1 = Math.min(x1, p.px); x2 = Math.max(x2, p.px);
            y1 = Math.min(y1, p.py); y2 = Math.max(y2, p.py);
          }
      const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
      if (!ok || mx < 0 || my < 0 || mx > c.w || my > c.h) continue;
      boxes.push([
        Math.max(0, x1 + n()), Math.max(0, y1 + n()), Math.min(c.w, x2 + n()), Math.min(c.h, y2 + n()),
        Math.round((0.62 + Math.random() * 0.35) * 100) / 100, 0,
      ]);
    }
    const markers = cornerMarkers(this.settings.layout)
      .map((m) => ({ id: m.id, ...c.project([m.x, m.y, 0.6]) }))
      .filter((m) => m.depth > 0 && m.px >= 0 && m.py >= 0 && m.px <= c.w && m.py <= c.h)
      .map((m) => ({ id: m.id, x: m.px + n() * 0.3, y: m.py + n() * 0.3 }));
    return { t: 'vision', w: c.w, h: c.h, boxes, markers, names: { 0: 'plastic' }, src: 'sim' };
  }

  /** Render the simulated camera image into a 2D canvas context. */
  renderTo(ctx) {
    const r = this.twin.renderer;
    const { width: w, height: h } = this.rt;
    const prev = r.getRenderTarget();
    r.setRenderTarget(this.rt);
    r.clear();
    r.render(this.scene, this.vcam);
    r.setRenderTarget(prev);
    r.readRenderTargetPixels(this.rt, 0, 0, w, h, this.pixels);
    const row = w * 4;
    for (let y = 0; y < h; y++) this.img.data.set(this.pixels.subarray((h - 1 - y) * row, (h - y) * row), y * row);
    this.off.getContext('2d').putImageData(this.img, 0, 0);
    ctx.drawImage(this.off, 0, 0, this.cam.w, this.cam.h);
  }

  // ---------------------------------------------------------------- detections in 3D
  showTracks(tracks, highlight = null) {
    this.detGroup.clear();
    for (const t of tracks) {
      const color = t.id === highlight ? 0xffd166 : t.pickable ? 0x3ecf8e : t.stable ? 0xf28c28 : 0x8a92a3;
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(Math.max(12, (t.len || 30) / 2), Math.max(12, (t.len || 30) / 2) + 3, 32),
        new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }),
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.copy(T(t.x, t.y, 2));
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(1.5, 1.5, 40, 6), new THREE.MeshBasicMaterial({ color }));
      pin.position.copy(T(t.x, t.y, 20));
      this.detGroup.add(ring, pin);
    }
    toHelperLayer(this.detGroup);
  }
}
