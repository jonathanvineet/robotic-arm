// Converts the SolidWorks STEP export of the arm into web/models/arm-model.json:
//   * meshes (identical parts such as the 6 servos are stored once + a transform)
//   * which kinematic link every part belongs to
//   * the arm's kinematics measured from the CAD: joint axes from the servo shafts,
//     link lengths, offsets, the pose the arm was modelled in, and the claw motion.
//
//   npm i --no-save occt-import-js@0.0.23
//   node tools/step-to-model.mjs Final_assembly.STEP
//
// Coordinates: CAD/three.js (Y up). Robot frame = (x, -z, y).
import { createRequire } from 'node:module';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
let occtFactory;
try {
  occtFactory = require('occt-import-js');
} catch {
  occtFactory = require(join(process.env.OCCT_PATH || '.', 'node_modules', 'occt-import-js'));
}

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const stepPath = process.argv[2] || join(root, 'Final_assembly.STEP');

// ------------------------------------------------------------------ vector helpers
const sub = (a, b) => a.map((v, i) => v - b[i]);
const add = (a, b) => a.map((v, i) => v + b[i]);
const mul = (a, k) => a.map((v) => v * k);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = (a) => Math.hypot(...a);
const nrm = (a) => mul(a, 1 / len(a));
const mean = (pts) => mul(pts.reduce((s, p) => add(s, p), [0, 0, 0]), 1 / pts.length);
const deg = (r) => (r * 180) / Math.PI;
function rotAxis(v, k, th) {
  // Rodrigues: rotate v about unit axis k by th
  const c = Math.cos(th), s = Math.sin(th);
  return add(add(mul(v, c), mul(cross(k, v), s)), mul(k, dot(k, v) * (1 - c)));
}
const rotAbout = (p, c, k, th) => add(c, rotAxis(sub(p, c), k, th));
const verts = (m) => {
  const a = m.attributes.position.array, out = [];
  for (let i = 0; i < a.length; i += 3) out.push([a[i], a[i + 1], a[i + 2]]);
  return out;
};

// Horn's method: rigid transform A -> B (same vertex order)
function horn(A, B) {
  const ca = mean(A), cb = mean(B);
  const S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < A.length; i++) {
    const a = sub(A[i], ca), b = sub(B[i], cb);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) S[r][c] += a[r] * b[c];
  }
  const [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]] = S;
  const N = [
    [xx + yy + zz, yz - zy, zx - xz, xy - yx],
    [yz - zy, xx - yy - zz, xy + yx, zx + xz],
    [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
    [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
  ];
  let q = [1, 0.1, 0.2, 0.3];
  const shift = 1e7;
  for (let it = 0; it < 800; it++) {
    const nq = N.map((row, r) => row.reduce((s, v, c) => s + v * q[c], 0) + shift * q[r]);
    q = mul(nq, 1 / len(nq.slice(0, 3).concat(nq[3])));
    const l = Math.hypot(...q);
    q = q.map((v) => v / l);
  }
  const [w, x, y, z] = q;
  const R = [
    [w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z],
  ];
  const Rx = (p) => R.map((row) => dot(row, p));
  const t = sub(cb, Rx(ca));
  const app = (p) => add(Rx(p), t);
  let err = 0;
  for (let i = 0; i < A.length; i += 5) err = Math.max(err, len(sub(app(A[i]), B[i])));
  // column-major 4x4 for three.js Matrix4.fromArray
  const M = [R[0][0], R[1][0], R[2][0], 0, R[0][1], R[1][1], R[2][1], 0, R[0][2], R[1][2], R[2][2], 0, t[0], t[1], t[2], 1];
  return { Rx, app, err, M };
}

function pca(pts) {
  const c = mean(pts);
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const p of pts) {
    const d = sub(p, c);
    for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) C[a][b] += d[a] * d[b];
  }
  const axes = [];
  let M = C.map((r) => [...r]);
  for (let k = 0; k < 3; k++) {
    let v = nrm([1, 0.37, 0.71]);
    for (let it = 0; it < 400; it++) v = nrm(M.map((row) => dot(row, v)));
    const lam = dot(v, M.map((row) => dot(row, v)));
    axes.push(v);
    M = M.map((row, a) => row.map((x, b) => x - lam * v[a] * v[b]));
  }
  return { c, axes }; // largest variance first
}

/** True box axes of a part: the three orthogonal face-normal directions with the most area. */
function boxAxes(pts, idx) {
  const bins = [];
  for (let t = 0; t < idx.length; t += 3) {
    const A = pts[idx[t]], B = pts[idx[t + 1]], C = pts[idx[t + 2]];
    const n = cross(sub(B, A), sub(C, A));
    const area = len(n) / 2;
    if (area < 1e-9) continue;
    const u = nrm(n);
    const bin = bins.find((b) => Math.abs(dot(b.n, u)) > 0.999);
    if (bin) bin.area += area;
    else bins.push({ n: u, area });
  }
  bins.sort((x, y) => y.area - x.area);
  const n1 = bins[0].n;
  const n2 = bins.find((b) => Math.abs(dot(b.n, n1)) < 0.02).n;
  return [n1, nrm(sub(n2, mul(n1, dot(n1, n2)))), nrm(cross(n1, n2))];
}

/** Servo output shaft = the small round "tip" at one extreme of the body. */
function findShaft(pts, idx) {
  const c = mean(pts);
  const axes = boxAxes(pts, idx);
  let best = null;
  for (const ax of axes)
    for (const sg of [1, -1]) {
      const d = mul(ax, sg);
      const pr = pts.map((p) => dot(sub(p, c), d));
      const mx = Math.max(...pr);
      const ext = pts.filter((p, k) => pr[k] > mx - 0.3);
      const o1 = nrm(cross(d, Math.abs(d[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]));
      const o2 = cross(d, o1);
      const w = (o) => Math.max(...ext.map((p) => dot(p, o))) - Math.min(...ext.map((p) => dot(p, o)));
      const w1 = w(o1), w2 = w(o2);
      // round, 3.5–7.5 mm across: the output spline (mounting ears are narrower and not round)
      if (Math.min(w1, w2) >= 3.5 && Math.max(w1, w2) < 7.5 && Math.abs(w1 - w2) < 1.5 && ext.length >= 8 && (!best || ext.length > best.n))
        best = { p: mean(ext), d, n: ext.length };
    }
  if (!best) throw new Error('servo shaft not found');
  return best;
}

// ------------------------------------------------------------------ read STEP
const occt = await occtFactory();
const res = occt.ReadStepFile(new Uint8Array(readFileSync(stepPath)), {
  linearUnit: 'millimeter',
  linearDeflectionType: 'bounding_box_ratio',
  linearDeflection: 0.002,
  angularDeflection: 0.6,
});
if (!res.success) throw new Error('STEP import failed');
const meshes = res.meshes;
// parent assembly of each mesh
const parentOf = [];
(function walk(n, path) {
  for (const i of n.meshes || []) parentOf[i] = path;
  for (const ch of n.children || []) walk(ch, [...path, ch.name]);
})(res.root, []);
const P = meshes.map((m, i) => ({ i, name: m.name, v: verts(m), inGripper: (parentOf[i] || []).includes('gripper_assm'), m }));
console.log(`read ${P.length} meshes, ${P.reduce((s, p) => s + p.m.index.array.length / 3, 0)} triangles`);

// ------------------------------------------------------------------ instancing
const unique = [];
for (const p of P) {
  const u = unique.find((q) => q.name === p.name && q.v.length === p.v.length);
  if (u) {
    const T = horn(u.v, p.v);
    if (T.err < 0.02) {
      p.mesh = u.id;
      p.T = T;
      continue;
    }
  }
  p.mesh = unique.length;
  p.T = { M: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], app: (x) => x, Rx: (x) => x };
  unique.push({ id: p.mesh, name: p.name, v: p.v, m: p.m });
}
console.log(`${unique.length} unique meshes`);

// ------------------------------------------------------------------ servos & axes
const shaftOf = (p) => {
  const u = unique[p.mesh];
  u.shaft ||= findShaft(u.v, u.m.index.array);
  return { p: p.T.app(u.shaft.p), d: nrm(p.T.Rx(u.shaft.d)) };
};
const big = P.filter((p) => /MG996R/i.test(p.name)).sort((a, b) => mean(a.v)[1] - mean(b.v)[1]);
const small = P.filter((p) => /MG90S/i.test(p.name));
const gripServo = small.find((p) => p.inGripper);
const wristServos = small.filter((p) => !p.inGripper).sort((a, b) => mean(a.v)[1] - mean(b.v)[1]);
const [yawServo, shoulderServo, elbowServo] = big;
const shoulderAx = shaftOf(shoulderServo);
const elbowAx = shaftOf(elbowServo);
const yawAx = shaftOf(yawServo);
// the wrist servo whose shaft is parallel to the shoulder axis is the PITCH joint
const wa = wristServos.map((p) => ({ p, ax: shaftOf(p) }));
const pitchIdx = Math.abs(dot(wa[0].ax.d, shoulderAx.d)) > Math.abs(dot(wa[1].ax.d, shoulderAx.d)) ? 0 : 1;
const pitch = wa[pitchIdx], roll = wa[1 - pitchIdx];
const rollFirst = mean(roll.p.v)[1] < mean(pitch.p.v)[1];
console.log(`wrist order: ${rollFirst ? 'ROLL then PITCH' : 'PITCH then ROLL'} (along the arm)`);

const UP = [0, 1, 0];
const gripCentre = mean(P.filter((p) => p.inGripper).flatMap((p) => p.v));
let a = nrm(sub(shoulderAx.d, mul(UP, dot(shoulderAx.d, UP)))); // horizontal shoulder axis
let d = nrm(cross(a, UP)); // forward in the arm plane
if (dot(d, gripCentre) < 0) d = mul(d, -1);
const left = [d[2], 0, -d[0]]; // robot-left in CAD coords
const yaw0 = Math.atan2(-d[2], d[0]);
const plane = (p) => ({ h: dot(p, d), y: p[1], s: dot(p, left) });

// closest points between two lines
function closest(p1, d1, p2, d2) {
  const w = sub(p1, p2);
  const A = dot(d1, d1), B = dot(d1, d2), C = dot(d2, d2), D = dot(d1, w), E = dot(d2, w);
  const den = A * C - B * B;
  const t = (B * E - C * D) / den, u = (A * E - B * D) / den;
  return [add(p1, mul(d1, t)), add(p2, mul(d2, u))];
}
// pivots in the arm plane (project the shaft lines onto the plane through the shoulder)
const S = shoulderAx.p, E = elbowAx.p;
const Sp = plane(S), Ep = plane(E);
let rDir = roll.ax.d;
if (dot(rDir, sub(gripCentre, E)) < 0) rDir = mul(rDir, -1);
const [Wc] = closest(roll.ax.p, rDir, pitch.ax.p, pitch.ax.d);
const Wp = plane(Wc);

const q1_0 = Math.atan2(Ep.h - Sp.h, Ep.y - Sp.y);
const t2_0 = Math.atan2(Wp.h - Ep.h, Wp.y - Ep.y);
const upper = Math.hypot(Ep.h - Sp.h, Ep.y - Sp.y);
const fore = Math.hypot(Wp.h - Ep.h, Wp.y - Ep.y);
const u2 = nrm(add(mul(d, Math.sin(t2_0)), mul(UP, Math.cos(t2_0))));
// roll0: pitch axis vs the arm-plane normal (twin local +Z = robot right = -left), about the forearm
const right = mul(left, -1);
const pa = nrm(sub(pitch.ax.d, mul(u2, dot(pitch.ax.d, u2))));
const ang = (from, to) => Math.atan2(dot(cross(from, to), u2), dot(from, to));
let roll0 = ang(right, pa);
if (Math.abs(roll0) > Math.PI / 2) roll0 = ang(right, mul(pa, -1));

// ------------------------------------------------------------------ gripper
const gears = P.filter((p) => p.inGripper && /gear/i.test(p.name)).map((p) => {
  const { c, axes } = pca(p.v);
  return { p, c, axis: axes[2] }; // flat disc: smallest variance = rotation axis
});
const sideOf = (p) => {
  const c = mean(p.v);
  return len(sub(c, gears[0].c)) < len(sub(c, gears[1].c)) ? 0 : 1;
};
const moving = P.filter((p) => p.inGripper && /gripper$|gripper_link|gear/i.test(p.name) && !/gripper_base/i.test(p.name));
const claws = P.filter((p) => p.inGripper && p.name === 'gripper');
const handDir0 = (() => {
  // direction from the wrist centre towards the claws
  const cc = mean(claws.flatMap((c) => c.v));
  return nrm(sub(cc, Wc));
})();
const tipOf = (cl) => {
  const pr = cl.v.map((p) => dot(p, handDir0));
  const mx = Math.max(...pr);
  return mean(cl.v.filter((p, k) => pr[k] > mx - 1.5));
};
const tips = claws.map((cl) => ({ side: sideOf(cl), tip: tipOf(cl) }));
// both gears turn by the same amount in opposite directions; find the closing angle
let ax0 = gears[0].axis, ax1 = gears[1].axis;
if (dot(ax0, ax1) < 0) ax1 = mul(ax1, -1);
const tipAt = (side, th) => {
  const t = tips.find((x) => x.side === side).tip;
  const g = gears[side];
  return rotAbout(t, g.c, side === 0 ? ax0 : ax1, side === 0 ? th : -th);
};
let best = { gap: Infinity, th: 0 };
for (let k = -1400; k <= 1400; k++) {
  const th = (k / 1400) * (Math.PI / 2.2);
  const gap = len(sub(tipAt(0, th), tipAt(1, th)));
  if (gap >= 3 && gap < best.gap) best = { gap, th };
}
const openGap = len(sub(tipAt(0, 0), tipAt(1, 0)));
const tcpClosed = mul(add(tipAt(0, best.th), tipAt(1, best.th)), 0.5);
// hand vector in the un-rolled arm plane (pick the un-roll direction that puts the
// gripper back in the arm plane — the roll axis sign from the CAD is arbitrary)
const lat = (th) => dot(rotAxis(sub(tcpClosed, Wc), u2, th), left);
if (Math.abs(lat(roll0)) < Math.abs(lat(-roll0))) roll0 = -roll0;
const hv = rotAxis(sub(tcpClosed, Wc), u2, -roll0);
const Hp = { h: dot(hv, d), y: hv[1], s: dot(hv, left) };
const t3_0 = Math.atan2(Hp.h, Hp.y);
const q2_0 = t2_0 - q1_0;
const q3_0 = t3_0 - t2_0;
const hand = Math.hypot(Hp.h, Hp.y);

const kin = {
  yaw0: deg(yaw0), q1_0: deg(q1_0), q2_0: deg(q2_0), roll0: deg(roll0), q3_0: deg(q3_0),
  links: {
    base: +Sp.y.toFixed(2),
    fwd: +Sp.h.toFixed(2),
    // sideways offset of the FINGERTIPS from the base axis (all lateral offsets lie along the
    // pitch axes, so they only shift the TCP sideways — that's what picking needs)
    side: +(Wp.s + Hp.s).toFixed(2),
    upper: +upper.toFixed(2),
    fore: +fore.toFixed(2),
    wrist: +(hand * 0.45).toFixed(2),
    tool: +(hand * 0.55).toFixed(2),
  },
  rollFirst,
  checks: {
    yawServoShaftFromCentre_mm: +Math.hypot(yawAx.p[0], yawAx.p[2]).toFixed(2),
    elbowAxisParallel: +Math.abs(dot(elbowAx.d, shoulderAx.d)).toFixed(4),
    pitchAxisParallel: +Math.abs(dot(pitch.ax.d, shoulderAx.d)).toFixed(4),
    rollAxisOffElbow_mm: +len(sub(closest(roll.ax.p, rDir, E, shoulderAx.d)[0], closest(roll.ax.p, rDir, E, shoulderAx.d)[1])).toFixed(2),
    handLateral_mm: +Hp.s.toFixed(2),
    shoulderLateral_mm: +Sp.s.toFixed(2),
    elbowLateral_mm: +(Ep.s - Sp.s).toFixed(2),
    wristLateral_mm: +(Wp.s - Sp.s).toFixed(2),
    clawGapOpen_mm: +openGap.toFixed(1),
    clawGapClosed_mm: +best.gap.toFixed(1),
  },
};

// ------------------------------------------------------------------ link assignment
function linkOf(p) {
  if (p.inGripper) {
    if (moving.includes(p)) return `finger${sideOf(p)}`;
    return 'pitch';
  }
  if (p === yawServo || /^1\.base/.test(p.name)) return 'base';
  if (p === shoulderServo || /waist/.test(p.name)) return 'yaw';
  if (/^arm_1/.test(p.name)) return 'upper';
  if (p === elbowServo) {
    // servo body belongs to whichever arm part surrounds it more
    const c = mean(p.v);
    const near = (name) => Math.min(...P.find((q) => q.name === name).v.map((v) => len(sub(v, c))));
    return near('arm_1') < near('arm_2') ? 'upper' : 'fore';
  }
  if (/^arm_2/.test(p.name)) return 'fore';
  if (p === roll.p) return rollFirst ? 'fore' : 'pitch';
  if (/^arm_3/.test(p.name) || p === pitch.p) return rollFirst ? 'roll' : 'fore';
  return 'pitch';
}
const COLORS = {
  '1.base': '#1f4fd1', '2.waist': '#c8102e', arm_1: '#e9ecef', arm_2: '#2457d6', arm_3: '#c8102e',
  gripper_base: '#2457d6', gripper: '#f28c28', new_gripper_link: '#f6a04d', gear_1_2: '#f28c28',
};
const colorOf = (p) => (/MG9|TowerPro/i.test(p.name) ? '#2b2d33' : COLORS[p.name] || '#9aa3b2');

// ------------------------------------------------------------------ pack meshes
const b64 = (typed) => Buffer.from(typed.buffer, typed.byteOffset, typed.byteLength).toString('base64');
const outMeshes = unique.map((u) => {
  const pos = u.m.attributes.position.array;
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < pos.length; i += 3) for (let k = 0; k < 3; k++) {
    mn[k] = Math.min(mn[k], pos[i + k]);
    mx[k] = Math.max(mx[k], pos[i + k]);
  }
  const scale = mx.map((v, k) => Math.max(1e-6, v - mn[k]) / 65535);
  const q = new Uint16Array(pos.length);
  for (let i = 0; i < pos.length; i += 3) for (let k = 0; k < 3; k++) q[i + k] = Math.round((pos[i + k] - mn[k]) / scale[k]);
  const idx = u.m.index.array;
  const I = pos.length / 3 > 65535 ? new Uint32Array(idx) : new Uint16Array(idx);
  return { name: u.name, min: mn, scale, pos: b64(q), idx: b64(I), idx32: I instanceof Uint32Array };
});
const parts = P.map((p) => ({ name: p.name, mesh: p.mesh, link: linkOf(p), color: colorOf(p), matrix: p.T.M.map((v) => +v.toFixed(6)) }));
const model = {
  source: stepPath.split('/').pop(),
  units: 'mm',
  frame: 'three.js world = CAD (Y up); robot = (x, -z, y)',
  kin,
  gripper: {
    pivots: gears.map((g, i) => ({ side: sideOf(g.p), centre: g.c, axis: i === 0 ? ax0 : ax1, sign: i === 0 ? 1 : -1 })),
    closeAngle: deg(best.th),
  },
  meshes: outMeshes,
  parts,
};
mkdirSync(join(root, 'web', 'models'), { recursive: true });
const outPath = join(root, 'web', 'models', 'arm-model.json');
writeFileSync(outPath, JSON.stringify(model));
console.log(JSON.stringify(kin, null, 2));
console.log('parts:', parts.map((p) => `${p.name}→${p.link}`).join(', '));
console.log(`gripper close angle ${model.gripper.closeAngle.toFixed(1)}°`);
console.log(`wrote ${outPath} (${(JSON.stringify(model).length / 1024).toFixed(0)} KB)`);
