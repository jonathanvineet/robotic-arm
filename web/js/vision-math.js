// vision-math.js — pure (no DOM / no Three) maths for camera-guided picking.
//   * pinhole camera model (used by the simulated camera)
//   * homography: image pixels <-> table plane (mm, robot frame)
//   * calibration-mat layout (ArUco marker positions)
//   * detection -> world objects, tracker, pick planning
// Robot frame: x forward from the arm, y to the arm's left, z up, origin = base centre on the table.

import { ik, fk, toJoint, toServo } from './kinematics.js';

// ------------------------------------------------------------------ workspace markers
// Four ArUco markers at the corners of a square centred on the robot's base.
// Keep in sync with Plastic-Segregation-AI/make_markers.py
export const LAYOUT_DEFAULT = {
  B: 360, // marker centres at (±B, ±B) mm from the base centre
  size: 100, // printed marker side, mm (black square)
  ids: [0, 1, 2, 3], // front-left, front-right, back-right, back-left
  dict: 'DICT_4X4_50',
  points: null, // optional measured centres [{id, x, y}] — overrides B
};
export const CORNERS = ['front-left', 'front-right', 'back-right', 'back-left'];

/** World (x,y) of each marker centre. "Front" = the arm's forward direction (base servo at its zero). */
export function cornerMarkers(lay = LAYOUT_DEFAULT) {
  const L = { ...LAYOUT_DEFAULT, ...lay };
  if (Array.isArray(L.points) && L.points.length >= 4) return L.points.map((p, i) => ({ id: p.id, x: p.x, y: p.y, corner: CORNERS[i] || '' }));
  const B = L.B;
  const xy = [[B, B], [B, -B], [-B, -B], [-B, B]];
  return xy.map(([x, y], i) => ({ id: L.ids[i], x, y, corner: CORNERS[i] }));
}

/** Pick area = the square between the marker centres (the markers sit in the corners, out of reach). */
export function layoutArea(lay = LAYOUT_DEFAULT) {
  const L = { ...LAYOUT_DEFAULT, ...lay };
  const m = cornerMarkers(L);
  const xs = m.map((p) => p.x), ys = m.map((p) => p.y);
  const inset = 0;
  return { x0: Math.min(...xs) + inset, x1: Math.max(...xs) - inset, y0: Math.min(...ys) + inset, y1: Math.max(...ys) - inset };
}

// ------------------------------------------------------------------ vectors
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => {
  const l = Math.hypot(...a) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};

// ------------------------------------------------------------------ camera
/**
 * Camera rig: position (mm, robot frame), looking at a point on the table,
 * image "up" pointing as close as possible to +x (away from the arm).
 * hfov = horizontal field of view in degrees. w,h = image size in px.
 */
export function makeCamera({ x = 0, y = 0, z = 1150, lookX = 0, lookY = 0, hfov = 70, w = 1920, h = 1080, roll = 0 } = {}) {
  const pos = [x, y, z];
  const f = norm(sub([lookX, lookY, 0], pos));
  let up0 = [1, 0, 0];
  if (Math.abs(dot(up0, f)) > 0.98) up0 = [0, 1, 0];
  const u0 = norm(sub(up0, f.map((v) => v * dot(up0, f))));
  // roll = camera mounted rotated about its viewing axis (any direction works)
  const rr = (roll * Math.PI) / 180;
  const r0 = cross(f, u0);
  const u = norm(u0.map((v, i) => v * Math.cos(rr) + r0[i] * Math.sin(rr)));
  const r = cross(f, u);
  const fx = w / 2 / Math.tan(((hfov / 2) * Math.PI) / 180);
  return {
    pos, f, u, r, w, h, fx, hfov,
    vfov: (2 * Math.atan(Math.tan(((hfov / 2) * Math.PI) / 180) * (h / w)) * 180) / Math.PI,
    look: [lookX, lookY, 0],
    /** world point [x,y,z] -> {px, py, depth} (depth <= 0 means behind the camera) */
    project(p) {
      const d = sub(p, pos);
      const zc = dot(d, f);
      return { px: w / 2 + (fx * dot(d, r)) / zc, py: h / 2 - (fx * dot(d, u)) / zc, depth: zc };
    },
  };
}

// ------------------------------------------------------------------ homography
function solve(A, b) {
  // Gaussian elimination with partial pivoting
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-12) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const k = M[r][c] / M[c][c];
      for (let k2 = c; k2 <= n; k2++) M[r][k2] -= k * M[c][k2];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

/**
 * Least-squares homography H (3x3, row-major, h33 = 1) mapping src -> dst.
 * pairs: [{px, py, x, y}] (pixel -> world mm). Needs >= 4 non-collinear points.
 */
export function solveHomography(pairs, from = (p) => [p.px, p.py], to = (p) => [p.x, p.y]) {
  if (pairs.length < 4) return null;
  // normalise both sides for numerical stability
  const stats = (pts) => {
    const mx = pts.reduce((s, p) => s + p[0], 0) / pts.length;
    const my = pts.reduce((s, p) => s + p[1], 0) / pts.length;
    const d = pts.reduce((s, p) => s + Math.hypot(p[0] - mx, p[1] - my), 0) / pts.length || 1;
    const s = Math.SQRT2 / d;
    return { T: [s, 0, -s * mx, 0, s, -s * my, 0, 0, 1], Tinv: [1 / s, 0, mx, 0, 1 / s, my, 0, 0, 1] };
  };
  const S = pairs.map(from);
  const D = pairs.map(to);
  const ns = stats(S);
  const nd = stats(D);
  const A = Array.from({ length: 8 }, () => new Array(8).fill(0));
  const b = new Array(8).fill(0);
  for (let i = 0; i < pairs.length; i++) {
    const [u, v] = applyH(ns.T, S[i]);
    const [X, Y] = applyH(nd.T, D[i]);
    const rows = [
      [[u, v, 1, 0, 0, 0, -u * X, -v * X], X],
      [[0, 0, 0, u, v, 1, -u * Y, -v * Y], Y],
    ];
    for (const [row, rhs] of rows) {
      for (let r = 0; r < 8; r++) {
        b[r] += row[r] * rhs;
        for (let c = 0; c < 8; c++) A[r][c] += row[r] * row[c];
      }
    }
  }
  const h = solve(A, b);
  if (!h) return null;
  const Hn = [...h, 1];
  const H = mul3(nd.Tinv, mul3(Hn, ns.T));
  return H.map((v) => v / H[8]);
}

function mul3(A, B) {
  const C = new Array(9).fill(0);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) for (let k = 0; k < 3; k++) C[r * 3 + c] += A[r * 3 + k] * B[k * 3 + c];
  return C;
}

export function applyH(H, [u, v]) {
  const w = H[6] * u + H[7] * v + H[8];
  return [(H[0] * u + H[1] * v + H[2]) / w, (H[3] * u + H[4] * v + H[5]) / w];
}

export function invertH(H) {
  const [a, b, c, d, e, f, g, h, i] = H;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  const inv = [A, -(b * i - c * h), b * f - c * e, B, a * i - c * g, -(a * f - c * d), C, -(a * h - b * g), a * e - b * d];
  return inv.map((v) => v / det);
}

/** RMS error (mm) of a homography over its calibration pairs. */
export function reprojError(H, pairs) {
  if (!H || !pairs.length) return Infinity;
  const s = pairs.reduce((acc, p) => {
    const [x, y] = applyH(H, [p.px, p.py]);
    return acc + (x - p.x) ** 2 + (y - p.y) ** 2;
  }, 0);
  return Math.sqrt(s / pairs.length);
}

// ------------------------------------------------------------------ camera pose from the markers
/**
 * Where the camera is (mm, robot frame), recovered from the pixel->table homography.
 * Principal point = image centre, square pixels. The focal length is measured from
 * the homography when the camera is tilted enough to tell; otherwise (looking
 * straight down) it comes from hfovPrior — then only the height depends on it.
 */
export function cameraFromHomography(Hpw, w, h, hfovPrior = 70) {
  const G = mul3([1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 1], invertH(Hpw)); // world -> centred pixels
  const col = (k) => [G[k], G[3 + k], G[6 + k]];
  const [g1, g2, g3] = [col(0), col(1), col(2)];
  const fPrior = w / 2 / Math.tan((hfovPrior * Math.PI) / 360);
  const pose = (fx) => {
    const Ki = (v) => [v[0] / fx, v[1] / fx, v[2]];
    const a = Ki(g1), b = Ki(g2), c = Ki(g3);
    const lam = 2 / (Math.hypot(...a) + Math.hypot(...b));
    const r1 = a.map((v) => v * lam), r2 = b.map((v) => v * lam), t = c.map((v) => v * lam);
    const r3 = cross(r1, r2);
    let C = [-dot(r1, t), -dot(r2, t), -dot(r3, t)];
    if (C[2] < 0) C = [C[0], C[1], -C[2]];
    const axis = [r1[2], r2[2], r3[2]]; // optical axis in world
    const tilt = (Math.acos(Math.min(1, Math.abs(axis[2]))) * 180) / Math.PI;
    return { C, tilt, fx };
  };
  let res = pose(fPrior);
  let focal = 'assumed';
  if (res.tilt > 8) {
    const est = [];
    const d1 = g1[2] * g2[2];
    if (Math.abs(d1) > 1e-15) est.push(-(g1[0] * g2[0] + g1[1] * g2[1]) / d1);
    const d2 = g1[2] ** 2 - g2[2] ** 2;
    if (Math.abs(d2) > 1e-15) est.push(-(g1[0] ** 2 + g1[1] ** 2 - g2[0] ** 2 - g2[1] ** 2) / d2);
    const ok = est.filter((v) => v > 0 && Number.isFinite(v)).map(Math.sqrt).filter((f) => f > fPrior / 3 && f < fPrior * 3);
    if (ok.length) {
      res = pose(ok.reduce((s, v) => s + v, 0) / ok.length);
      focal = 'measured';
    }
  }
  const hfov = (2 * Math.atan(w / 2 / res.fx) * 180) / Math.PI;
  return { pos: res.C, tilt: res.tilt, hfov, focal };
}

/**
 * Angled-camera correction. A detection box centre is (roughly) where the
 * object's middle — `centreZ` mm above the table — appears, but the table
 * homography maps it onto the table, which pushes it away from the camera.
 * Slide it back along the camera ray to that height.
 */
export function correctHeight(o, camPos, centreZ) {
  if (!camPos || !(centreZ > 0) || camPos[2] <= centreZ) return o;
  const k = centreZ / camPos[2];
  return { ...o, x: o.x + (camPos[0] - o.x) * k, y: o.y + (camPos[1] - o.y) * k, raw: { x: o.x, y: o.y } };
}

// ------------------------------------------------------------------ detections -> world
/**
 * box: [x1,y1,x2,y2,conf,cls] in pixels. Returns the object's table position,
 * approximate footprint and long-axis angle (deg, robot frame).
 */
export function boxToWorld(H, box) {
  const [x1, y1, x2, y2, conf = 1, cls = 0] = box;
  const [x, y] = applyH(H, [(x1 + x2) / 2, (y1 + y2) / 2]);
  const c = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]].map((p) => applyH(H, p));
  const e1 = [(c[1][0] - c[0][0] + c[2][0] - c[3][0]) / 2, (c[1][1] - c[0][1] + c[2][1] - c[3][1]) / 2];
  const e2 = [(c[3][0] - c[0][0] + c[2][0] - c[1][0]) / 2, (c[3][1] - c[0][1] + c[2][1] - c[1][1]) / 2];
  const l1 = Math.hypot(...e1);
  const l2 = Math.hypot(...e2);
  const long = l1 >= l2 ? e1 : e2;
  return {
    x, y, conf, cls,
    len: Math.max(l1, l2),
    wid: Math.min(l1, l2),
    angle: (Math.atan2(long[1], long[0]) * 180) / Math.PI,
    elongated: Math.max(l1, l2) > 1.4 * Math.min(l1, l2),
  };
}

// ------------------------------------------------------------------ tracker
export class Tracker {
  constructor({ radius = 20, minHits = 5, maxAgeMs = 700, alpha = 0.35 } = {}) {
    Object.assign(this, { radius, minHits, maxAgeMs, alpha });
    this.tracks = [];
    this.nextId = 1;
  }

  update(objs, now) {
    const free = new Set(this.tracks);
    for (const o of objs) {
      let best = null;
      let bd = this.radius;
      for (const t of free) {
        const d = Math.hypot(t.x - o.x, t.y - o.y);
        if (d < bd) {
          bd = d;
          best = t;
        }
      }
      if (best) {
        free.delete(best);
        const a = this.alpha;
        best.x += a * (o.x - best.x);
        best.y += a * (o.y - best.y);
        Object.assign(best, { conf: o.conf, len: o.len, wid: o.wid, angle: o.angle, elongated: o.elongated, cls: o.cls });
        best.hits++;
        best.last = now;
      } else {
        this.tracks.push({ ...o, id: this.nextId++, hits: 1, first: now, last: now });
      }
    }
    this.tracks = this.tracks.filter((t) => now - t.last <= this.maxAgeMs);
    return this.tracks;
  }

  stable(now) {
    return this.tracks.filter((t) => t.hits >= this.minHits && now - t.last < 300);
  }

  clear() {
    this.tracks = [];
  }
}

// ------------------------------------------------------------------ pick planning
export const PICK_DEFAULTS = {
  pickZ: 14, // fingertip height when gripping (mm above table)
  approachZ: 60, // hover height above the object
  bin: { x: -120, y: 210, z: 150 }, // drop point (fingertip) above the bin (back-left of the arm)
  binSize: 120, // bin footprint (square, mm) — detections inside are ignored
  noPickRadius: 120, // ignore detections this close to the base (the arm itself / unreachable)
  area: null, // {x0,x1,y0,y1} pick area in mm; null = anywhere (the app sets it to the mat)
  gripOpen: null, // servo angle; null = gripper max
  gripClose: null, // servo angle; null = gripper min + 15
  alignRoll: false, // turn the wrist to grip across long objects (only for arms whose roll comes AFTER pitch)
  heightFix: true, // angled-camera correction (see correctHeight)
  hfov: 70, // camera horizontal field of view guess (deg), used when looking straight down
  pitches: Array.from({ length: 25 }, (_, i) => 180 - i * 5), // 180 (straight down) .. 60
};

const wrap180 = (a) => ((((a + 180) % 360) + 360) % 360) - 180;

/** Wrist-roll servo angle that makes the fingers close across the object's long axis. */
export function rollFor(obj, yawDeg, J, settings) {
  if (!settings.alignRoll || !obj.elongated) return toServo(0, J[4]);
  const closing = obj.angle + 90;
  let roll = wrap180(yawDeg - closing);
  if (roll > 90) roll -= 180;
  if (roll < -90) roll += 180;
  for (const r of [roll, roll + 180, roll - 180]) {
    const s = toServo(r, J[4]);
    if (s >= J[4].min && s <= J[4].max) return s;
  }
  return Math.min(J[4].max, Math.max(J[4].min, toServo(roll, J[4])));
}

function solveAt(p, J, L, cur, pitches) {
  for (const pitch of pitches) {
    const r = ik({ ...p, pitch }, J, L, cur);
    if (r.ok) return { ...r, pitch };
  }
  return null;
}

export function inBin(o, s) {
  return Math.abs(o.x - s.bin.x) < s.binSize / 2 && Math.abs(o.y - s.bin.y) < s.binSize / 2;
}

/** Why an object can't be picked (or null). */
export function rejectReason(o, s) {
  if (Math.hypot(o.x, o.y) < s.noPickRadius) return 'too close to the base';
  const a = s.area;
  if (a && (o.x < a.x0 || o.x > a.x1 || o.y < a.y0 || o.y > a.y1)) return 'outside pick area';
  if (inBin(o, s)) return 'inside the bin';
  return null;
}

/**
 * Full pick-and-place plan as a list of moves for runMoves():
 * open -> hover -> descend -> grip -> lift -> bin -> release -> park.
 */
export function planPick(obj, J, L, current, settings = PICK_DEFAULTS, park = null) {
  const s = { ...PICK_DEFAULTS, ...settings };
  const why = rejectReason(obj, s);
  if (why) return { ok: false, reason: why };
  const open = s.gripOpen ?? J[5].max;
  const close = s.gripClose ?? J[5].min + 15;

  // Prefer the same pitch for hover and grip (gripper goes straight down);
  // otherwise use the hover pitch closest to the grip pitch.
  let hover = null;
  let grip = null;
  for (const pitch of s.pitches) {
    const g = ik({ x: obj.x, y: obj.y, z: s.pickZ, pitch }, J, L, current);
    if (!g.ok) continue;
    const order = [...s.pitches].sort((a, b) => Math.abs(a - pitch) - Math.abs(b - pitch));
    for (const hp of order) {
      const h = ik({ x: obj.x, y: obj.y, z: s.pickZ + s.approachZ, pitch: hp }, J, L, current);
      if (h.ok) {
        hover = { ...h, pitch: hp };
        break;
      }
    }
    if (hover) {
      grip = { ...g, pitch };
      break;
    }
  }
  if (!grip) return { ok: false, reason: 'out of reach' };
  const drop = solveAt(s.bin, J, L, current, s.pitches);
  if (!drop) return { ok: false, reason: 'bin out of reach' };

  const yaw = toJoint(grip.servos[0], J[0]);
  const roll = rollFor(obj, yaw, J, s);
  const pose = (sv, g, rl = roll) => sv.map((v, i) => (i === 4 ? rl : i === 5 ? g : v));
  const neutralRoll = toServo(0, J[4]);
  const parkPose = park ?? J.map((j) => j.home);
  const moves = [
    { j: pose(hover.servos, open), label: 'hover above object' },
    { j: pose(grip.servos, open), label: 'descend' },
    { j: pose(grip.servos, close), label: 'grip', dwell: 350 },
    { j: pose(hover.servos, close), label: 'lift' },
    { j: pose(drop.servos, close, neutralRoll), label: 'over the bin' },
    { j: pose(drop.servos, open, neutralRoll), label: 'release', dwell: 300 },
    { j: parkPose.map((v, i) => (i === 5 ? open : v)), label: 'park' },
  ];
  return { ok: true, moves, pitch: grip.pitch, yaw, roll, tcp: fk(grip.servos, J, L).tcp };
}
