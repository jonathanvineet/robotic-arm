// kinematics.js — forward & inverse kinematics for the 6-servo arm.
//
// Robot frame (mm): x forward, y left, z up, origin at the base on the floor.
// Joint angles come from servo angles via: q = (servo - zero) * dir.
// Kinematic zero = whole arm pointing straight up. Positive pitch tilts forward.

// Measured from Final_assembly.STEP by tools/step-to-model.mjs (mm).
export const DEFAULT_LINKS = {
  base: 96.28, // table -> shoulder axis
  fwd: 13.91, // shoulder axis is this far in front of the base rotation axis
  side: -17.21, // fingertips sit this far to the arm's left (+) / right (-) of the base axis
  upper: 120, // shoulder -> elbow
  fore: 119.17, // elbow -> wrist pitch axis
  wrist: 55.85, // wrist pitch axis -> gripper (split only matters for the drawn fallback arm)
  tool: 68.26, // ... -> closed fingertips (TCP)
  rollFirst: true, // this arm's wrist ROLL comes before the PITCH joint
};

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

export const toJoint = (servo, j) => (servo - j.zero) * j.dir;
export const toServo = (q, j) => j.zero + q * j.dir;

/** Gripper opening 0 (closed) .. 1 (open). */
export function gripOpen(servo, j) {
  const f = (servo - j.min) / Math.max(1e-6, j.max - j.min);
  const g = Math.min(1, Math.max(0, f));
  return j.dir < 0 ? 1 - g : g;
}

/**
 * @param {number[]} s servo angles (6)
 * @param {object[]} J joint configs (zero/dir/min/max)
 * @param {object} L link lengths
 */
export function fk(s, J, L) {
  const fwd = L.fwd || 0;
  const side = L.side || 0;
  const yaw = toJoint(s[0], J[0]) * D2R;
  const q1 = toJoint(s[1], J[1]) * D2R;
  const q2 = toJoint(s[2], J[2]) * D2R;
  const q3 = toJoint(s[3], J[3]) * D2R;
  const rho = toJoint(s[4], J[4]) * D2R;
  const t1 = q1, t2 = q1 + q2, t3 = q1 + q2 + q3;
  const hand = L.wrist + L.tool;
  // arm-plane coordinates: h = forward, v = up, w = robot-right (right-handed with h, v)
  const S = [fwd, L.base, 0];
  const E = [S[0] + L.upper * Math.sin(t1), S[1] + L.upper * Math.cos(t1), 0];
  const W = [E[0] + L.fore * Math.sin(t2), E[1] + L.fore * Math.cos(t2), 0];
  let hv = [hand * Math.sin(t3), hand * Math.cos(t3), 0];
  if (L.rollFirst && rho) {
    // roll turns everything after it about the forearm axis
    const k = [Math.sin(t2), Math.cos(t2), 0];
    const c = Math.cos(rho), sn = Math.sin(rho);
    const kx = [k[1] * hv[2] - k[2] * hv[1], k[2] * hv[0] - k[0] * hv[2], k[0] * hv[1] - k[1] * hv[0]];
    const kd = k[0] * hv[0] + k[1] * hv[1] + k[2] * hv[2];
    hv = hv.map((v, i) => v * c + kx[i] * sn + k[i] * kd * (1 - c));
  }
  const T = [W[0] + hv[0], W[1] + hv[1], hv[2]];
  const c = Math.cos(yaw), sn = Math.sin(yaw);
  const world = ([h, v, w]) => {
    const lat = side - w; // robot-left offset
    return { x: h * c - lat * sn, y: h * sn + lat * c, z: v };
  };
  const points = [[0, 0, 0], S, E, W, T].map(world);
  points[0] = { x: 0, y: 0, z: 0 };
  // pitch is measured in the vertical plane facing the TCP (same as ik()),
  // so it stays meaningful when the arm leans back over its base
  const facingBack = T[0] < -1e-9;
  return {
    points, // base, shoulder, elbow, wrist, tcp
    tcp: points[4],
    yaw: yaw * R2D,
    pitch: (facingBack ? -t3 : t3) * R2D, // tool angle from vertical: 90 = horizontal (pointing away), 180 = down
    roll: rho * R2D,
    grip: gripOpen(s[5], J[5]),
  };
}

const within = (s, j) => s >= j.min - 1e-6 && s <= j.max + 1e-6;

/**
 * Inverse kinematics for a TCP position and tool pitch.
 * Roll and gripper are copied from `current`.
 * @returns {{ok:true, servos:number[], elbow:'up'|'down'} | {ok:false, reason:string}}
 */
export function ik(target, J, L, current) {
  const { x, y, z } = target;
  const phi = (target.pitch ?? 90) * D2R;
  const fwd = L.fwd || 0;
  const side = L.side || 0;
  const R = Math.hypot(x, y);
  const hand = L.wrist + L.tool;
  const theta = R < 1e-6 ? toJoint(current[0], J[0]) * D2R : Math.atan2(y, x);
  if (R < Math.abs(side)) return { ok: false, reason: 'Too close to the base' };
  const along = Math.sqrt(R * R - side * side);
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  let reason = 'Out of reach';
  // sigma = +1: face the target. -1: turn the base the other way and lean back over it.
  for (const sigma of [1, -1]) {
    const rr = sigma * along;
    const yaw = wrap(theta - Math.atan2(side, rr));
    const ph = sigma * phi;
    const rw = rr - fwd - hand * Math.sin(ph);
    const zw = z - L.base - hand * Math.cos(ph);
    const c2 = (rw * rw + zw * zw - L.upper ** 2 - L.fore ** 2) / (2 * L.upper * L.fore);
    if (c2 > 1 + 1e-9 || c2 < -1 - 1e-9) continue;
    const yawServo = toServo(yaw * R2D, J[0]);
    if (!within(yawServo, J[0])) {
      if (reason === 'Out of reach') reason = 'Base rotation outside limits';
      continue;
    }
    for (const sgn of [1, -1]) {
      const q2 = sgn * Math.acos(Math.min(1, Math.max(-1, c2)));
      const q1 = Math.atan2(rw, zw) - Math.atan2(L.fore * Math.sin(q2), L.upper + L.fore * Math.cos(q2));
      const q3 = ph - q1 - q2;
      const s = [...current];
      s[0] = yawServo;
      s[1] = toServo(q1 * R2D, J[1]);
      s[2] = toServo(q2 * R2D, J[2]);
      s[3] = toServo(q3 * R2D, J[3]);
      if (L.rollFirst) s[4] = toServo(0, J[4]); // a rolled wrist would swing the fingertips sideways
      const bad = [1, 2, 3].find((i) => !within(s[i], J[i]));
      if (bad === undefined) return { ok: true, servos: s, elbow: sgn > 0 ? 'up' : 'down' };
      reason = `${J[bad].name ?? 'Joint ' + bad} outside limits`;
    }
  }
  return { ok: false, reason };
}

/** Maximum horizontal reach (for drawing the workspace ring). */
export const maxReach = (L) => (L.fwd || 0) + L.upper + L.fore + L.wrist + L.tool;
