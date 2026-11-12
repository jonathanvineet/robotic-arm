// kinematics.js — forward & inverse kinematics for the 6-servo arm.
//
// Robot frame (mm): x forward, y left, z up, origin at the base on the floor.
// Joint angles come from servo angles via: q = (servo - zero) * dir.
// Kinematic zero = whole arm pointing straight up. Positive pitch tilts forward.

export const DEFAULT_LINKS = {
  base: 80, // floor -> shoulder axis
  upper: 105, // shoulder -> elbow
  fore: 100, // elbow -> wrist pitch axis
  wrist: 65, // wrist pitch axis -> gripper palm
  tool: 60, // palm -> fingertip (TCP)
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
  const yaw = toJoint(s[0], J[0]) * D2R;
  const q1 = toJoint(s[1], J[1]) * D2R;
  const q2 = toJoint(s[2], J[2]) * D2R;
  const q3 = toJoint(s[3], J[3]) * D2R;
  const t1 = q1,
    t2 = q1 + q2,
    t3 = q1 + q2 + q3;
  const hand = L.wrist + L.tool;
  const planar = [[0, 0], [0, L.base]];
  const add = (len, th) => {
    const [r, z] = planar[planar.length - 1];
    planar.push([r + len * Math.sin(th), z + len * Math.cos(th)]);
  };
  add(L.upper, t1);
  add(L.fore, t2);
  add(hand, t3);
  const c = Math.cos(yaw),
    sn = Math.sin(yaw);
  const points = planar.map(([r, z]) => ({ x: r * c, y: r * sn, z }));
  // pitch is measured in the vertical plane facing the TCP (same as ik()),
  // so it stays meaningful when the arm leans back over its base
  const facingBack = planar[4][0] < -1e-9;
  return {
    points, // base, shoulder, elbow, wrist, tcp
    tcp: points[4],
    yaw: yaw * R2D,
    pitch: (facingBack ? -t3 : t3) * R2D, // tool angle from vertical: 90 = horizontal (pointing away), 180 = down
    roll: toJoint(s[4], J[4]),
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
  const r = Math.hypot(x, y);
  const hand = L.wrist + L.tool;
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  const yaw0 = r < 1e-6 ? toJoint(current[0], J[0]) * D2R : Math.atan2(y, x);
  // Normal: face the target. Flipped: turn the base 180° and lean back over it.
  const configs = [
    { yaw: yaw0, r, phi },
    { yaw: wrap(yaw0 + Math.PI), r: -r, phi: -phi },
  ];
  let reason = 'Out of reach';
  for (const cf of configs) {
    const rw = cf.r - hand * Math.sin(cf.phi);
    const zw = z - L.base - hand * Math.cos(cf.phi);
    const c2 = (rw * rw + zw * zw - L.upper ** 2 - L.fore ** 2) / (2 * L.upper * L.fore);
    if (c2 > 1 + 1e-9 || c2 < -1 - 1e-9) continue;
    const yawServo = toServo(cf.yaw * R2D, J[0]);
    if (!within(yawServo, J[0])) {
      if (reason === 'Out of reach') reason = 'Base rotation outside limits';
      continue;
    }
    for (const sgn of [1, -1]) {
      const q2 = sgn * Math.acos(Math.min(1, Math.max(-1, c2)));
      const q1 = Math.atan2(rw, zw) - Math.atan2(L.fore * Math.sin(q2), L.upper + L.fore * Math.cos(q2));
      const q3 = cf.phi - q1 - q2;
      const s = [...current];
      s[0] = yawServo;
      s[1] = toServo(q1 * R2D, J[1]);
      s[2] = toServo(q2 * R2D, J[2]);
      s[3] = toServo(q3 * R2D, J[3]);
      const bad = [1, 2, 3].find((i) => !within(s[i], J[i]));
      if (bad === undefined) return { ok: true, servos: s, elbow: sgn > 0 ? 'up' : 'down' };
      reason = `${J[bad].name ?? 'Joint ' + bad} outside limits`;
    }
  }
  return { ok: false, reason };
}

/** Maximum horizontal reach (for drawing the workspace ring). */
export const maxReach = (L) => L.upper + L.fore + L.wrist + L.tool;
