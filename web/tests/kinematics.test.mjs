import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fk, ik, DEFAULT_LINKS, toJoint, toServo } from '../js/kinematics.js';
import { defaultConfig } from '../js/sim.js';

const J = defaultConfig().j;
const L = DEFAULT_LINKS; // measured from the CAD (with offsets, roll before pitch)
const L0 = { base: 80, fwd: 0, side: 0, upper: 105, fore: 100, wrist: 65, tool: 60, rollFirst: false };
const close = (a, b, eps = 1e-6) => Math.abs(a - b) < eps;

test('home pose points straight up (with the CAD shoulder/tip offsets)', () => {
  const f = fk([90, 90, 90, 90, 90, 60], J, L);
  assert.ok(close(f.tcp.x, L.fwd) && close(f.tcp.y, L.side));
  assert.ok(close(f.tcp.z, L.base + L.upper + L.fore + L.wrist + L.tool));
  assert.ok(close(f.pitch, 0));
});

test('base rotation turns the arm toward +y', () => {
  const f = fk([135, 135, 90, 90, 90, 60], J, L0); // yaw +45, shoulder tilted forward 45
  assert.ok(f.tcp.x > 0 && f.tcp.y > 0 && close(f.tcp.x, f.tcp.y, 1e-6));
});

test('roll before pitch: rolling a bent wrist swings the tip sideways; IK keeps roll neutral', () => {
  const s = [90, 120, 60, 60, 90, 60];
  const a = fk(s, J, L).tcp;
  const b = fk(s.map((v, i) => (i === 4 ? 130 : v)), J, L).tcp;
  assert.ok(Math.hypot(a.x - b.x, a.y - b.y) > 10, 'roll moves the tip');
  const c = fk(s.map((v, i) => (i === 4 ? 130 : v)), J, { ...L, rollFirst: false }).tcp;
  assert.ok(Math.hypot(a.x - c.x, a.y - c.y, a.z - c.z) < 1e-9, 'roll after pitch would not');
  const rolled = s.map((v, i) => (i === 4 ? 130 : v));
  const r = [180, 170, 160, 150, 140, 130, 120].map((pitch) => ik({ x: 230, y: 40, z: 20, pitch }, J, L, rolled)).find((x) => x.ok);
  assert.ok(r && r.servos[4] === J[4].zero, 'IK solution has a neutral wrist roll');
  const t = fk(r.servos, J, L).tcp;
  assert.ok(Math.hypot(t.x - 230, t.y - 40, t.z - 20) < 1e-6);
});

test('servo <-> joint mapping honours zero and direction', () => {
  const j = { zero: 100, dir: -1 };
  assert.equal(toJoint(130, j), -30);
  assert.equal(toServo(-30, j), 130);
});

test('IK inverts FK for random reachable poses', () => {
  let solved = 0;
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let k = 0; k < 500; k++) {
    const s = J.map((j) => j.min + rnd() * (j.max - j.min));
    s[4] = J[4].zero; // roll neutral (IK keeps it there on this arm)
    const f = fk(s, J, L);
    const r = ik({ ...f.tcp, pitch: f.pitch }, J, L, s);
    if (!r.ok) continue;
    solved++;
    const g = fk(r.servos, J, L);
    assert.ok(close(g.tcp.x, f.tcp.x, 1e-6) && close(g.tcp.y, f.tcp.y, 1e-6) && close(g.tcp.z, f.tcp.z, 1e-6));
    assert.ok(close(g.pitch, f.pitch, 1e-6));
  }
  assert.ok(solved > 300, `solved ${solved}/500`);
});

test('IK reports unreachable targets', () => {
  const r = ik({ x: 2000, y: 0, z: 100, pitch: 90 }, J, L, J.map((j) => j.home));
  assert.equal(r.ok, false);
  assert.match(r.reason, /reach/i);
});

test('IK reaches behind the base by leaning back over it', () => {
  const r = ik({ x: -230, y: -10, z: 20, pitch: 150 }, J, L, J.map((j) => j.home));
  assert.ok(r.ok, r.reason);
  const f = fk(r.servos, J, L).tcp;
  assert.ok(Math.hypot(f.x + 230, f.y + 10, f.z - 20) < 1e-6);
});
