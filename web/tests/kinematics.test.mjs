import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fk, ik, DEFAULT_LINKS, toJoint, toServo } from '../js/kinematics.js';
import { defaultConfig } from '../js/sim.js';

const J = defaultConfig().j;
const L = DEFAULT_LINKS;
const close = (a, b, eps = 1e-6) => Math.abs(a - b) < eps;

test('home pose points straight up', () => {
  const f = fk([90, 90, 90, 90, 90, 60], J, L);
  assert.ok(close(f.tcp.x, 0) && close(f.tcp.y, 0));
  assert.ok(close(f.tcp.z, L.base + L.upper + L.fore + L.wrist + L.tool));
  assert.ok(close(f.pitch, 0));
});

test('base rotation turns the arm toward +y', () => {
  const f = fk([135, 135, 90, 90, 90, 60], J, L); // yaw +45, shoulder tilted forward 45
  assert.ok(f.tcp.x > 0 && f.tcp.y > 0 && close(f.tcp.x, f.tcp.y, 1e-6));
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

test('IK rejects targets behind the base (outside base limits)', () => {
  const r = ik({ x: -150, y: -10, z: 150, pitch: 90 }, J, L, J.map((j) => j.home));
  assert.equal(r.ok, false);
});
