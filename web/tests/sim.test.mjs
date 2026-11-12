// Protocol tests against the VirtualArm — the same checks you would run on
// the real ESP (see the Test tab in the app).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { VirtualArm } from '../js/sim.js';

function arm(flashStore = { v: null }) {
  const out = [];
  const a = new VirtualArm({
    emit: (m) => out.push(m),
    flash: { load: () => flashStore.v, save: (c) => (flashStore.v = c) },
  });
  let now = 0;
  const run = (ms) => {
    for (const end = now + ms; now < end; now += 5) a.loop(now);
  };
  const last = (t) => [...out].reverse().find((m) => m.t === t);
  return { a, out, run, last };
}

test('get returns info, cfg and state', () => {
  const { a, out } = arm();
  a.handle({ c: 'get' });
  assert.deepEqual(out.map((m) => m.t), ['info', 'cfg', 's']);
  assert.equal(out[1].j.length, 6);
});

test('joint moves smoothly and reports state', () => {
  const { a, run, last } = arm();
  a.handle({ c: 'joint', i: 0, a: 120 });
  run(100);
  const mid = last('s');
  assert.equal(mid.m, 1);
  assert.ok(mid.p[0] > 90 && mid.p[0] < 120);
  run(3000);
  assert.equal(last('s').p[0], 120);
  assert.equal(last('s').m, 0);
});

test('targets are clamped to soft limits', () => {
  const { a } = arm();
  a.handle({ c: 'joint', i: 1, a: 999 });
  assert.equal(a.P.ax[1].tgt, 165);
  a.handle({ c: 'set', j: [-50, null, null, null, null, 5] });
  assert.equal(a.P.ax[0].tgt, 0);
  assert.equal(a.P.ax[5].tgt, 30);
  assert.equal(a.P.ax[1].tgt, 165, 'null leaves joint unchanged');
});

test('jog is relative', () => {
  const { a } = arm();
  a.handle({ c: 'jog', i: 3, d: -15 });
  a.handle({ c: 'jog', i: 3, d: -15 });
  assert.equal(a.P.ax[3].tgt, 60);
});

test('E-stop freezes and latches until reset', () => {
  const { a, run, last } = arm();
  a.handle({ c: 'joint', i: 2, a: 170 });
  run(300);
  a.handle({ c: 'estop' });
  const frozen = a.P.ax[2].pos;
  run(500);
  assert.equal(a.P.ax[2].pos, frozen);
  assert.equal(last('s').e, 1);
  a.handle({ c: 'joint', i: 2, a: 10 });
  assert.equal(a.P.ax[2].tgt, frozen, 'moves rejected while latched');
  assert.match(last('log').m, /E-STOP/);
  a.handle({ c: 'reset' });
  a.handle({ c: 'joint', i: 2, a: 10 });
  assert.equal(a.P.ax[2].tgt, 10);
});

test('relax blocks motion until hold', () => {
  const { a } = arm();
  a.handle({ c: 'relax' });
  a.handle({ c: 'joint', i: 0, a: 10 });
  assert.equal(a.P.ax[0].tgt, 90);
  a.handle({ c: 'hold' });
  a.handle({ c: 'joint', i: 0, a: 10 });
  assert.equal(a.P.ax[0].tgt, 10);
});

test('home returns to configured home', () => {
  const { a, run } = arm();
  a.handle({ c: 'set', j: [10, 30, 170, 20, 160, 110] });
  run(6000);
  a.handle({ c: 'home' });
  run(6000);
  assert.deepEqual(a.P.ax.map((x) => x.pos), [90, 90, 90, 90, 90, 60]);
});

test('speed command updates planner and cfg', () => {
  const { a, last } = arm();
  a.handle({ c: 'speed', v: 30, a: 100, sync: false });
  assert.equal(a.P.vmax, 30);
  assert.equal(a.P.amax, 100);
  assert.equal(last('cfg').sync, false);
  a.handle({ c: 'speed', v: 99999 });
  assert.equal(a.P.vmax, 720, 'speed clamped');
});

test('calibration: cfg, save, reboot, factory', () => {
  const flash = { v: null };
  const { a, last } = arm(flash);
  a.handle({ c: 'cfg', j: [{ i: 0, min: 20, max: 160, home: 200, dir: -1, pmin: 600 }] });
  const j0 = last('cfg').j[0];
  assert.deepEqual([j0.min, j0.max, j0.home, j0.dir, j0.pmin], [20, 160, 160, 1 * -1, 600]);
  a.handle({ c: 'save' });
  a.handle({ c: 'reboot' });
  assert.equal(a.S.j[0].min, 20, 'survives reboot');
  a.handle({ c: 'factory' });
  assert.equal(a.S.j[0].min, 0);
});

test('swapped min/max are fixed up', () => {
  const { a } = arm();
  a.handle({ c: 'cfg', j: [{ i: 4, min: 150, max: 40 }] });
  assert.deepEqual([a.S.j[4].min, a.S.j[4].max], [40, 150]);
});

test('client disconnect makes the arm stop', () => {
  const { a, run } = arm();
  a.handle({ c: 'joint', i: 0, a: 0 });
  run(300);
  a.disconnect();
  run(3000);
  assert.ok(a.P.ax[0].pos > 0, 'did not continue to the old target');
  assert.equal(a.P.moving(), false);
});

test('telemetry: 20 Hz while moving, 1 Hz idle', () => {
  const { a, out, run } = arm();
  run(2000);
  out.length = 0;
  run(3000);
  const idle = out.filter((m) => m.t === 's').length;
  assert.ok(idle >= 2 && idle <= 4, `idle ${idle}`);
  a.handle({ c: 'joint', i: 0, a: 0 });
  out.length = 0;
  run(500);
  const busy = out.filter((m) => m.t === 's').length;
  assert.ok(busy >= 9 && busy <= 11, `moving ${busy}`);
});

test('unknown command is reported', () => {
  const { a, last } = arm();
  a.handle({ c: 'dance' });
  assert.match(last('log').m, /Unknown/);
});
