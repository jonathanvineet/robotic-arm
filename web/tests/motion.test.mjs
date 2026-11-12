import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Planner } from '../js/motion.js';

const DT = 0.02;
const run = (P, maxSteps = 5000) => {
  let n = 0;
  while (P.moving() && n < maxSteps) {
    P.update(DT);
    n++;
  }
  return n;
};

function make() {
  const P = new Planner(6);
  P.init([90, 90, 90, 90, 90, 90]);
  P.vmax = 90;
  P.amax = 240;
  P.jvmax = [120, 60, 90, 120, 180, 180];
  return P;
}

test('reaches every target exactly', () => {
  const P = make();
  const t = [0, 165, 12.5, 180, 33, 120];
  P.setTargets(t, Array(6).fill(true), true);
  const n = run(P);
  assert.ok(n < 5000);
  P.ax.forEach((a, i) => assert.equal(a.pos, t[i]));
});

test('never exceeds speed caps or acceleration', () => {
  const P = make();
  P.setTargets([0, 0, 0, 0, 0, 0], Array(6).fill(true), false);
  const last = Array(6).fill(0);
  for (let s = 0; s < 400; s++) {
    P.update(DT);
    P.ax.forEach((a, i) => {
      assert.ok(Math.abs(a.vel) <= P.capOf(i) + 1e-9, `joint ${i} speed ${a.vel}`);
      if (a.vel !== 0) assert.ok(Math.abs(a.vel - last[i]) / DT <= P.amax + 1e-6, `joint ${i} accel`);
      last[i] = a.vel;
    });
  }
});

test('the slow shoulder cap is honoured', () => {
  const P = make();
  P.setTarget(1, 165);
  let peak = 0;
  for (let s = 0; s < 200; s++) {
    P.update(DT);
    peak = Math.max(peak, Math.abs(P.ax[1].vel));
  }
  assert.ok(peak <= 60 + 1e-9 && peak > 55, `peak ${peak}`);
});

test('synced move: all joints arrive together', () => {
  const P = make();
  const t = [170, 100, 30, 95, 0, 92];
  P.setTargets(t, Array(6).fill(true), true);
  const arrive = Array(6).fill(-1);
  for (let s = 0; s < 3000; s++) {
    P.update(DT);
    P.ax.forEach((a, i) => {
      if (arrive[i] < 0 && a.pos === t[i] && a.vel === 0) arrive[i] = s;
    });
  }
  assert.ok(Math.max(...arrive) - Math.min(...arrive) <= 15, `arrivals ${arrive}`);
});

test('unsynced move: small moves finish early', () => {
  const P = make();
  P.setTargets([170, 95, 90, 90, 90, 90], Array(6).fill(true), false);
  for (let s = 0; s < 25; s++) P.update(DT);
  assert.equal(P.ax[1].pos, 95);
  assert.ok(P.ax[0].pos < 170);
});

test('stop glides to a halt, halt freezes instantly', () => {
  const P = make();
  P.setTarget(0, 0);
  for (let s = 0; s < 30; s++) P.update(DT);
  const at = P.ax[0].pos;
  P.stop();
  run(P);
  assert.ok(P.ax[0].pos < at && P.ax[0].pos > 0);
  P.setTarget(0, 180);
  for (let s = 0; s < 20; s++) P.update(DT);
  P.halt();
  assert.equal(P.moving(), false);
});

test('streaming targets (slider drag) track smoothly without overshoot', () => {
  const P = make();
  let target = 90;
  for (let s = 0; s < 150; s++) {
    if (s % 2 === 0) target = Math.min(150, target + 1.5);
    P.setTarget(2, target);
    P.update(DT);
    assert.ok(P.ax[2].pos <= target + 1e-9);
  }
  run(P);
  assert.equal(P.ax[2].pos, 150);
});
