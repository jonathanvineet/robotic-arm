// Proves the simulator moves exactly like the firmware: compiles
// firmware/RoboArm/motion.h natively, runs the same scenario in C++ and JS,
// and compares every sample. Skips if no C++ compiler is installed.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { Planner } from '../js/motion.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DT = 0.02;

function scenario(P, step) {
  const all = [true, true, true, true, true, true];
  if (step === 0) P.setTargets([0, 150, 45, 120, 10, 170], all, true);
  if (step === 60) P.setTarget(2, 160);
  if (step === 120) P.stop();
  if (step === 150) P.setTargets([180, 20, 90, 90, 90, 40], [true, true, false, true, false, true], false);
  if (step === 260) P.halt();
  if (step === 270) P.setTargets([90, 90, 90, 90, 90, 90], all, true);
}

function jsTrace() {
  const P = new Planner(6);
  P.init([90, 90, 90, 90, 90, 90]);
  P.vmax = 90;
  P.amax = 240;
  P.jvmax = [120, 60, 90, 120, 180, 180];
  const rows = [];
  for (let s = 0; s < 600; s++) {
    scenario(P, s);
    P.update(DT);
    rows.push(P.ax.map((a) => a.pos));
  }
  return rows;
}

function compiler() {
  for (const c of ['g++', 'clang++', 'c++']) {
    try {
      execFileSync(c, ['--version'], { stdio: 'ignore' });
      return c;
    } catch {}
  }
  return null;
}

const cxx = compiler();

test('JS planner == firmware planner (sample-by-sample)', { skip: !cxx && 'no C++ compiler' }, () => {
  const out = join(mkdtempSync(join(tmpdir(), 'roboarm-')), 'trace');
  execFileSync(cxx, [
    '-std=c++17', '-O2', '-DMOTION_REAL=double',
    `-I${join(root, 'firmware', 'RoboArm')}`,
    join(root, 'firmware', 'tests', 'native', 'test_motion.cpp'),
    '-o', out,
  ]);
  const cpp = execFileSync(out, ['trace'], { encoding: 'utf8' })
    .trim()
    .split('\n')
    .map((l) => l.split(',').map(Number));
  const js = jsTrace();
  assert.equal(cpp.length, js.length);
  let worst = 0;
  for (let s = 0; s < js.length; s++)
    for (let i = 0; i < 6; i++) worst = Math.max(worst, Math.abs(cpp[s][i] - js[s][i]));
  assert.ok(worst < 1e-5, `max difference ${worst}°`);
});

test('firmware native unit tests pass', { skip: !cxx && 'no C++ compiler' }, () => {
  const out = join(mkdtempSync(join(tmpdir(), 'roboarm-')), 'test_motion');
  // float, like the ESP
  execFileSync(cxx, [
    '-std=c++17', '-O2',
    `-I${join(root, 'firmware', 'RoboArm')}`,
    join(root, 'firmware', 'tests', 'native', 'test_motion.cpp'),
    '-o', out,
  ]);
  const res = execFileSync(out, { encoding: 'utf8' });
  assert.match(res, /^OK/m);
});
