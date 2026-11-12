import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  makeCamera, solveHomography, applyH, invertH, reprojError, cornerMarkers, layoutArea, boxToWorld,
  Tracker, planPick, PICK_DEFAULTS, rollFor, LAYOUT_DEFAULT,
} from '../js/vision-math.js';
import { fk, DEFAULT_LINKS } from '../js/kinematics.js';
import { defaultConfig } from '../js/sim.js';

const J = defaultConfig().j;
const L = DEFAULT_LINKS;
const home = J.map((j) => j.home);
const S = { ...PICK_DEFAULTS, area: layoutArea(LAYOUT_DEFAULT) };
const inView = (cam, p) => p.depth > 0 && p.px >= 0 && p.py >= 0 && p.px <= cam.w && p.py <= cam.h;

test('corner markers sit at (±B, ±B) in the documented order', () => {
  const m = cornerMarkers({ B: 300 });
  assert.deepEqual(m.map((p) => [p.id, p.x, p.y]), [[0, 300, 300], [1, 300, -300], [2, -300, -300], [3, -300, 300]]);
  const custom = cornerMarkers({ points: [{ id: 7, x: 1, y: 2 }, { id: 8, x: 3, y: 4 }, { id: 9, x: 5, y: 6 }, { id: 10, x: 7, y: 8 }] });
  assert.deepEqual(custom.map((p) => p.id), [7, 8, 9, 10]);
});

test('default camera: straight above the base, sees all 4 corner markers + bin, clears the arm', () => {
  const cam = makeCamera();
  assert.deepEqual([cam.pos[0], cam.pos[1]], [0, 0]);
  const c = cam.project([0, 0, 0]);
  assert.ok(Math.abs(c.px - cam.w / 2) < 1e-6 && Math.abs(c.py - cam.h / 2) < 1e-6, 'base is in the image centre');
  const half = LAYOUT_DEFAULT.size / 2 + 10;
  for (const m of cornerMarkers())
    for (const [dx, dy] of [[-half, -half], [half, half], [-half, half], [half, -half]])
      assert.ok(inView(cam, cam.project([m.x + dx, m.y + dy, 0])), `marker ${m.id} cut off`);
  const b = PICK_DEFAULTS.bin;
  assert.ok(inView(cam, cam.project([b.x, b.y, 0])));
  const armTop = L.base + L.upper + L.fore + L.wrist + L.tool;
  assert.ok(cam.pos[2] - armTop >= 150, `camera ${cam.pos[2]} too close to the arm top ${armTop}`);
});

test('homography from the corner markers is exact for any camera rotation / tilt', () => {
  for (const [roll, cx, cy, lx, ly] of [[0, 0, 0, 0, 0], [90, 0, 0, 0, 0], [137, 30, -20, 0, 10], [180, -50, 40, 20, 0]]) {
    const cam = makeCamera({ x: cx, y: cy, z: 1050, lookX: lx, lookY: ly, roll });
    const pairs = cornerMarkers().map((m) => ({ ...cam.project([m.x, m.y, 0]), x: m.x, y: m.y }));
    const H = solveHomography(pairs);
    assert.ok(reprojError(H, pairs) < 1e-6);
    for (const [x, y] of [[250, 0], [-200, 150], [0, -280], [-260, -260]]) {
      const p = cam.project([x, y, 0]);
      const [wx, wy] = applyH(H, [p.px, p.py]);
      assert.ok(Math.hypot(wx - x, wy - y) < 1e-6, `roll ${roll}: ${wx},${wy}`);
    }
    const Hi = invertH(H);
    const back = applyH(H, applyH(Hi, [120, -40]));
    assert.ok(Math.hypot(back[0] - 120, back[1] + 40) < 1e-6);
  }
});

test('noisy clicks give a small error, collinear points are rejected', () => {
  const cam = makeCamera();
  const pts = [[300, 300], [300, -300], [-300, -300], [-300, 300], [100, 0], [0, 150]];
  const pairs = pts.map(([x, y], i) => {
    const p = cam.project([x, y, 0]);
    return { px: p.px + (i % 2 ? 1 : -1), py: p.py + (i % 3 ? 1 : -1), x, y };
  });
  assert.ok(reprojError(solveHomography(pairs), pairs) < 3);
  const line = [0, 1, 2, 3].map((i) => ({ px: i * 10, py: i * 10, x: i, y: i }));
  assert.equal(solveHomography(line), null);
});

test('boxToWorld finds centre and long axis of an elongated object', () => {
  const cam = makeCamera();
  const H = solveHomography(cornerMarkers().map((m) => ({ ...cam.project([m.x, m.y, 0]), x: m.x, y: m.y })));
  const a = cam.project([180 - 14, 20 + 35, 0]);
  const b = cam.project([180 + 14, 20 - 35, 0]);
  const box = [Math.min(a.px, b.px), Math.min(a.py, b.py), Math.max(a.px, b.px), Math.max(a.py, b.py), 0.9, 0];
  const o = boxToWorld(H, box);
  assert.ok(Math.hypot(o.x - 180, o.y - 20) < 1e-6);
  assert.ok(Math.abs(o.len - 70) < 1e-6 && Math.abs(o.wid - 28) < 1e-6 && o.elongated);
  assert.ok(Math.abs(Math.abs(o.angle) - 90) < 1e-6, `angle ${o.angle}`);
});

test('tracker only confirms objects seen in several consecutive frames', () => {
  const t = new Tracker({ minHits: 3 });
  t.update([{ x: 150, y: 0 }], 0);
  t.update([{ x: 151, y: 1 }], 100);
  assert.equal(t.stable(100).length, 0);
  t.update([{ x: 150, y: -1 }, { x: 250, y: 80 }], 200);
  assert.equal(t.stable(200).length, 1);
  t.update([], 1200);
  assert.equal(t.tracks.length, 0, 'stale tracks expire');
});

test('pick plan grips at the object and drops in the bin, all around the robot', () => {
  for (const deg of [0, 60, 150, 180, -90, -150]) {
    const x = 240 * Math.cos((deg * Math.PI) / 180), y = 240 * Math.sin((deg * Math.PI) / 180);
    const plan = planPick({ x, y, conf: 0.9 }, J, L, home, S);
    assert.ok(plan.ok, `${deg}°: ${plan.reason}`);
    const grip = fk(plan.moves[1].j, J, L).tcp;
    assert.ok(Math.hypot(grip.x - x, grip.y - y) < 1e-6 && Math.abs(grip.z - PICK_DEFAULTS.pickZ) < 1e-6, `${deg}° grip`);
    const bin = fk(plan.moves[4].j, J, L).tcp;
    assert.ok(Math.hypot(bin.x - PICK_DEFAULTS.bin.x, bin.y - PICK_DEFAULTS.bin.y) < 1e-6, 'bin');
    assert.ok(plan.moves[2].j[5] < plan.moves[1].j[5], 'gripper closes');
    for (const m of plan.moves) m.j.forEach((v, i) => assert.ok(v >= J[i].min - 1e-6 && v <= J[i].max + 1e-6));
  }
  assert.deepEqual(planPick({ x: 240, y: 0 }, J, L, home, S).moves[0].label, 'hover above object');
});

test('pick plan refuses unreachable / unsafe targets', () => {
  assert.equal(planPick({ x: 600, y: 0 }, J, L, home, { ...S, area: null }).reason, 'out of reach');
  assert.equal(planPick({ x: 60, y: 10 }, J, L, home, S).reason, 'too close to the base');
  assert.equal(planPick({ x: PICK_DEFAULTS.bin.x, y: PICK_DEFAULTS.bin.y }, J, L, home, S).reason, 'inside the bin');
  assert.equal(planPick({ x: 400, y: 0 }, J, L, home, S).reason, 'outside pick area');
});

test('wrist roll alignment (only when enabled)', () => {
  const on = { ...PICK_DEFAULTS, alignRoll: true };
  assert.equal(rollFor({ elongated: true, angle: 0 }, 0, J, PICK_DEFAULTS), J[4].zero, 'off by default for roll-before-pitch arms');
  assert.equal(rollFor({ elongated: false, angle: 30 }, 0, J, on), J[4].zero);
  assert.equal(Math.abs(rollFor({ elongated: true, angle: 0 }, 0, J, on) - J[4].zero), 90);
  assert.equal(rollFor({ elongated: true, angle: 90 }, 0, J, on), J[4].zero);
});
