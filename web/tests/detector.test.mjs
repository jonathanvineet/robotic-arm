import { test } from 'node:test';
import assert from 'node:assert/strict';
import { decodeYolo, nms, letterbox, isEnd2End } from '../js/detector.js';

// build a fake YOLO head output [1, 4+nc, N]
function head(anchors, nc = 1, N = 50) {
  const C = 4 + nc;
  const d = new Float32Array(C * N);
  anchors.forEach(([cx, cy, w, h, scores], i) => {
    d[i] = cx; d[N + i] = cy; d[2 * N + i] = w; d[3 * N + i] = h;
    scores.forEach((s, c) => (d[(4 + c) * N + i] = s));
  });
  return { d, dims: [1, C, N] };
}

test('decodes boxes above the confidence threshold', () => {
  const { d, dims } = head([[100, 100, 40, 20, [0.9]], [300, 200, 10, 10, [0.3]]]);
  const b = decodeYolo(d, dims, { conf: 0.5 });
  assert.equal(b.length, 1);
  assert.deepEqual(b[0].slice(0, 4), [80, 90, 120, 110]);
  assert.ok(Math.abs(b[0][4] - 0.9) < 1e-6);
});

test('NMS removes overlapping duplicates, keeps separate objects', () => {
  const { d, dims } = head([[100, 100, 40, 40, [0.9]], [104, 102, 40, 40, [0.8]], [300, 300, 40, 40, [0.7]]]);
  const b = decodeYolo(d, dims, { conf: 0.5 });
  assert.equal(b.length, 2);
  assert.deepEqual(b.map((x) => x[4] > 0.85), [true, false]);
});

test('transposed [1, N, 4+nc] layout and multi-class', () => {
  const N = 2100, C = 6; // real raw heads have thousands of anchors (8400 at 640 px)
  const d = new Float32Array(N * C);
  d.set([50, 50, 10, 10, 0.1, 0.8], 0);
  d.set([200, 50, 10, 10, 0.95, 0.2], C);
  const b = decodeYolo(d, [1, N, C], { conf: 0.5 });
  assert.deepEqual(b.map((x) => x[5]).sort(), [0, 1]);
});

test('letterbox maps crop pixels back exactly', () => {
  const lb = letterbox(1200, 600, 640);
  assert.equal(lb.r, 640 / 1200);
  assert.equal(lb.dx, 0);
  assert.equal(lb.dy, (640 - 320) / 2);
  const x = 900, y = 450;
  const mx = x * lb.r + lb.dx, my = y * lb.r + lb.dy;
  assert.ok(Math.abs((mx - lb.dx) / lb.r - x) < 1e-9 && Math.abs((my - lb.dy) / lb.r - y) < 1e-9);
});

test('nms keeps different classes even if they overlap', () => {
  const k = nms([[0, 0, 10, 10, 0.9, 0], [0, 0, 10, 10, 0.8, 1]], 0.45);
  assert.equal(k.length, 2);
});

test('YOLO26 end-to-end output [1, 300, 6] = x1,y1,x2,y2,score,class', () => {
  const K = 300;
  const d = new Float32Array(K * 6);
  d.set([113.6, 271.79, 200.21, 358.54, 0.9, 0], 0);
  d.set([10, 10, 50, 40, 0.2, 0], 6);
  const dims = [1, K, 6];
  assert.ok(isEnd2End(dims) && !isEnd2End([1, 5, 8400]));
  const b = decodeYolo(d, dims, { conf: 0.5 });
  assert.equal(b.length, 1);
  assert.deepEqual(b[0].map((v) => Math.round(v * 100) / 100), [113.6, 271.79, 200.21, 358.54, 0.9, 0]);
});

import { computeEnhance, applyEnhance, tileGrid } from '../js/detector.js';

function washedOut(w = 64, h = 48) {
  // low-contrast, bright image: values squeezed into 150..230 (like the user's whitish webcam)
  const d = new Uint8ClampedArray(w * h * 4);
  for (let i = 0; i < w * h; i++) {
    const v = 150 + ((i * 37) % 81);
    d.set([v, v - 5, v + 3, 255], i * 4);
  }
  return d;
}
const lum = (d) => {
  let lo = 255, hi = 0, s = 0;
  for (let i = 0; i < d.length; i += 4) {
    const l = 0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2];
    lo = Math.min(lo, l); hi = Math.max(hi, l); s += l;
  }
  return { lo, hi, mean: s / (d.length / 4) };
};

test('washed-out frame: auto contrast + brightness restore a full range and mid-grey average', () => {
  const d = washedOut();
  const before = lum(d);
  const P = computeEnhance(d, 64, 48, null, { auto: true, gamma: 0, sat: 1.15, wb: false });
  applyEnhance(d, 64, 48, P);
  const after = lum(d);
  assert.ok(before.hi - before.lo < 90 && after.hi - after.lo > 200, `range ${before.hi - before.lo} -> ${after.hi - after.lo}`);
  assert.ok(Math.abs(after.mean - 117) < 30, `mean ${after.mean}`);
});

test('enhancement only touches the given rectangle (letterbox padding stays 114)', () => {
  const d = washedOut(10, 10);
  for (let i = 0; i < 10; i++) d.set([114, 114, 114, 255], i * 4); // first row = padding
  applyEnhance(d, 10, 10, computeEnhance(d, 10, 10, [0, 1, 10, 10], { auto: true, sat: 1.15 }), [0, 1, 10, 10]);
  for (let i = 0; i < 10; i++) assert.deepEqual([...d.slice(i * 4, i * 4 + 3)], [114, 114, 114]);
});

test('all corrections off = no change', () => {
  assert.equal(computeEnhance(washedOut(), 64, 48, null, { auto: false, gamma: 0 && 1, sat: 1, wb: false }), null);
});

test('zoom tiles cover the region with overlap; auto picks more tiles for bigger regions', () => {
  const t = tileGrid([0, 0, 1000, 800], 2);
  assert.equal(t.length, 4);
  assert.deepEqual(t[0].slice(0, 2), [0, 0]);
  assert.deepEqual(t[3].slice(2), [1000, 800]);
  assert.ok(t[0][2] > t[1][0], 'neighbouring tiles overlap');
  assert.equal(tileGrid([0, 0, 500, 400], 'auto').length, 1);
  assert.equal(tileGrid([0, 0, 900, 900], 'auto').length, 4);
  assert.ok(tileGrid([0, 0, 1920, 1080], 'auto').length <= 6);
  assert.equal(tileGrid([0, 0, 1920, 1080], 1).length, 1);
});
