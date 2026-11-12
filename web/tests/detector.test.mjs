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
