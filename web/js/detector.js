// detector.js — runs the plastic YOLO model IN THE BROWSER (onnxruntime-web, WebAssembly).
// Model: web/models/plastic.onnx (+ plastic.json with class names), made from best.pt by
// Plastic-Segregation-AI/export_onnx.py. No Python needed at run time.

/** Non-maximum suppression (per class). boxes: [x1,y1,x2,y2,score,cls] */
export function nms(boxes, iouThr = 0.45, maxDet = 100) {
  const sorted = [...boxes].sort((a, b) => b[4] - a[4]);
  const keep = [];
  const iou = (a, b) => {
    const w = Math.max(0, Math.min(a[2], b[2]) - Math.max(a[0], b[0]));
    const h = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[1], b[1]));
    const inter = w * h;
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter || 1);
  };
  for (const b of sorted) {
    if (keep.length >= maxDet) break;
    if (keep.every((k) => k[5] !== b[5] || iou(k, b) < iouThr)) keep.push(b);
  }
  return keep;
}

/** End-to-end models (YOLO26, YOLOv10): [1, K, 6] = x1, y1, x2, y2, score, class — already NMS-free. */
export function isEnd2End(dims) {
  return dims.length === 3 && dims[2] === 6 && dims[1] <= 1000;
}

/**
 * Decode a detection output into boxes in model-input pixels.
 *  - YOLO26 / YOLOv10 end-to-end: [1, K, 6]
 *  - YOLOv8 / YOLO11 raw head:     [1, 4+nc, N] (ultralytics default) or [1, N, 4+nc]
 */
export function decodeYolo(data, dims, { conf = 0.5, iou = 0.45, maxDet = 100 } = {}) {
  if (isEnd2End(dims)) {
    const out = [];
    for (let i = 0; i < dims[1]; i++) {
      const o = i * 6;
      if (data[o + 4] >= conf) out.push([data[o], data[o + 1], data[o + 2], data[o + 3], data[o + 4], Math.round(data[o + 5])]);
    }
    return nms(out, 0.7, maxDet); // the model already removed duplicates; this only catches exact repeats
  }
  const transposed = dims[2] > dims[1]; // [1, 4+nc, N]
  const C = transposed ? dims[1] : dims[2];
  const N = transposed ? dims[2] : dims[1];
  const nc = C - 4;
  const at = transposed ? (c, i) => data[c * N + i] : (c, i) => data[i * C + c];
  const out = [];
  for (let i = 0; i < N; i++) {
    let best = 0;
    let cls = 0;
    for (let c = 0; c < nc; c++) {
      const s = at(4 + c, i);
      if (s > best) {
        best = s;
        cls = c;
      }
    }
    if (best < conf) continue;
    const cx = at(0, i), cy = at(1, i), w = at(2, i), h = at(3, i);
    out.push([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, best, cls]);
  }
  return nms(out, iou, maxDet);
}

/** Letterbox geometry for fitting a (sw x sh) crop into an S x S model input. */
export function letterbox(sw, sh, S) {
  const r = Math.min(S / sw, S / sh);
  return { r, dx: (S - sw * r) / 2, dy: (S - sh * r) / 2, nw: sw * r, nh: sh * r };
}

export class YoloDetector {
  async load(url = 'models/plastic.onnx') {
    const res = await fetch(url);
    if (!res.ok) throw new Error(`model not found at web/${url} — run "python export_onnx.py" in Plastic-Segregation-AI`);
    const buf = new Uint8Array(await res.arrayBuffer());
    const base = new URL('../vendor/ort/', import.meta.url).href;
    // 1) GPU (WebGPU) — fastest, Chrome/Edge on Mac & Windows
    if (navigator.gpu && (await navigator.gpu.requestAdapter().catch(() => null))) {
      try {
        const ort = await import('../vendor/ort/ort.webgpu.min.mjs');
        ort.env.wasm.wasmPaths = base;
        ort.env.wasm.numThreads = 1;
        this.session = await ort.InferenceSession.create(buf, { executionProviders: ['webgpu', 'wasm'], graphOptimizationLevel: 'all' });
        this.ort = ort;
        this.backend = 'GPU (WebGPU)';
      } catch (e) {
        console.warn('WebGPU backend failed, using CPU', e);
        this.session = null;
      }
    }
    // 2) CPU (WebAssembly): several threads, in a background worker so the page stays smooth
    if (!this.session) {
      const ort = await import('../vendor/ort/ort.wasm.min.mjs');
      ort.env.wasm.wasmPaths = base;
      ort.env.wasm.numThreads = globalThis.crossOriginIsolated ? Math.min(4, navigator.hardwareConcurrency || 2) : 1;
      for (const proxy of [true, false]) {
        try {
          ort.env.wasm.proxy = proxy;
          this.session = await ort.InferenceSession.create(buf.slice(), { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
          break;
        } catch (e) {
          if (!proxy) throw e;
          console.warn('worker mode failed, running on the page thread', e);
        }
      }
      this.ort = ort;
      this.backend = `CPU, ${ort.env.wasm.numThreads} thread${ort.env.wasm.numThreads > 1 ? 's' : ''}${ort.env.wasm.proxy ? ', worker' : ''}`;
    }
    this.threads = this.ort.env.wasm.numThreads;
    this.input = this.session.inputNames[0];
    this.output = this.session.outputNames[0];
    this.size = 640;
    this.names = { 0: 'plastic' };
    try {
      const meta = await (await fetch(url.replace(/\.onnx$/, '.json'))).json();
      if (meta.imgsz) this.size = +meta.imgsz;
      if (meta.names) this.names = meta.names;
    } catch {}
    this.canvas = document.createElement('canvas');
    this.canvas.width = this.canvas.height = this.size;
    this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });
    // warm-up (WebGPU compiles its shaders on the first run)
    const S = this.size;
    await this.session.run({ [this.input]: new this.ort.Tensor('float32', new Float32Array(3 * S * S), [1, 3, S, S]) });
    return this;
  }

  /** src: canvas/video/image; roi: [x1,y1,x2,y2] in src pixels or null. Returns full-frame boxes. */
  async detect(src, srcW, srcH, roi, conf = 0.5) {
    const S = this.size;
    const [sx, sy, ex, ey] = roi ? roi.map(Math.round) : [0, 0, srcW, srcH];
    const sw = Math.max(1, ex - sx), sh = Math.max(1, ey - sy);
    const lb = letterbox(sw, sh, S);
    const g = this.ctx;
    g.fillStyle = 'rgb(114,114,114)';
    g.fillRect(0, 0, S, S);
    g.drawImage(src, sx, sy, sw, sh, lb.dx, lb.dy, lb.nw, lb.nh);
    const px = g.getImageData(0, 0, S, S).data;
    const n = S * S;
    const f = new Float32Array(3 * n);
    for (let i = 0; i < n; i++) {
      f[i] = px[i * 4] / 255;
      f[n + i] = px[i * 4 + 1] / 255;
      f[2 * n + i] = px[i * 4 + 2] / 255;
    }
    const t0 = performance.now();
    const out = await this.session.run({ [this.input]: new this.ort.Tensor('float32', f, [1, 3, S, S]) });
    this.ms = performance.now() - t0;
    const o = out[this.output];
    return decodeYolo(o.data, o.dims, { conf }).map(([x1, y1, x2, y2, s, c]) => [
      Math.max(0, (x1 - lb.dx) / lb.r + sx), Math.max(0, (y1 - lb.dy) / lb.r + sy),
      Math.min(srcW, (x2 - lb.dx) / lb.r + sx), Math.min(srcH, (y2 - lb.dy) / lb.r + sy),
      Math.round(s * 1000) / 1000, c,
    ]);
  }
}
