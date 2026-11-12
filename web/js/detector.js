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

// ------------------------------------------------------------------ image correction
export const ENHANCE_DEFAULTS = {
  // Off by default: on the user's washed-out test frame the model scored the plastic higher on the raw
  // image (0.75) than after any correction (0.68-0.70). Turn on only if your lighting is much worse.
  auto: false, // stretch contrast between the 1st and 99.5th brightness percentile
  gamma: 0, // 0 = automatic brightness (only with auto); otherwise 0.5 (brighter) .. 2 (darker)
  sat: 1, // colour saturation
  wb: false, // grey-world white balance (fixes a blue/yellow cast)
};

/**
 * Work out a correction from an RGBA frame (or a region of it). Returns per-channel
 * lookup tables + saturation, or null when nothing needs changing.
 */
export function computeEnhance(data, w, h, rect = null, opts = ENHANCE_DEFAULTS) {
  const o = { ...ENHANCE_DEFAULTS, ...opts };
  if (!o.auto && !o.gamma && o.sat === 1 && !o.wb) return null;
  const [x0, y0, x1, y1] = rect ? rect.map(Math.round) : [0, 0, w, h];
  const hist = new Uint32Array(256);
  let sr = 0, sg = 0, sb = 0, n = 0;
  for (let y = Math.max(0, y0); y < Math.min(h, y1); y += 2)
    for (let x = Math.max(0, x0); x < Math.min(w, x1); x += 2) {
      const i = (y * w + x) * 4;
      const r = data[i], g = data[i + 1], b = data[i + 2];
      hist[(r * 77 + g * 150 + b * 29) >> 8]++;
      sr += r; sg += g; sb += b; n++;
    }
  if (!n) return null;
  const pct = (p) => {
    let acc = 0;
    for (let v = 0; v < 256; v++) if ((acc += hist[v]) >= p * n) return v;
    return 255;
  };
  let lo = 0, hi = 255;
  if (o.auto) {
    lo = pct(0.01);
    hi = pct(0.995);
    if (hi - lo < 40) [lo, hi] = [Math.max(0, lo - 20), Math.min(255, hi + 20)]; // flat image: don't blow up noise
  }
  const mean = (sr * 0.299 + sg * 0.587 + sb * 0.114) / n;
  const gray = (sr + sg + sb) / 3 / n;
  const gain = o.wb ? [sr, sg, sb].map((s) => Math.min(1.4, Math.max(0.7, gray / (s / n || 1)))) : [1, 1, 1];
  let gamma = o.gamma || 1;
  if (!o.gamma && o.auto) {
    const m = Math.min(0.95, Math.max(0.05, (mean - lo) / Math.max(1, hi - lo)));
    gamma = Math.min(1.8, Math.max(0.6, Math.log(0.46) / Math.log(m))); // aim for a mid-grey average
  }
  const luts = gain.map((gn) => {
    const t = new Uint8ClampedArray(256);
    for (let v = 0; v < 256; v++) {
      const x = Math.min(1, Math.max(0, (v * gn - lo) / Math.max(1, hi - lo)));
      t[v] = Math.round(255 * x ** gamma);
    }
    return t;
  });
  return { luts, sat: o.sat, lo, hi, gamma, gain };
}

/** Apply a correction in place to an RGBA buffer (optionally only inside rect). */
export function applyEnhance(data, w, h, P, rect = null) {
  if (!P) return;
  const [R, G, B] = P.luts;
  const s = P.sat;
  const [x0, y0, x1, y1] = rect ? rect.map(Math.round) : [0, 0, w, h];
  for (let y = Math.max(0, y0); y < Math.min(h, y1); y++) {
    let i = (y * w + Math.max(0, x0)) * 4;
    for (let x = Math.max(0, x0); x < Math.min(w, x1); x++, i += 4) {
      let r = R[data[i]], g = G[data[i + 1]], b = B[data[i + 2]];
      if (s !== 1) {
        const l = 0.299 * r + 0.587 * g + 0.114 * b;
        r = l + (r - l) * s;
        g = l + (g - l) * s;
        b = l + (b - l) * s;
      }
      data[i] = r;
      data[i + 1] = g;
      data[i + 2] = b;
    }
  }
}

// ------------------------------------------------------------------ zoom tiles
/**
 * Split a region into overlapping tiles so small objects appear bigger to the model.
 * mode: 'auto' | 1 | 2 | 3 (tiles per side). Returns [x1,y1,x2,y2] rectangles.
 */
export function tileGrid(region, mode = 'auto', { overlap = 0.2, target = 560, maxTiles = 6 } = {}) {
  const [x0, y0, x1, y1] = region;
  const rw = x1 - x0, rh = y1 - y0;
  let nx, ny;
  if (mode === 'auto') {
    nx = Math.min(3, Math.max(1, Math.ceil(rw / target)));
    ny = Math.min(3, Math.max(1, Math.ceil(rh / target)));
    while (nx * ny > maxTiles) nx >= ny ? nx-- : ny--;
  } else nx = ny = Math.max(1, +mode || 1);
  if (nx === 1 && ny === 1) return [[x0, y0, x1, y1]];
  // tile size so that n tiles with the given overlap cover the region exactly
  const tw = rw / (nx - (nx - 1) * overlap), th = rh / (ny - (ny - 1) * overlap);
  const tiles = [];
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++) {
      const tx = x0 + i * tw * (1 - overlap), ty = y0 + j * th * (1 - overlap);
      tiles.push([tx, ty, Math.min(x1, tx + tw), Math.min(y1, ty + th)].map(Math.round));
    }
  return tiles;
}

/** Letterbox geometry for fitting a (sw x sh) crop into an S x S model input. */
export function letterbox(sw, sh, S) {
  const r = Math.min(S / sw, S / sh);
  return { r, dx: (S - sw * r) / 2, dy: (S - sh * r) / 2, nw: sw * r, nh: sh * r };
}

export class YoloDetector {
  /** backend: 'auto' (GPU if available, else CPU) | 'gpu' | 'cpu' */
  async load(url = 'models/plastic.onnx', { backend = 'auto' } = {}) {
    const res = await fetch(url);
    if (!res.ok) throw new Error(`model not found at web/${url} — run "python export_onnx.py" in Plastic-Segregation-AI`);
    const buf = new Uint8Array(await res.arrayBuffer());
    const base = new URL('../vendor/ort/', import.meta.url).href;
    // 1) GPU (WebGPU) — fastest. Chrome/Edge: Metal on Mac, DirectX 12 on Windows (NVIDIA RTX etc.).
    //    Ask for the high-performance GPU so gaming laptops use the NVIDIA card, not the integrated one.
    const adapter = backend !== 'cpu' && navigator.gpu ? await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' }).catch(() => null) : null;
    if (adapter) {
      try {
        const ort = await import('../vendor/ort/ort.webgpu.min.mjs');
        ort.env.wasm.wasmPaths = base;
        ort.env.wasm.numThreads = 1;
        ort.env.webgpu.powerPreference = 'high-performance';
        ort.env.webgpu.adapter = adapter; // use exactly this adapter
        this.session = await ort.InferenceSession.create(buf, { executionProviders: ['webgpu', 'wasm'], graphOptimizationLevel: 'all' });
        this.ort = ort;
        const info = adapter.info || (await adapter.requestAdapterInfo?.().catch(() => null)) || {};
        this.gpuName = [info.vendor, info.architecture, info.description].filter(Boolean).join(' ').trim() || 'unknown GPU';
        this.integrated = /intel|integrated|uhd|iris|radeon\(tm\) graphics/i.test(this.gpuName) && !/nvidia/i.test(this.gpuName);
        this.backend = `GPU (${this.gpuName})`;
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

  /**
   * src: canvas/video/image; roi: [x1,y1,x2,y2] in src pixels or null.
   * opts.enhance: computeEnhance() result applied to the model input; opts.tiles: 'auto' | 1 | 2 | 3.
   * Returns full-frame boxes [x1,y1,x2,y2,score,cls].
   */
  async detect(src, srcW, srcH, roi, conf = 0.5, opts = {}) {
    const region = roi ? roi.map(Math.round) : [0, 0, srcW, srcH];
    const tiles = tileGrid(region, opts.tiles ?? 1, { maxTiles: opts.maxTiles ?? 6 });
    const all = [];
    const t0 = performance.now();
    for (const t of tiles) all.push(...(await this.detectTile(src, srcW, srcH, t, conf, opts.enhance)));
    this.ms = performance.now() - t0;
    this.tiles = tiles;
    // objects seen by two overlapping tiles: keep the best box
    return tiles.length > 1 ? nms(all, 0.5) : all;
  }

  async detectTile(src, srcW, srcH, [sx, sy, ex, ey], conf, enhance) {
    const S = this.size;
    const sw = Math.max(1, ex - sx), sh = Math.max(1, ey - sy);
    const lb = letterbox(sw, sh, S);
    const g = this.ctx;
    g.fillStyle = 'rgb(114,114,114)';
    g.fillRect(0, 0, S, S);
    g.drawImage(src, sx, sy, sw, sh, lb.dx, lb.dy, lb.nw, lb.nh);
    const img = g.getImageData(0, 0, S, S);
    const px = img.data;
    if (enhance) {
      applyEnhance(px, S, S, enhance, [lb.dx, lb.dy, lb.dx + lb.nw, lb.dy + lb.nh]);
      if (this.keepInput) g.putImageData(img, 0, 0); // so the "model view" shows the corrected input
    }
    const n = S * S;
    const f = new Float32Array(3 * n);
    for (let i = 0; i < n; i++) {
      f[i] = px[i * 4] / 255;
      f[n + i] = px[i * 4 + 1] / 255;
      f[2 * n + i] = px[i * 4 + 2] / 255;
    }
    const out = await this.session.run({ [this.input]: new this.ort.Tensor('float32', f, [1, 3, S, S]) });
    const o = out[this.output];
    return decodeYolo(o.data, o.dims, { conf }).map(([x1, y1, x2, y2, s, c]) => [
      Math.max(0, (x1 - lb.dx) / lb.r + sx), Math.max(0, (y1 - lb.dy) / lb.r + sy),
      Math.min(srcW, (x2 - lb.dx) / lb.r + sx), Math.min(srcH, (y2 - lb.dy) / lb.r + sy),
      Math.round(s * 1000) / 1000, c,
    ]);
  }
}
