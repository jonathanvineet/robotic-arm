// vision.js — camera -> detections -> table coordinates -> pick & place.
// Sources: the simulated camera in the twin, or the Python vision bridge
// (Plastic-Segregation-AI/vision_bridge.py: USB camera + YOLO + ArUco).

import { World } from './world.js';
import { YoloDetector, computeEnhance, applyEnhance, ENHANCE_DEFAULTS } from './detector.js';
import { AR } from '../vendor/aruco.mjs';
import { fk } from './kinematics.js';
import {
  solveHomography, applyH, invertH, reprojError, cornerMarkers, layoutArea, boxToWorld, Tracker, planPick,
  PICK_DEFAULTS, LAYOUT_DEFAULT, CORNERS, rejectReason, cameraFromHomography, correctHeight,
} from './vision-math.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
const store = {
  get(k, d) {
    try {
      const v = localStorage.getItem('roboarm.vision.' + k);
      return v == null ? d : JSON.parse(v);
    } catch {
      return d;
    }
  },
  set(k, v) {
    try {
      localStorage.setItem('roboarm.vision.' + k, JSON.stringify(v));
    } catch {}
  },
};

export function initVision(app) {
  const { S, send, twin, startJob, stopJob, runMoves, appLog } = app;
  const world = new World(twin);
  const savedSim = store.get('sim5', null);
  if (savedSim) world.applySettings(savedSim);
  world.spawn(3);

  const V = {
    source: null,
    ws: null,
    last: null, // last vision message
    img: null, // last bridge frame (Image)
    fps: 0,
    frames: 0,
    fpsT: performance.now(),
    cal: null, // { pairs, H, err, w, h }
    tracker: new Tracker({ minHits: 5 }),
    settings: { ...PICK_DEFAULTS, minHits: 5, ...store.get('pick', {}) },
    busy: false,
    cooldownUntil: 0,
    target: null,
    picked: 0,
    attempts: [], // {x, y, n}: same spot picked again = the last grasp missed
  };
  V.settings.bin = { ...PICK_DEFAULTS.bin, ...(V.settings.bin || {}) };
  V.settings.enh = { ...ENHANCE_DEFAULTS, ...(V.settings.enh || {}) };
  V.settings.tiles ??= 'auto';
  V.settings.preview ??= true;
  V.tracker.minHits = V.settings.minHits;

  // reachable ring at grip height (for the overlay)
  V.reach = (() => {
    let r0 = null, r1 = null;
    for (let r = 60; r <= 500; r += 5) {
      const ok = planPick({ x: r, y: 0 }, S.cfg.j, S.links, S.cfg.j.map((j) => j.home), { ...V.settings, area: null, noPickRadius: 0 }).ok;
      if (ok && r0 == null) r0 = r;
      if (ok) r1 = r;
    }
    return r0 == null ? null : { r0, r1 };
  })();
  // camera FOV guess — only used when the camera looks (almost) straight down
  function hfovPrior() {
    return V.source === 'sim' ? world.settings.cam.hfov : +V.settings.hfov || 70;
  }
  const canvas = $('#feed');
  const ctx = canvas.getContext('2d');

  // ------------------------------------------------------------ calibration storage
  const calKey = () => `cal.${V.source}`;
  function loadCal() {
    V.cal = store.get(calKey(), null);
    if (V.cal?.H && V.cal.w) V.cal.cam = cameraFromHomography(V.cal.H, V.cal.w, V.cal.h, hfovPrior());
    renderCal();
  }
  function saveCal() {
    store.set(calKey(), V.cal);
    renderCal();
  }
  function setH(pairs, how) {
    const H = solveHomography(pairs);
    if (!H) {
      appLog('Calibration failed: need 4+ well-spread points', 'error');
      return false;
    }
    const err = reprojError(H, pairs);
    const w = V.last?.w, h = V.last?.h;
    const cam = w ? cameraFromHomography(H, w, h, hfovPrior()) : null;
    V.cal = { pairs, H, Hinv: invertH(H), err, w, h, how, cam };
    saveCal();
    sendRoi();
    appLog(`Camera calibrated from ${pairs.length} points (${how}), error ${err.toFixed(1)} mm`, err > 5 ? 'warn' : 'info');
    if (cam) {
      const [x, y, z] = cam.pos;
      appLog(`Camera found at X ${x.toFixed(0)} Y ${y.toFixed(0)} mm, ${z.toFixed(0)} mm above the table, tilted ${cam.tilt.toFixed(0)}° (FOV ${cam.hfov.toFixed(0)}°, ${cam.focal}). Measure it — if this is far off, the markers are misplaced.`);
    }
    V.tracker.clear();
    return true;
  }
  function renderCal() {
    const tag = $('#calState');
    if (V.cal?.H) {
      const c = V.cal.cam;
      tag.textContent = `${V.cal.how} · ${V.cal.err.toFixed(1)} mm` + (c ? ` · cam ${c.pos[2].toFixed(0)} mm up, ${c.tilt.toFixed(0)}° tilt` : '');
      world.showCamEstimate(V.source === 'sim' || $('#showGrid').checked ? c?.pos : null);
      tag.className = `tag ${V.cal.err > 5 ? 'warn' : 'ok'}`;
    } else {
      tag.textContent = 'not calibrated';
      world.showCamEstimate(null);
      tag.className = 'tag warn';
    }
    const pairs = V.touch || [];
    $('#calPairs').innerHTML = pairs
      .map((p, k) => `<li><span class="nm ang">px ${p.px.toFixed(0)},${p.py.toFixed(0)} → X ${p.x.toFixed(0)} Y ${p.y.toFixed(0)} mm</span><button class="btn sm danger" data-k="${k}">✕</button></li>`)
      .join('');
  }
  $('#calPairs').onclick = (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    V.touch.splice(+b.dataset.k, 1);
    renderCal();
  };

  // ---------------- workspace markers (4 corners around the robot)
  const saveWorld = () => store.set('sim5', world.settings);
  const layout = () => ({ ...LAYOUT_DEFAULT, ...world.settings.layout });
  V.settings.area = layoutArea(layout());
  function renderLayout() {
    const L = layout();
    $('#layB').value = L.B;
    $('#laySize').value = L.size;
    $('#layDict').textContent = L.dict;
    $('#layTable').innerHTML =
      '<tr><th>Corner</th><th>ID</th><th>X mm</th><th>Y mm</th></tr>' +
      cornerMarkers(L)
        .map((m, i) => `<tr data-i="${i}"><td>${CORNERS[i]}</td><td><input type="number" data-f="id" value="${m.id}" step="1"></td><td><input type="number" data-f="x" value="${m.x.toFixed(1)}" step="1"></td><td><input type="number" data-f="y" value="${m.y.toFixed(1)}" step="1"></td></tr>`)
        .join('');
  }
  function setLayout(patch) {
    world.applySettings({ layout: { ...layout(), ...patch } });
    saveWorld();
    V.settings.area = layoutArea(layout());
    renderLayout();
    camCheck?.();
    if (V.source === 'sim') resetCal();
    sendRoi();
  }
  $('#layB').onchange = (e) => setLayout({ B: Math.max(100, +e.target.value || LAYOUT_DEFAULT.B), points: null });
  $('#laySize').onchange = (e) => setLayout({ size: Math.max(20, +e.target.value || LAYOUT_DEFAULT.size) });
  $('#layTable').addEventListener('change', () => {
    const points = $$('#layTable tr[data-i]').map((tr) => ({
      id: Math.round(+tr.querySelector('[data-f=id]').value),
      x: +tr.querySelector('[data-f=x]').value,
      y: +tr.querySelector('[data-f=y]').value,
    }));
    setLayout({ points, ids: points.map((p) => p.id) });
  });
  $('#layReset').onclick = () => setLayout({ ...LAYOUT_DEFAULT });
  renderLayout();
  function resetCal() {
    V.cal = null;
    saveCal();
  }

  // Tell the bridge to run YOLO only on the mat (+ margin): small objects stay
  // large enough for the model, and the 3D-printed arm itself is never "plastic".
  /** Pixel rectangle around the pick area (+ margin): YOLO only looks there. */
  function roiPx() {
    if (!V.cal?.Hinv) return null;
    const a = V.settings.area;
    const m = 25;
    const pts = [[a.x0 - m, a.y0 - m], [a.x0 - m, a.y1 + m], [a.x1 + m, a.y0 - m], [a.x1 + m, a.y1 + m]].map((p) => applyH(V.cal.Hinv, p));
    const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
    const W = V.cal.w || V.last?.w || 1e9, Hh = V.cal.h || V.last?.h || 1e9;
    const r = [Math.max(0, Math.min(...xs)), Math.max(0, Math.min(...ys)), Math.min(W, Math.max(...xs)), Math.min(Hh, Math.max(...ys))].map(Math.round);
    return r[2] - r[0] > 32 && r[3] - r[1] > 32 ? r : null;
  }
  function sendRoi() {
    if (V.source !== 'bridge' || V.ws?.readyState !== WebSocket.OPEN) return;
    V.ws.send(JSON.stringify({ c: 'roi', roi: roiPx() }));
  }

  $('#calMarkers').onclick = () => {
    const ms = V.last?.markers || [];
    const known = cornerMarkers(layout());
    const pairs = ms
      .map((m) => {
        const k = known.find((q) => q.id === m.id);
        return k && { px: m.x, py: m.y, x: k.x, y: k.y };
      })
      .filter(Boolean);
    if (pairs.length < 4) {
      const seen = ms.map((m) => m.id).join(', ') || 'none';
      appLog(`Only ${pairs.length}/4 corner markers found (camera sees IDs: ${seen}; expected ${known.map((k) => k.id).join(', ')}). Check all 4 are in view, flat and evenly lit.`, 'warn');
      return;
    }
    setH(pairs, 'markers');
  };
  $('#calTouch').onchange = (e) => {
    V.touch = e.target.checked ? V.touch || [] : V.touch;
    $('#touchHint').hidden = !e.target.checked;
    renderCal();
  };
  $('#calSolve').onclick = () => (V.touch?.length >= 4 ? setH(V.touch, 'touch') : appLog('Touch at least 4 points first', 'warn'));
  $('#calClear').onclick = () => {
    V.touch = [];
    resetCal();
    sendRoi();
  };

  // ------------------------------------------------------------ sources
  function setSource(src) {
    if (V.ws) {
      const w = V.ws;
      V.ws = null;
      w.close();
    }
    stopWebcam();
    V.source = src;
    V.last = null;
    V.img = null;
    V.tracker.clear();
    store.set('source', src);
    $('#visUrl').hidden = src !== 'bridge';
    $('#visConnect').hidden = src !== 'bridge';
    $('#camSelect').hidden = src !== 'webcam';
    world.setSimVisible(src === 'sim');
    loadCal();
    if (src === 'bridge') connectBridge();
    if (src === 'webcam') startWebcam();
  }

  // ------------------------------------------------------------ webcam + YOLO + ArUco, all in the browser
  const arDetector = new AR.Detector({ dictionaryName: 'ARUCO_4X4_1000' }); // IDs 0-49 = OpenCV DICT_4X4_50
  function feedMsg(text) {
    $('#feedMsg').hidden = !text;
    $('#feedMsg').textContent = text || '';
  }
  function stopWebcam() {
    V.camRun = (V.camRun || 0) + 1; // stops the running loop
    V.stream?.getTracks().forEach((t) => t.stop());
    V.stream = null;
  }
  async function loadModel() {
    if (V.yolo || V.yoloLoading) return;
    V.yoloLoading = true;
    try {
      V.yolo = await new YoloDetector().load('models/plastic.onnx');
      V.modelError = null;
      appLog(`Plastic detector running in the browser on ${V.yolo.backend} (${V.yolo.size}px model)`);
    } catch (e) {
      V.modelError = e.message || String(e);
      appLog(`Plastic detector: ${V.modelError}`, 'warn');
    } finally {
      V.yoloLoading = false;
    }
  }
  async function listCameras() {
    try {
      const cams = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
      const sel = $('#camSelect');
      const cur = V.camId || store.get('camId', '');
      sel.innerHTML = cams.map((c, i) => `<option value="${esc(c.deviceId)}">${esc(c.label || `Camera ${i + 1}`)}</option>`).join('');
      if (cams.some((c) => c.deviceId === cur)) sel.value = cur;
    } catch {}
  }
  async function startWebcam() {
    stopWebcam();
    const run = V.camRun;
    if (!navigator.mediaDevices?.getUserMedia) {
      feedMsg(window.isSecureContext ? 'This browser has no camera access.' : 'Camera needs http://localhost (open the app with npm start on this PC).');
      return;
    }
    feedMsg('Opening camera… (allow camera access if the browser asks)');
    const camId = store.get('camId', '');
    try {
      V.stream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: { deviceId: camId ? { exact: camId } : undefined, width: { ideal: 1920 }, height: { ideal: 1080 } },
      });
    } catch (e) {
      if (camId) {
        store.set('camId', '');
        return startWebcam(); // saved camera gone: use the default one
      }
      feedMsg(`Camera: ${e.message || e.name}`);
      return;
    }
    if (run !== V.camRun) return V.stream.getTracks().forEach((t) => t.stop());
    V.camId = V.stream.getVideoTracks()[0]?.getSettings().deviceId;
    listCameras();
    buildCamControls();
    V.video ||= Object.assign(document.createElement('video'), { muted: true, playsInline: true });
    V.video.srcObject = V.stream;
    await V.video.play().catch(() => {});
    V.frame ||= document.createElement('canvas');
    loadModel();
    feedMsg('');
    let markerT = 0;
    let markers = [];
    while (run === V.camRun) {
      const v = V.video;
      const w = v.videoWidth, h = v.videoHeight;
      if (!w) {
        await new Promise((r) => setTimeout(r, 100));
        continue;
      }
      if (V.frame.width !== w || V.frame.height !== h) Object.assign(V.frame, { width: w, height: h });
      const fx = V.frame.getContext('2d', { willReadFrequently: true });
      fx.drawImage(v, 0, 0, w, h);
      const now = performance.now();
      // markers: on a <=1280 px copy, twice a second while calibrating, then every 3 s
      if (now - markerT > (V.cal?.H ? 3000 : 500)) {
        markerT = now;
        const k = Math.min(1, 1280 / w);
        const mw = Math.round(w * k), mh = Math.round(h * k);
        V.mframe ||= document.createElement('canvas');
        if (V.mframe.width !== mw) Object.assign(V.mframe, { width: mw, height: mh });
        const mx = V.mframe.getContext('2d', { willReadFrequently: true });
        mx.drawImage(V.frame, 0, 0, mw, mh);
        markers = arDetector.detect(mx.getImageData(0, 0, mw, mh)).map((m) => ({
          id: m.id,
          x: m.corners.reduce((s, c) => s + c.x, 0) / 4 / k,
          y: m.corners.reduce((s, c) => s + c.y, 0) / 4 / k,
        }));
      }
      // image correction: measured on a 960 px copy (inside the pick area once calibrated),
      // applied to the model input and to the preview
      const k2 = Math.min(1, 960 / w);
      const vw = Math.round(w * k2), vh = Math.round(h * k2);
      V.small ||= document.createElement('canvas');
      if (V.small.width !== vw) Object.assign(V.small, { width: vw, height: vh });
      const sx = V.small.getContext('2d', { willReadFrequently: true });
      sx.drawImage(V.frame, 0, 0, vw, vh);
      const sdata = sx.getImageData(0, 0, vw, vh);
      const roi = roiPx();
      V.enh = computeEnhance(sdata.data, vw, vh, roi && roi.map((v) => v * k2), V.settings.enh);
      if (V.settings.preview && V.enh) {
        applyEnhance(sdata.data, vw, vh, V.enh);
        sx.putImageData(sdata, 0, 0);
        V.view = V.small;
      } else V.view = null;
      let boxes = [];
      if (V.yolo && !V.busy) {
        try {
          const gpu = /GPU/.test(V.yolo.backend);
          V.yolo.keepInput = true;
          // Auto: zoom tiles only inside the calibrated pick area (whole-frame tiling is too slow), GPU only
          const auto = V.settings.tiles === 'auto';
          const tiles = auto ? (roi && gpu ? 'auto' : 1) : V.settings.tiles;
          boxes = await V.yolo.detect(V.frame, w, h, roi, V.settings.conf ?? 0.5, { enhance: V.enh, tiles, maxTiles: auto ? 4 : 9 });
          const mv = $('#modelView');
          if (mv && V.yolo.canvas) mv.getContext('2d').drawImage(V.yolo.canvas, 0, 0, mv.width, mv.height);
        } catch (e) {
          V.modelError = `detection failed: ${e.message || e}`;
          V.yolo = null;
          appLog(V.modelError, 'error');
        }
      }
      if (run !== V.camRun) break;
      onVision({ t: 'vision', src: 'webcam', w, h, boxes, markers, names: V.yolo?.names, model: !!V.yolo, modelError: V.modelError, ms: V.yolo?.ms });
      await new Promise((r) => setTimeout(r, V.yolo ? 0 : 60));
    }
  }
  // ---------------- image correction + camera hardware controls
  function fillImage() {
    const e = V.settings.enh;
    $('#enhAuto').checked = !!e.auto;
    $('#enhWb').checked = !!e.wb;
    $('#enhGamma').value = e.gamma || 0;
    $('#enhGammaVal').textContent = e.gamma ? e.gamma.toFixed(2) : 'auto';
    $('#enhSat').value = e.sat;
    $('#enhSatVal').textContent = (+e.sat).toFixed(2);
    $('#enhPreview').checked = !!V.settings.preview;
    $('#tiles').value = String(V.settings.tiles);
  }
  const saveSettings = () => store.set('pick', V.settings);
  $('#enhAuto').onchange = (e) => ((V.settings.enh.auto = e.target.checked), saveSettings());
  $('#enhWb').onchange = (e) => ((V.settings.enh.wb = e.target.checked), saveSettings());
  $('#enhGamma').oninput = (e) => {
    const v = +e.target.value;
    V.settings.enh.gamma = v < 0.55 ? 0 : v; // far left = auto
    $('#enhGammaVal').textContent = V.settings.enh.gamma ? V.settings.enh.gamma.toFixed(2) : 'auto';
    saveSettings();
  };
  $('#enhSat').oninput = (e) => {
    V.settings.enh.sat = +e.target.value;
    $('#enhSatVal').textContent = (+e.target.value).toFixed(2);
    saveSettings();
  };
  $('#enhPreview').onchange = (e) => ((V.settings.preview = e.target.checked), saveSettings());
  $('#tiles').onchange = (e) => ((V.settings.tiles = e.target.value === 'auto' ? 'auto' : +e.target.value), saveSettings());
  $('#enhReset').onclick = () => {
    V.settings.enh = { ...ENHANCE_DEFAULTS };
    V.settings.tiles = 'auto';
    V.settings.preview = true;
    saveSettings();
    fillImage();
  };
  fillImage();

  /** Exposure / contrast / white balance on the camera itself (whatever this camera + browser allow). */
  function buildCamControls() {
    const box = $('#camCtl');
    const track = V.stream?.getVideoTracks()[0];
    const caps = track?.getCapabilities?.() || {};
    const cur = track?.getSettings?.() || {};
    const KEYS = ['exposureMode', 'exposureCompensation', 'exposureTime', 'brightness', 'contrast', 'saturation', 'sharpness', 'whiteBalanceMode', 'colorTemperature', 'focusMode', 'focusDistance', 'zoom'];
    const keys = KEYS.filter((k) => caps[k] != null);
    if (!keys.length) {
      box.innerHTML = '<p class="hint">This camera/browser exposes no hardware controls — the software correction above still works. (Windows + Chrome usually exposes more than macOS.)</p>';
      return;
    }
    const saved = store.get(`camctl.${V.camId}`, {});
    box.innerHTML = keys
      .map((k) => {
        const c = caps[k];
        const label = k.replace(/([A-Z])/g, ' $1').toLowerCase();
        if (Array.isArray(c))
          return `<label class="inline">${label} <select data-cc="${k}">${c.map((v) => `<option ${v === cur[k] ? 'selected' : ''}>${v}</option>`).join('')}</select></label>`;
        return `<label class="field"><span>${label} <b class="mono" data-ccv="${k}">${cur[k] ?? ''}</b></span><input type="range" data-cc="${k}" min="${c.min}" max="${c.max}" step="${c.step || (c.max - c.min) / 100}" value="${cur[k] ?? c.min}"></label>`;
      })
      .join('');
    const apply = async (k, v) => {
      try {
        const extra = {};
        if (k === 'exposureTime' || k === 'exposureCompensation') extra.exposureMode = k === 'exposureTime' ? 'manual' : 'continuous';
        if (k === 'colorTemperature') extra.whiteBalanceMode = 'manual';
        if (k === 'focusDistance') extra.focusMode = 'manual';
        await track.applyConstraints({ advanced: [{ ...extra, [k]: v }] });
        const all = { ...store.get(`camctl.${V.camId}`, {}), ...extra, [k]: v };
        store.set(`camctl.${V.camId}`, all);
      } catch (e) {
        appLog(`Camera ${k}: ${e.message || e}`, 'warn');
      }
    };
    box.querySelectorAll('[data-cc]').forEach((el) => {
      el.onchange = el.oninput = () => {
        const k = el.dataset.cc;
        const v = el.tagName === 'SELECT' ? el.value : +el.value;
        const lab = box.querySelector(`[data-ccv="${k}"]`);
        if (lab) lab.textContent = v;
        apply(k, v);
      };
    });
    // re-apply what was set last time for this camera
    if (Object.keys(saved).length) track.applyConstraints({ advanced: [saved] }).catch(() => {});
  }

  $('#saveFrame').onclick = () => {
    if (!V.frame?.width) return appLog('No camera frame yet', 'warn');
    V.frame.toBlob(
      (b) => {
        const a = document.createElement('a');
        a.href = URL.createObjectURL(b);
        a.download = `plastic_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.jpg`;
        a.click();
        setTimeout(() => URL.revokeObjectURL(a.href), 2000);
        appLog(`Saved ${a.download} (${V.frame.width}×${V.frame.height}) — label these in Roboflow and retrain for your camera height`);
      },
      'image/jpeg',
      0.95,
    );
  };

  $('#camSelect').onchange = (e) => {
    store.set('camId', e.target.value);
    startWebcam();
  };

  function connectBridge() {
    const url = $('#visUrl').value.trim() || 'ws://localhost:8765/ws';
    store.set('url', url);
    $('#feedMsg').textContent = `Connecting to ${url}…`;
    let ws;
    try {
      ws = new WebSocket(url);
    } catch (e) {
      $('#feedMsg').textContent = e.message;
      return;
    }
    V.ws = ws;
    ws.onopen = () => {
      appLog(`Vision bridge connected (${url})`);
      setTimeout(sendRoi, 300);
    };
    ws.onmessage = (ev) => {
      let m;
      try {
        m = JSON.parse(ev.data);
      } catch {
        return;
      }
      if (m.t === 'bstatus') {
        // bridge is up but has no camera yet
        if (V.source === 'bridge') {
          $('#feedMsg').hidden = false;
          $('#feedMsg').textContent = `${m.msg}${m.model ? '' : ` · ${m.modelError}`}`;
          $('#visStat').textContent = 'PC camera: waiting… (switch the source to "Simulated camera" to work without it)';
        }
        return;
      }
      if (m.t !== 'vision') return;
      if (m.modelError && !V.modelWarned) {
        V.modelWarned = true;
        appLog(`Vision bridge: ${m.modelError} — markers only, no plastic detection`, 'warn');
      }
      if (m.img) {
        const im = new Image();
        im.onload = () => (V.img = im);
        im.src = m.img;
      }
      onVision(m);
    };
    ws.onclose = () => {
      if (V.ws !== ws) return;
      V.ws = null;
      $('#feedMsg').textContent = 'Bridge disconnected — is vision_bridge.py running? Retrying…';
      $('#feedMsg').hidden = false;
      setTimeout(() => V.source === 'bridge' && !V.ws && connectBridge(), 2000);
    };
  }
  $('#visConnect').onclick = connectBridge;
  $('#visSource').onchange = (e) => setSource(e.target.value);

  // ------------------------------------------------------------ per-frame pipeline
  function onVision(m) {
    V.last = m;
    V.frames++;
    const now = performance.now();
    if (now - V.fpsT > 1000) {
      V.fps = (V.frames * 1000) / (now - V.fpsT);
      V.frames = 0;
      V.fpsT = now;
    }
    if (m.src !== 'webcam' || !m.modelError) $('#feedMsg').hidden = true;
    else feedMsg(`Camera OK — ${m.modelError}`);
    if (V.cal?.w && (V.cal.w !== m.w || V.cal.h !== m.h)) {
      appLog(`Camera resolution changed (${m.w}x${m.h}) — recalibrate`, 'warn');
      V.cal = null;
      saveCal();
    }
    let objs = [];
    if (V.cal?.H) {
      // angled-camera correction: slide each detection back along the camera ray to the object's mid-height
      const camPos = V.settings.heightFix !== false ? V.cal.cam?.pos : null;
      objs = m.boxes.map((b) => ({ ...correctHeight(boxToWorld(V.cal.H, b), camPos, V.settings.pickZ), box: b }));
    }
    if (!V.busy) V.tracker.update(objs, now);
    const stable = new Set(V.tracker.stable(now).map((t) => t.id));
    for (const t of V.tracker.tracks) {
      t.stable = stable.has(t.id);
      t.why = rejectReason(t, V.settings) || (triesAt(t) >= 3 ? 'gave up after 3 tries' : null);
      t.pickable = t.stable && !t.why && planPick(t, S.cfg.j, S.links, S.tgt, V.settings).ok;
      if (t.stable && !t.why && !t.pickable) t.why = 'out of reach';
    }
    world.showTracks(V.busy ? [] : V.tracker.tracks, V.target?.id);
    $('#visStat').textContent =
      `${m.src === 'sim' ? 'SIM' : 'CAM'} ${m.w}×${m.h} · ${V.fps.toFixed(0)} fps · ${m.boxes.length} det · ` +
      `${m.markers?.length || 0}/4 markers · ${V.tracker.tracks.filter((t) => t.pickable).length} pickable` +
      (m.src !== 'sim' && m.model === false ? ' · NO MODEL (markers only)' : '') +
      (m.ms ? ` · YOLO ${m.ms.toFixed(0)} ms on ${V.yolo?.backend || '?'}` : '');
    autoPick(now);
    if (/[?&]debug\b/.test(location.search) && now - (V.dbgT || 0) > 2000) {
      V.dbgT = now;
      console.log('[vision]', $('#visStat').textContent, '| tiles', V.yolo?.tiles?.length ?? 0, '| enh', V.enh ? `lo ${V.enh.lo} hi ${V.enh.hi} gamma ${V.enh.gamma.toFixed(2)}` : 'off', '| boxes', JSON.stringify(m.boxes.map((b) => b.slice(0, 4).map(Math.round).concat(+(+b[4]).toFixed(2)))));
    }
  }

  // ------------------------------------------------------------ drawing
  function draw() {
    const m = V.last;
    if (!m) return;
    if (canvas.width !== m.w || canvas.height !== m.h) {
      canvas.width = m.w;
      canvas.height = m.h;
    }
    if (V.source === 'sim') world.renderTo(ctx);
    else if (V.source === 'webcam' && (V.view || V.frame)?.width) ctx.drawImage(V.view || V.frame, 0, 0, m.w, m.h);
    else if (V.img) ctx.drawImage(V.img, 0, 0, m.w, m.h);
    else {
      ctx.fillStyle = '#000';
      ctx.fillRect(0, 0, m.w, m.h);
    }
    const s = m.w / 640;
    ctx.lineWidth = 2 * s;
    ctx.font = `${12 * s}px system-ui`;
    // calibration overlay: 50 mm grid inside the marker square + reach circles around the base
    if (V.cal?.Hinv && $('#showGrid').checked) {
      const P = (x, y) => applyH(V.cal.Hinv, [x, y]);
      const A = layoutArea(layout());
      ctx.strokeStyle = 'rgba(63,208,201,.4)';
      ctx.lineWidth = 1 * s;
      ctx.beginPath();
      for (let x = Math.ceil(A.x0 / 50) * 50; x <= A.x1; x += 50) {
        ctx.moveTo(...P(x, A.y0));
        ctx.lineTo(...P(x, A.y1));
      }
      for (let y = Math.ceil(A.y0 / 50) * 50; y <= A.y1; y += 50) {
        ctx.moveTo(...P(A.x0, y));
        ctx.lineTo(...P(A.x1, y));
      }
      ctx.stroke();
      // forward axis (base servo at its zero) and the reachable ring
      ctx.strokeStyle = 'rgba(242,140,40,.9)';
      ctx.lineWidth = 2 * s;
      ctx.beginPath();
      ctx.moveTo(...P(0, 0));
      ctx.lineTo(...P(A.x1, 0));
      ctx.stroke();
      ctx.fillStyle = 'rgba(242,140,40,.9)';
      ctx.fillText('front', ...P(A.x1 - 30, 8));
      ctx.setLineDash([6 * s, 6 * s]);
      for (const rr of [V.reach?.r0, V.reach?.r1].filter(Boolean)) {
        ctx.beginPath();
        for (let k = 0; k <= 72; k++) {
          const th = (k / 72) * Math.PI * 2;
          const p = P(rr * Math.cos(th), rr * Math.sin(th));
          k ? ctx.lineTo(...p) : ctx.moveTo(...p);
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);
      // bin footprint
      const B = V.settings.binSize / 2;
      const bx = V.settings.bin.x, by = V.settings.bin.y;
      ctx.strokeStyle = 'rgba(62,207,142,.9)';
      ctx.beginPath();
      [[bx - B, by - B], [bx + B, by - B], [bx + B, by + B], [bx - B, by + B], [bx - B, by - B]].forEach((p, i) =>
        i ? ctx.lineTo(...P(...p)) : ctx.moveTo(...P(...p)),
      );
      ctx.stroke();
      ctx.lineWidth = 2 * s;
    }
    // zoom tiles the model looked at (webcam)
    if (V.source === 'webcam' && V.yolo?.tiles?.length > 1) {
      ctx.save();
      ctx.setLineDash([8 * s, 8 * s]);
      ctx.strokeStyle = 'rgba(255,255,255,.35)';
      ctx.lineWidth = 1 * s;
      for (const t of V.yolo.tiles) ctx.strokeRect(t[0], t[1], t[2] - t[0], t[3] - t[1]);
      ctx.restore();
    }
    for (const mk of m.markers || []) {
      ctx.strokeStyle = '#ffd166';
      ctx.strokeRect(mk.x - 6 * s, mk.y - 6 * s, 12 * s, 12 * s);
      ctx.fillStyle = '#ffd166';
      ctx.fillText(`#${mk.id}`, mk.x + 8 * s, mk.y - 8 * s);
    }
    for (const b of m.boxes) {
      const [x1, y1, x2, y2, conf] = b;
      let t = null;
      if (V.cal?.H) {
        const o = boxToWorld(V.cal.H, b);
        t = V.tracker.tracks.find((q) => Math.hypot(q.x - o.x, q.y - o.y) < 25);
      }
      const color = t?.id === V.target?.id ? '#ffd166' : t?.pickable ? '#3ecf8e' : t?.stable ? '#f28c28' : '#c9ced8';
      ctx.strokeStyle = color;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      ctx.fillStyle = color;
      const label = t ? `${(conf * 100).toFixed(0)}% · ${t.x.toFixed(0)},${t.y.toFixed(0)}mm${t.why ? ' · ' + t.why : ''}` : `${(conf * 100).toFixed(0)}%`;
      ctx.fillText(label, x1 + 2, Math.max(12 * s, y1 - 4 * s));
    }
    for (const p of V.touch || []) {
      ctx.strokeStyle = '#ff4d5e';
      ctx.beginPath();
      ctx.arc(p.px, p.py, 6 * s, 0, Math.PI * 2);
      ctx.stroke();
    }
    if (!V.cal?.H) {
      ctx.fillStyle = 'rgba(0,0,0,.6)';
      ctx.fillRect(0, m.h - 26 * s, m.w, 26 * s);
      ctx.fillStyle = '#ffd166';
      ctx.fillText('Not calibrated — click “Calibrate from mat”', 8 * s, m.h - 9 * s);
    }
  }

  canvas.addEventListener('click', (e) => {
    const m = V.last;
    if (!m) return;
    const px = (e.offsetX / canvas.clientWidth) * m.w;
    const py = (e.offsetY / canvas.clientHeight) * m.h;
    if ($('#calTouch').checked) {
      const tcp = fk(S.pos, S.cfg.j, S.links).tcp;
      if (tcp.z > 25) appLog(`Fingertip is ${tcp.z.toFixed(0)} mm above the table — lower it to touch the spot first`, 'warn');
      V.touch = [...(V.touch || []), { px, py, x: tcp.x, y: tcp.y }];
      renderCal();
      return;
    }
    if (!V.cal?.H) return;
    const [x, y] = applyH(V.cal.H, [px, py]);
    const t = V.tracker.tracks.find((q) => Math.hypot(q.x - x, q.y - y) < 35);
    if (t) pick(t);
    else appLog(`Clicked table point X ${x.toFixed(0)} Y ${y.toFixed(0)} mm (no object there)`);
  });

  // ------------------------------------------------------------ picking
  function triesAt(t) {
    return V.attempts.find((a) => Math.hypot(a.x - t.x, a.y - t.y) < 25)?.n || 0;
  }

  async function pick(t) {
    if (V.busy) return;
    const a = V.attempts.find((q) => Math.hypot(q.x - t.x, q.y - t.y) < 25);
    if (a) a.n++;
    else V.attempts.push({ x: t.x, y: t.y, n: 1 });
    if (V.attempts.length > 30) V.attempts.shift();
    const plan = planPick(t, S.cfg.j, S.links, S.tgt, V.settings);
    if (!plan.ok) {
      appLog(`Can't pick object at ${t.x.toFixed(0)},${t.y.toFixed(0)}: ${plan.reason}`, 'warn');
      return;
    }
    const job = startJob('Pick');
    if (!job) return;
    V.busy = true;
    V.target = t;
    appLog(`Picking at X ${t.x.toFixed(0)} Y ${t.y.toFixed(0)} mm (pitch ${plan.pitch}°)`);
    if (/debug/.test(location.search)) console.log('[vision] pick', t.x.toFixed(1), t.y.toFixed(1), 'grip tcp', JSON.stringify(plan.tcp));
    try {
      await runMoves(job, plan.moves, (k) => ($('#pickStat').textContent = `#${t.id}: ${plan.moves[k].label}`));
      V.picked++;
      $('#pickStat').textContent = `done · ${V.picked} picked`;
      console.log(`[vision] pick done #${V.picked}; bin=${world.sorted}`);
      if (S.job === job) S.job = null;
    } catch (e) {
      $('#pickStat').textContent = e.message;
    } finally {
      V.busy = false;
      V.target = null;
      V.tracker.clear();
      V.cooldownUntil = performance.now() + 900; // let the camera settle after the arm leaves
    }
  }

  function best() {
    return V.tracker.tracks
      .filter((t) => t.pickable)
      .sort((a, b) => Math.hypot(a.x, a.y) - Math.hypot(b.x, b.y))[0];
  }

  function autoPick(now) {
    if (!$('#autoPick').checked || V.busy || S.job || !S.online || S.estop || S.relaxed || S.moving) return;
    if (now < V.cooldownUntil || !V.cal?.H) return;
    const t = best();
    if (t) pick(t);
  }

  $('#pickNow').onclick = () => {
    if (!V.cal?.H) return appLog('Calibrate the camera first', 'warn');
    const t = best();
    t ? pick(t) : appLog('No confirmed, reachable plastic in view', 'warn');
  };
  $('#pickStop').onclick = () => {
    $('#autoPick').checked = false;
    stopJob();
    send({ c: 'stop' });
  };

  // ------------------------------------------------------------ settings
  const getPath = (o, p) => p.split('.').reduce((a, k) => a?.[k], o);
  const setPath = (o, p, v) => {
    const ks = p.split('.');
    const last = ks.pop();
    ks.reduce((a, k) => a[k], o)[last] = v;
  };
  function fillPick() {
    for (const inp of $$('[data-ps]')) {
      const v = getPath(V.settings, inp.dataset.ps);
      if (inp.type === 'checkbox') inp.checked = !!v;
      else inp.value = v ?? '';
      if (inp.dataset.ps === 'gripOpen' && v == null) inp.placeholder = S.cfg.j[5].max;
      if (inp.dataset.ps === 'gripClose' && v == null) inp.placeholder = S.cfg.j[5].min + 15;
    }
  }
  for (const inp of $$('[data-ps]')) {
    inp.onchange = () => {
      const k = inp.dataset.ps;
      const v = inp.type === 'checkbox' ? inp.checked : inp.value === '' ? null : +inp.value;
      setPath(V.settings, k, v);
      if (k === 'minHits') V.tracker.minHits = Math.max(1, v || 1);
      if (k === 'hfov' && V.cal?.H && V.cal.w) {
        V.cal.cam = cameraFromHomography(V.cal.H, V.cal.w, V.cal.h, hfovPrior());
        saveCal();
      }
      if (k.startsWith('bin.')) {
        world.applySettings({ bin: { ...V.settings.bin } });
        saveWorld();
      }
      store.set('pick', V.settings);
    };
  }
  world.applySettings({ bin: { ...V.settings.bin } });
  fillPick();

  function camCheck() {
    const L = S.links;
    const top = L.base + L.upper + L.fore + L.wrist + L.tool;
    const c = world.settings.cam;
    const el = $('#camWarn');
    const msgs = [];
    if (c.z < top + 150) msgs.push(`⚠ Camera is ${c.z} mm high but the arm reaches ${top} mm when stretched up — mount it at least ${top + 150} mm high.`);
    const mm = (2 * Math.hypot(c.x - c.lookX, c.y - c.lookY, c.z) * Math.tan((c.hfov * Math.PI) / 360)) / c.w;
    // everything that must be fully in frame: the 4 markers (with their size) and the bin
    const cam = world.cam;
    const half = layout().size / 2 + 8;
    const need = [
      ...cornerMarkers(layout()).flatMap((m) => [[m.x - half, m.y - half], [m.x + half, m.y + half], [m.x - half, m.y + half], [m.x + half, m.y - half]]),
      [V.settings.bin.x, V.settings.bin.y],
    ];
    const out = need.filter(([x, y]) => {
      const p = cam.project([x, y, 0]);
      return !(p.depth > 0 && p.px >= 0 && p.py >= 0 && p.px <= cam.w && p.py <= cam.h);
    }).length;
    if (out) msgs.push('⚠ Part of the mat/bin is out of frame — raise the camera, turn it, or aim it at the middle of the mat.');
    msgs.push(`≈ ${(1 / mm).toFixed(2)} px/mm at the table — a 30 mm cap is ~${Math.round(30 / mm)} px wide.`);
    el.textContent = msgs.join(' ');
    el.className = `hint ${c.z < top + 150 || out ? 'bad' : ''}`;
  }
  for (const inp of $$('[data-cam]')) {
    inp.value = world.settings.cam[inp.dataset.cam];
    inp.onchange = () => {
      world.applySettings({ cam: { ...world.settings.cam, [inp.dataset.cam]: +inp.value } });
      saveWorld();
      camCheck();
      if (V.source === 'sim') resetCal(); // moved camera = recalibrate (auto in the simulator)
    };
  }
  camCheck();
  for (const inp of $$('[data-sim]')) {
    inp.value = world.settings[inp.dataset.sim];
    inp.onchange = () => {
      world.applySettings({ [inp.dataset.sim]: Math.max(0, +inp.value || 0) });
      saveWorld();
    };
  }
  $('#simSpawn').onclick = () => world.spawn(3);
  $('#simClear').onclick = () => {
    world.clearObjects();
    V.attempts = [];
  };

  // ------------------------------------------------------------ loops
  setInterval(() => {
    if (V.source !== 'sim') return;
    const m = world.detect();
    // first time in the simulator: calibrate automatically, like you would with the mat
    if (!V.cal?.H && m.markers.length === 4) {
      V.last = m;
      $('#calMarkers').click();
    }
    onVision(m);
  }, 100);
  const visible = () => $('.tab[data-tab="vision"]').classList.contains('on');
  (function loop() {
    if (visible()) draw();
    $('#simScore').textContent = `${world.sorted} in bin · ${world.objects.filter((o) => o.userData.state === 'table').length} on table`;
    setTimeout(() => requestAnimationFrame(loop), 90);
  })();

  // ------------------------------------------------------------ boot
  const fromBridge = location.port === '8765';
  $('#visUrl').value = store.get('url', fromBridge ? `ws://${location.host}/ws` : 'ws://localhost:8765/ws');
  const canCam = !!navigator.mediaDevices?.getUserMedia && window.isSecureContext;
  const src = /[?&]sim\b/.test(location.search) ? 'sim' : store.get('source', canCam ? 'webcam' : 'sim');
  $('#visSource').value = src;
  setSource(src);
  if (/[?&]autopick\b/.test(location.search)) $('#autoPick').checked = true; // ?autopick for demos/tests
  return { world, V };
}
