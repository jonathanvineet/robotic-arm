// vision.js — camera -> detections -> table coordinates -> pick & place.
// Sources: the simulated camera in the twin, or the Python vision bridge
// (Plastic-Segregation-AI/vision_bridge.py: USB camera + YOLO + ArUco).

import { World } from './world.js';
import { fk } from './kinematics.js';
import {
  solveHomography, applyH, invertH, reprojError, cornerMarkers, layoutArea, boxToWorld, Tracker, planPick,
  PICK_DEFAULTS, LAYOUT_DEFAULT, CORNERS, rejectReason,
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
  const canvas = $('#feed');
  const ctx = canvas.getContext('2d');

  // ------------------------------------------------------------ calibration storage
  const calKey = () => `cal.${V.source}`;
  function loadCal() {
    V.cal = store.get(calKey(), null);
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
    V.cal = { pairs, H, Hinv: invertH(H), err, w: V.last?.w, h: V.last?.h, how };
    saveCal();
    sendRoi();
    appLog(`Camera calibrated from ${pairs.length} points (${how}), error ${err.toFixed(1)} mm`, err > 5 ? 'warn' : 'info');
    V.tracker.clear();
    return true;
  }
  function renderCal() {
    const tag = $('#calState');
    if (V.cal?.H) {
      tag.textContent = `${V.cal.how} · ${V.cal.err.toFixed(1)} mm`;
      tag.className = `tag ${V.cal.err > 5 ? 'warn' : 'ok'}`;
    } else {
      tag.textContent = 'not calibrated';
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
  function sendRoi() {
    if (V.source !== 'bridge' || V.ws?.readyState !== WebSocket.OPEN) return;
    if (!V.cal?.Hinv) return V.ws.send(JSON.stringify({ c: 'roi', roi: null }));
    const a = V.settings.area;
    const m = 25;
    const pts = [[a.x0 - m, a.y0 - m], [a.x0 - m, a.y1 + m], [a.x1 + m, a.y0 - m], [a.x1 + m, a.y1 + m]].map((p) => applyH(V.cal.Hinv, p));
    const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
    const W = V.cal.w || V.last?.w || 1e9, Hh = V.cal.h || V.last?.h || 1e9;
    V.ws.send(JSON.stringify({
      c: 'roi',
      roi: [Math.max(0, Math.min(...xs)), Math.max(0, Math.min(...ys)), Math.min(W, Math.max(...xs)), Math.min(Hh, Math.max(...ys))].map(Math.round),
    }));
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
    V.source = src;
    V.last = null;
    V.img = null;
    V.tracker.clear();
    store.set('source', src);
    $('#visUrl').hidden = src !== 'bridge';
    $('#visConnect').hidden = src !== 'bridge';
    world.setSimVisible(src === 'sim');
    loadCal();
    if (src === 'bridge') connectBridge();
  }

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
      if (m.t !== 'vision') return;
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
    $('#feedMsg').hidden = true;
    if (V.cal?.w && (V.cal.w !== m.w || V.cal.h !== m.h)) {
      appLog(`Camera resolution changed (${m.w}x${m.h}) — recalibrate`, 'warn');
      V.cal = null;
      saveCal();
    }
    let objs = [];
    if (V.cal?.H) objs = m.boxes.map((b) => ({ ...boxToWorld(V.cal.H, b), box: b }));
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
      `${m.markers?.length || 0}/4 markers · ${V.tracker.tracks.filter((t) => t.pickable).length} pickable`;
    autoPick(now);
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
  const src = fromBridge ? 'bridge' : store.get('source', 'sim');
  $('#visSource').value = src;
  setSource(src);
  if (/[?&]autopick\b/.test(location.search)) $('#autoPick').checked = true; // ?autopick for demos/tests
  return { world, V };
}
