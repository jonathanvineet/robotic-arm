// app.js — RoboArm Studio: controller UI, digital twin, poses, sequences,
// calibration, tests, keyboard + gamepad input.

import { Twin } from './twin.js';
import { SimTransport, WsTransport, SerialTransport } from './transport.js';
import { fk, ik, DEFAULT_LINKS } from './kinematics.js';
import { defaultConfig } from './sim.js';

// ------------------------------------------------------------------ utils
const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
const r1 = (v) => Math.round(v * 10) / 10;
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
const store = {
  get(k, d) {
    try {
      const v = localStorage.getItem('roboarm.' + k);
      return v == null ? d : JSON.parse(v);
    } catch {
      return d;
    }
  },
  set(k, v) {
    try {
      localStorage.setItem('roboarm.' + k, JSON.stringify(v));
    } catch {}
  },
};

// ------------------------------------------------------------------ state
const cfg0 = defaultConfig();
const S = {
  cfg: cfg0,
  info: null,
  pos: cfg0.j.map((j) => j.home),
  tgt: cfg0.j.map((j) => j.home),
  moving: false,
  estop: false,
  relaxed: false,
  stateN: 0,
  lastStateAt: 0,
  links: { ...DEFAULT_LINKS, ...store.get('links', {}) },
  poses: store.get('poses', []),
  seq: store.get('seq', []),
  step: store.get('step', 5),
  mmStep: store.get('mmStep', 10),
  sel: 0,
  link: null,
  online: false,
  job: null,
  calDirty: false,
  dragging: new Set(),
};

// ------------------------------------------------------------------ log
function log(level, text, src = 'dev') {
  const box = $('#log');
  const d = document.createElement('div');
  const t = new Date().toLocaleTimeString([], { hour12: false });
  d.className = `${level} ${src === 'app' ? 'app' : ''}`;
  d.innerHTML = `<span class="t">${t}</span>${esc(text)}`;
  const stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 8;
  box.appendChild(d);
  while (box.childElementCount > 400) box.firstChild.remove();
  if (stick) box.scrollTop = box.scrollHeight;
}
const appLog = (text, level = 'info') => log(level, text, 'app');

// ------------------------------------------------------------------ twin
const twin = new Twin($('#twin'));
twin.setLinks(S.links);
twin.setPose(S.pos, S.cfg.j);

// ------------------------------------------------------------------ connection
function send(obj) {
  if (!S.link || !S.online) return false;
  S.link.send(obj);
  return true;
}

async function connect() {
  disconnect();
  const mode = $('#mode').value;
  let link;
  if (mode === 'sim') {
    link = new SimTransport({ latency: +$('#simLat').value || 0, jitter: +$('#simJit').value || 0 });
  } else if (mode === 'ws') {
    const host = $('#host').value.trim() || '192.168.4.1';
    store.set('host', host);
    link = new WsTransport(host);
  } else {
    link = new SerialTransport();
  }
  S.link = link;
  link.onmessage = onMessage;
  link.onlog = (t) => log('info', t, 'dev');
  link.onstatus = (st, detail) => {
    if (S.link !== link) return;
    S.online = st === 'online';
    const pill = $('#status');
    pill.className = `pill ${st === 'online' ? 'online' : st === 'connecting' ? 'connecting' : 'off'}`;
    pill.textContent = st === 'online' ? `online · ${link.label}` : st;
    pill.title = detail || '';
    if (detail && st !== 'online') appLog(`${st}: ${detail}`, st === 'offline' ? 'warn' : 'info');
    if (st === 'online') appLog(`Connected to ${link.label}`);
    $('#connectBtn').textContent = st === 'offline' ? 'Connect' : 'Disconnect';
    $('#hudMode').textContent = st === 'online' ? (mode === 'sim' ? 'SIMULATOR' : `LIVE · ${link.label}`) : 'OFFLINE';
    if (st !== 'online') $('#rtt').textContent = '';
  };
  store.set('mode', mode);
  await link.connect();
}

function disconnect() {
  stopJob();
  if (S.link) {
    const l = S.link;
    S.link = null;
    S.online = false;
    l.close();
  }
  $('#status').className = 'pill off';
  $('#status').textContent = 'offline';
  $('#connectBtn').textContent = 'Connect';
  $('#hudMode').textContent = 'OFFLINE';
}

$('#connectBtn').onclick = () => (S.link && S.link.status !== 'offline' ? disconnect() : connect());
$('#mode').onchange = () => {
  $('#host').classList.toggle('hidden', $('#mode').value !== 'ws');
  if (S.link) connect();
};
$('#host').addEventListener('keydown', (e) => e.key === 'Enter' && connect());

setInterval(() => send({ c: 'ping', id: performance.now() }), 2000);

// ------------------------------------------------------------------ incoming
let renderQueued = false;
function queueRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(() => {
    renderQueued = false;
    render();
  });
}

function onMessage(m) {
  switch (m.t) {
    case 's':
      S.pos = m.p;
      S.tgt = m.g;
      S.moving = !!m.m;
      S.estop = !!m.e;
      S.relaxed = !!m.r;
      S.stateN++;
      S.lastStateAt = performance.now();
      queueRender();
      break;
    case 'cfg': {
      const structural = m.j.length !== S.cfg.j.length || m.j.some((j, i) => j.name !== S.cfg.j[i]?.name);
      S.cfg = m;
      if (structural) buildJoints();
      updateRanges();
      fillMotion();
      if (!S.calDirty) fillCal();
      queueRender();
      break;
    }
    case 'info':
      S.info = m;
      renderInfo();
      break;
    case 'log':
      log(m.l || 'info', m.m);
      break;
    case 'pong':
      if (typeof m.id === 'number') $('#rtt').textContent = `${Math.round(performance.now() - m.id)} ms`;
      break;
  }
}

// ------------------------------------------------------------------ joints panel
const STEPS = [0.5, 1, 5, 10, 30];
const MM_STEPS = [1, 5, 10, 25, 50];

function seg(el, values, current, onPick, fmt = (v) => v) {
  el.innerHTML = values.map((v) => `<button data-v="${v}" class="${v === current ? 'on' : ''}">${fmt(v)}</button>`).join('');
  el.onclick = (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    $$('button', el).forEach((x) => x.classList.toggle('on', x === b));
    onPick(+b.dataset.v);
  };
}
seg($('#stepSeg'), STEPS, S.step, (v) => store.set('step', (S.step = v)), (v) => `${v}°`);
seg($('#mmSeg'), MM_STEPS, S.mmStep, (v) => store.set('mmStep', (S.mmStep = v)), (v) => `${v}mm`);

function holdRepeat(btn, fn) {
  let t1, t2;
  const stop = () => {
    clearTimeout(t1);
    clearInterval(t2);
  };
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    fn();
    t1 = setTimeout(() => (t2 = setInterval(fn, 90)), 350);
  });
  ['pointerup', 'pointerleave', 'pointercancel'].forEach((ev) => btn.addEventListener(ev, stop));
}

const pending = new Map();
setInterval(() => {
  for (const [i, a] of pending) send({ c: 'joint', i, a: r1(a) });
  pending.clear();
}, 40);

function buildJoints() {
  const box = $('#joints');
  box.innerHTML = '';
  S.cfg.j.forEach((j, i) => {
    const el = document.createElement('div');
    el.className = 'joint';
    el.dataset.i = i;
    el.innerHTML = `
      <div class="jh"><span class="n">${i + 1}. ${esc(j.name)}</span><span class="ch">CH${j.ch}</span><span class="act" title="actual"></span></div>
      <div class="jc">
        <button data-d="-1" aria-label="jog minus">−</button>
        <input type="range" step="0.5" aria-label="${esc(j.name)} target">
        <button data-d="1" aria-label="jog plus">+</button>
        <input type="number" step="0.5" aria-label="${esc(j.name)} angle">
      </div>
      <div class="jbar" title="actual position within limits"><i></i></div>`;
    const range = $('input[type=range]', el);
    const num = $('input[type=number]', el);
    range.addEventListener('pointerdown', () => S.dragging.add(i));
    range.addEventListener('pointerup', () => S.dragging.delete(i));
    range.addEventListener('input', () => {
      S.sel = i;
      num.value = range.value;
      pending.set(i, +range.value);
    });
    range.addEventListener('change', () => {
      S.dragging.delete(i);
      pending.set(i, +range.value);
    });
    num.addEventListener('change', () => send({ c: 'joint', i, a: +num.value }));
    $$('button[data-d]', el).forEach((b) => holdRepeat(b, () => {
      S.sel = i;
      send({ c: 'jog', i, d: +b.dataset.d * S.step });
    }));
    el.addEventListener('click', () => {
      S.sel = i;
      queueRender();
    });
    box.appendChild(el);
  });
  buildCal();
  buildPulses();
}

function updateRanges() {
  $$('#joints .joint').forEach((el, i) => {
    const j = S.cfg.j[i];
    for (const inp of $$('input', el)) {
      inp.min = j.min;
      inp.max = j.max;
    }
    $('.ch', el).textContent = `CH${j.ch}`;
  });
}

function render() {
  const J = S.cfg.j;
  $$('#joints .joint').forEach((el, i) => {
    const j = J[i];
    el.classList.toggle('sel', i === S.sel);
    $('.act', el).textContent = `${S.pos[i].toFixed(1)}°`;
    const range = $('input[type=range]', el);
    const num = $('input[type=number]', el);
    if (!S.dragging.has(i) && !pending.has(i)) range.value = S.tgt[i];
    if (document.activeElement !== num && !S.dragging.has(i)) num.value = r1(S.tgt[i]);
    const f = (S.pos[i] - j.min) / Math.max(1e-6, j.max - j.min);
    $('.jbar i', el).style.left = `${clamp(f, 0, 1) * 100}%`;
  });

  twin.setPose(S.pos, J);
  twin.setGhost(S.tgt);

  const a = fk(S.pos, J, S.links);
  const g = fk(S.tgt, J, S.links);
  const p = a.tcp;
  $('#tcp').textContent =
    `TCP   X ${p.x.toFixed(0).padStart(5)}  Y ${p.y.toFixed(0).padStart(5)}  Z ${p.z.toFixed(0).padStart(5)} mm\n` +
    `      pitch ${a.pitch.toFixed(0).padStart(4)}°  roll ${a.roll.toFixed(0).padStart(4)}°  grip ${(a.grip * 100).toFixed(0).padStart(3)}%\n` +
    `GOAL  X ${g.tcp.x.toFixed(0).padStart(5)}  Y ${g.tcp.y.toFixed(0).padStart(5)}  Z ${g.tcp.z.toFixed(0).padStart(5)} mm`;

  const fm = $('#flagMove');
  fm.textContent = S.moving ? 'MOVING' : 'IDLE';
  fm.className = `flag ${S.moving ? 'act' : ''}`;
  const fr = $('#flagRelax');
  fr.textContent = S.relaxed ? 'RELAXED' : 'TORQUE';
  fr.className = `flag ${S.relaxed ? 'warn' : 'ok'}`;
  $('#relaxBtn').textContent = S.relaxed ? 'Hold' : 'Relax';
  $('#estopBtn').classList.toggle('latched', S.estop);
  $('#estopBanner').classList.toggle('hidden', !S.estop);
  renderPulses();
}

// ------------------------------------------------------------------ actions
$('#homeBtn').onclick = () => send({ c: 'home' });
$('#stopBtn').onclick = () => {
  stopJob();
  send({ c: 'stop' });
};
$('#relaxBtn').onclick = () => send({ c: S.relaxed ? 'hold' : 'relax' });
$('#devHold').onclick = () => send({ c: 'hold' });
function eStop() {
  stopJob();
  if (!send({ c: 'estop' })) appLog('E-stop pressed but not connected', 'warn');
}
$('#estopBtn').onclick = eStop;
$('#estopReset').onclick = () => send({ c: 'reset' });

// motion settings
function fillMotion() {
  const c = S.cfg;
  if (document.activeElement !== $('#speed')) $('#speed').value = c.speed;
  if (document.activeElement !== $('#accel')) $('#accel').value = c.accel;
  $('#speedVal').textContent = Math.round(c.speed);
  $('#accelVal').textContent = Math.round(c.accel);
  $('#sync').checked = !!c.sync;
}
$('#speed').oninput = () => ($('#speedVal').textContent = $('#speed').value);
$('#accel').oninput = () => ($('#accelVal').textContent = $('#accel').value);
$('#speed').onchange = () => send({ c: 'speed', v: +$('#speed').value });
$('#accel').onchange = () => send({ c: 'speed', a: +$('#accel').value });
$('#sync').onchange = () => send({ c: 'speed', sync: $('#sync').checked });
$$('[data-speed]').forEach((b) => (b.onclick = () => {
  const [v, a] = b.dataset.speed.split(',').map(Number);
  send({ c: 'speed', v, a });
}));

// twin options
$('#optGhost').onchange = (e) => twin.setOptions({ ghost: e.target.checked });
$('#optTrail').onchange = (e) => twin.setOptions({ trail: e.target.checked });
$('#optEnv').onchange = (e) => twin.setOptions({ envelope: e.target.checked });
$('#clearTrail').onclick = () => twin.clearTrail();
$('#viewSeg').onclick = (e) => {
  const b = e.target.closest('button');
  if (b) twin.view(b.dataset.view);
};

// tabs
$('#tabs').onclick = (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  $$('#tabs button').forEach((x) => x.classList.toggle('on', x === b));
  $$('.tab').forEach((t) => t.classList.toggle('on', t.dataset.tab === b.dataset.tab));
  store.set('tab', b.dataset.tab);
};
{
  const t = store.get('tab', 'xyz');
  $(`#tabs button[data-tab="${t}"]`)?.click();
}

// ------------------------------------------------------------------ XYZ / IK
function moveTo(servos, sync = true) {
  return send({ c: 'set', j: servos.map(r1), sync });
}

function ikMsg(text, cls) {
  const el = $('#ikMsg');
  el.textContent = text;
  el.className = `ikmsg ${cls || ''}`;
}

function goXYZ(t) {
  const res = ik(t, S.cfg.j, S.links, S.tgt);
  twin.showMarker(t);
  if (!res.ok) {
    ikMsg(`✕ ${res.reason}`, 'bad');
    return false;
  }
  ikMsg(`✓ elbow ${res.elbow} → [${res.servos.map((v) => v.toFixed(0)).join(', ')}]`, 'ok');
  moveTo(res.servos);
  return true;
}

function fillXYZ() {
  const g = fk(S.tgt, S.cfg.j, S.links);
  $('#ikX').value = g.tcp.x.toFixed(0);
  $('#ikY').value = g.tcp.y.toFixed(0);
  $('#ikZ').value = g.tcp.z.toFixed(0);
  $('#ikP').value = g.pitch.toFixed(0);
}
$('#ikUse').onclick = fillXYZ;
$('#ikGo').onclick = () => goXYZ({ x: +$('#ikX').value, y: +$('#ikY').value, z: +$('#ikZ').value, pitch: +$('#ikP').value });

function cartJog(axis, dir) {
  const g = fk(S.tgt, S.cfg.j, S.links);
  const t = { ...g.tcp, pitch: g.pitch };
  if (axis === 'p') t.pitch += dir * 5;
  else t[axis] += dir * S.mmStep;
  if (goXYZ(t)) {
    $('#ikX').value = t.x.toFixed(0);
    $('#ikY').value = t.y.toFixed(0);
    $('#ikZ').value = t.z.toFixed(0);
    $('#ikP').value = t.pitch.toFixed(0);
  }
}
$$('[data-cj]').forEach((b) => {
  const [axis, dir] = b.dataset.cj.split(',');
  holdRepeat(b, () => cartJog(axis, +dir));
});

// ------------------------------------------------------------------ poses
function savePoses() {
  store.set('poses', S.poses);
  renderPoses();
}

function renderPoses() {
  const ul = $('#poseList');
  const all = [{ name: 'Home', j: S.cfg.j.map((j) => j.home), builtin: true }, ...S.poses];
  ul.innerHTML = all
    .map((p, k) => `<li data-k="${k - 1}">
      <span class="nm">${esc(p.name)}</span><span class="ang">${p.j.map((v) => Math.round(v)).join(' ')}</span>
      <button class="btn sm" data-a="go">Go</button>
      ${p.builtin ? '' : '<button class="btn sm" data-a="upd" title="Overwrite with current">⟳</button><button class="btn sm danger" data-a="del" title="Delete">✕</button>'}
    </li>`)
    .join('');
  ul.onclick = (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    const k = +b.closest('li').dataset.k;
    if (b.dataset.a === 'go') k < 0 ? send({ c: 'home' }) : moveTo(S.poses[k].j);
    if (b.dataset.a === 'upd') {
      S.poses[k].j = S.tgt.map(r1);
      savePoses();
    }
    if (b.dataset.a === 'del') {
      S.poses.splice(k, 1);
      savePoses();
    }
  };
}
$('#poseSave').onclick = () => {
  const name = $('#poseName').value.trim() || `Pose ${S.poses.length + 1}`;
  S.poses.push({ name, j: S.tgt.map(r1) });
  $('#poseName').value = '';
  savePoses();
};
renderPoses();

// ------------------------------------------------------------------ jobs (sequence / tests)
function startJob(name) {
  stopJob();
  if (!S.online) {
    appLog('Connect first (Simulator works without hardware)', 'warn');
    return null;
  }
  if (S.estop) {
    appLog('Reset the E-stop first', 'warn');
    return null;
  }
  S.job = { name, aborted: false };
  return S.job;
}

function stopJob() {
  if (S.job) {
    S.job.aborted = true;
    appLog(`${S.job.name} stopped`, 'warn');
    S.job = null;
  }
}

function check(job) {
  if (job.aborted) throw new Error('aborted');
  if (S.estop) throw new Error('E-stop');
  if (!S.online) throw new Error('disconnected');
}

async function jobSleep(job, ms) {
  const t0 = performance.now();
  while (performance.now() - t0 < ms) {
    check(job);
    await sleep(Math.min(25, ms));
  }
}

/** Wait for a fresh state report that says the arm has stopped. */
async function waitIdle(job, timeout = 30000) {
  const sentAt = performance.now();
  for (;;) {
    check(job);
    if (S.lastStateAt > sentAt + 30 && !S.moving) return;
    if (performance.now() - sentAt > timeout) throw new Error('timeout waiting for the arm');
    await sleep(20);
  }
}

async function runMoves(job, moves, onStep) {
  for (let k = 0; k < moves.length; k++) {
    check(job);
    onStep?.(k);
    const m = moves[k];
    if (m.home) send({ c: 'home' });
    else if (m.joint != null) send({ c: 'joint', i: m.joint, a: m.a });
    else moveTo(m.j, true);
    await waitIdle(job);
    if (m.dwell) await jobSleep(job, m.dwell);
  }
}

// ------------------------------------------------------------------ sequence
function saveSeq() {
  store.set('seq', S.seq);
  renderSeq();
}

function renderSeq(active = -1) {
  const ol = $('#seqList');
  if (!S.seq.length) {
    ol.innerHTML = '<div class="empty">No steps yet. Move the arm, then “Add current pose”.</div>';
    return;
  }
  ol.innerHTML = S.seq
    .map((s, k) => `<li data-k="${k}" class="${k === active ? 'playing' : ''}">
      <span class="nm ang">${s.j.map((v) => Math.round(v)).join(' ')}</span>
      <input type="number" value="${s.dwell}" min="0" step="100" title="pause after (ms)" data-a="dwell">
      <button class="btn sm" data-a="go" title="Go">▶</button>
      <button class="btn sm" data-a="up" title="Move up">↑</button>
      <button class="btn sm danger" data-a="del" title="Delete">✕</button>
    </li>`)
    .join('');
}
$('#seqList').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  const k = +b.closest('li').dataset.k;
  if (b.dataset.a === 'go') moveTo(S.seq[k].j);
  if (b.dataset.a === 'up' && k > 0) [S.seq[k - 1], S.seq[k]] = [S.seq[k], S.seq[k - 1]];
  if (b.dataset.a === 'del') S.seq.splice(k, 1);
  saveSeq();
});
$('#seqList').addEventListener('change', (e) => {
  if (e.target.dataset.a !== 'dwell') return;
  S.seq[+e.target.closest('li').dataset.k].dwell = Math.max(0, +e.target.value || 0);
  store.set('seq', S.seq);
});
$('#seqAdd').onclick = () => {
  S.seq.push({ j: S.tgt.map(r1), dwell: Math.max(0, +$('#seqDwell').value || 0) });
  saveSeq();
};
$('#seqClear').onclick = () => {
  if (S.seq.length && confirm(`Delete all ${S.seq.length} steps?`)) {
    S.seq = [];
    saveSeq();
  }
};
$('#seqStop').onclick = () => {
  stopJob();
  send({ c: 'stop' });
};
$('#seqPlay').onclick = async () => {
  if (!S.seq.length) return appLog('Sequence is empty', 'warn');
  const job = startJob('Sequence');
  if (!job) return;
  let lap = 0;
  try {
    do {
      lap++;
      await runMoves(job, S.seq, (k) => {
        renderSeq(k);
        $('#seqStatus').textContent = `step ${k + 1}/${S.seq.length}${$('#seqLoop').checked ? ` · lap ${lap}` : ''}`;
      });
    } while ($('#seqLoop').checked);
    $('#seqStatus').textContent = 'done';
    appLog('Sequence finished');
    if (S.job === job) S.job = null;
  } catch (e) {
    $('#seqStatus').textContent = e.message;
  }
  renderSeq();
};
$('#seqExport').onclick = () => {
  const data = { app: 'roboarm', version: 1, sequence: S.seq, poses: S.poses, links: S.links };
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  a.download = `roboarm-${new Date().toISOString().slice(0, 10)}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};
$('#seqImport').onclick = () => $('#seqFile').click();
$('#seqFile').onchange = async (e) => {
  const f = e.target.files[0];
  if (!f) return;
  try {
    const d = JSON.parse(await f.text());
    const ok = (p) => Array.isArray(p?.j) && p.j.length === S.cfg.j.length && p.j.every(Number.isFinite);
    if (Array.isArray(d.sequence)) S.seq = d.sequence.filter(ok).map((s) => ({ j: s.j, dwell: +s.dwell || 0 }));
    if (Array.isArray(d.poses)) S.poses = d.poses.filter(ok).map((p) => ({ name: String(p.name || 'Pose'), j: p.j }));
    if (d.links) applyLinks({ ...S.links, ...d.links });
    saveSeq();
    savePoses();
    appLog(`Imported ${S.seq.length} steps, ${S.poses.length} poses`);
  } catch (err) {
    appLog(`Import failed: ${err.message}`, 'error');
  }
  e.target.value = '';
};
renderSeq();

// ------------------------------------------------------------------ calibration
const CAL_FIELDS = [
  ['ch', 'Ch', 1],
  ['min', 'Min', 0.5],
  ['max', 'Max', 0.5],
  ['home', 'Home', 0.5],
  ['zero', 'Zero', 0.5],
  ['pmin', 'µs@0°', 10],
  ['pmax', 'µs@180°', 10],
  ['vmax', 'Max °/s', 5],
];

function buildCal() {
  const t = $('#calTable');
  t.innerHTML =
    `<tr><th>Joint</th>${CAL_FIELDS.map((f) => `<th>${f[1]}</th>`).join('')}<th title="Invert direction">Inv</th><th></th></tr>` +
    S.cfg.j
      .map((j, i) => `<tr data-i="${i}"><td class="jn">${esc(j.name)}</td>
        ${CAL_FIELDS.map(([k, , st]) => `<td><input type="number" data-k="${k}" step="${st}"></td>`).join('')}
        <td><input type="checkbox" data-k="dir"></td>
        <td class="row gap"><button class="btn mini" data-set="home" title="Home = current target">H←</button><button class="btn mini" data-set="zero" title="Zero = current target">Z←</button></td></tr>`)
      .join('');
  fillCal();
}

function fillCal() {
  $$('#calTable tr[data-i]').forEach((tr) => {
    const j = S.cfg.j[+tr.dataset.i];
    for (const inp of $$('input', tr)) {
      if (inp.dataset.k === 'dir') inp.checked = j.dir < 0;
      else inp.value = j[inp.dataset.k];
    }
  });
  $('#calOsc').value = S.cfg.osc;
  $('#calBootRelax').checked = !!S.cfg.bootRelax;
  S.calDirty = false;
}

$('#calTable').addEventListener('input', () => (S.calDirty = true));
$('#calTable').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-set]');
  if (!b) return;
  const tr = b.closest('tr');
  $(`input[data-k="${b.dataset.set}"]`, tr).value = r1(S.tgt[+tr.dataset.i]);
  S.calDirty = true;
});
$('#calApply').onclick = () => {
  const j = $$('#calTable tr[data-i]').map((tr) => {
    const o = { i: +tr.dataset.i };
    for (const inp of $$('input', tr)) {
      if (inp.dataset.k === 'dir') o.dir = inp.checked ? -1 : 1;
      else if (inp.value !== '') o[inp.dataset.k] = +inp.value;
    }
    return o;
  });
  const msg = { c: 'cfg', j, bootRelax: $('#calBootRelax').checked };
  if ($('#calOsc').value) msg.osc = +$('#calOsc').value;
  if (send(msg)) {
    S.calDirty = false;
    appLog('Calibration applied (not yet saved to flash)');
  }
};
$('#calSave').onclick = () => send({ c: 'save' });
$('#calRevert').onclick = () => {
  S.calDirty = false;
  send({ c: 'get' }) || fillCal();
};
$('#calFactory').onclick = () => confirm('Restore factory calibration and speeds on the arm?') && send({ c: 'factory' });

// geometry
const LINK_LABELS = { base: 'Base', upper: 'Upper arm', fore: 'Forearm', wrist: 'Wrist', tool: 'Gripper' };
function applyLinks(L) {
  for (const k of Object.keys(DEFAULT_LINKS)) L[k] = clamp(+L[k] || DEFAULT_LINKS[k], k === 'base' ? 40 : 10, 1000);
  S.links = L;
  store.set('links', L);
  twin.setLinks(L);
  buildLinks();
  queueRender();
}
function buildLinks() {
  $('#links').innerHTML = Object.entries(LINK_LABELS)
    .map(([k, l]) => `<label>${l}<input type="number" data-l="${k}" value="${S.links[k]}" step="1"></label>`)
    .join('');
}
$('#links').addEventListener('change', (e) => {
  const k = e.target.dataset.l;
  if (k) applyLinks({ ...S.links, [k]: +e.target.value });
});
buildLinks();

// ------------------------------------------------------------------ tests
const bar = (f) => ($('#testBar').style.width = `${Math.round(f * 100)}%`);
const testStatus = (t) => ($('#testStatus').textContent = t);

async function runTest(name, buildMoves) {
  const job = startJob(name);
  if (!job) return;
  const moves = buildMoves();
  appLog(`${name}: ${moves.length} moves`);
  try {
    await runMoves(job, moves, (k) => {
      bar(k / moves.length);
      testStatus(`${name} · ${moves[k].label || `move ${k + 1}`} (${k + 1}/${moves.length})`);
    });
    bar(1);
    testStatus(`${name} ✓ passed`);
    appLog(`${name} passed`);
    if (S.job === job) S.job = null;
  } catch (e) {
    testStatus(`${name} ✕ ${e.message}`);
    appLog(`${name}: ${e.message}`, 'warn');
  }
}

$('#testSweep').onclick = () =>
  runTest('Self-test sweep', () => {
    const moves = [{ home: true, label: 'home' }];
    S.cfg.j.forEach((j, i) => {
      moves.push({ joint: i, a: j.min, label: `${j.name} → min ${j.min}°` });
      moves.push({ joint: i, a: j.max, label: `${j.name} → max ${j.max}°` });
      moves.push({ joint: i, a: j.home, label: `${j.name} → home`, dwell: 150 });
    });
    return moves;
  });

$('#testRandom').onclick = () =>
  runTest('Random poses', () => {
    const J = S.cfg.j;
    const moves = Array.from({ length: 10 }, (_, k) => ({
      j: J.map((j) => {
        // stay in the middle 60% of each range to be gentle on the hardware
        const lo = j.min + (j.max - j.min) * 0.2;
        return r1(lo + Math.random() * (j.max - j.min) * 0.6);
      }),
      label: `random ${k + 1}`,
      dwell: 200,
    }));
    moves.push({ home: true, label: 'home' });
    return moves;
  });

$('#testLimits').onclick = async () => {
  const job = startJob('Safety check');
  if (!job) return;
  const results = [];
  const expect = (ok, what) => {
    results.push(ok);
    appLog(`${ok ? '✓' : '✕'} ${what}`, ok ? 'info' : 'error');
  };
  try {
    bar(0);
    testStatus('Safety check · soft limits');
    send({ c: 'home' });
    await waitIdle(job);
    const i = 2;
    const j = S.cfg.j[i];
    send({ c: 'joint', i, a: j.max + 90 });
    await jobSleep(job, 200);
    expect(Math.abs(S.tgt[i] - j.max) < 0.2, `${j.name} target clamped to max (${S.tgt[i]} ≤ ${j.max})`);
    send({ c: 'joint', i, a: j.min - 90 });
    await jobSleep(job, 200);
    expect(Math.abs(S.tgt[i] - j.min) < 0.2, `${j.name} target clamped to min (${S.tgt[i]} ≥ ${j.min})`);
    bar(0.4);
    testStatus('Safety check · E-stop while moving');
    await waitIdle(job);
    send({ c: 'joint', i, a: j.max });
    await jobSleep(job, 250);
    send({ c: 'estop' });
    const t0 = performance.now();
    while (!(S.estop && !S.moving) && performance.now() - t0 < 1500) await sleep(10);
    expect(S.estop && !S.moving, `E-stop froze motion in ${Math.round(performance.now() - t0)} ms`);
    const frozen = S.pos[i];
    send({ c: 'joint', i, a: j.home });
    await sleep(250);
    expect(Math.abs(S.tgt[i] - frozen) < 0.2, 'moves are rejected while E-stop is latched');
    send({ c: 'reset' });
    await sleep(250);
    expect(!S.estop, 'E-stop reset clears the latch');
    bar(0.8);
    testStatus('Safety check · stop');
    send({ c: 'joint', i, a: j.min });
    await jobSleep(job, 250);
    send({ c: 'stop' });
    await waitIdle(job);
    expect(Math.abs(S.pos[i] - j.min) > 1, 'Stop decelerates before reaching the goal');
    send({ c: 'home' });
    await waitIdle(job);
    bar(1);
    const pass = results.every(Boolean);
    testStatus(`Safety check ${pass ? '✓ all passed' : `✕ ${results.filter((x) => !x).length} failed`}`);
    if (S.job === job) S.job = null;
  } catch (e) {
    testStatus(`Safety check ✕ ${e.message}`);
    if (S.estop) send({ c: 'reset' });
  }
};
$('#testAbort').onclick = () => {
  stopJob();
  send({ c: 'stop' });
  testStatus('aborted');
};

for (const id of ['simLat', 'simJit']) {
  $('#' + id).value = store.get(id, 0);
  $('#' + id).onchange = (e) => store.set(id, +e.target.value || 0);
}

function buildPulses() {
  $('#pulses').innerHTML = S.cfg.j
    .map((j, i) => `<span>CH${j.ch} ${esc(j.name)}</span><div class="bar"><i data-p="${i}"></i></div><span data-u="${i}"></span>`)
    .join('');
}
function renderPulses() {
  if (!$('.tab[data-tab="test"]').classList.contains('on')) return;
  S.cfg.j.forEach((j, i) => {
    const us = S.relaxed ? 0 : j.pmin + (S.pos[i] / 180) * (j.pmax - j.pmin);
    $(`[data-p="${i}"]`).style.width = `${clamp((us - 400) / 2200, 0, 1) * 100}%`;
    $(`[data-u="${i}"]`).textContent = S.relaxed ? 'off' : `${Math.round(us)} µs`;
  });
}

// ------------------------------------------------------------------ device
function renderInfo() {
  const i = S.info;
  const rows = [
    ['Board', i.board],
    ['Firmware', i.fw],
    ['Network', `${i.mode} · ${i.ssid}${i.rssi ? ` (${i.rssi} dBm)` : ''}`],
    ['Address', i.ip],
    ['PCA9685', i.pca ? 'found' : 'NOT FOUND — check wiring'],
    ['Free heap', i.heap ? `${(i.heap / 1024).toFixed(1)} KB` : '—'],
    ['Uptime', `${i.up}s`],
  ];
  $('#devInfo').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join('');
  const fp = $('#flagPca');
  fp.className = `flag ${i.pca ? 'ok' : 'bad'}`;
  fp.textContent = i.pca ? 'PCA9685' : 'NO PCA9685';
  if (!i.pca) appLog('PCA9685 not detected — check SDA/SCL, VCC 3.3V and common GND', 'error');
}
$('#devGet').onclick = () => send({ c: 'get' });
$('#devScan').onclick = () => send({ c: 'scan' });
$('#devReboot').onclick = () => confirm('Reboot the controller?') && send({ c: 'reboot' });
$('#wifiSave').onclick = () => {
  const ssid = $('#wifiSsid').value.trim();
  if (!ssid && !confirm('Empty SSID = always use the RoboArm hotspot. Continue?')) return;
  send({ c: 'wifi', ssid, pass: $('#wifiPass').value });
};
$('#rawSend').onclick = () => {
  try {
    send(JSON.parse($('#rawCmd').value));
  } catch (e) {
    appLog(`Bad JSON: ${e.message}`, 'error');
  }
};
$('#rawCmd').addEventListener('keydown', (e) => e.key === 'Enter' && $('#rawSend').click());
$('#logClear').onclick = () => ($('#log').innerHTML = '');

// ------------------------------------------------------------------ keyboard
window.addEventListener('keydown', (e) => {
  const tag = e.target.tagName;
  const typing = (tag === 'INPUT' && e.target.type !== 'range' && e.target.type !== 'checkbox') || tag === 'TEXTAREA' || tag === 'SELECT';
  if (typing || e.metaKey || e.ctrlKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (e.key === ' ') {
    e.preventDefault();
    eStop();
  } else if (e.key === 'Escape') {
    stopJob();
    send({ c: 'stop' });
  } else if (k === 'h') send({ c: 'home' });
  else if (k >= '1' && k <= '6') {
    S.sel = +k - 1;
    queueRender();
  } else if ((e.key === 'ArrowLeft' || e.key === 'ArrowRight') && e.target.type !== 'range') {
    e.preventDefault();
    send({ c: 'jog', i: S.sel, d: (e.key === 'ArrowRight' ? 1 : -1) * S.step * (e.shiftKey ? 5 : 1) });
  } else if ('wsadrf'.includes(k) && k.length === 1) {
    const map = { w: ['x', 1], s: ['x', -1], a: ['y', 1], d: ['y', -1], r: ['z', 1], f: ['z', -1] };
    cartJog(...map[k]);
  }
});

// ------------------------------------------------------------------ gamepad
// Left stick: base / shoulder · Right stick: wrist roll / elbow
// D-pad ↑↓: wrist pitch · LT/RT: gripper · A: home · B: E-STOP · X: stop
const pad = { tgt: null, lastSend: 0, last: 0, buttons: [], idleSince: 0 };
const RATES = [90, 60, 90, 90, 120, 120]; // deg/s at full stick
function pollPad(now) {
  const gp = [...(navigator.getGamepads?.() || [])].find(Boolean);
  if (gp) {
    const dt = pad.last ? Math.min(0.1, (now - pad.last) / 1000) : 0;
    const dz = (v) => (Math.abs(v) < 0.15 ? 0 : (v - Math.sign(v) * 0.15) / 0.85);
    const btn = (i) => gp.buttons[i]?.value ?? 0;
    const pressed = (i) => (gp.buttons[i]?.pressed && !pad.buttons[i]);
    if (pressed(1)) eStop();
    if (pressed(0)) send({ c: 'home' });
    if (pressed(2)) send({ c: 'stop' });
    pad.buttons = gp.buttons.map((b) => b.pressed);
    const rate = [
      -dz(gp.axes[0] || 0),
      -dz(gp.axes[1] || 0),
      -dz(gp.axes[3] || 0),
      (btn(13) - btn(12)),
      dz(gp.axes[2] || 0),
      btn(7) - btn(6),
    ];
    const active = rate.some((r) => Math.abs(r) > 0.01);
    if (active && !S.estop && !S.relaxed) {
      if (!pad.tgt) pad.tgt = [...S.tgt];
      pad.idleSince = 0;
      pad.tgt = pad.tgt.map((v, i) => clamp(v + rate[i] * RATES[i] * dt, S.cfg.j[i].min, S.cfg.j[i].max));
      if (now - pad.lastSend > 50) {
        pad.lastSend = now;
        send({ c: 'set', j: pad.tgt.map((v, i) => (Math.abs(rate[i]) > 0.01 ? r1(v) : null)), sync: false });
      }
    } else if (pad.tgt) {
      if (!pad.idleSince) pad.idleSince = now;
      if (now - pad.idleSince > 300) pad.tgt = null;
    }
    $('#padStatus').textContent = `Gamepad: ${gp.id.slice(0, 40)} — sticks move joints, A home, B E-STOP, X stop`;
  }
  pad.last = now;
  requestAnimationFrame(pollPad);
}
requestAnimationFrame(pollPad);

// ------------------------------------------------------------------ boot
buildJoints();
updateRanges();
fillMotion();
render();

{
  const servedByArm = location.protocol === 'http:' && !['localhost', '127.0.0.1', ''].includes(location.hostname) && !location.port;
  $('#host').value = servedByArm ? location.hostname : store.get('host', '192.168.4.1');
  const mode = servedByArm ? 'ws' : store.get('mode', 'sim');
  $('#mode').value = mode === 'serial' ? 'sim' : mode; // serial needs a click (browser rule)
  if (!SerialTransport.supported()) $('#mode option[value=serial]').disabled = true;
  $('#host').classList.toggle('hidden', $('#mode').value !== 'ws');
  connect();
}
