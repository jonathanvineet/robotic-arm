// sim.js — VirtualArm: a JavaScript copy of the firmware (RoboArm.ino).
// Same commands, same replies, same motion planner, same telemetry timing,
// so the web app cannot tell it apart from the real ESP.

import { Planner } from './motion.js';

export const N = 6;
export const CONTROL_HZ = 50;
export const STATE_HZ = 20;

// keep in sync with firmware/RoboArm/config.h
export const JOINT_DEFAULTS = [
  { name: 'Base', ch: 0, min: 0, max: 180, home: 90, zero: 90, dir: 1, pmin: 500, pmax: 2500, vmax: 120 },
  { name: 'Shoulder', ch: 1, min: 15, max: 165, home: 90, zero: 90, dir: 1, pmin: 500, pmax: 2500, vmax: 60 },
  { name: 'Elbow', ch: 2, min: 0, max: 180, home: 90, zero: 90, dir: 1, pmin: 500, pmax: 2500, vmax: 90 },
  { name: 'Wrist Pitch', ch: 3, min: 0, max: 180, home: 90, zero: 90, dir: 1, pmin: 500, pmax: 2500, vmax: 120 },
  { name: 'Wrist Roll', ch: 4, min: 0, max: 180, home: 90, zero: 90, dir: 1, pmin: 500, pmax: 2500, vmax: 180 },
  { name: 'Gripper', ch: 5, min: 30, max: 120, home: 60, zero: 90, dir: 1, pmin: 500, pmax: 2500, vmax: 180 },
];

export function defaultConfig() {
  return {
    t: 'cfg',
    speed: 90,
    accel: 240,
    sync: true,
    bootRelax: false,
    osc: 25000000,
    j: JOINT_DEFAULTS.map((j) => ({ ...j })),
  };
}

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const r1 = (v) => Math.round(v * 10) / 10;
const isNum = (v) => typeof v === 'number' && Number.isFinite(v);

export class VirtualArm {
  /**
   * @param {object} o
   * @param {(msg:object)=>void} o.emit  receives every message the "device" broadcasts
   * @param {{load:()=>object|null, save:(cfg:object)=>void}} [o.flash]  emulated EEPROM
   */
  constructor({ emit, flash } = {}) {
    this.emit = emit || (() => {});
    this.flash = flash || { load: () => null, save: () => {} };
    this.bootMs = 0;
    this.boot(0);
  }

  boot(nowMs) {
    const saved = this.flash.load();
    this.S = saved ? structuredClone(saved) : defaultConfig();
    this.S.t = 'cfg';
    this.P = new Planner(N);
    this.applyMotion();
    this.P.init(this.S.j.map((j, i) => this.clampJoint(i, j.home)));
    this.estop = false;
    this.relaxed = !!this.S.bootRelax;
    this.pcaOk = true;
    this.bootMs = nowMs;
    this.lastCtl = nowMs;
    this.lastState = -Infinity;
    this.wasMoving = true;
  }

  // ---------------------------------------------------------- messaging
  log(l, m) {
    this.emit({ t: 'log', l, m });
  }

  stateMsg() {
    return {
      t: 's',
      p: this.P.ax.map((a) => r1(a.pos)),
      g: this.P.ax.map((a) => r1(a.tgt)),
      m: this.P.moving() ? 1 : 0,
      e: this.estop ? 1 : 0,
      r: this.relaxed ? 1 : 0,
    };
  }

  cfgMsg() {
    return structuredClone(this.S);
  }

  infoMsg(nowMs = this.lastCtl) {
    return {
      t: 'info',
      fw: '1.0.0-sim',
      board: 'Simulator',
      ip: 'virtual',
      mode: 'SIM',
      ssid: '-',
      rssi: 0,
      pca: this.pcaOk,
      heap: 0,
      up: Math.floor((nowMs - this.bootMs) / 1000),
      n: N,
    };
  }

  sendAll() {
    this.emit(this.infoMsg());
    this.emit(this.cfgMsg());
    this.emit(this.stateMsg());
  }

  // ---------------------------------------------------------- helpers
  applyMotion() {
    this.P.vmax = this.S.speed;
    this.P.amax = this.S.accel;
    this.P.sync = this.S.sync;
    this.S.j.forEach((j, i) => (this.P.jvmax[i] = j.vmax));
  }

  clampJoint(i, a) {
    return clamp(a, this.S.j[i].min, this.S.j[i].max);
  }

  clampTargets() {
    this.P.ax.forEach((a, i) => (a.tgt = this.clampJoint(i, a.tgt)));
  }

  canMove() {
    if (this.estop) {
      this.log('warn', 'E-STOP active - send reset first');
      return false;
    }
    if (this.relaxed) {
      this.log('warn', 'Servos relaxed - press Hold first');
      return false;
    }
    return true;
  }

  /** Pulse width in microseconds the PCA9685 would output for joint i. */
  pulseUs(i) {
    const j = this.S.j[i];
    return j.pmin + (this.P.ax[i].pos / 180) * (j.pmax - j.pmin);
  }

  // ---------------------------------------------------------- commands
  handle(d) {
    const c = d?.c ?? '';
    const S = this.S;
    switch (c) {
      case 'ping':
        this.emit({ t: 'pong', id: d.id ?? null });
        break;
      case 'get':
        this.sendAll();
        break;
      case 'set': {
        if (!this.canMove()) return;
        const arr = Array.isArray(d.j) ? d.j : [];
        const mask = [];
        const t = [];
        for (let i = 0; i < N; i++) {
          mask[i] = i < arr.length && arr[i] != null;
          t[i] = mask[i] ? this.clampJoint(i, Number(arr[i])) : this.P.ax[i].tgt;
        }
        this.P.setTargets(t, mask, d.sync ?? !!S.sync);
        break;
      }
      case 'joint':
      case 'jog': {
        if (!this.canMove()) return;
        const i = d.i ?? -1;
        if (i < 0 || i >= N) return;
        const a = c === 'jog' ? this.P.ax[i].tgt + (d.d ?? 0) : d.a ?? this.P.ax[i].tgt;
        this.P.setTarget(i, this.clampJoint(i, a));
        break;
      }
      case 'home':
        if (!this.canMove()) return;
        this.P.setTargets(
          S.j.map((j, i) => this.clampJoint(i, j.home)),
          new Array(N).fill(true),
          true,
        );
        this.log('info', 'Homing');
        break;
      case 'stop':
        this.P.stop();
        this.clampTargets();
        break;
      case 'estop':
        this.estop = true;
        this.P.halt();
        this.log('error', 'E-STOP');
        break;
      case 'reset':
        this.estop = false;
        this.log('info', 'E-stop cleared');
        break;
      case 'relax':
        this.relaxed = true;
        this.P.halt();
        this.log('warn', 'Servos relaxed (no torque)');
        break;
      case 'hold':
        this.relaxed = false;
        this.log('info', 'Servos holding');
        break;
      case 'speed':
        if (isNum(d.v)) S.speed = clamp(d.v, 1, 720);
        if (isNum(d.a)) S.accel = clamp(d.a, 1, 5000);
        if (typeof d.sync === 'boolean') S.sync = d.sync;
        this.applyMotion();
        this.emit(this.cfgMsg());
        break;
      case 'cfg': {
        if (typeof d.bootRelax === 'boolean') S.bootRelax = d.bootRelax;
        if (isNum(d.osc)) S.osc = clamp(Math.round(d.osc), 20000000, 30000000);
        for (const o of Array.isArray(d.j) ? d.j : []) {
          const i = o.i ?? -1;
          if (i < 0 || i >= N) continue;
          const j = S.j[i];
          if (isNum(o.ch)) j.ch = clamp(Math.round(o.ch), 0, 15);
          if (isNum(o.min)) j.min = clamp(o.min, 0, 180);
          if (isNum(o.max)) j.max = clamp(o.max, 0, 180);
          if (j.max < j.min) [j.min, j.max] = [j.max, j.min];
          if (isNum(o.home)) j.home = o.home;
          j.home = this.clampJoint(i, j.home);
          if (isNum(o.zero)) j.zero = clamp(o.zero, -180, 360);
          if (isNum(o.dir)) j.dir = o.dir < 0 ? -1 : 1;
          if (isNum(o.pmin)) j.pmin = clamp(Math.round(o.pmin), 300, 2900);
          if (isNum(o.pmax)) j.pmax = clamp(Math.round(o.pmax), 300, 2900);
          if (isNum(o.vmax)) j.vmax = clamp(o.vmax, 1, 720);
        }
        this.clampTargets();
        this.applyMotion();
        this.emit(this.cfgMsg());
        break;
      }
      case 'save':
        this.flash.save(structuredClone(S));
        this.log('info', 'Settings saved to flash');
        break;
      case 'factory':
        this.S = defaultConfig();
        this.flash.save(structuredClone(this.S));
        this.applyMotion();
        this.clampTargets();
        this.emit(this.cfgMsg());
        this.log('warn', 'Factory defaults restored');
        break;
      case 'wifi':
        this.log('warn', `Wi-Fi set to '${d.ssid ?? ''}' (ignored in simulator)`);
        break;
      case 'reboot':
        this.log('warn', 'Rebooting...');
        this.boot(this.lastCtl);
        this.sendAll();
        break;
      case 'scan':
        this.log('info', 'I2C devices: 0x40 (simulated PCA9685)');
        break;
      default:
        this.log('warn', `Unknown command: ${c}`);
    }
  }

  /** Called when the last client disconnects (same as firmware). */
  disconnect() {
    if (this.P.moving()) {
      this.P.stop();
      this.clampTargets();
    }
  }

  // ---------------------------------------------------------- loop
  /** Mirror of loop(): fixed-step planner + throttled telemetry. */
  loop(nowMs) {
    const period = 1000 / CONTROL_HZ;
    let steps = 0;
    while (nowMs - this.lastCtl >= period && steps < 5) {
      this.P.update(1 / CONTROL_HZ);
      this.lastCtl += period;
      steps++;
    }
    if (nowMs - this.lastCtl >= period) this.lastCtl = nowMs;

    const moving = this.P.moving();
    const interval = moving || this.wasMoving ? 1000 / STATE_HZ : 1000;
    if (nowMs - this.lastState >= interval) {
      this.lastState = nowMs;
      this.emit(this.stateMsg());
      this.wasMoving = moving;
    }
  }
}
