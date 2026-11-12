// motion.js — line-by-line port of firmware/RoboArm/motion.h.
// web/tests/crosscheck.test.mjs proves the two produce identical trajectories.

export const EPS_POS = 0.01;
export const MIN_SCALE = 0.02;

const sign = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

export function stepAxis(a, dt, vmax, amax) {
  const s = a.scale < MIN_SCALE ? MIN_SCALE : a.scale;
  const v = vmax * s;
  const acc = amax * s;
  const err = a.tgt - a.pos;
  if (Math.abs(err) <= EPS_POS && Math.abs(a.vel) <= amax * dt) {
    a.pos = a.tgt;
    a.vel = 0;
    return;
  }
  const vdes = sign(err) * Math.min(v, Math.sqrt(2 * acc * Math.abs(err)));
  const dv = vdes - a.vel;
  const braking = a.vel !== 0 && sign(dv) !== sign(a.vel);
  const lim = (braking ? amax : acc) * dt;
  a.vel += clamp(dv, -lim, lim);
  const step = a.vel * dt;
  if (sign(step) === sign(err) && Math.abs(step) >= Math.abs(err)) {
    a.pos = a.tgt;
    a.vel = 0;
  } else {
    a.pos += step;
  }
}

export class Planner {
  constructor(n) {
    this.n = n;
    this.ax = Array.from({ length: n }, () => ({ pos: 90, vel: 0, tgt: 90, scale: 1 }));
    this.vmax = 90;
    this.amax = 240;
    this.jvmax = new Array(n).fill(360);
    this.sync = true;
  }

  init(p) {
    this.ax.forEach((a, i) => {
      a.pos = a.tgt = p[i];
      a.vel = 0;
      a.scale = 1;
    });
  }

  capOf(i) {
    return Math.min(this.vmax, this.jvmax[i]);
  }

  setTargets(t, mask, doSync) {
    let tmax = 0;
    const ti = new Array(this.n).fill(0);
    for (let i = 0; i < this.n; i++) {
      if (!mask[i]) continue;
      ti[i] = Math.abs(t[i] - this.ax[i].pos) / Math.max(this.capOf(i), 0.001);
      tmax = Math.max(tmax, ti[i]);
    }
    for (let i = 0; i < this.n; i++) {
      if (!mask[i]) continue;
      this.ax[i].tgt = t[i];
      this.ax[i].scale = doSync && tmax > 0 ? clamp(ti[i] / tmax, MIN_SCALE, 1) : 1;
    }
  }

  setTarget(i, t) {
    this.ax[i].tgt = t;
    this.ax[i].scale = 1;
  }

  stop() {
    for (const a of this.ax) {
      const d = (a.vel * Math.abs(a.vel)) / (2 * Math.max(this.amax, 1));
      a.tgt = a.pos + d;
      a.scale = 1;
    }
  }

  halt() {
    for (const a of this.ax) {
      a.tgt = a.pos;
      a.vel = 0;
    }
  }

  update(dt) {
    for (let i = 0; i < this.n; i++) stepAxis(this.ax[i], dt, this.capOf(i), this.amax);
  }

  moving() {
    return this.ax.some((a) => a.vel !== 0 || Math.abs(a.tgt - a.pos) > EPS_POS);
  }
}
