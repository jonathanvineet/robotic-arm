// motion.h — joint-space motion planner (pure C++, no Arduino dependencies).
//
// The browser simulator (web/js/motion.js) is a line-by-line port of this file,
// and web/tests/crosscheck.test.mjs compiles this header natively and checks
// that both produce identical trajectories. Change one, change the other.
//
// Each axis tracks its target with a velocity + acceleration limit and
// brakes early so it lands on the target without overshoot. Targets can be
// changed at any time (slider streaming, gamepad), which a fixed trapezoid
// can't handle. In sync mode every joint of a multi-joint move is scaled so
// that all joints arrive together.
#pragma once
#include <math.h>

#ifndef MOTION_REAL
#define MOTION_REAL float
#endif
typedef MOTION_REAL mreal;

namespace motion {

static const mreal EPS_POS = (mreal)0.01;   // degrees
static const mreal MIN_SCALE = (mreal)0.02;

inline mreal mabs(mreal v) { return v < 0 ? -v : v; }
inline mreal msign(mreal v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }
inline mreal mmin(mreal a, mreal b) { return a < b ? a : b; }
inline mreal mmax(mreal a, mreal b) { return a > b ? a : b; }
inline mreal mclamp(mreal v, mreal lo, mreal hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Axis {
  mreal pos = 90;
  mreal vel = 0;
  mreal tgt = 90;
  mreal scale = 1;  // sync factor 0..1 applied to vmax and accel
};

// Advance one axis by dt seconds with velocity cap vmax (deg/s) and accel amax (deg/s^2).
inline void stepAxis(Axis& a, mreal dt, mreal vmax, mreal amax) {
  mreal s = a.scale < MIN_SCALE ? MIN_SCALE : a.scale;
  mreal v = vmax * s;
  mreal acc = amax * s;
  mreal err = a.tgt - a.pos;
  if (mabs(err) <= EPS_POS && mabs(a.vel) <= amax * dt) {
    a.pos = a.tgt;
    a.vel = 0;
    return;
  }
  // fastest speed from which we can still stop exactly on target
  mreal vdes = msign(err) * mmin(v, (mreal)sqrt(2 * acc * mabs(err)));
  mreal dv = vdes - a.vel;
  // speeding up uses the (sync-scaled) accel; braking may use the full accel
  bool braking = (a.vel != 0) && (msign(dv) != msign(a.vel));
  mreal lim = (braking ? amax : acc) * dt;
  a.vel += mclamp(dv, -lim, lim);
  mreal step = a.vel * dt;
  if (msign(step) == msign(err) && mabs(step) >= mabs(err)) {
    a.pos = a.tgt;
    a.vel = 0;
  } else {
    a.pos += step;
  }
}

template <int N>
struct Planner {
  Axis ax[N];
  mreal vmax = 90;      // global speed, deg/s
  mreal amax = 240;     // global accel, deg/s^2
  mreal jvmax[N];       // per-joint speed cap, deg/s
  bool sync = true;

  Planner() {
    for (int i = 0; i < N; i++) jvmax[i] = 360;
  }

  void init(const mreal* p) {
    for (int i = 0; i < N; i++) {
      ax[i].pos = ax[i].tgt = p[i];
      ax[i].vel = 0;
      ax[i].scale = 1;
    }
  }

  mreal capOf(int i) const { return mmin(vmax, jvmax[i]); }

  // mask[i] == false leaves joint i untouched. doSync overrides `sync` for this move.
  void setTargets(const mreal* t, const bool* mask, bool doSync) {
    mreal tmax = 0;
    mreal ti[N];
    for (int i = 0; i < N; i++) {
      ti[i] = 0;
      if (!mask[i]) continue;
      ti[i] = mabs(t[i] - ax[i].pos) / mmax(capOf(i), (mreal)0.001);
      tmax = mmax(tmax, ti[i]);
    }
    for (int i = 0; i < N; i++) {
      if (!mask[i]) continue;
      ax[i].tgt = t[i];
      ax[i].scale = (doSync && tmax > 0) ? mclamp(ti[i] / tmax, MIN_SCALE, 1) : 1;
    }
  }

  void setTarget(int i, mreal t) {
    ax[i].tgt = t;
    ax[i].scale = 1;
  }

  // Controlled stop: aim each axis at its braking point.
  void stop() {
    for (int i = 0; i < N; i++) {
      Axis& a = ax[i];
      mreal d = a.vel * mabs(a.vel) / (2 * mmax(amax, (mreal)1));
      a.tgt = a.pos + d;
      a.scale = 1;
    }
  }

  // Instant freeze (emergency stop).
  void halt() {
    for (int i = 0; i < N; i++) {
      ax[i].tgt = ax[i].pos;
      ax[i].vel = 0;
    }
  }

  void update(mreal dt) {
    for (int i = 0; i < N; i++) stepAxis(ax[i], dt, capOf(i), amax);
  }

  bool moving() const {
    for (int i = 0; i < N; i++)
      if (ax[i].vel != 0 || mabs(ax[i].tgt - ax[i].pos) > EPS_POS) return true;
    return false;
  }
};

}  // namespace motion
