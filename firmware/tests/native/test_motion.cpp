// Native (desktop) tests for the firmware motion planner — no hardware needed.
//   g++ -std=c++17 -O2 -I../../RoboArm test_motion.cpp -o test_motion && ./test_motion
//   ./test_motion trace   -> prints a CSV trajectory (used by web/tests/crosscheck.test.mjs)
#include <cstdio>
#include <cstring>
#include <cmath>
#include "motion.h"

using namespace motion;
static int failures = 0, checks = 0;
#define CHECK(cond, ...)                                    \
  do {                                                      \
    checks++;                                               \
    if (!(cond)) {                                          \
      failures++;                                           \
      printf("FAIL %s:%d  ", __FILE__, __LINE__);           \
      printf(__VA_ARGS__);                                  \
      printf("\n");                                         \
    }                                                       \
  } while (0)

static const mreal DT = (mreal)0.02;

// Shared scenario — keep identical to web/tests/crosscheck.test.mjs
static void scenario(Planner<6>& P, int step) {
  if (step == 0) {
    mreal t[6] = {0, 150, 45, 120, 10, 170};
    bool m[6] = {true, true, true, true, true, true};
    P.setTargets(t, m, true);
  }
  if (step == 60) P.setTarget(2, 160);       // retarget mid-move (reverses the elbow)
  if (step == 120) P.stop();                 // controlled stop
  if (step == 150) {
    mreal t[6] = {180, 20, 90, 90, 90, 40};
    bool m[6] = {true, true, false, true, false, true};
    P.setTargets(t, m, false);
  }
  if (step == 260) P.halt();
  if (step == 270) {
    mreal t[6] = {90, 90, 90, 90, 90, 90};
    bool m[6] = {true, true, true, true, true, true};
    P.setTargets(t, m, true);
  }
}

static void setupPlanner(Planner<6>& P) {
  mreal start[6] = {90, 90, 90, 90, 90, 90};
  P.init(start);
  P.vmax = 90;
  P.amax = 240;
  mreal caps[6] = {120, 60, 90, 120, 180, 180};
  for (int i = 0; i < 6; i++) P.jvmax[i] = caps[i];
}

static int trace() {
  Planner<6> P;
  setupPlanner(P);
  for (int s = 0; s < 600; s++) {
    scenario(P, s);
    P.update(DT);
    for (int i = 0; i < 6; i++) printf(i ? ",%.6f" : "%.6f", (double)P.ax[i].pos);
    printf("\n");
  }
  return 0;
}

static void testReachesTargetNoOvershoot() {
  Planner<6> P;
  setupPlanner(P);
  mreal t[6] = {10, 160, 0, 180, 0, 120};
  bool m[6] = {true, true, true, true, true, true};
  P.setTargets(t, m, true);
  mreal prev[6];
  for (int i = 0; i < 6; i++) prev[i] = P.ax[i].pos;
  int steps = 0;
  while (P.moving() && steps < 5000) {
    P.update(DT);
    for (int i = 0; i < 6; i++) {
      mreal lo = prev[i] < t[i] ? prev[i] : t[i], hi = prev[i] > t[i] ? prev[i] : t[i];
      (void)lo; (void)hi;
      // never passes the target
      if (t[i] >= 90) CHECK(P.ax[i].pos <= t[i] + 1e-4, "axis %d overshot: %f > %f", i, (double)P.ax[i].pos, (double)t[i]);
      else CHECK(P.ax[i].pos >= t[i] - 1e-4, "axis %d overshot: %f < %f", i, (double)P.ax[i].pos, (double)t[i]);
    }
    steps++;
  }
  CHECK(steps < 5000, "did not settle");
  for (int i = 0; i < 6; i++) CHECK(fabs(P.ax[i].pos - t[i]) < 1e-4, "axis %d ended at %f, wanted %f", i, (double)P.ax[i].pos, (double)t[i]);
}

static void testRespectsLimits() {
  Planner<6> P;
  setupPlanner(P);
  mreal t[6] = {0, 0, 0, 0, 0, 0};
  bool m[6] = {true, true, true, true, true, true};
  P.setTargets(t, m, false);
  mreal lastVel[6] = {0};
  for (int s = 0; s < 500; s++) {
    P.update(DT);
    for (int i = 0; i < 6; i++) {
      mreal cap = P.capOf(i);
      CHECK(fabs(P.ax[i].vel) <= cap + 1e-3, "axis %d speed %f > cap %f", i, (double)P.ax[i].vel, (double)cap);
      if (P.ax[i].vel != 0) {
        mreal acc = fabs(P.ax[i].vel - lastVel[i]) / DT;
        CHECK(acc <= P.amax + 1e-2, "axis %d accel %f > %f", i, (double)acc, (double)P.amax);
      }
      lastVel[i] = P.ax[i].vel;
    }
  }
}

static void testSyncArrival() {
  Planner<6> P;
  setupPlanner(P);
  mreal t[6] = {170, 100, 30, 95, 0, 92};
  bool m[6] = {true, true, true, true, true, true};
  P.setTargets(t, m, true);
  int arrive[6] = {-1, -1, -1, -1, -1, -1};
  for (int s = 0; s < 3000; s++) {
    P.update(DT);
    for (int i = 0; i < 6; i++)
      if (arrive[i] < 0 && P.ax[i].pos == t[i] && P.ax[i].vel == 0) arrive[i] = s;
  }
  int lo = 1 << 30, hi = 0;
  for (int i = 0; i < 6; i++) {
    CHECK(arrive[i] >= 0, "axis %d never arrived", i);
    if (arrive[i] < lo) lo = arrive[i];
    if (arrive[i] > hi) hi = arrive[i];
  }
  // within 0.3 s of each other (accel phases differ slightly)
  CHECK(hi - lo <= 15, "sync spread %d steps (%d..%d)", hi - lo, lo, hi);
}

static void testStopAndHalt() {
  Planner<6> P;
  setupPlanner(P);
  P.setTarget(0, 0);
  for (int s = 0; s < 30; s++) P.update(DT);
  mreal v = P.ax[0].vel;
  CHECK(v < 0, "expected base moving negative");
  P.stop();
  mreal at = P.ax[0].pos;
  int s = 0;
  while (P.moving() && s < 1000) { P.update(DT); s++; }
  CHECK(P.ax[0].pos < at && P.ax[0].pos > 0, "stop should glide a little and stop short: %f", (double)P.ax[0].pos);
  P.setTarget(0, 180);
  for (int k = 0; k < 20; k++) P.update(DT);
  P.halt();
  CHECK(!P.moving(), "halt must freeze instantly");
}

int main(int argc, char** argv) {
  if (argc > 1 && !strcmp(argv[1], "trace")) return trace();
  testReachesTargetNoOvershoot();
  testRespectsLimits();
  testSyncArrival();
  testStopAndHalt();
  printf("%s: %d/%d checks passed\n", failures ? "FAILED" : "OK", checks - failures, checks);
  return failures ? 1 : 0;
}
