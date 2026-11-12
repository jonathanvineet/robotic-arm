// config.h — compile-time settings. Everything marked (runtime) can also be
// changed live from the web app (Calibrate tab) and saved to flash, so you
// rarely need to edit this file after the first upload.
#pragma once

#define FW_VERSION "1.0.0"
#define NUM_JOINTS 6

// ---------------------------------------------------------------- I2C pins
// ESP8266 (NodeMCU / Wemos D1 mini): SDA = D2 (GPIO4), SCL = D1 (GPIO5)
// ESP32 DevKit:                      SDA = GPIO21,    SCL = GPIO22
#if defined(ESP8266)
#define I2C_SDA 4
#define I2C_SCL 5
#else
#define I2C_SDA 21
#define I2C_SCL 22
#endif

#define PCA9685_ADDR 0x40
#define SERVO_FREQ 50                // Hz
#define PCA9685_OSC_DEFAULT 25000000 // (runtime) tune if angles are slightly off

// ---------------------------------------------------------------- Wi-Fi
// Leave WIFI_SSID empty to always start the access point.
// You can also set Wi-Fi from the web app without re-flashing.
#define WIFI_SSID ""
#define WIFI_PASS ""
#define AP_SSID "RoboArm"
#define AP_PASS "roboarm123"   // min 8 chars
#define HOSTNAME "roboarm"     // http://roboarm.local
#define WIFI_CONNECT_TIMEOUT_MS 12000

#define HTTP_PORT 80
#define WS_PORT 81
#define SERIAL_BAUD 115200

// ---------------------------------------------------------------- timing
#define CONTROL_HZ 50   // motion planner rate
#define STATE_HZ 20     // telemetry rate while moving

// ---------------------------------------------------------------- motion (runtime)
#define DEFAULT_SPEED 90.0f    // deg/s
#define DEFAULT_ACCEL 240.0f   // deg/s^2
#define DEFAULT_SYNC true      // all joints arrive together
#define DEFAULT_BOOT_RELAX false  // true = servos stay limp until first command

// ---------------------------------------------------------------- joints (runtime)
// Angles are servo degrees 0..180.
//  min/max : soft limits — the arm will never be commanded outside these
//  home    : reset / home pose
//  zero    : servo angle where the joint is at its kinematic zero (arm straight up)
//  dir     : +1 / -1 to flip the direction in the digital twin & IK
//  pmin/pmax : pulse width (us) for 0 and 180 degrees
//  vmax    : per-joint speed cap (deg/s) — keep the heavy shoulder slow
struct JointDefaults {
  const char* name;
  uint8_t ch;
  float min, max, home, zero;
  int8_t dir;
  uint16_t pmin, pmax;
  float vmax;
};

static const JointDefaults JOINT_DEFAULTS[NUM_JOINTS] = {
  // name           ch  min   max  home zero dir pmin  pmax  vmax
  {"Base",          0,  0,  180,  90,  90,  1,  500, 2500, 120},  // MG995
  {"Shoulder",      1, 15,  165,  90,  90,  1,  500, 2500,  60},  // high-torque
  {"Elbow",         2,  0,  180,  90,  90,  1,  500, 2500,  90},  // MG996R
  {"Wrist Pitch",   3,  0,  180,  90,  90,  1,  500, 2500, 120},  // MG995
  {"Wrist Roll",    4,  0,  180,  90,  90,  1,  500, 2500, 180},  // MG90S
  {"Gripper",       5, 30,  120,  60,  90,  1,  500, 2500, 180},  // MG90S
};
