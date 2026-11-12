// STEP 2 of hardware bring-up: test & calibrate each servo on its own.
// Upload, open Serial Monitor @115200 (line ending: Newline). Commands:
//   c <0-15>     select channel            u <us>     raw pulse width (500..2500)
//   a <deg>      angle with current range  + / -      nudge pulse by step
//   s <us>       nudge step (default 10)   w          slow sweep min->max->mid
//   lo / hi      remember current pulse as this servo's 0° / 180°
//   off          release channel           all off    release everything
//   p            print summary (copy into the Calibrate tab: µs@0° / µs@180°)
// Start in the middle (1500 us) and move in small steps to find the mechanical
// end stops. Back off ~20 us from where the servo buzzes / stalls.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#if defined(ESP8266)
#define SDA_PIN 4
#define SCL_PIN 5
#else
#define SDA_PIN 21
#define SCL_PIN 22
#endif

Adafruit_PWMServoDriver pwm(0x40, Wire);
const char* NAMES[6] = {"Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"};
int ch = 0, us = 1500, stepUs = 10;
int lo[16], hi[16];
float ticksPerUs;

void out(int c, int u) {
  u = constrain(u, 300, 2900);
  pwm.setPWM(c, 0, (uint16_t)(u * ticksPerUs + 0.5f));
  Serial.printf("CH%d %s -> %d us\n", c, c < 6 ? NAMES[c] : "", u);
}

void summary() {
  Serial.println("ch  name          us@0°  us@180°");
  for (int i = 0; i < 6; i++) Serial.printf("%-3d %-12s  %5d  %5d\n", i, NAMES[i], lo[i], hi[i]);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(50);
  float pre = roundf(25000000.0f / (4096 * 50)) - 1;
  ticksPerUs = 4096.0f * (25000000.0f / (4096 * (pre + 1))) / 1e6f;
  for (int i = 0; i < 16; i++) { lo[i] = 500; hi[i] = 2500; }
  Serial.println("\nServo tester. 'c 0' selects base, 'u 1500' centres it. Type commands, press Enter.");
  Serial.println("!! Servo power must come from the external 6 V supply on PCA9685 V+ !!");
}

void loop() {
  if (!Serial.available()) return;
  String l = Serial.readStringUntil('\n');
  l.trim();
  if (!l.length()) return;
  int sp = l.indexOf(' ');
  String cmd = sp < 0 ? l : l.substring(0, sp);
  int arg = sp < 0 ? 0 : l.substring(sp + 1).toInt();
  if (cmd == "c") { ch = constrain(arg, 0, 15); Serial.printf("channel %d selected\n", ch); }
  else if (cmd == "u") { us = arg; out(ch, us); }
  else if (cmd == "a") { us = lo[ch] + (hi[ch] - lo[ch]) * constrain(arg, 0, 180) / 180; out(ch, us); }
  else if (cmd == "+") { us += stepUs; out(ch, us); }
  else if (cmd == "-") { us -= stepUs; out(ch, us); }
  else if (cmd == "s") { stepUs = max(1, arg); Serial.printf("step %d us\n", stepUs); }
  else if (cmd == "lo") { lo[ch] = us; Serial.printf("CH%d 0° = %d us\n", ch, us); }
  else if (cmd == "hi") { hi[ch] = us; Serial.printf("CH%d 180° = %d us\n", ch, us); }
  else if (cmd == "w") {
    for (int u = lo[ch]; u <= hi[ch]; u += 10) { pwm.setPWM(ch, 0, u * ticksPerUs); delay(25); }
    for (int u = hi[ch]; u >= (lo[ch] + hi[ch]) / 2; u -= 10) { pwm.setPWM(ch, 0, u * ticksPerUs); delay(25); }
    us = (lo[ch] + hi[ch]) / 2;
    Serial.println("sweep done");
  }
  else if (cmd == "off") { pwm.setPin(ch, 0); Serial.printf("CH%d released\n", ch); }
  else if (l == "all off") { for (int i = 0; i < 16; i++) pwm.setPin(i, 0); Serial.println("all released"); }
  else if (cmd == "p") summary();
  else Serial.println("? commands: c u a + - s w lo hi off 'all off' p");
}
