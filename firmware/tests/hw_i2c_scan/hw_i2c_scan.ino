// STEP 1 of hardware bring-up: is the PCA9685 wired correctly?
// Upload, open Serial Monitor @115200. You should see "0x40 <- PCA9685".
// (0x70 is the PCA9685 "all-call" address and is normal too.)
#include <Wire.h>

#if defined(ESP8266)
#define SDA_PIN 4  // D2
#define SCL_PIN 5  // D1
#else
#define SDA_PIN 21
#define SCL_PIN 22
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.printf("\nI2C scanner  SDA=GPIO%d SCL=GPIO%d\n", SDA_PIN, SCL_PIN);
}

void loop() {
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X%s\n", a, a == 0x40 ? "  <- PCA9685" : a == 0x70 ? "  (PCA9685 all-call)" : "");
      found++;
    }
  }
  if (!found) Serial.println("  nothing found: check SDA/SCL swapped, VCC=3.3V, GND shared with ESP");
  Serial.println("--- rescanning in 3 s");
  delay(3000);
}
