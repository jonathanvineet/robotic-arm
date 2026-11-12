# RoboArm — 6-DOF arm controller, digital twin & simulator

ESP8266 (or ESP32) + PCA9685 firmware, a browser controller with a live 3D digital twin,
and a simulator that behaves exactly like the real board, so you can test everything without hardware.

```
firmware/RoboArm/        ← upload this sketch (UI is embedded, nothing else to upload)
firmware/tests/          ← hardware bring-up sketches + native C++ tests
web/                     ← controller + digital twin + simulator (also served by the ESP)
tools/                   ← dev server, UI embedder, compile script
```

## 1. Wiring

### Controller → PCA9685

| PCA9685 | **ESP8266** (NodeMCU / D1 mini) | ESP32 DevKit |
|---------|------------------------|--------------|
| SDA     | **D2** (GPIO4)         | GPIO21       |
| SCL     | **D1** (GPIO5)         | GPIO22       |
| VCC     | 3V3                    | 3V3          |
| GND     | GND                    | GND          |

> The ESP8266 has no GPIO21/22. Use **D2/D1**. The firmware picks the right pins for each board automatically.

### Servos

| PCA9685 | Servo | Joint |
|---|---|---|
| CH0 | MG995 | Base rotation |
| CH1 | high-torque | Shoulder |
| CH2 | MG996R | Elbow |
| CH3 | MG995 | Wrist pitch |
| CH4 | MG90S | Wrist roll |
| CH5 | MG90S | Gripper |

Power: an external **6 V supply goes to the PCA9685 V+ / GND terminal block**. Connect the ESP GND to the PCA9685 GND.
Never power the servos from the ESP. Use a supply of at least **5 A**: MG995 and MG996R servos pull up to 2.5 A each when stalled.
A 1000 µF capacitor across V+/GND helps with brown-outs.

## 2. Firmware

**Arduino IDE**
1. Boards Manager: install **esp8266** by ESP8266 Community.
2. Library Manager: install **Adafruit PWM Servo Driver Library**, **ArduinoJson** (v7), and **WebSockets** by Markus Sattler.
3. Open `firmware/RoboArm/RoboArm.ino`, choose the board **NodeMCU 1.0 (ESP-12E)** (or LOLIN(WEMOS) D1 mini), and upload.
4. Open the Serial Monitor at 115200 baud. It prints the IP address and whether the PCA9685 was found.

For an ESP32, choose **ESP32 Dev Module** and set *Partition Scheme* to **Minimal SPIFFS (1.9MB APP with OTA)**. The default partition works, but the sketch fills 97% of it.

Optional settings are in `config.h`: Wi-Fi network, pin numbers and default limits. All of the limits can also be changed live from the web app.

**Compile check from the terminal:** `sh tools/compile-firmware.sh` builds for both ESP8266 and ESP32.

### Connecting
- On first boot the arm creates its own Wi-Fi hotspot, **RoboArm** (password `roboarm123`). Join it and open **http://192.168.4.1**.
- To put the arm on your home Wi-Fi, go to the **Device** tab, enter the SSID and password, and click **Save & reboot**. After that, open **http://roboarm.local** or the IP address shown on the serial monitor.
- **USB cable:** run the app from your PC (see below), choose **USB serial**, and click Connect (Chrome or Edge only).
- **Serial Monitor:** type `help` for plain-text commands (`home`, `j 2 120`, `estop`, `status` …).
- **OTA:** after the first USB upload, the board shows up as a network port called `roboarm` in the Arduino IDE.

## 3. Bring-up order (do this once)

1. `firmware/tests/hw_i2c_scan`: confirms the PCA9685 answers at `0x40`.
2. `firmware/tests/hw_servo_test`: moves **one servo at a time** with raw pulse widths so you can find each servo's safe end stops. Type `p` to print the µs values.
3. Upload `RoboArm` and open the **Calibrate** tab. Enter each joint's min/max/home, the µs values at 0° and 180°, and the **Zero** angle (the servo angle at which that link points straight up). Tick **Inv** if the twin moves the opposite way to the real arm. Click **Apply**, then **Save to flash**.
4. Measure your link lengths (axis to axis) under **Twin geometry**. The twin and the XYZ control will then match the real arm.
5. Open the **Test** tab and run the **Self-test sweep** at the *Gentle* speed preset.

**First power-up tip:** the arm drives to its home pose on boot. If you're unsure your home values are safe, tick **Start limp on power-up**. The servos then stay relaxed until you press **Hold**.

## 4. Web app

Run it without hardware: `npm start`, then open http://localhost:8080. It connects to the **Simulator** automatically.

| Area | What it does |
|---|---|
| Joints | Slider and number box per joint, hold **−/+** to jog, a jog step selector, Home / Stop / Relax |
| Motion | Speed (°/s), acceleration (°/s²), synchronised moves, Gentle/Normal/Fast presets |
| Twin | Solid arm = the real position reported by the board; cyan ghost = where it's heading; TCP trail; reach ring; 3D/Front/Side/Top views |
| XYZ | Move the fingertip in mm (inverse kinematics), plus Cartesian jog buttons |
| Poses | Save, go to, overwrite and delete named poses |
| Sequence | Record steps with pauses, play once or in a loop, export/import JSON |
| Calibrate | Per-joint channel, limits, home, zero, invert, pulse range and speed cap; PCA9685 oscillator trim; factory reset |
| Test | Self-test sweep, random poses, a safety check (limits, E-stop, stop), simulated latency/jitter, live pulse monitor |
| Device | Board info, I2C scan, Wi-Fi setup, reboot, raw JSON commands |

**Keys:** `Space` E-stop · `Esc` stop · `H` home · `1`–`6` select a joint · `←/→` jog (`Shift` ×5) · `W/S` X · `A/D` Y · `R/F` Z
**Gamepad:** left stick = base/shoulder · right stick = wrist roll/elbow · D-pad = wrist pitch · LT/RT = gripper · A home · **B E-STOP** · X stop

**E-stop** freezes the arm where it is, with torque on so it doesn't drop, and rejects every move until you press **Reset E-stop**.
**Relax** switches the PWM off so the joints go limp. Hold the arm when you do this, because it will fall.
If the last browser disconnects while the arm is moving, the arm brakes to a stop.

## 5. Tests

```sh
npm test          # 28 tests: motion planner, kinematics, simulator protocol,
                  # + compiles the firmware planner natively and checks the simulator
                  # matches it sample-for-sample
cd firmware/tests/native && g++ -std=c++17 -I../../RoboArm test_motion.cpp -o t && ./t
```

After changing anything in `web/`, run `npm run embed` to regenerate `firmware/RoboArm/web_assets.h`, then re-upload.

## 6. Protocol

JSON messages over WebSocket `ws://<ip>:81`, newline-delimited JSON over serial at 115200, or `POST /api/cmd`.

| Command | Meaning |
|---|---|
| `{"c":"set","j":[90,80,null,…],"sync":true}` | move several joints (`null` = leave as is) |
| `{"c":"joint","i":2,"a":120}` / `{"c":"jog","i":2,"d":-5}` | move or jog one joint |
| `{"c":"home"}` `{"c":"stop"}` `{"c":"estop"}` `{"c":"reset"}` | home, controlled stop, E-stop, clear E-stop |
| `{"c":"relax"}` `{"c":"hold"}` | torque off / on |
| `{"c":"speed","v":90,"a":240,"sync":true}` | motion settings |
| `{"c":"cfg","j":[{"i":0,"min":10,"max":170,"home":90,"zero":90,"dir":1,"pmin":500,"pmax":2500,"vmax":120,"ch":0}],"osc":25000000,"bootRelax":false}` | calibration (any subset of fields) |
| `{"c":"save"}` `{"c":"factory"}` `{"c":"get"}` `{"c":"scan"}` `{"c":"reboot"}` `{"c":"wifi","ssid":"…","pass":"…"}` | settings, info, I2C scan, reboot, Wi-Fi |

Replies: `{"t":"s","p":[actual],"g":[target],"m":moving,"e":estop,"r":relaxed}` at 20 Hz while moving (1 Hz idle), plus `{"t":"cfg",…}`, `{"t":"info",…}`, `{"t":"log","l":"info|warn|error","m":"…"}` and `{"t":"pong","id":…}`.
`GET /api/state` returns the current position for scripts.
