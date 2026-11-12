# RoboArm — 6-DOF arm controller, digital twin & simulator

ESP8266 (or ESP32) + PCA9685 firmware, a browser controller with a live 3D digital twin,
a simulator that behaves exactly like the real board, and **camera-guided pick & place**:
a camera on the PC spots plastic with YOLO and the arm picks it up and drops it in a bin.
Everything, including the camera, can be tested in the simulator without hardware.

```
firmware/RoboArm/        ← upload this sketch (UI is embedded, nothing else to upload)
firmware/tests/          ← hardware bring-up sketches + native C++ tests
web/                     ← controller + digital twin + simulator (also served by the ESP)
tools/                   ← dev server, UI embedder, compile script
Plastic-Segregation-AI/  ← YOLO model, vision_bridge.py (camera → web app), make_markers.py
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

Run it without hardware: `npm run web` (or `npm start`, which also starts the camera bridge). It connects to the **Simulator** automatically.

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
npm test          # 40 tests: motion planner, kinematics, simulator protocol,
                  # vision maths (homography, tracker, pick planning), and compiles the
                  # firmware planner natively to check the simulator matches it sample-for-sample
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

## 7. Camera pick & place (plastic sorting)

```
 camera (PC, USB) ──► vision_bridge.py ──► web app ──► arm (Wi-Fi or USB)
    YOLO boxes + ArUco markers (pixels)     │  pixels → table mm (homography from the 4 corner markers)
                                            │  tracking, reachability, inverse kinematics
                                            └► hover → descend → grip → lift → bin → release → park
```

The same pipeline runs in the simulator. The twin has a simulated overhead camera that renders the scene
and outputs YOLO-style boxes, so you can test calibration, tracking and picking with no camera or arm.

### How the real work cell must be laid out

Top view. The robot frame has its origin at the centre of the arm's base, with **X = front** (where the arm points when the base servo is at its zero angle) and **Y = the arm's left**.

```
   ID3 back-left ┌────────────────────────────────────┐ ID0 front-left      ▲ +X front
   (-360,+360)   │        · · · reach ring · · ·      │ (+360,+360)
                 │     ·    180–300 mm all round   ·  │
                 │   ·                               ·│
   ┌─────┐       │   ·            ( ◎ )              ·│   ◎ arm base = origin
   │ BIN │       │   ·     camera 1150 mm straight  · │   camera on the base axis,
   └─────┘       │     ·   above the base, looking  · │   looking straight down
  (-120,210)     │        · ·  down at everything  ·  │
   ID2 back-right└────────────────────────────────────┘ ID1 front-right
   (-360,-360)                                          (+360,-360)
               +Y ◄──────────────────────────────────────►  -Y
```

| Item | Where / how |
|---|---|
| **Arm** | In the middle of the table. Clamp or bolt the base so it can never shift. |
| **Corner markers** | Run `python make_markers.py` and print the PDF at 100% (one 100 mm marker per page; check the 50 mm scale bar). Tape the markers flat with their **centres at (±360, ±360) mm** from the base centre: ID0 front-left, ID1 front-right, ID2 back-right, ID3 back-left. Each page's arrow points to the front. Easiest method: draw a cross through the base centre (one line along the arm's front direction, one across), then measure 360 mm along each line and 360 mm across. If your markers end up somewhere else, measure their centres and type them into Vision → Calibration → Corner markers. Picks are only as accurate as these numbers. |
| **Pick area** | Everything inside the square. The arm (as built in the CAD) reaches the table in a **ring 180–300 mm from the base, all the way round**: it leans back over itself for targets behind. Two places are out of reach: the area right next to the base, and a thin wedge exactly on the arm's left, which would need a little more than the base servo's 180°. Objects there are boxed in orange as *out of reach*. |
| **Camera** | **1150 mm straight above the centre of the base, looking straight down** (anywhere from 1120 to 1300 mm works), at **1920×1080**. It can be rotated any way, but all 4 markers must be fully in the picture. Mount it on a gallows stand: a pole behind the robot, outside its reach, with a boom over the top. It must never move after calibration. |
| **Bin** | Back-left at X −120, Y 210 mm (changeable in Pick settings), under 120 mm tall. The bin's footprint is ignored for detections. |
| **Light** | Even, diffuse light from above or the sides. Avoid glare spots and hard shadows. |
| **Background** | A plain matte surface that contrasts with plastic. |

### Camera height
- **Too low:** the arm stretched straight up is about 460 mm tall. Keep at least 150 mm above that, so 610 mm or more.
- **Wide enough:** to see all 4 corners (±360 mm plus the 100 mm markers and their white borders) on the short side of a 16:9, 70° webcam, the camera must be at least about 1110 mm high. With a 4:3 camera or a wider lens you can go lower.
- **Detail:** at 1150 mm and 1920×1080 you get about 1.2 px/mm, so a 30 mm bottle cap is about 36 px. The bridge runs YOLO only on the marker square, which keeps objects large for the model and stops the arm itself being detected.
- **Accuracy:** higher means less error from tall objects leaning in the view (about 4 mm at the edge of the square from 1150 mm).
- **Test your model at this height.** If it misses small objects, try `--imgsz 960`.

### Calibrating (once per camera position)
1. Park the arm at **Home** (straight up). Seen from directly above, it's just a dot in the middle of the picture. Open the **Vision** tab: all 4 markers should be boxed in yellow.
2. Click **Calibrate from markers**. The error should be under 3 mm. Then check the overlay: the orange **front** line should point the way your arm faces at base 0°, and the cyan grid should look square.
3. Optional cross-check: tick **Touch-point mode**, jog the fingertip down onto a spot on the table, and click that spot in the image. Repeat for 4 or more spots around the ring, then click **Solve**.

### Arm model
The 3D arm in the twin is the real CAD model, `Final_assembly.STEP`, converted by `tools/step-to-model.mjs`. It also measures the kinematics from the CAD:
- joint axes from the servo shafts;
- shoulder 96.3 mm high and 13.9 mm forward of the base axis;
- upper arm 120.0 mm and forearm 119.2 mm;
- wrist to fingertips 124.1 mm;
- fingertips 17 mm to the right of the base axis;
- **the wrist rolls before it pitches**.

Because of that joint order, picking keeps the wrist roll neutral. If you change the CAD, re-export the STEP and run:
```sh
npm i --no-save occt-import-js@0.0.23
node tools/step-to-model.mjs Final_assembly.STEP
npm run embed
```

### Plastic detection in the browser
The Vision tab's **Webcam (detect in browser)** source does everything in the page: the camera, your YOLO model and the ArUco markers. No Python is needed at run time.
- **The model:** your model is a **YOLO26n** end-to-end model, converted once with `python export_onnx.py` to `web/models/plastic.onnx`. Its boxes match Ultralytics' Python output to within 0.1 px.
- **Speed:** it runs on the **GPU via WebGPU** when the browser supports it (Chrome or Edge, Mac and Windows), at about 30 ms per frame on this Mac. Otherwise it falls back to several CPU threads in a background worker, at roughly 150–450 ms per frame.
- **Checking it:** the status line under the camera view shows which backend is in use and the time per frame.
- **Camera access:** the browser only allows it on `http://localhost`, so open the app with `npm start` on the PC the camera is plugged into. The page served by the ESP can't use the camera.
- **Image correction** (Vision → Image & small objects): auto contrast, automatic or manual brightness, saturation, and optional white balance. These are applied to the model's input and, if ticked, to the preview. They fix a washed-out or whitish webcam. Under *Camera controls* you'll find any hardware settings the camera exposes (exposure, contrast, white balance, focus). Windows usually exposes more than macOS.
- **Small objects from a high camera:** the **Zoom tiles** setting runs the model on overlapping tiles, so small items appear 2–3× bigger to it. *Auto* uses 2×2 tiles inside the calibrated pick area, on the GPU only. Each tile costs one extra model run. The small *model view* square shows exactly what the model sees.
- **The real fix for small objects is training data from your actual setup.** Use **Save frame for training** to collect frames from the mounted camera, label them in Roboflow, retrain, then run `python export_onnx.py` again.

### Running on the HP Omen (Windows + NVIDIA RTX)
**In the browser (recommended):** use Chrome or Edge, run `npm start`, and use **Webcam (detect in browser)**. WebGPU runs on the RTX through DirectX 12. The detector asks for the *high-performance* GPU, and the Log and status line show which GPU it got, e.g. `GPU (nvidia ampere …)`.

**If it reports Intel, or "integrated GPU" in the Log:**
1. Open Windows **Settings → System → Display → Graphics**.
2. Add Chrome (or Edge) and set it to **High performance**, then restart the browser.
3. Optionally, in **OMEN Gaming Hub**, set the GPU mode to **Discrete** for the best speed.
4. Make sure the NVIDIA driver is up to date.

A fast GPU (under 15 ms per tile) automatically gets zoom tiles over the whole frame, up to 3×3, even before calibration. That's better for small objects seen from a high camera.

**Python alternative on the RTX (CUDA):**
```bat
cd robotic-arm\Plastic-Segregation-AI
py -3.11 -m venv .venv
.venv\Scripts\activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
pip install -r requirements.txt
cd ..
npm run bridge -- --device 0
```
Then choose **Python vision bridge** in the Vision tab.

### Running it

**One command:**
```sh
npm start                     # web app on http://localhost:8080, opens the browser (detection runs in the page)
npm run bridge                # optional: the Python vision bridge instead (camera + YOLO in Python)
```
`npm start` finds Python (preferring `Plastic-Segregation-AI/.venv` if it exists) and runs `vision_bridge.py`, which also serves the web app at **http://localhost:8765**. It keeps working when things are missing:
- **No camera plugged in:** the app still runs and the bridge retries the camera every 5 s.
- **YOLO (ultralytics) not installed:** markers-only mode, with a message telling you what to install.
- **No Python or OpenCV:** it falls back to the plain web server so the simulator works, and prints what to install.

**First time on Windows:**
```bat
cd robotic-arm\Plastic-Segregation-AI
py -3.11 -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
cd ..
npm start
```
In the app, connect to the arm in the header (*Wi-Fi* with the arm's IP, or *USB serial*). In the Vision tab, use **PC camera**, calibrate, then **Pick now** or **Auto pick**. To work without hardware, switch the Vision source to *Simulated camera* and the header to *Simulator*.

### Camera position and the angled-camera correction
The camera doesn't have to be straight above the robot. When you calibrate, the app works out where the camera is from the 4 markers: X, Y, height and tilt, plus the lens's field of view whenever the camera is tilted. Check it against a tape measure: the log prints it, and in the simulator a yellow wire ball marks it in 3D.

With that camera position, **Angled-camera correction** (on by default, under Calibration) moves each detection back along the camera's line of sight to the object's mid-height (Grip Z). This removes the error from seeing objects from the side:

| Camera | Worst error without | With correction |
|---|---|---|
| Straight above, 1150 mm | 4 mm | 0.4 mm |
| 300 mm off-centre (15° tilt) | 8 mm | 0.5 mm |
| From a corner (33° tilt) | 15 mm | 0.6 mm |
| Steep side view (45° tilt) | 22 mm | 0.8 mm |

These numbers are for a 30 mm-tall object, with Grip Z set to its mid-height. If the camera looks straight down, enter your webcam's horizontal FOV in Calibration; it only affects the computed height in that case. A steep angle still has drawbacks: the arm hides more of the table and far objects get fewer pixels. Mostly-overhead remains best.

`vision_bridge.py` options: `--no-model` (markers only), `--image photo.png` / `--video clip.mp4` (no camera), `--dict DICT_5X5_100` (other marker set), `--conf 0.5`, `--fps 10`, `--imgsz 960`.

**Safety:** auto-pick only starts when the arm is idle, connected and not E-stopped. It needs an object to be seen in 5 consecutive frames. It gives up on a spot after 3 missed grasps. **Space** or **Stop** cancels a pick immediately.
