# Quadruped Spider Robot

<img width="1179" height="1451" alt="IMG_1272" src="https://github.com/user-attachments/assets/f929e8d6-cc07-4266-8d5a-cb2376de141d" />

Welcome to the repository for the Quadruped Spider Robot. This project is a complete robotics platform featuring a 3D-printed chassis and a fully custom-designed Printed Circuit Board (PCB).

The robot walks on twelve servos, keeps itself level using an on-board IMU, watches the room with an ultrasonic sensor, animates a pair of eyes on an OLED, and is driven from a phone over its own Wi-Fi network — no app to install.

## 🕷️ Features

*   **Brain:** An **ESP32-S3-WROOM-1-N8R8** (dual core, 8 MB flash, 8 MB octal PSRAM). One core runs the gait, the sensors and the web server; the other is dedicated to the eye animation, so the display never stutters while the robot walks.
*   **Custom PCB:** A completely customized, purpose-built motherboard integrating power management, logic and signal routing, eliminating messy wiring.
*   **Servo control:** An on-board **PCA9685** PWM driver commands **12 servo motors**, 3 degrees of freedom per leg, over a dedicated I²C bus, with a hardware output-enable line that can relax every servo at once.
*   **Attitude sensing:** A **BMI270 IMU** on the SPI bus, read at 100 Hz, feeds a **closed loop that keeps the body level in real time** (see below).
*   **Distance sensing:** An HC-SR04 ultrasonic sensor at the front, plus two spare headers for VL53L1 time-of-flight laser sensors.
*   **Display:** An **OLED screen** animating the robot's "eyes" — expressions, blinking, gaze direction — and doubling as a battery and status indicator.
*   **Battery monitoring:** A resistor divider lets the robot watch its own 3S LiPo and shut the servos down before the pack is damaged.
*   **Phone control:** The robot raises its own Wi-Fi access point and serves a two-joystick control interface, in English or Italian.

## ⚡ Custom Hardware

To ensure stable power delivery to all 12 servos and keep the form factor compact, I designed and assembled a custom PCB.

| Top view | Bottom view |
|:---:|:---:|
| ![PCB top view](https://github.com/user-attachments/assets/6e3345c5-753e-4091-a538-5938418c70bc) | ![PCB bottom view](https://github.com/user-attachments/assets/f13c50d8-3f89-4029-b026-bb6768b39f33) |

### What the board carries

| Subsystem | Part | Interface | Notes |
|---|---|---|---|
| Microcontroller | ESP32-S3-WROOM-1-N8R8 | — | Native USB, no USB-to-serial chip. BOOT and RESET buttons on board. |
| Servo driver | PCA9685 @ `0x40` | I²C bus 0 (GPIO4/5) | 16 channels, 12 used for the legs, 4 free. `~OE` on GPIO1 relaxes every servo in hardware. |
| **IMU** | **BMI270** | **SPI** (CS 10, MOSI 11, SCK 12, MISO 13, INT1 15) | Roll and pitch, 100 Hz. Drives the levelling loop. |
| Display | SSD1306 OLED @ `0x3C` | I²C bus 1 (GPIO17/18) | The eyes. |
| Ultrasonic | HC-SR04 | GPIO21 trigger, GPIO38 echo | Echo level-shifted by an on-board divider. |
| Laser range | 2 × VL53L1 headers @ `0x29` | I²C bus 1 | Wired and ready; not yet used by the firmware. |
| Battery sense | 1 MΩ / 270 kΩ divider | GPIO2 (ADC1) | Reads the 3S pack down to a per-cell voltage. |

### The IMU keeps the robot level, continuously

The BMI270 is not just a tilt read-out — it is **inside the control loop**. Fifty times a second, the firmware:

1. reads roll and pitch from the IMU (fused from accelerometer and gyroscope by a complementary filter at 100 Hz),
2. multiplies them by an adjustable gain and clamps the result,
3. counter-rotates the **body** by that amount through the inverse kinematics, while the feet stay planted.

The result is that if you tilt the surface the spider is standing on, it re-levels its chassis and stays upright. The correction gain is tunable live from the phone, and the whole loop can be switched off with one button. The same sensor also detects a genuine fall: past a threshold angle the robot stops trying to walk, sits down, and waits until it is set upright again.

**The two I²C buses are deliberately separate.** The servo driver sits alone on bus 0 while the display and the laser sensors share bus 1, so redrawing the eyes can never steal bandwidth from the motors.

## 📱 Phone Interface

The robot creates its own Wi-Fi access point — **network `Spider`, password `spider1234`** — and serves the whole interface from flash at **http://192.168.4.1**. There is nothing to install: it is a single self-contained page, so any phone with a browser works, iPhone or Android.

Three screens: **Drive** for the two joysticks and live telemetry, **Poses** for every gesture and body pose, **Settings** for live tuning and the interface language.

| Drive | Poses | Settings |
|:---:|:---:|:---:|
| ![Drive screen](https://github.com/user-attachments/assets/0724fdc5-6d4c-427b-bebe-dfd1dcd5ea98) | ![Poses screen](https://github.com/user-attachments/assets/3ace7da4-c0b3-45b0-a331-06f25c769dc3) | ![Settings screen](https://github.com/user-attachments/assets/dad87398-d8bf-4774-b5a9-5392bf7b3f70) |


*   **Left stick — drive.** Direction and *analogue speed*: the further you push, the faster it walks.
*   **Right stick — gaze.** Up and down tilt the muzzle; left and right yaw the body and carry the eyes' pupils with it, so the robot really does look where you point it.
*   **Live telemetry.** Obstacle distance, current speed, battery voltage and level, loop rate, free RAM, roll and pitch, and the eye animation frame counter.
*   **Gestures and poses.** Giggle, stretch, nod, shake, bow, dance, wave, and every static body attitude — lean, yaw, weight shift, crouch, tiptoe — plus an "all servos to 90°" pose for checking calibration.
*   **Live tuning.** Stride, lift, support time, speed, levelling gain and the two obstacle distances, all adjustable while the robot is running.
*   **Automatic mode.** Left to itself the robot cruises, decelerates proportionally as it approaches an obstacle, scans left and right to pick the clearer side, startles and backs off if something appears right in front of it, and throws in randomised idle poses so it never looks scripted.

## 🎥 Video Demonstration

Check out the spider robot in action!

| Phone control and walking | Earlier build |
|:---:|:---:|
| [![Watch the video](https://img.youtube.com/vi/KWrzHBKVxug/sddefault.jpg)](https://www.youtube.com/watch?v=KWrzHBKVxug) | [![Watch the video](https://img.youtube.com/vi/jzssCQmDDpo/maxresdefault.jpg)](https://www.youtube.com/watch?v=jzssCQmDDpo) |

## 🔧 Firmware

The firmware lives in [`spider_ESP32/spider_wifi_EN/`](spider_ESP32/spider_wifi_EN/) and is split into independent, non-blocking modules:

| File | Responsibility |
|---|---|
| `spider_wifi_EN.ino` | Startup, the main loop, automatic mode, the single command entry point |
| `Legs.*` | Inverse kinematics, gaits (crawl and trot), poses, gestures, body attitude |
| `Attitude.*` | BMI270 driver, complementary filter, fall detection |
| `Eyes.*` | OLED eye animation, running as its own task on the second core |
| `Distance.*` | Interrupt-timed HC-SR04 |
| `Battery.*` | Voltage, level thresholds, low-battery protection |
| `Remote.*`, `page.h` | Wi-Fi access point, web server and the control page |

Nothing in the loop blocks: gaits, gestures and sensors are all state machines, so the robot keeps answering the phone while it walks.

### Building

Arduino IDE with the **esp32** core installed. Select **ESP32S3 Dev Module** and set:

| Setting | Value |
|---|---|
| USB CDC On Boot | **Enabled** (default is Disabled — without it the serial monitor stays silent) |
| Flash Size | **8MB (64Mb)** |
| PSRAM | **OPI PSRAM** |
| Partition Scheme | 8M with SPIFFS |

Or with `arduino-cli`:

```
esp32:esp32:esp32s3:FlashSize=8M,PartitionScheme=default_8MB,PSRAM=opi,CDCOnBoot=cdc
```

Libraries: `Adafruit GFX`, `Adafruit SSD1306`, `Adafruit PWM Servo Driver`, `SparkFun BMI270`.

> **First upload:** there is no auto-reset circuit, so hold **BOOT**, tap **RESET**, release **BOOT**, then select the new COM port. Later uploads reset themselves.

---
*Project created by Marco La Barbera*
