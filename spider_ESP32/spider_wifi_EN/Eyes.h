#ifndef EYES_H
#define EYES_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================================================
// === EYES - NON-BLOCKING EYE ENGINE ===
// === ESP32-S3-WROOM-1 / SSD1306 128x64 ===
// ==================================================
//
// A module ready to be merged into the full firmware: no delay(), no wait
// loops. You ask for an expression and go straight back to your own work;
// the animation carries on by itself.
//
// --- WIRING (connector J3 on the PCB, B4B-XH-A 4 pin) ---
//   J3.1 -> GND
//   J3.2 -> +3.3V
//   J3.3 -> SCL2  = GPIO18 (physical pin 11)
//   J3.4 -> SDA2  = GPIO17 (physical pin 10)
//
// --- THE BOARD'S TWO I2C BUSES ---
//   BUS 0 (Wire)  -> SDA1/SCL1 = GPIO4 / GPIO5    -> PCA9685 (servos)
//   BUS 1 (Wire1) -> SDA2/SCL2 = GPIO17 / GPIO18  -> OLED (J3) + laser VL53L1 (J5, J6)
// The display sits on Wire1 and never steals bandwidth from the servo
// driver. Addresses on bus 1: OLED 0x3C, VL53L1 0x29 -> no clash.
// 4.7k pull-ups are already on the PCB (R27/R28).
//
// --- TWO WAYS TO USE IT ---
//   A) update() inside loop():  eyes.update();  every pass. Simple.
//      A frame that touches the display costs ~25 ms of I2C, so now and
//      then the loop stretches (only during transitions, see below).
//   B) startTask():  the eyes run on a FreeRTOS task pinned to core 0,
//      while loop() (gait, servos, sensors) stays on core 1 and NEVER
//      stalls. This is the recommended way once merged into the firmware.
//      TwoWire on the ESP32 already holds a per-bus mutex, so concurrent
//      access to Wire1 (eyes on core 0 + lasers on core 1) is safe.
//
// Either way the display is refreshed ONLY when the image really changes:
// with the eyes still, I2C traffic is exactly zero.

// --- CONFIGURATION ---
// Change them here (Eyes.cpp is compiled separately, so redefining them
// inside the .ino would have no effect), or pass them to the compiler with
// -D if you use PlatformIO.
#ifndef EYES_SDA
#define EYES_SDA 17              // physical pin 10
#endif
#ifndef EYES_SCL
#define EYES_SCL 18              // physical pin 11
#endif
#ifndef EYES_I2C_FREQ
#define EYES_I2C_FREQ 400000UL
#endif
#ifndef EYES_ADDR
#define EYES_ADDR 0x3C
#endif
#ifndef EYES_FRAME_MS
#define EYES_FRAME_MS 33         // ~30 fps durante le animazioni
#endif

// Display orientation: 0 = normal, 2 = upside down by 180 degrees.
// On this robot the panel is mounted the other way up for mechanical
// reasons.
#ifndef EYES_ROTATION
#define EYES_ROTATION 2
#endif

// Height of the differently coloured band on yellow/blue panels, in pixels
// (0 if your display is monochrome). It keeps the eyes entirely inside the
// blue area: the band is always on the same physical edge of the glass, so
// rotating by 180 degrees moves it from top to bottom and the eyes have to
// move up accordingly. The arithmetic in Eyes.cpp handles it.
#ifndef EYES_YELLOW_BAND
#define EYES_YELLOW_BAND 16
#endif

// --- EXPRESSIONS (same "mood" codes as the ATMega version) ---
enum {
  EYES_NEUTRAL     = 0,
  EYES_ANGRY = 1,
  EYES_HAPPY     = 2,
  EYES_SLEEPY      = 3,
  EYES_PUZZLED    = 4,
  EYES_SCARED  = 5
};

class Eyes {
public:
  explicit Eyes(TwoWire *bus = &Wire1);

  // --- STARTUP ---
  // startBus = false when Wire1 has already been brought up by someone else
  // (the VL53L1 init, for instance: they sit on the same bus).
  bool begin(bool avviaBus = true);
  bool ready() const { return _ready; }

  // How many frames were actually pushed to the display. If it does not
  // climb, the eye task is not running; if it climbs but the screen stays
  // black, the problem is the display or the bus, not the software.
  uint32_t frames() const { return _frames; }

  // Way B: move the animation onto a dedicated task.
  // Call it AFTER begin(). From then on update() is no longer needed.
  bool startTask(uint8_t core = 0, uint32_t stack = 4096, UBaseType_t priorita = 1);

  // Way A: call it on every pass of loop(). Returns at once if the next
  // frame is not due yet. Harmless if you are using the task.
  void update();

  // --- COMMANDS (all non-blocking: they set and return) ---
  // instant = true skips the smooth transition and snaps straight into the
  // new pose: what a startle reaction needs (a sudden obstacle).
  void setExpression(uint8_t expression, bool instant = false);
  uint8_t expression() const { return _moodWanted; }

  void look(int dx, int dy = 0);      // sguardo in pixel, circa -15..+15
  void blink();                // un ammicco singolo
  void setAutoBlink(bool enabled); // ammicco spontaneo (enabled di default)

  // True while a transition or a blink is running: useful if you want to
  // let one expression finish before starting another.
  bool moving() const;

  // --- SHORTCUTS (same names as the ATMega version) ---
  void faceNormal()    { setExpression(EYES_NEUTRAL); }
  void faceAngry() { setExpression(EYES_ANGRY); }
  void faceHappy()     { setExpression(EYES_HAPPY); }
  void facePuzzled()    { setExpression(EYES_PUZZLED); }
  void faceScared()  { setExpression(EYES_SCARED); }
  void sleepy()            { setExpression(EYES_SLEEPY); }
  void wakeUp()          { setExpression(EYES_NEUTRAL); }
  void lookRight()     { look(15); }
  void lookLeft()   { look(-15); }
  void lookAhead()     { look(0); }

  // --- BATTERY ---
  // The icon sits bottom left, inside the panel's yellow band: out of the
  // eyes' way and still readable.
  //   level 0 full, 1 half, 2 low, 3 critical
  // At level 2 it blinks even when the display is switched off, and at
  // level 3 it takes the whole screen: a pack that is being damaged has to
  // make itself noticed, not wait for someone to ask.
  void setBattery(uint8_t level, int percent, float volt);
  void setShowBattery(bool mostra);
  bool showBattery() const { return _battShow; }

  // --- DIAGNOSTICS ---
  // Lists on the serial port whatever answers on the display's bus.
  void scanBus();

private:
  TwoWire *_bus;
  Adafruit_SSD1306 _display;
  bool _ready;
  bool _useTask;
  TaskHandle_t _task;
  portMUX_TYPE _mux;

  // State requested from outside (guarded by _mux)
  volatile uint8_t _moodWanted;
  volatile int _xWanted;
  volatile int _yWanted;
  volatile bool _blinkWanted;
  volatile bool _blinkAuto;
  volatile bool _instant;

  // Animated state: touched only by _animateFrame()
  uint8_t _mood;
  uint8_t _phase;            // 0 = stabile, 1 = uscita, 2 = entrata
  int _xOff, _yOff;
  int _topH, _botH;
  int _blinkH;
  uint8_t _blinkPhase;       // 0 = idle, 1 = closing, 2 = opening
  unsigned long _tFrame;
  unsigned long _tBlink;

  // Last frame sent to the display, so an identical one is not resent
  volatile uint8_t _battLevel;
  volatile int _battPerc;
  volatile int _battMilliVolt;     // in mV, per non tenere un float volatile
  volatile bool _battShow;
  bool _pBattBlink;

  volatile uint32_t _frames;
  bool _neverDrawn;
  int _pX, _pY, _pTop, _pBot, _pBlink;
  uint8_t _pBattLevel;
  int _pBattPerc;
  bool _pBattShow;
  uint8_t _pMood;

  void _animateFrame();
  void _draw(int xOff, int yOff, int topH, int botH, int blinkH, uint8_t mood);
  void _drawBatteryIcon();
  void _drawPlugIcon();
  void _drawBigBattery();
  void _scheduleBlink();
  static void _taskLoop(void *param);
  static int _approach(int value, int target, int step);
};

#endif // EYES_H
