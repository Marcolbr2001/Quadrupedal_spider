#ifndef LEGS_H
#define LEGS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================================================
// === LEGS - NON-BLOCKING MOTION ENGINE ===
// === ESP32-S3-WROOM-1 + PCA9685 ===
// ==================================================
//
// The kinematics of spider_ATMega/spider/spider.ino ported onto the
// PCA9685: no FlexiTimer2, no Servo.h and above all no wait_all_reach().
// The old code sat in a while() at every single waypoint. Here a state
// machine advances pose by pose on each tick, so loop() stays free for the
// eyes, the sensors and bluetooth.
//
// --- WIRING ---
//   PCA9685 (U2) su bus I2C 0 = Wire : SDA GPIO4 (pin 4), SCL GPIO5 (pin 5)
//   Address 0x40 (A0..A5 all grounded)
//   The PCA9685's ~OE on GPIO1 (physical pin 39): LOW = servos live,
//   HIGH = outputs off and the robot goes limp.
//   Servos on J7..J18 = PCA channels 0..11, fed from +5V_Servo.
//
//   WATCH OUT: there is no pull-up resistor on ~OE on this board. Until
//   GPIO1 is configured the pin floats, so begin() drives it HIGH (servos
//   off) as its very first action.
//
// --- CHANNEL MAP ---
//   Leg i, joint j -> PCA channel (i * 3 + j), that is:
//     leg 0 -> J7,  J8,  J9    (channels 0, 1, 2)
//     leg 1 -> J10, J11, J12   (channels 3, 4, 5)
//     leg 2 -> J13, J14, J15   (channels 6, 7, 8)
//     leg 3 -> J16, J17, J18   (channels 9, 10, 11)
//   Same order as pins 2..13 on the ATMega version. If you wired it
//   differently, change CHANNEL[][] in Legs.cpp: nothing else needs
//   touching.

// --- BUS AND PINS ---
#ifndef LEGS_SDA
#define LEGS_SDA 4               // physical pin 4
#endif
#ifndef LEGS_SCL
#define LEGS_SCL 5               // physical pin 5
#endif
#ifndef LEGS_OE
#define LEGS_OE 1                // physical pin 39, the PCA9685's ~OE
#endif
#ifndef LEGS_ADDR
#define LEGS_ADDR 0x40
#endif
#ifndef LEGS_I2C_FREQ
#define LEGS_I2C_FREQ 400000UL
#endif

// Servo PWM frequency. 50 Hz is the safe value for analogue servos. If you
// fit digital ones that take more (100-333 Hz), raise it: the motion gets
// visibly smoother because the servo is corrected more often. Do NOT raise
// it with analogue servos, they overheat.
#ifndef LEGS_PWM_HZ
#define LEGS_PWM_HZ 50
#endif

// How often to recompute the position. No point going faster than the PWM.
#ifndef LEGS_TICK_MS
#define LEGS_TICK_MS 20
#endif

// --- GAITS ---
enum {
  GAIT_STOP = 0,
  GAIT_FORWARD,
  GAIT_BACK,
  GAIT_LEFT,
  GAIT_RIGHT
};

// Step type for forward/backward:
//   CRAWL = one leg at a time, the ATMega version. Slow but always
//           balanced on three feet.
//   TROT  = two opposite legs together. Around four times faster and far
//           more natural, but it balances on two feet only: try it the
//           first time with the robot held off the ground.
enum {
  STEP_CRAWL = 0,
  STEP_TROT
};

class Legs {
public:
  Legs();

  // startBus = false when Wire has already been brought up by someone else.
  bool begin(bool avviaBus = true);
  bool ready() const { return _ready; }

  // Optional: motion runs on a dedicated task (core 1 by default, leaving
  // core 0 to the eyes). After this call update() is no longer needed.
  bool startTask(uint8_t core = 1, uint32_t stack = 4096, UBaseType_t priorita = 2);

  // Call it on every pass of loop() if you are NOT using the task.
  void update();

  // --- COMMANDS (non-blocking: they set the intent and return) ---
  void stand();
  void sit();
  void forward();
  void backward();
  void turnLeft();
  void turnRight();
  void wave();
  // Finishes the step cycle in progress and stops standing. Stopping
  // mid-step would leave the robot on three legs.
  void halt();

  // --- BODY ATTITUDE ---
  // Applied AFTER the gait and BEFORE the kinematics, so it works with any
  // step and while standing still. Angles in degrees, shifts in mm.
  //   roll     + = the right side drops
  //   pitch    + = the nose drops
  //   yaw      + = the body turns to the right
  //   height   + = the body rises
  //   lateral  + = the body shifts to the right
  //   forward  + = the body shifts forwards
  void setAttitude(float roll, float pitch, float yaw = 0,
                  float height = 0, float lateral = 0, float forward = 0);
  void resetAttitude() { setAttitude(0, 0, 0, 0, 0, 0); }

  // --- TROT TUNING ---
  // The trot stands on two legs at a time, so it is not statically
  // balanced: whether it works depends on these numbers, and only the real
  // robot can say what they should be. They are adjustable on the fly.
  //   strideMm    stride length (the crawl uses 80)
  //   liftMm      how far the foot leaves the ground
  //   supportMs   dwell with all four legs down between one diagonal and
  //               the next: that is what gives it its balance back
  void setTrot(float strideMm, float liftMm, uint16_t supportMs);
  float trotStride() const      { return _trStride; }
  float trotLift() const     { return _trLift; }
  uint16_t trotSupport() const { return _trSupport; }

  void setStep(uint8_t tipo);          // STEP_CRAWL / STEP_TROT
  uint8_t step() const { return _step; }
  void setSpeed(float multiplier); // 1.0 = come l'originale

  // Cuts and restores power to the servos through ~OE. Useful to stop them
  // buzzing while idle and to save battery.
  void relax();
  void enable();
  bool enabled() const { return _enabled; }

  uint8_t gait() const { return _gait; }
  bool moving() const { return _seq != nullptr; }

  // --- CALIBRATION CHECK ---
  // Puts all twelve servos exactly at mid-travel, the same as zampe_init:
  // the pose in which the horns should look square. The kinematics stay
  // suspended until endCalibration() is called, and that brings us back
  // with a ramp rather than a snap (from the 90 pose to standing there are
  // tens of degrees per joint).
  void calibrate90();
  void endCalibration();
  bool calibrating() const { return _calActive; }

  // Square stance: all four legs at the same fore-aft distance. The
  // starting pose inherited from the ATMega code is deliberately
  // asymmetric; this one is not.
  void squareStance();

  // --- CALIBRATION (use it with the robot resting on a stand) ---
  // Drives a leg to the raw angles asked for, bypassing the kinematics.
  void rawAngles(int leg, float alpha, float beta, float gamma);
  // Lifts and lowers a leg so you can tell which index is which physical
  // leg: essential before trusting the trot.
  void identifyLeg(int leg);

private:
  Adafruit_PWMServoDriver _pwm;
  bool _ready;
  bool _enabled;
  bool _useTask;
  TaskHandle_t _task;
  portMUX_TYPE _mux;

  // Commands requested from outside
  volatile uint8_t _gaitWanted;
  volatile uint8_t _step;
  volatile float _multiplier;
  volatile bool _waveWanted;

  // Attitude requested from outside (guarded by _mux)
  volatile float _roll, _pitch, _yaw;
  volatile float _height, _lateral, _fwd;

  // The attitude actually applied: it chases the requested one at a bounded
  // rate, so an abrupt command does not become a yank.
  float _rollNow, _pitchNow, _yawNow;
  float _heightNow, _lateralNow, _fwdNow;

  // Motion state
  uint8_t _gait;
  const void *_seq;          // sequenza di pose in corso (nullptr = ferma)
  uint8_t _nSeq;
  uint8_t _iSeq;
  bool _phaseB;               // for the trot: whose turn the diagonal is

  // Interpolation of the current pose
  float _siteNow[4][3];
  float _siteFrom[4][3];
  float _siteTo[4][3];
  unsigned long _tPose;
  unsigned long _poseTime;

  unsigned long _tTick;
  int _usWritten[16];        // last value mandato, per non riscriverlo

  // Blends between the 90-degree pose (0) and the real kinematics (1).
  bool _calActive;
  float _calBlend;

  void _tick();
  void _startSequence(const void *seq, uint8_t n);
  void _nextPose();
  void _chooseSequence();
  // Trot parameters and tables, rebuilt whenever they change.
  float _trStride, _trLift;
  uint16_t _trSupport;
  bool _trRebuild;
  volatile uint8_t _trStance;   // 0 = da fare, 1 = pronta forward, 2 = backward

  void _applyPose(const float p[4][3], float speed, float attesaMs);
  void _buildTrot();
  void _applyAttitude(const float dentro[4][3], float fuori[4][3]);
  void _writeServos();
  static void _taskLoop(void *param);
};

#endif // LEGS_H
