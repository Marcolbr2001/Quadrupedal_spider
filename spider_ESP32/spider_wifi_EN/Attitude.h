#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <Arduino.h>
#include <SPI.h>
#include <SparkFun_BMI270_Arduino_Library.h>

// ==================================================
// === ATTITUDE - NON-BLOCKING BMI270 ===
// === ESP32-S3-WROOM-1 ===
// ==================================================
//
// Tells how the body is sitting: roll (side down) and pitch (nose down),
// in degrees. Used to keep the chassis level on a floor that is not, and to
// give the trot the weight transfer it lacks.
//
// Fuses accelerometer and gyro with a complementary filter: the
// accelerometer alone tells the truth but shakes on every step, the gyro
// alone is smooth but drifts. Together they work.
//
// --- WIRING (SPI, see PINOUT.md) ---
//   CS   GPIO10 (pin 18)
//   MOSI GPIO11 (pin 19)
//   SCK  GPIO12 (pin 20)
//   MISO GPIO13 (pin 21)
//   INT1 GPIO15 (pin 8), not used yet

#ifndef IMU_CS
#define IMU_CS   10
#endif
#ifndef IMU_MOSI
#define IMU_MOSI 11
#endif
#ifndef IMU_SCK
#define IMU_SCK  12
#endif
#ifndef IMU_MISO
#define IMU_MISO 13
#endif

// How often to read the sensor.
#ifndef IMU_PERIOD_MS
#define IMU_PERIOD_MS 10         // 100 Hz
#endif

// How much the gyro weighs in the complementary filter. Higher = smoother
// but slower to correct drift; lower = more responsive but jumpier while
// walking.
#ifndef IMU_ALPHA
#define IMU_ALPHA 0.98f
#endif

// --- AXIS MAPPING ---
// Index 0 = sensor X, 1 = Y, 2 = Z. The sign straightens the direction.
// Measured on the robot with the check mode: dropping the right side made
// the sensor's Y axis rise, and dropping the nose made X fall. So Y points
// right and X points backwards: forward is -X.
#ifndef IMU_IDX_RIGHT
#define IMU_IDX_RIGHT 1          // sensor Y -> robot's right
#define IMU_SGN_RIGHT (+1.0f)
#endif
#ifndef IMU_IDX_FWD
#define IMU_IDX_FWD 0            // sensor X -> backwards, so -X is forward
#define IMU_SGN_FWD (-1.0f)
#endif
#ifndef IMU_IDX_UP
#define IMU_IDX_UP 2              // sensor Z -> up
#define IMU_SGN_UP (+1.0f)
#endif

// Gyro senses. There are TWO and they are independent: the sense of a
// rotation does not follow the accelerometer's on the same axis, because it
// comes from the right-hand rule about that axis, not from the direction
// the axis points. With this mapping they end up opposite to each other.
// To check: tilt the robot slowly and watch the two printed columns. If the
// fused value chases the accelerometer one, the sense is right; if it runs
// off the wrong way, flip the sign of THAT axis only.
#ifndef IMU_SGN_GYRO_ROLL
#define IMU_SGN_GYRO_ROLL (+1.0f)
#endif
#ifndef IMU_SGN_GYRO_PITCH
#define IMU_SGN_GYRO_PITCH (-1.0f)
#endif

class Attitude {
public:
  Attitude();

  bool begin();
  bool ready() const { return _ready; }

  // Call it on every pass of loop(): it reads only when due and never
  // waits for anybody.
  void update();

  // Degrees. roll + = right side down, pitch + = nose down. Already net of
  // the zero taken with zero().
  float roll() const     { return _roll - _zeroRoll; }
  float pitch() const { return _pitch - _zeroPitch; }

  // Raw accelerometer-only angles: used to check the axis mapping and the
  // gyro senses.
  float rollAccel() const     { return _rollAcc; }
  float pitchAccel() const { return _pitchAcc; }

  // Takes the current position as "level". Call it with the robot still
  // and standing on a flat surface: the sensor is never perfectly aligned
  // with the chassis, and this cancels the fixed error.
  void zero();

  // True when the robot is tilted past the threshold: useful for noticing
  // it is about to topple and sitting down instead of pressing on.
  bool tipped(float degrees = 45.0f) const;

  BMI270 &sensor() { return _imu; }

private:
  BMI270 _imu;
  bool _ready;
  unsigned long _tReading;
  float _roll, _pitch;           // fused
  float _rollAcc, _pitchAcc;     // accelerometer only
  float _zeroRoll, _zeroPitch;
};

#endif // ATTITUDE_H
