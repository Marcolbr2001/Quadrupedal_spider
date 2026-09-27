#include "Attitude.h"

static const float RAD = 180.0f / 3.14159265f;

Attitude::Attitude()
  : _ready(false),
    _tReading(0),
    _roll(0), _pitch(0),
    _rollAcc(0), _pitchAcc(0),
    _zeroRoll(0), _zeroPitch(0) {}

bool Attitude::begin() {
  // Same syntax as esp32_gyro_accel: SPI.begin(SCK, MISO, MOSI, CS) with
  // CS at -1, because the sensor library handles it itself.
  SPI.begin(IMU_SCK, IMU_MISO, IMU_MOSI, -1);

  int8_t status = _imu.beginSPI(IMU_CS);
  if (status != BMI2_OK) {
    Serial.printf("ATTITUDE: BMI270 not found (code %d).\n", status);
    Serial.println("  Check the solder joints and the 3.3 V rail. Carrying on without the IMU.");
    return false;
  }

  _ready = true;
  _tReading = millis();

  // A first read so we start from sensible angles instead of from zero.
  _imu.getSensorData();
  update();
  _roll = _rollAcc;
  _pitch = _pitchAcc;

  Serial.println("ATTITUDE: BMI270 ready.");
  return true;
}

void Attitude::update() {
  if (!_ready) return;

  unsigned long ora = millis();
  unsigned long dtMs = ora - _tReading;
  if (dtMs < IMU_PERIOD_MS) return;
  _tReading = ora;
  float dt = dtMs / 1000.0f;

  _imu.getSensorData();

  // From the sensor's axes to the robot's (right, forward, up).
  float a[3] = { _imu.data.accelX, _imu.data.accelY, _imu.data.accelZ };
  float g[3] = { _imu.data.gyroX,  _imu.data.gyroY,  _imu.data.gyroZ  };

  float aRight = IMU_SGN_RIGHT * a[IMU_IDX_RIGHT];
  float aFwd = IMU_SGN_FWD * a[IMU_IDX_FWD];
  float aUp = IMU_SGN_UP     * a[IMU_IDX_UP];

  // Gravity points down: if the right side drops, some of it lands on the
  // "right" axis and aRight grows. Same idea for the nose.
  _rollAcc     = atan2(aRight, aUp) * RAD;
  _pitchAcc = atan2(aFwd, aUp) * RAD;

  // Angular rates, in degrees per second, about those same axes: roll
  // turns about the forward axis, pitch about the right one.
  float gRoll     = IMU_SGN_GYRO_ROLL     * g[IMU_IDX_FWD];
  float gPitch = IMU_SGN_GYRO_PITCH * g[IMU_IDX_RIGHT];

  // Complementary filter: the gyro carries the angle forward instant by
  // instant, the accelerometer slowly pulls it back to true vertical.
  _roll     = IMU_ALPHA * (_roll     + gRoll     * dt) + (1 - IMU_ALPHA) * _rollAcc;
  _pitch = IMU_ALPHA * (_pitch + gPitch * dt) + (1 - IMU_ALPHA) * _pitchAcc;
}

void Attitude::zero() {
  if (!_ready) return;
  _zeroRoll = _roll;
  _zeroPitch = _pitch;
  Serial.printf("ATTITUDE: zero taken (roll %.1f, pitch %.1f).\n",
                _zeroRoll, _zeroPitch);
}

bool Attitude::tipped(float degrees) const {
  if (!_ready) return false;
  return fabs(roll()) > degrees || fabs(pitch()) > degrees;
}
