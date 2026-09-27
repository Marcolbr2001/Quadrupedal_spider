#include "Battery.h"

// What the voltage at the pin must be multiplied by to get back to the
// pack's: (R29 + R30) / R30 = 4.70 with the values on this board.
static const float FACTOR = (BATT_R_TOP + BATT_R_BOTTOM) / BATT_R_BOTTOM;

Battery::Battery()
  : _volt(0),
    _level(BATT_LVL_FULL),
    _critical(false),
    _confirms(0),
    _tReading(0) {}

bool Battery::begin() {
  // 12 dB of attenuation: full scale reaches about 3.1 V, and 2.7 is all
  // we need even with a full pack.
  analogSetPinAttenuation(BATT_PIN, ADC_11db);

  // A full unfiltered first reading, so at power-on the value is already
  // the real one instead of creeping up to it from zero.
  long sum = 0;
  for (int i = 0; i < BATT_SAMPLES; i++) sum += analogReadMilliVolts(BATT_PIN);
  _volt = (sum / (float)BATT_SAMPLES) / 1000.0f * FACTOR;
  _tReading = millis();

  if (_volt < BATT_V_ABSENT) {
    _level = BATT_LVL_USB;
    Serial.printf("BATTERY: %.2f V, no pack connected: running on cable.\n", _volt);
    return false;
  }
  Serial.printf("BATTERY: %.2f V (%d%%)\n", _volt, percent());
  return true;
}

void Battery::update() {
  if (millis() - _tReading < BATT_PERIOD_MS) return;
  _tReading = millis();

  long sum = 0;
  for (int i = 0; i < BATT_SAMPLES; i++) sum += analogReadMilliVolts(BATT_PIN);
  float v = (sum / (float)BATT_SAMPLES) / 1000.0f * FACTOR;

  // Battery just plugged in while the robot was running on the cable: we
  // restart from the true value instead of letting the filter creep up to
  // it. Without this, the climb from 0 to 12 V would pass through the
  // "critical" band and three seconds later the alarm would fire on a full
  // pack.
  if (_level == BATT_LVL_USB && v >= BATT_V_ABSENT) _volt = v;
  else _volt = _volt + BATT_FILTER * (v - _volt);   // exponential filter:
                                                    // under the servos the
                                                    // voltage swings a lot

  // First of all: is there a pack? If the +12V line is unconnected the pin
  // sits at ground and any threshold would be a false emergency.
  if (_volt < BATT_V_ABSENT) {
    _level = BATT_LVL_USB;
    _confirms = 0;
    return;
  }

  if      (_volt >= BATT_V_FULL) _level = BATT_LVL_FULL;
  else if (_volt >= BATT_V_HALF) _level = BATT_LVL_HALF;
  else if (_volt >= BATT_V_LOW) _level = BATT_LVL_LOW;
  else                            _level = BATT_LVL_CRITICAL;

  // The critical level needs confirming: a single collapse is most likely
  // the servos' inrush, not a flat pack.
  if (_level == BATT_LVL_CRITICAL) {
    if (_confirms < BATT_CONFIRMS) _confirms++;
    if (_confirms >= BATT_CONFIRMS && !_critical) {
      _critical = true;
      Serial.printf("BATTERY: CRITICAL at %.2f V, releasing the servos.\n", _volt);
    }
  } else {
    _confirms = 0;
  }
}

int Battery::percent() const {
  if (_level == BATT_LVL_USB) return 0;   // meaningless on the cable
  float p = (_volt - BATT_V_EMPTY) / (BATT_V_CHARGED - BATT_V_EMPTY) * 100.0f;
  return (int)constrain(p, 0.0f, 100.0f);
}

void Battery::clearCritical() {
  _critical = false;
  _confirms = 0;
  Serial.println("BATTERY: alarm cleared by hand.");
}
