#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// ==================================================
// === BATTERY - READING THE 3S PACK ===
// === ESP32-S3-WROOM-1 ===
// ==================================================
//
// The 12 V pack reaches the micro through the R29 / R30 divider
// (1 M / 270 k), which divides it by 4.70: at 12.6 V of battery the pin
// sees 2.68 V, comfortably inside the ADC's range.
//
//   +12V_Battery --[ R29 1M ]--+-- Battery_LVL -> GPIO2 (physical pin 38)
//                              |
//                          [ R30 270k ]   [ C21 100nF ]
//                              |               |
//                             GND             GND
//
// The divider has a high impedance (212 k seen from the pin) and the
// ESP32's ADC would like far less to sample cleanly. Capacitor C21 on the
// pin acts as a reservoir and fixes most of it; averaging several samples
// and the filter below take care of the rest.
//
// GPIO2 is on ADC1, the only one that keeps working with WiFi running.

#ifndef BATT_PIN
#define BATT_PIN 2                 // physical pin 38
#endif
#ifndef BATT_R_TOP
#define BATT_R_TOP 1000000.0f     // R29
#endif
#ifndef BATT_R_BOTTOM
#define BATT_R_BOTTOM 270000.0f     // R30
#endif

#ifndef BATT_PERIOD_MS
#define BATT_PERIOD_MS 500
#endif
#ifndef BATT_SAMPLES
#define BATT_SAMPLES 16           // averaged on each reading, against noise
#endif
#ifndef BATT_FILTER
#define BATT_FILTER 0.15f          // how much the new reading weighs
#endif

// Thresholds for a 3S pack: 4.2 V/cell when full, 3.5 V/cell the floor you
// should not go under in discharge if you want to keep it healthy.
#ifndef BATT_V_FULL
#define BATT_V_FULL 11.8f         // 3.93 V/cell
#endif
#ifndef BATT_V_HALF
#define BATT_V_HALF 11.0f         // 3.67 V/cell
#endif
#ifndef BATT_V_LOW
#define BATT_V_LOW 10.4f          // 3.47 V/cell: below this everything drops out
#endif
#ifndef BATT_V_EMPTY
#define BATT_V_EMPTY 10.2f         // reference for the percentage
#endif

// Below this voltage the pack is not flat: it is simply not there. Running
// the board from the USB cable alone leaves the +12V line unconnected, R30
// pulls the pin to ground and the ADC reads nearly zero. Without this check
// the robot would go into safe mode every time you plug it into a computer.
// A genuinely flat 3S never gets this low: under 9 V the cell protection
// has already cut everything off.
#ifndef BATT_V_ABSENT
#define BATT_V_ABSENT 5.0f
#endif
#ifndef BATT_V_CHARGED
#define BATT_V_CHARGED 12.6f
#endif

// How many critical readings in a row before believing it. Twelve servos
// starting together collapse the voltage for a moment: without this filter
// the robot would shut down on its first step.
#ifndef BATT_CONFIRMS
#define BATT_CONFIRMS 6            // 6 x 500 ms = 3 seconds
#endif

enum {
  BATT_LVL_FULL = 0,
  BATT_LVL_HALF,
  BATT_LVL_LOW,
  BATT_LVL_CRITICAL,
  BATT_LVL_USB        // no pack: we are running off the cable
};

class Battery {
public:
  Battery();

  bool begin();
  void update();                          // non-blocking

  float volt() const      { return _volt; }
  int percent() const;
  uint8_t level() const { return _level; }

  // True when the board runs off the cable and there is no battery. The
  // voltage then means nothing, nothing goes into safe mode, and all that
  // is shown is the plug.
  bool onUsb() const      { return _level == BATT_LVL_USB; }

  // Once tripped it does not clear itself: releasing the servos lets the
  // voltage recover and the robot would set off only to collapse again. The
  // only way out is clearCritical(), i.e. after changing or recharging.
  bool critical() const    { return _critical; }
  void clearCritical();

private:
  float _volt;
  uint8_t _level;
  bool _critical;
  uint8_t _confirms;
  unsigned long _tReading;
};

#endif // BATTERY_H
