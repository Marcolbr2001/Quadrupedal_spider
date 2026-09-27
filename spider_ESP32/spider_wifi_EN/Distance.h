#ifndef DISTANCE_H
#define DISTANCE_H

#include <Arduino.h>

// ==================================================
// === DISTANCE - NON-BLOCKING HC-SR04 ===
// === ESP32-S3-WROOM-1 ===
// ==================================================
//
// No pulseIn(): that call sits and waits for the echo for up to 23 ms,
// which on this robot means losing a whole servo window. Here the TRIG is
// fired every 50 ms (a 10 us pulse, negligible) and the echo is timed by an
// interrupt on both edges, so loop() never waits for anybody.
//
// --- WIRING (connector J4 on the PCB, B4B-XH-A 4 pin) ---
//   J4.1 -> +5V_Board   (the HC-SR04 wants 5 V, not 3.3 V)
//   J4.2 -> TRIG  = GPIO21 (physical pin 23)
//   J4.3 -> ECHO  = 5 V, through the R25/R26 divider
//   J4.4 -> GND
//
// The 5 V echo does NOT reach the micro directly: a 10k (R25) / 20k (R26)
// divider on the board brings it to 5 * 20/30 = 3.33 V before it enters
// GPIO38 (physical pin 31). TRIG goes out at 3.3 V and the HC-SR04 is
// perfectly happy with that.

#ifndef DIST_TRIG
#define DIST_TRIG 21              // physical pin 23
#endif
#ifndef DIST_ECHO
#define DIST_ECHO 38              // physical pin 31, after the divider
#endif

// How often to fire a ping. 50 ms, same as the ATMega version.
#ifndef DIST_PERIOD_MS
#define DIST_PERIOD_MS 50
#endif

// Beyond this we do not care, and the echo would not come back anyway.
#ifndef DIST_MAX_CM
#define DIST_MAX_CM 400
#endif

// Reported when no echo comes back: "all clear", the same thing the
// original did by setting currentDist = 100 when NewPing returned 0.
#ifndef DIST_FAR_CM
#define DIST_FAR_CM 100
#endif

class Distance {
public:
  Distance();

  bool begin();

  // Call it on every pass of loop(): it fires the ping when due and picks
  // up whatever the interrupt measured. It always returns immediately.
  void update();

  // Last known distance in centimetres. Never blocks and never waits: if
  // the echo has not come back yet you get the last good reading.
  int cm() const { return _cm; }

  bool near(int sogliaCm) const { return _cm > 0 && _cm < sogliaCm; }

  // millis() of the last successful reading (0 = never).
  unsigned long lastReading() const { return _tReading; }

  // How many readings timed out in a row. If it only ever grows, the
  // sensor is unplugged or has no 5 V.
  uint16_t timeouts() const { return _timeout; }

private:
  int _cm;
  unsigned long _tPing;       // when the last TRIG went out
  unsigned long _tReading;
  bool _waiting;             // TRIG fired, echo not back yet
  uint16_t _timeout;

  void _fireTrigger();
};

#endif // DISTANCE_H
