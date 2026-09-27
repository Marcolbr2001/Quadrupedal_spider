#include "Distance.h"

// Microseconds of echo per centimetre, there and back. It is the same
// US_ROUNDTRIP_CM the NewPing library used on the ATMega, so the distances
// come out identical to before.
static const unsigned long US_PER_CM = 57;

// If no echo has arrived by now, the way is clear.
// DIST_MAX_CM * 57 us, plus some margin for the sensor's own turnaround.
static const unsigned long TIMEOUT_US = (unsigned long)DIST_MAX_CM * US_PER_CM + 5000UL;

// --- State shared with the interrupt ---
// The ISR touches only these: all volatile and one word wide.
static volatile unsigned long isrRise = 0;
static volatile unsigned long isrWidth = 0;
static volatile bool isrReady = false;

// Times the echo: rising edge starts the clock, falling edge stops it.
// IRAM_ATTR keeps the function in internal RAM, which is mandatory for an
// ISR that must be able to fire while the flash is busy.
static void IRAM_ATTR isrEcho() {
  if (digitalRead(DIST_ECHO)) {
    isrRise = micros();
  } else if (isrRise != 0) {
    isrWidth = micros() - isrRise;
    isrRise = 0;
    isrReady = true;
  }
}

Distance::Distance()
  : _cm(DIST_FAR_CM),
    _tPing(0),
    _tReading(0),
    _waiting(false),
    _timeout(0) {}

bool Distance::begin() {
  pinMode(DIST_TRIG, OUTPUT);
  digitalWrite(DIST_TRIG, LOW);
  pinMode(DIST_ECHO, INPUT);

  attachInterrupt(digitalPinToInterrupt(DIST_ECHO), isrEcho, CHANGE);

  _cm = DIST_FAR_CM;
  _tPing = millis();
  Serial.printf("DIST: HC-SR04 on TRIG=GPIO%d ECHO=GPIO%d, one ping every %d ms.\n",
                DIST_TRIG, DIST_ECHO, DIST_PERIOD_MS);
  return true;
}

void Distance::_fireTrigger() {
  // A 10 us pulse: the only wait in the whole module, and it is measured
  // in microseconds, so it does not even bother the servos.
  digitalWrite(DIST_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(DIST_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(DIST_TRIG, LOW);

  _tPing = micros();
  _waiting = true;
}

void Distance::update() {
  // 1. Did an echo come back? Turn it into centimetres.
  if (isrReady) {
    unsigned long width;
    noInterrupts();
    width = isrWidth;
    isrReady = false;
    interrupts();

    _waiting = false;
    _timeout = 0;

    int reading = (int)(width / US_PER_CM);
    // Same filter as the ATMega version: zero and out-of-range both mean
    // "all clear", so a lost echo cannot raise a false alarm.
    _cm = (reading > 0 && reading <= DIST_MAX_CM) ? reading : DIST_FAR_CM;
    _tReading = millis();
  }

  // 2. No echo at all: nothing ahead, or the sensor is unplugged.
  if (_waiting && (micros() - _tPing) > TIMEOUT_US) {
    _waiting = false;
    _cm = DIST_FAR_CM;
    if (_timeout < 65535) _timeout++;
  }

  // 3. Time for the next ping?
  if (!_waiting && (micros() - _tPing) >= (unsigned long)DIST_PERIOD_MS * 1000UL) {
    _fireTrigger();
  }
}
