#include "Eyes.h"
#include "Distance.h"
#include "Legs.h"
#include "Attitude.h"
#include "Battery.h"
#include "Remote.h"

// ==================================================
// === SPIDER - FULL FIRMWARE ===
// === ESP32-S3-WROOM-1 ===
// ==================================================
//
// Brings together the modules tested one by one in single_parts/ESP32:
//
//   Eyes      OLED display on I2C bus 1 (J3), animated on core 0
//   Distance  HC-SR04 on J4, interrupt-timed reading every 50 ms
//   Legs      12 servos via PCA9685 on I2C bus 0, gaits on core 1
//   Attitude  BMI270 on SPI, keeps the chassis level
//
// Nothing blocks and nothing treads on anything else:
//   - the servos are on I2C bus 0, display and lasers on bus 1
//   - the eyes run on core 0, the legs on core 1
//   - TwoWire holds a per-bus mutex, so crossing access is safe
//
// The behaviour is that of the ATMega version (spider_ATMega/spider), but
// without the waiting: if something is close the spider takes fright and
// backs off, if it has room it explores, if the distance is right it rests.
//
// Every command goes through runCommand(), called both from the serial
// port and from the WiFi remote: two roads, a single entry point.
//
// --- REMOTE ---
// The spider brings up the "Spider" network (password spider1234). Join it
// with the phone and open http://192.168.4.1 . Two joysticks:
//   left   drive: forward, back, turn. The further you push, the faster.
//   right  gaze: up and down raise and lower the muzzle, right and left
//          yaw the body and move the pupils with it.

// --- AUTOMATIC MODE: distance thresholds ---
// Three bands, not one: under 12 cm it is a startle, under 25 cm it stops
// and looks around, and between 30 and 50 it simply slows down. That way
// the spider reacts gradually instead of jumping straight from "walk" to
// "run away".
#define D_PANIC_CM      12   // comparsa improvvisa: soprassalto e fuga
#define D_SCAN_CM   25   // ordinary obstacle: stop and look to the sides
#define D_SLOW_CM       30   // below this it goes at minimum speed
#define D_CLEAR_CM      50   // sopra questa va a tutta

// --- AUTOMATIC MODE: speeds ---
#define VEL_MIN          0.8  // gait guardinga
#define VEL_MAX          2.0  // gait di crociera

// --- AUTOMATIC MODE: scanning ---
#define SCAN_TURN_MS   800  // quanto gira per sbirciare di lato
#define SCAN_SETTLE_MS   450  // dwell prima di leggere, per non misurare mentre balla
#define SCAN_SAMPLES    5    // letture da mediare
#define SCAN_MARGIN_CM  5    // below this difference the two sides are a draw

// --- AUTOMATIC MODE: fleeing ---
#define FLEE_BACK_MS 1300
#define FLEE_TURN_MIN_MS 1000
#define FLEE_TURN_MAX_MS 2200

// --- AUTOMATIC MODE: whims ---
// Now and then, when it has room, it does something unnecessary. That is
// what separates a robot on patrol from a little animal pottering about.
#define WHIM_MIN_MS  7000
#define WHIM_MAX_MS 15000

// --- GAIT ---
#define STEP_INITIAL   STEP_TROT
#define SPEED         2.0
#define TROT_STRIDE     70.0     // valori trovati provando sul ragno
#define TROT_LIFT    18.0
#define TROT_SUPPORT  100

// Which core runs the eye animation.
// Core 0 also carries WiFi and the network stack; core 1 has loop() and the
// leg task (which only uses about 15 per cent of the time). A full frame
// costs ~25 ms of I2C, so on core 1 it slows loop() and therefore the
// server a little, while on core 0 it risks fighting the WiFi.
#define CORE_EYES 1

// --- TELECOMANDO ---
#define STICK_DEADZONE      18     // below this push the stick counts as centred
#define STICK_PITCH 18.0   // degrees di muso su/giu' a fondo corsa
#define STICK_YAW  18.0   // degrees of body yaw at full deflection
#define STICK_PUPILS    15     // pixels of pupil travel

// --- LEVELLING ---
#define LEVEL_GAIN 0.6
#define MAX_CORRECTION   12.0
#define FALL_DEGREES    40.0     // oltre questa inclinazione si siede

// Declared up here rather than down with the gestures: the Arduino IDE
// generates the function prototypes itself and puts them at the top of the
// file, so a type used as a parameter must already be known by then.
struct Gesture {
  float roll, pitch, yaw, height, lateral, forward;
  uint16_t durationMs;
  int8_t expression;      // -1 = leave the eyes as they are
};

Eyes eyes;    // OLED  (Wire1, J3)
Distance  sonar;    // HC-SR04 (J4)
Legs     legs;    // servi (Wire, PCA9685)
Attitude   imu;      // BMI270 (SPI)
Battery  batt;     // partitore R29/R30 su GPIO2
Remote remote;   // rete WiFi + pagina di comando

// --- AUTOMATIC MODE STATES ---
enum {
  AUTO_START,            // wave iniziale
  AUTO_CRUISE,         // walks, with speed tied to distance
  AUTO_WHIM,        // still, putting on a little show
  AUTO_TURN_AND_GO,   // gira per un tempo dato, poi riprende a camminare
  AUTO_SCAN_R,          // sbircia a right
  AUTO_READ_R,         // still, reading to the right
  AUTO_SCAN_L,          // sbircia a left
  AUTO_READ_L,         // still, reading to the left
  AUTO_STARTLE,      // spavento: pose e eyes
  AUTO_FLEE_BACK,    // si allontana in retromarcia
  AUTO_FALLEN            // troppo inclinato: si siede
};

uint8_t status = AUTO_START;
unsigned long tState = 0;
unsigned long phaseTime = 0;        // for the states with a variable duration
unsigned long tNextWhim = 0;
int distR = 0, distL = 0;          // scan readings
float speedNow = -1;              // ultima speed' mandata alle legs

// Adjustable from the Settings page: they used to be #defines, now they
// are variables so a slider can move them without recompiling.
float speedMax    = VEL_MAX;
float gain  = LEVEL_GAIN;
int   dPanic   = D_PANIC_CM;
int   dScan = D_SCAN_CM;
bool levelling = true;
// It starts in manual, not automatic, so the spider does not wander off
// while you are still getting your phone out.
bool automatic = false;
// True when the body is being held in a pose chosen by hand or by a
// gesture: levelling then stands aside, otherwise it would rewrite the
// attitude fifty times a second and the pose would never be seen.
bool manualPose = false;

void setup() {
  Serial.begin(115200);
  // With the native CDC, if the monitor is not open a write can sit
  // waiting for someone to drain the buffer, and every printf becomes a
  // brake on the whole loop. At zero the prints are dropped instead of
  // blocking, which is what you want when the spider runs on battery.
  Serial.setTxTimeoutMs(0);

  // Short wait: the spider has to start on battery too, with no USB. To see
  // the opening lines, press RESET with the monitor already open.
  unsigned long tWait = millis();
  while (!Serial && millis() - tWait < 1500) delay(10);

  Serial.println("\n========================================");
  Serial.println(">>> SPIDER - full firmware <<<");
  Serial.println("========================================\n");

  // 1. Eyes first: that way, if anything goes wrong later, it shows on its face.
  eyes.begin();                    // brings up I2C bus 1
  eyes.startTask(CORE_EYES);

  Serial.printf("EYES: %s, task on core %d\n",
                eyes.ready() ? "display ready" : "DISPLAY MISSING", CORE_EYES);

  // 2. Sensors
  sonar.begin();
  imu.begin();                      // if absent, we carry on without levelling
  batt.begin();

  // 3. Remote, BEFORE anything starts moving. Bringing up the WiFi is the
  //    chip's hungriest moment, and twelve servos repositioning at the same
  //    instant can drag the rail down far enough to make softAP() fail. It
  //    also means that if the legs ever misbehave, the robot is still
  //    reachable from the phone.
  remote.setCommand(runCommand);
  remote.setJoystick(joystick);
  remote.setParameter(parameter);
  remote.setTelemetry(telemetry);
  if (!remote.begin()) Serial.println("REMOTE: no network, carrying on without it.");

  // 4. Legs
  if (!legs.begin()) {             // brings up I2C bus 0
    Serial.println("CRITICAL ERROR: PCA9685 not answering, no movement.");
    eyes.setExpression(EYES_PUZZLED);
    for (;;) delay(1000);
  }
  legs.setStep(STEP_INITIAL);
  legs.setTrot(TROT_STRIDE, TROT_LIFT, TROT_SUPPORT);
  legs.setSpeed(SPEED);
  legs.startTask(1);               // movimento sul core 1

  // 5. Stand up, then take the IMU zero with the spider still and level
  eyes.faceNormal();
  legs.stand();
  Serial.println("Standing up, do not touch me...");
  waitMs(2500);
  imu.zero();

  // 6. A wave, just as the ATMega version did
  startWave();
  Serial.println("Starting in MANUAL: press 'm' or the Automatic button to let it loose.");
  status = AUTO_START;
  tState = millis();
  speedNow = SPEED;

  printHelp();
}

// How many passes a second the loop makes. If it drops below a few
// hundred, something is blocking it and the remote feels it immediately.
unsigned long loopCount = 0, tLoopCount = 0;
int loopHz = 0;

void loop() {
  loopCount++;
  if (millis() - tLoopCount >= 1000) {
    loopHz = loopCount;
    loopCount = 0;
    tLoopCount = millis();
  }

  // The modules with their own task (eyes, legs) are not touched here.
  sonar.update();
  imu.update();
  batt.update();
  batteryCheck();
  waveCheck();

  remote.update();
  readSerial();
  updateGesture();
  updateLevelling();
  fallCheck();
  if (automatic) updateBehaviour();
  printStatus();
}

// A wait that keeps the sensors alive: only needed in setup.
void waitMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    sonar.update();
    imu.update();
    delay(5);
  }
}

// ==================================================
// === POSES AND GESTURES ===
// ==================================================
//
// A pose is a body attitude held still; a gesture is a sequence of poses
// with their timings, plus an eye expression. Nothing else is needed: the
// spider already has everything required to put on a show.
//
// The durations are not free: the attitude moves at most 60 degrees per
// second and 80 mm per second, so a 20-degree move needs at least 330 ms.
// Ask for more in less time and the movement is clipped, and the gesture
// loses its bite.

// Giggle: short bounces, like the shoulders of someone laughing quietly.
const Gesture G_GIGGLE[] = {
  {  0,  0,  0,  +7,  0, 0, 170, EYES_HAPPY },
  {  0,  0,  0,  -5,  0, 0, 170, -1 },
  {  0,  0,  0,  +7,  0, 0, 170, -1 },
  {  0,  0,  0,  -5,  0, 0, 170, -1 },
  {  0,  0,  0,  +7,  0, 0, 170, -1 },
  {  0,  0,  0,  -5,  0, 0, 170, -1 },
  {  0,  0,  0,  +7,  0, 0, 170, -1 },   // quattro sobbalzi, non tre
  {  0,  0,  0,   0,  0, 0, 300, -1 },
  {  0,  0,  0,   0,  0, 0, 400, EYES_NEUTRAL }
};

// Stretch: slow, like a cat just woken up.
const Gesture G_STRETCH[] = {
  {  0, +15,  0, -10,  0, 0, 900, EYES_NEUTRAL },
  {  0, -12,  0, +15,  0, 0, 900, -1 },
  {  0,   0,  0,   0,  0, 0, 700, EYES_HAPPY },
  {  0,   0,  0,   0,  0, 0, 400, EYES_NEUTRAL }
};

// Nods yes.
const Gesture G_NOD[] = {
  {  0, +12,  0,   0,  0, 0, 360, EYES_HAPPY },
  {  0,  -8,  0,   0,  0, 0, 360, -1 },
  {  0, +12,  0,   0,  0, 0, 360, -1 },
  {  0,   0,  0,   0,  0, 0, 300, -1 },
  {  0,   0,  0,   0,  0, 0, 400, EYES_NEUTRAL }
};

// Shakes its head.
const Gesture G_SHAKE_NO[] = {
  {  0,  0, +14,   0,  0, 0, 400, EYES_ANGRY },
  {  0,  0, -14,   0,  0, 0, 480, -1 },
  {  0,  0, +14,   0,  0, 0, 480, -1 },
  {  0,  0,   0,   0,  0, 0, 400, EYES_NEUTRAL }
};

// Curious: head tilted to one side, like a dog that does not understand.
const Gesture G_CURIOUS[] = {
  { +18,  0,  0,   0,  0, 0, 500, EYES_PUZZLED },
  { +18,  0,  0,   0,  0, 0, 900, -1 },
  {   0,  0,  0,   0,  0, 0, 500, EYES_NEUTRAL }
};

// Bow.
const Gesture G_BOW[] = {
  {  0, +20,  0, -12,  0, 0, 700, EYES_HAPPY },
  {  0, +20,  0, -12,  0, 0, 600, -1 },
  {  0,   0,  0,   0,  0, 0, 700, -1 },
  {  0,   0,  0,   0,  0, 0, 400, EYES_NEUTRAL }
};

// Sways from one side to the other.
const Gesture G_DANCE[] = {
  {  0,  0,  0,   0, +20, 0, 400, EYES_HAPPY },
  {  0,  0,  0,   0, -20, 0, 520, -1 },
  {  0,  0,  0,   0, +20, 0, 520, -1 },
  {  0,  0,  0,   0, -20, 0, 520, -1 },
  {  0,  0,  0,   0,   0, 0, 400, -1 },
  {  0,   0,  0,   0,  0, 0, 400, EYES_NEUTRAL }
};

// Fright: it huddles down sharply and then rises slowly.
const Gesture G_FRIGHT[] = {
  {  0, -10,  0, -18,  0, 0, 260, EYES_SCARED },
  {  0, -10,  0, -18,  0, 0, 800, -1 },
  {  0,   0,  0,   0,  0, 0, 900, EYES_NEUTRAL }
};

// A short version of the fright for automatic mode: it flinches down,
// stays there a moment and returns to neutral quickly, because straight
// afterwards it has to back away and cannot do that crouched.
const Gesture G_STARTLE[] = {
  {  0,  -8,  0, -16,  0, 0, 260, EYES_SCARED },
  {  0,  -8,  0, -16,  0, 0, 450, -1 },
  {  0,   0,  0,   0,  0, 0, 350, -1 }
};

const Gesture *gesture = nullptr;
uint8_t gestureN = 0, gestureI = 0;
unsigned long tGesture = 0;

// Just starts a gesture: it touches neither the mode nor the legs. That
// way automatic mode can use it too, since sometimes it wants to laugh
// while walking and sometimes it wants to stop first.
void startGesture(const Gesture *g, uint8_t n, const char *name) {
  gesture = g;
  gestureN = n;
  gestureI = 0;
  tGesture = millis();
  manualPose = true;
  Serial.printf("* %s\n", name);
  applyGesture(g[0]);
}

void applyGesture(const Gesture &g) {
  legs.setAttitude(g.roll, g.pitch, g.yaw,
                   g.height, g.lateral, g.forward);
  if (g.expression >= 0) eyes.setExpression((uint8_t)g.expression);
}

void updateGesture() {
  if (gesture == nullptr) return;
  if (millis() - tGesture < gesture[gestureI].durationMs) return;

  gestureI++;
  tGesture = millis();
  if (gestureI < gestureN) {
    applyGesture(gesture[gestureI]);
    return;
  }

  // Done: back to neutral and levelling takes over again.
  gesture = nullptr;
  legs.resetAttitude();
  manualPose = false;
}

// A single pose, held until 'c' is pressed.
void pose(float rol, float bec, float imb, float alt, float lat, const char *name) {
  toManual();
  gesture = nullptr;
  manualPose = true;
  legs.setAttitude(rol, bec, imb, alt, lat, 0);
  Serial.printf("pose: %s\n", name);
}

// The version for hand commands: it switches automatic off and stops the legs.
void manualGesture(const Gesture *g, uint8_t n, const char *name) {
  toManual();
  legs.halt();
  startGesture(g, n, name);
}

#define N_GESTURE(a) (uint8_t)(sizeof(a) / sizeof(Gesture))

// ==================================================
// === BEHAVIOUR ===
// ==================================================

void changeState(uint8_t next) {
  status = next;
  tState = millis();
}

// Speed proportional to the room ahead: full above 50 cm, minimum below
// 30, and a ramp in between. Braking gradually instead of slamming on is
// the difference between a robot and a cautious little animal.
void adjustSpeed(int d) {
  float v;
  if (d <= 0 || d >= D_CLEAR_CM)      v = speedMax;    // d <= 0 = nessun eco = via libera
  else if (d <= D_SLOW_CM)            v = VEL_MIN;
  else v = VEL_MIN + (speedMax - VEL_MIN) *
           (float)(d - D_SLOW_CM) / (float)(D_CLEAR_CM - D_SLOW_CM);

  if (fabs(v - speedNow) > 0.05f) {
    speedNow = v;
    legs.setSpeed(v);
  }
}

// --- Average of several readings, so no decision rests on one echo ---
int samples = 0;
long sampleSum = 0;
unsigned long tLastSample = 0;

void startReading() {
  samples = 0;
  sampleSum = 0;
  tLastSample = sonar.lastReading();
}

// Returns true once it has gathered enough NEW readings (recognised by the
// Distance module's timestamp, so the same echo is never counted twice).
bool readingReady(int &risultato) {
  if (sonar.lastReading() != tLastSample) {
    tLastSample = sonar.lastReading();
    sampleSum += sonar.cm();
    samples++;
  }
  if (samples >= SCAN_SAMPLES) {
    risultato = (int)(sampleSum / samples);
    return true;
  }
  return false;
}

void resumeCruise() {
  eyes.lookAhead();
  eyes.faceNormal();
  eyes.setAutoBlink(true);
  legs.forward();
  tNextWhim = millis() + random(WHIM_MIN_MS, WHIM_MAX_MS);
  changeState(AUTO_CRUISE);
}

void turnFor(bool right, unsigned long ms) {
  if (right) { legs.turnRight();   eyes.lookRight();   }
  else        { legs.turnLeft(); eyes.lookLeft(); }
  phaseTime = ms;
  changeState(AUTO_TURN_AND_GO);
}

// Now and then it does something that serves no purpose: that is why it
// looks alive rather than programmed.
void whim(int d) {
  if ((long)(millis() - tNextWhim) < 0) return;
  tNextWhim = millis() + random(WHIM_MIN_MS, WHIM_MAX_MS);

  // No playing about if something is ahead: driving comes first.
  if (d > 0 && d < D_CLEAR_CM) return;

  int choice = random(0, 100);

  if (choice < 30) {
    // Small enough to do while walking.
    if (random(0, 2)) startGesture(G_GIGGLE, N_GESTURE(G_GIGGLE), "giggle");
    else              startGesture(G_NOD, N_GESTURE(G_NOD), "nod");

  } else if (choice < 55) {
    // A change of course for no reason.
    turnFor(random(0, 2), random(700, 1600));

  } else {
    // It stops and puts on a whole show.
    legs.halt();
    if (choice < 75)      startGesture(G_CURIOUS,      N_GESTURE(G_CURIOUS),      "curious");
    else if (choice < 88) startGesture(G_STRETCH, N_GESTURE(G_STRETCH), "stretch");
    else                  startGesture(G_DANCE,        N_GESTURE(G_DANCE),        "dance");
    changeState(AUTO_WHIM);
  }
}

void updateBehaviour() {
  int d = sonar.cm();
  unsigned long inState = millis() - tState;

  // Startle: it takes priority over everything except having fallen, and
  // it does not re-trigger while already fleeing.
  if (d > 0 && d < dPanic &&
      status != AUTO_STARTLE && status != AUTO_FLEE_BACK &&
      status != AUTO_TURN_AND_GO && status != AUTO_FALLEN) {
    Serial.printf("! %d cm out of nowhere\n", d);
    eyes.setExpression(EYES_SCARED, true);   // scatto, niente dissolvenza
    eyes.setAutoBlink(false);
    legs.halt();
    legs.setSpeed(speedMax);
    speedNow = speedMax;
    startGesture(G_STARTLE, N_GESTURE(G_STARTLE), "startle");
    changeState(AUTO_STARTLE);
    return;
  }

  switch (status) {

    case AUTO_START:
      if (!legs.moving() && inState > 1500) {
        eyes.faceNormal();
        resumeCruise();
      }
      break;

    case AUTO_CRUISE:
      adjustSpeed(d);

      // Obstacle at normal range: stop and look for a way through.
      if (d > 0 && d < dScan) {
        Serial.printf("  obstacle at %d cm, looking around\n", d);
        eyes.facePuzzled();
        legs.setSpeed(speedMax);       // le giratine di scan a speed' piena
        speedNow = speedMax;
        legs.turnRight();
        eyes.lookRight();
        changeState(AUTO_SCAN_R);
        break;
      }

      if (legs.gait() != GAIT_FORWARD) legs.forward();
      whim(d);
      break;

    case AUTO_WHIM:
      // The gesture clears itself when it ends.
      if (gesture == nullptr) resumeCruise();
      break;

    case AUTO_TURN_AND_GO:
      if (inState >= phaseTime) resumeCruise();
      break;

    // --- Scan: right, read, left, read, decide ---
    case AUTO_SCAN_R:
      if (inState >= SCAN_TURN_MS) {
        legs.halt();
        changeState(AUTO_READ_R);
      }
      break;

    case AUTO_READ_R:
      // Until it has settled we keep resetting, so the readings really do
      // start from standstill.
      if (inState < SCAN_SETTLE_MS) { startReading(); break; }
      if (readingReady(distR)) {
        legs.turnLeft();
        eyes.lookLeft();
        changeState(AUTO_SCAN_L);
      }
      break;

    case AUTO_SCAN_L:
      // Twice the time: it has to swing from the right side to the left.
      if (inState >= 2 * SCAN_TURN_MS) {
        legs.halt();
        changeState(AUTO_READ_L);
      }
      break;

    case AUTO_READ_L:
      if (inState < SCAN_SETTLE_MS) { startReading(); break; }
      if (readingReady(distL)) {
        Serial.printf("  right %d cm, left %d cm -> ", distR, distL);
        if (distR > distL + SCAN_MARGIN_CM) {
          // Better to the right: it is the other way round, so twice as far.
          Serial.println("going right");
          turnFor(true, 2 * SCAN_TURN_MS);
        } else if (distL > distR + SCAN_MARGIN_CM) {
          Serial.println("going left");
          turnFor(false, SCAN_TURN_MS / 2);
        } else {
          // A draw: neither way is open, so make a proper turn.
          Serial.println("blocked both ways, wide turn");
          eyes.faceAngry();
          turnFor(random(0, 2), 2500);
        }
      }
      break;

    // --- Fright and flight ---
    case AUTO_STARTLE:
      if (gesture == nullptr) {
        legs.backward();
        changeState(AUTO_FLEE_BACK);
      }
      break;

    case AUTO_FLEE_BACK:
      if (inState >= FLEE_BACK_MS) {
        turnFor(random(0, 2), random(FLEE_TURN_MIN_MS, FLEE_TURN_MAX_MS));
      }
      break;

    case AUTO_FALLEN:
      // fallCheck() gets it out once it is upright again.
      break;

    default:
      break;
  }
}

// ==================================================
// === BATTERY ===
// ==================================================

bool battCutOff = false;     // servos already released for a flat battery

// The wave does not go through the gesture player: it is a leg sequence
// with a happy face set by hand next to it, and nobody was putting that
// back. Here we wait for the legs to finish and return to neutral.
bool waving = false;
unsigned long tWave = 0;

void startWave() {
  legs.wave();
  eyes.faceHappy();
  waving = true;
  tWave = millis();
}

void waveCheck() {
  if (!waving) return;
  // A moment's grace: the sequence only starts at the next pose, and
  // without this wait moving() would still be false.
  if (millis() - tWave < 800) return;
  if (legs.moving()) return;
  waving = false;
  eyes.faceNormal();
}

void batteryCheck() {
  // The eyes draw the icon themselves: here we only hand them the numbers.
  eyes.setBattery(batt.level(), batt.percent(), batt.volt());

  if (!batt.critical() || battCutOff) return;

  // Below the limit there is no arguing: everything is released. Keeping
  // current flowing out of a pack at its floor damages it, and a cell taken
  // under 3 V often never comes back.
  battCutOff = true;
  automatic = false;
  legs.halt();
  legs.resetAttitude();
  legs.relax();
  Serial.printf("BATTERY FLAT (%.2f V): servos released. Recharge the pack.\n",
                batt.volt());
}

// ==================================================
// === SAFETY AND ATTITUDE ===
// ==================================================

// If it has tipped over, carrying on walking will not right it: better to
// sit down, stop correcting and wait to be set upright.
void fallCheck() {
  if (!imu.ready()) return;

  if (status != AUTO_FALLEN && imu.tipped(FALL_DEGREES)) {
    Serial.println("! leaning too far, sitting down");
    legs.halt();
    legs.resetAttitude();
    legs.sit();
    eyes.setExpression(EYES_SCARED, true);
    changeState(AUTO_FALLEN);
    return;
  }

  if (status == AUTO_FALLEN && !imu.tipped(FALL_DEGREES - 10) &&
      millis() - tState > 1500) {
    Serial.println("  upright again, carrying on");
    legs.stand();
    eyes.faceNormal();
    changeState(AUTO_START);
  }
}

void updateLevelling() {
  static unsigned long tLast = 0;
  if (millis() - tLast < 20) return;        // 50 Hz, same as the leg tick
  tLast = millis();

  if (!levelling || !imu.ready() || status == AUTO_FALLEN || manualPose) return;

  float r = constrain(-imu.roll()     * gain, -MAX_CORRECTION, MAX_CORRECTION);
  float b = constrain(-imu.pitch() * gain, -MAX_CORRECTION, MAX_CORRECTION);
  legs.setAttitude(r, b);
}

// ==================================================
// === COMMANDS ===
// ==================================================
// A single entry point: tomorrow BLE will call this very function, without
// touching anything else.

void runCommand(char c) {
  switch (c) {
    // --- driving ---
    case 'w': toManual(); legs.endCalibration(); legs.forward();       eyes.faceNormal();   break;
    case 's': toManual(); legs.endCalibration(); legs.backward();     eyes.faceNormal();   break;
    case 'a': toManual(); legs.endCalibration(); legs.turnLeft(); eyes.lookLeft();  break;
    case 'd': toManual(); legs.endCalibration(); legs.turnRight();   eyes.lookRight();    break;
    case 'x': toManual(); legs.halt();      eyes.lookAhead();    break;

    // --- postures ---
    case 'Q':
      toManual();
      gesture = nullptr;
      manualPose = false;
      legs.resetAttitude();
      legs.calibrate90();
      eyes.faceNormal();
      Serial.println("horn check: press 'u' or 'A' to come back");
      break;
    case 'A': toManual(); legs.squareStance(); Serial.println("square stance"); break;
    case 'u': toManual(); legs.endCalibration(); legs.stand(); break;
    case 'j': toManual(); legs.endCalibration(); legs.sit();  break;
    case 'h': toManual(); startWave(); break;

    // --- expressions ---
    case '1': eyes.faceNormal();    break;
    case '2': eyes.faceHappy();     break;
    case '3': eyes.faceAngry(); break;
    case '4': eyes.facePuzzled();    break;
    case '5': eyes.faceScared();  break;
    case '6': eyes.sleepy();            break;

    // --- body poses (they stay until you press 'c') ---
    case '7': pose(+15, 0, 0, 0, 0, "lean right");   break;
    case '8': pose(-15, 0, 0, 0, 0, "lean left"); break;
    case '9': pose(0, +15, 0, 0, 0, "nose down");             break;
    case '0': pose(0, -15, 0, 0, 0, "nose up");              break;
    case 'o': pose(0, 0, +15, 0, 0, "yaw right");        break;
    case 'p': pose(0, 0, -15, 0, 0, "yaw left");      break;
    case 'k': pose(0, 0, 0, -20, 0, "crouch");            break;
    case 'i': pose(0, 0, 0, +25, 0, "tiptoe");     break;
    case 'n': pose(0, 0, 0, 0, +25, "weight right");         break;
    case 'b': pose(0, 0, 0, 0, -25, "weight left");       break;
    case 'c':
      gesture = nullptr;
      manualPose = false;
      legs.resetAttitude();
      Serial.println("attitude neutral");
      break;

    // --- gestures (capitals) ---
    // By hand a gesture is done standing: take control first, then pose.
    case 'G': manualGesture(G_GIGGLE,     N_GESTURE(G_GIGGLE),     "giggle");     break;
    case 'S': manualGesture(G_STRETCH, N_GESTURE(G_STRETCH), "stretch"); break;
    case 'N': manualGesture(G_NOD,     N_GESTURE(G_NOD),     "nod");     break;
    case 'K': manualGesture(G_SHAKE_NO,         N_GESTURE(G_SHAKE_NO),         "shake no");     break;
    case 'C': manualGesture(G_CURIOUS,      N_GESTURE(G_CURIOUS),      "curious");    break;
    case 'B': manualGesture(G_BOW,      N_GESTURE(G_BOW),      "bow");      break;
    case 'D': manualGesture(G_DANCE,        N_GESTURE(G_DANCE),        "dance");        break;
    case 'P': manualGesture(G_FRIGHT,     N_GESTURE(G_FRIGHT),     "fright");     break;

    // --- modes ---
    case 'm':
      automatic = !automatic;
      if (automatic) {
        // We always come back in at cruise, with the speed reset.
        speedNow = -1;
        gesture = nullptr;
        manualPose = false;
        legs.resetAttitude();
        legs.endCalibration();
        resumeCruise();
      } else {
        legs.halt();
      }
      Serial.printf("mode %s\n", automatic ? "AUTOMATIC" : "manual");
      break;
    case 't': {
      bool trotto = (legs.step() != STEP_TROT);
      legs.setStep(trotto ? STEP_TROT : STEP_CRAWL);
      Serial.printf("gait %s\n", trotto ? "TROT" : "CRAWL");
      break;
    }
    case 'l':
      levelling = !levelling;
      if (!levelling) legs.resetAttitude();
      Serial.printf("levelling %s\n", levelling ? "on" : "off");
      break;
    case 'r': legs.relax(); Serial.println("servos relaxed"); break;
    case 'e': legs.enable();  Serial.println("servos enabled");    break;
    case 'v':
      eyes.setShowBattery(!eyes.showBattery());
      Serial.printf("battery icon %s (%.2f V, %d%%)\n",
                    eyes.showBattery() ? "on" : "off",
                    batt.volt(), batt.percent());
      break;
    case 'V':
      // Only to be used after changing or recharging the pack.
      batt.clearCritical();
      battCutOff = false;
      legs.enable();
      legs.stand();
      Serial.println("battery alarm cleared, servos re-enabled");
      break;
    case 'z': imu.zero(); break;
    case '?': printHelp(); break;

    default:
      Serial.printf("unknown command: '%c'. Press ? for the list.\n",
                    (c >= 32 && c < 127) ? c : '.');
      break;
  }
}

// Every driving command switches automatic off: otherwise the two fight
// over the legs and the spider looks indecisive.
void toManual() {
  if (battCutOff) return;      // a batteria scarica non si guida
  if (automatic) {
    automatic = false;
    Serial.println("manual mode");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    runCommand(c);
  }
}

void printHelp() {
  Serial.println("\n--- commands ---");
  Serial.println("  w s a d  forward, back, left, right");
  Serial.println("  x        stop");
  Serial.println("  u j h    stand, sit, wave");
  Serial.println("  Q        all servos to 90 degrees (horn check)");
  Serial.println("  A        square stance, legs even front and back");
  Serial.println("  1..6     normal, happy, angry, puzzled, scared, sleepy");
  Serial.println("  7 8      lean right / left");
  Serial.println("  9 0      nose down / nose up");
  Serial.println("  o p      yaw right / left");
  Serial.println("  k i      crouch / tiptoe");
  Serial.println("  n b      weight right / left");
  Serial.println("  c        neutral attitude (leaves pose mode)");
  Serial.println("  --- gestures (capitals) ---");
  Serial.println("  G giggle     S stretch        N nod        K shake no");
  Serial.println("  C curious    B bow            D dance      P fright");
  Serial.println("  --- modes ---");
  Serial.println("  m        automatic / manual");
  Serial.println("  t        trot / crawl");
  Serial.println("  l        levelling on/off");
  Serial.println("  r e      relax / enable the servos");
  Serial.println("  v        show/hide the battery icon");
  Serial.println("  V        clear the battery alarm (after recharging)");
  Serial.println("  z        re-take the IMU zero");
  Serial.println("  ?        this list");
}

// ==================================================
// === REMOTE ===
// ==================================================

// Left stick: driving. The axis pushed furthest decides what to do, and
// how far it is pushed decides the speed. It is not true omnidirectional
// walking (that would need a continuous-phase gait), but the speed really
// is analogue and you can feel it when driving.
void joystickDrive(int x, int y) {
  int push = max(abs(x), abs(y));

  if (push < STICK_DEADZONE) {
    if (legs.gait() != GAIT_STOP) legs.halt();
    return;
  }

  float v = VEL_MIN + (speedMax - VEL_MIN) *
            (float)(push - STICK_DEADZONE) / (float)(100 - STICK_DEADZONE);
  if (fabs(v - speedNow) > 0.05f) { speedNow = v; legs.setSpeed(v); }

  if (abs(y) >= abs(x)) {
    if (y > 0) { if (legs.gait() != GAIT_FORWARD)   legs.forward(); }
    else       { if (legs.gait() != GAIT_BACK) legs.backward(); }
  } else {
    if (x > 0) { if (legs.gait() != GAIT_RIGHT)   legs.turnRight(); }
    else       { if (legs.gait() != GAIT_LEFT) legs.turnLeft(); }
  }
}

// Right stick: the gaze. Up and down move the muzzle; right and left yaw
// the body and carry the pupils with them, so the spider really does look
// that way instead of merely twisting.
void joystickLook(int x, int y) {
  if (abs(x) < STICK_DEADZONE && abs(y) < STICK_DEADZONE) {
    if (manualPose && gesture == nullptr) {
      manualPose = false;          // released: levelling resumes
      legs.resetAttitude();
      eyes.lookAhead();
    }
    return;
  }

  // Pitch sense. On the real robot it came out reversed from what the
  // arithmetic led me to expect, so the measurement wins over the theory:
  // flipping this sign swaps nose up and nose down.
  float bec =  (float)y * STICK_PITCH / 100.0f;
  float imb =  (float)x * STICK_YAW  / 100.0f;

  manualPose = true;               // levelling stands aside
  legs.setAttitude(0, bec, imb, 0, 0, 0);
  eyes.look((x * STICK_PUPILS) / 100);
}

void joystick(int lx, int ly, int rx, int ry) {
  if (abs(lx) >= STICK_DEADZONE || abs(ly) >= STICK_DEADZONE) toManual();
  if (!automatic) joystickDrive(lx, ly);
  joystickLook(rx, ry);
}

// Sliders on the Settings page.
void parameter(const char *name, float v) {
  if      (!strcmp(name, "stride"))   legs.setTrot(v, legs.trotLift(), legs.trotSupport());
  else if (!strcmp(name, "lift"))    legs.setTrot(legs.trotStride(), v, legs.trotSupport());
  else if (!strcmp(name, "support"))  legs.setTrot(legs.trotStride(), legs.trotLift(), (uint16_t)v);
  else if (!strcmp(name, "speed"))  { speedMax = v; speedNow = -1; }
  else if (!strcmp(name, "gain"))  gain = v;
  else if (!strcmp(name, "panic"))    dPanic = (int)v;
  else if (!strcmp(name, "scan")) dScan = (int)v;
  else { Serial.printf("unknown parameter: %s\n", name); return; }
  Serial.printf("parameter %s = %.1f\n", name, v);
}

// A single reply carries everything the page needs to show.
String telemetry() {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"d\":%d,\"r\":%.1f,\"b\":%.1f,\"v\":%.2f,\"s\":\"%s\","
    "\"a\":%d,\"t\":%d,\"l\":%d,"
    "\"stride\":%.0f,\"lift\":%.0f,\"support\":%d,\"speed\":%.1f,"
    "\"gain\":%.1f,\"panic\":%d,\"scan\":%d,"
    "\"hz\":%d,\"ram\":%u,\"oc\":%d,\"of\":%u,"
    "\"bv\":%.2f,\"bp\":%d,\"bl\":%d,\"bm\":%d,\"pv\":\"" PAGE_VERSION "\"}",
    sonar.cm(), imu.roll(), imu.pitch(),
    speedNow < 0 ? 0.0f : speedNow,
    automatic ? stateName() : "manual",
    automatic ? 1 : 0,
    legs.step() == STEP_TROT ? 1 : 0,
    levelling ? 1 : 0,
    legs.trotStride(), legs.trotLift(), (int)legs.trotSupport(),
    speedMax, gain, dPanic, dScan,
    loopHz, (unsigned)ESP.getFreeHeap(),
    eyes.ready() ? 1 : 0, (unsigned)eyes.frames(),
    batt.volt(), batt.percent(), batt.level(),
    eyes.showBattery() ? 1 : 0);
  return String(buf);
}

// ==================================================
// === DIAGNOSTICS ===
// ==================================================

const char *stateName() {
  switch (status) {
    case AUTO_START:          return "start";
    case AUTO_CRUISE:       return "cruise";
    case AUTO_WHIM:      return "whim";
    case AUTO_TURN_AND_GO: return "turning";
    case AUTO_SCAN_R:        return "peek right";
    case AUTO_READ_R:       return "reading dx";
    case AUTO_SCAN_L:        return "peek left";
    case AUTO_READ_L:       return "reading sx";
    case AUTO_STARTLE:    return "fright!";
    case AUTO_FLEE_BACK:  return "backward!";
    case AUTO_FALLEN:         return "fallen";
    default:                  return "?";
  }
}

void printStatus() {
  static unsigned long tLast = 0;
  if (millis() - tLast < 2000) return;
  tLast = millis();

  Serial.printf("[%s] distance %3d cm | speed %.1f | incl r%+5.1f b%+5.1f | %s\n",
                automatic ? stateName() : "manual",
                sonar.cm(), speedNow < 0 ? 0.0f : speedNow,
                imu.roll(), imu.pitch(),
                levelling ? "levelled" : "free");
}
