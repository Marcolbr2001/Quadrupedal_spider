#include "Eyes.h"

// --- EYE GEOMETRY (identical to the ATMega version) ---
static const int SCREEN_W = 128;
static const int SCREEN_H = 64;
static const int w  = 40;
static const int h  = 40;   // Leggermente più bassi per stare nello schermo
static const int xL = 10;
static const int xR = 78;
static const int r  = 8;

// The eyes must be centred in the uniformly coloured part of the panel.
// With no rotation the yellow band is at the top, so we drop below it;
// flipping the display puts the band at the bottom and we go back up.
// With the default values (16 band, 40 tall eyes) this gives y = 20 with no
// rotation and y = 4 with 180, which is what this robot needs.
static const int USABLE_BAND = SCREEN_H - EYES_YELLOW_BAND;
#if EYES_ROTATION == 2
static const int y = (USABLE_BAND - h) / 2;
#else
static const int y = EYES_YELLOW_BAND + (USABLE_BAND - h) / 2;
#endif
static const int pSize = 14;

// --- ANIMATION SPEEDS (pixels per frame) ---
static const int STEP_LIDS = 3;   // apertura/chiusura per cambio expression
static const int STEP_BLINK    = 10;  // ammicco: veloce
static const int STEP_GAZE  = 5;   // spostamento pupille

// --- SPONTANEOUS BLINK ---
static const unsigned long BLINK_MIN_MS = 2500;
static const unsigned long BLINK_MAX_MS = 6500;

// The same codes as Battery.h, repeated here so the eye module is not tied
// to the battery one: all it knows how to do is draw the icon.
#define BATT_FULL_EYES   0
#define BATT_HALF_EYES   1
#define BATT_LOW_EYES   2
#define BATT_CRITICAL_EYES 3
#define BATT_USB_EYES     4
static const uint8_t BATT_LVL_FULL_EYES = BATT_FULL_EYES;

// Half a second on, half a second off.
static const unsigned long BATT_BLINK_MS = 500;

#define EYES_LOCK()   portENTER_CRITICAL(&_mux)
#define EYES_UNLOCK() portEXIT_CRITICAL(&_mux)

// Resting lids for each expression: where topH and botH should end up.
static void lidTargets(uint8_t mood, int &topT, int &botT) {
  switch (mood) {
    case EYES_ANGRY: topT = 20;      botT = 0;       break;  // brows lowered
    case EYES_HAPPY:     topT = 0;       botT = 25;      break;  // round cheeks rising
    case EYES_SLEEPY:      topT = h/2 + 2; botT = h/2 + 2; break;  // eyes shut
    default:               topT = 0;       botT = 0;       break;  // neutral, puzzled, scared
  }
}

// In expressions where the eyes are already covered the blink would be
// invisible (or would hide the sleep line), so there we skip it.
static bool blinkMakesSense(uint8_t mood) {
  return mood != EYES_SLEEPY;
}

Eyes::Eyes(TwoWire *bus)
  : _bus(bus),
    _display(SCREEN_W, SCREEN_H, bus, -1),
    _ready(false),
    _useTask(false),
    _task(nullptr),
    _mux(portMUX_INITIALIZER_UNLOCKED),
    _moodWanted(EYES_NEUTRAL),
    _xWanted(0),
    _yWanted(0),
    _blinkWanted(false),
    _blinkAuto(true),
    _instant(false),
    _mood(EYES_NEUTRAL),
    _phase(0),
    _xOff(0), _yOff(0),
    _topH(0), _botH(0),
    _blinkH(0),
    _blinkPhase(0),
    _tFrame(0),
    _tBlink(0),
    _battLevel(BATT_LVL_FULL_EYES), _battPerc(100), _battMilliVolt(0),
    _battShow(false), _pBattBlink(false),
    _frames(0),
    _neverDrawn(true),
    _pX(0), _pY(0), _pTop(0), _pBot(0), _pBlink(0),
    _pBattLevel(BATT_LVL_FULL_EYES), _pBattPerc(100), _pBattShow(false),
    _pMood(EYES_NEUTRAL) {}

// ==================================================
// === STARTUP ===
// ==================================================

bool Eyes::begin(bool avviaBus) {
  if (avviaBus) {
    if (!_bus->begin(EYES_SDA, EYES_SCL, EYES_I2C_FREQ)) {
      Serial.println("EYES: cannot start the I2C bus!");
      return false;
    }
  }

  // The last parameter (periphBegin = false) is ESSENTIAL on the ESP32: it
  // stops the library calling bus->begin() with no arguments, which would
  // wipe out the GPIO17/GPIO18 assignment.
  if (!_display.begin(SSD1306_SWITCHCAPVCC, EYES_ADDR, true, false)) {
    Serial.println("EYES: no display at address 0x3C.");
    scanBus();
    return false;
  }

  // Call it AFTER begin(): the display init resets the rotation.
  _display.setRotation(EYES_ROTATION);

  _display.clearDisplay();
  _display.display();

  _ready = true;
  _tFrame = millis();
  _scheduleBlink();
  Serial.println("EYES: display up, eyes online!");
  return true;
}

bool Eyes::startTask(uint8_t core, uint32_t stack, UBaseType_t priorita) {
  if (!_ready || _useTask) return false;
  BaseType_t outcome = xTaskCreatePinnedToCore(_taskLoop, "occhietti", stack,
                                             this, priorita, &_task, core);
  _useTask = (outcome == pdPASS);
  if (_useTask) {
    Serial.printf("EYES: animation on its own task (core %u).\n", core);
  } else {
    Serial.println("EYES: task not created, using update() from loop().");
  }
  return _useTask;
}

void Eyes::_taskLoop(void *param) {
  Eyes *self = static_cast<Eyes *>(param);
  const TickType_t period = pdMS_TO_TICKS(EYES_FRAME_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    self->_animateFrame();
    vTaskDelayUntil(&last, period);   // cadenza fissa, niente deriva
  }
}

void Eyes::update() {
  if (!_ready || _useTask) return;      // col task ci pensa l'altro core
  unsigned long ora = millis();
  if (ora - _tFrame < EYES_FRAME_MS) return;
  _tFrame = ora;
  _animateFrame();
}

// ==================================================
// === COMMANDS (non-blocking) ===
// ==================================================

void Eyes::setExpression(uint8_t expression, bool instant) {
  EYES_LOCK();
  _moodWanted = expression;
  if (instant) _instant = true;
  EYES_UNLOCK();
}

void Eyes::look(int dx, int dy) {
#if EYES_ROTATION == 2
  // The panel is mounted upside down, so a positive offset in the buffer
  // lands on the VIEWER'S LEFT. It gets straightened here, once: that way
  // lookRight() really does look right and none of the callers has to
  // remember the rotation.
  dx = -dx;
  dy = -dy;
#endif
  EYES_LOCK();
  _xWanted = dx;
  _yWanted = dy;
  EYES_UNLOCK();
}

void Eyes::setBattery(uint8_t level, int percent, float volt) {
  EYES_LOCK();
  _battLevel = level;
  _battPerc = percent;
  _battMilliVolt = (int)(volt * 1000.0f);
  EYES_UNLOCK();
}

void Eyes::setShowBattery(bool mostra) {
  EYES_LOCK();
  _battShow = mostra;
  EYES_UNLOCK();
}

void Eyes::blink() {
  EYES_LOCK();
  _blinkWanted = true;
  EYES_UNLOCK();
}

void Eyes::setAutoBlink(bool enabled) {
  EYES_LOCK();
  _blinkAuto = enabled;
  EYES_UNLOCK();
}

bool Eyes::moving() const {
  return _phase != 0 || _blinkPhase != 0 ||
         _xOff != _xWanted || _yOff != _yWanted;
}

// ==================================================
// === ANIMATION ENGINE (one frame) ===
// ==================================================

void Eyes::_animateFrame() {
  if (!_ready) return;

  // 1. Snapshot of the commands that arrived from outside.
  //    A very short critical section: scalar copies only.
  uint8_t moodWanted;
  int xWanted, yWanted;
  bool blinkWanted, blinkAuto, instant;
  EYES_LOCK();
  moodWanted  = _moodWanted;
  xWanted     = _xWanted;
  yWanted     = _yWanted;
  blinkWanted = _blinkWanted;
  blinkAuto   = _blinkAuto;
  instant   = _instant;
  _blinkWanted = false;
  _instant = false;
  EYES_UNLOCK();

  // 2. Expression changes in two stages: first the lids reopen with the
  //    old face, then it switches to the new one. That avoids the jerks
  //    you would see jumping straight from one pose to another.
  int topT, botT;

  // 2b. Startle: snap into the new pose with no transition.
  if (instant) {
    _mood = moodWanted;
    _phase = 0;
    lidTargets(_mood, topT, botT);
    _topH = topT;
    _botH = botT;
    _blinkH = 0;
    _blinkPhase = 0;
    blinkWanted = false;
  }

  if (moodWanted != _mood && _phase != 1) {
    _phase = 1;                          // leaving phase
  }

  if (_phase == 1) {
    topT = 0;                           // si torna a eyes aperti
    botT = 0;
  } else {
    lidTargets(_mood, topT, botT);
  }

  _topH = _approach(_topH, topT, STEP_LIDS);
  _botH = _approach(_botH, botT, STEP_LIDS);

  if (_phase == 1 && _topH == 0 && _botH == 0) {
    bool eraAddormentato = (_mood == EYES_SLEEPY);
    _mood = moodWanted;
    _phase = 2;                          // entering phase
    // Waking up: a blink as soon as the eyes reopen, just like the
    // wakeUp() of the blocking version.
    if (eraAddormentato && _mood != EYES_SLEEPY) blinkWanted = true;
  } else if (_phase == 2) {
    lidTargets(_mood, topT, botT);
    if (_topH == topT && _botH == botT) _phase = 0;
  }

  // 3. Gaze: the pupils slide towards the requested position
  _xOff = _approach(_xOff, xWanted, STEP_GAZE);
  _yOff = _approach(_yOff, yWanted, STEP_GAZE);

  // 4. Blink (drawn over the expression, so it works with all of them)
  if (blinkAuto && _blinkPhase == 0 && _phase == 0 &&
      blinkMakesSense(_mood) && (long)(millis() - _tBlink) >= 0) {
    blinkWanted = true;
  }
  if (blinkWanted && _blinkPhase == 0 && blinkMakesSense(_mood)) {
    _blinkPhase = 1;
  }
  if (_blinkPhase == 1) {
    _blinkH += STEP_BLINK;
    if (_blinkH >= h) { _blinkH = h; _blinkPhase = 2; }
  } else if (_blinkPhase == 2) {
    _blinkH -= STEP_BLINK;
    if (_blinkH <= 0) { _blinkH = 0; _blinkPhase = 0; _scheduleBlink(); }
  }

  // 5. The fear tremor: recomputed on every frame
  int jx = 0, jy = 0;
  if (_mood == EYES_SCARED) {
    jx = random(-2, 3);
    jy = random(-2, 3);
  }

  // 6. The display is touched ONLY if the image really changed.
  //    With the eyes still, the I2C bus stays completely free.
  int fx = _xOff + jx;
  int fy = _yOff + jy;

  // The battery joins the comparison like everything else, otherwise the
  // icon would never appear with the eyes still. The blink phase counts
  // too: that is precisely what has to force a redraw.
  bool blinking = ((millis() / BATT_BLINK_MS) % 2) == 0;

  if (_neverDrawn || fx != _pX || fy != _pY || _topH != _pTop ||
      _botH != _pBot || _blinkH != _pBlink || _mood != _pMood ||
      _battLevel != _pBattLevel || _battPerc != _pBattPerc ||
      _battShow != _pBattShow ||
      (_battLevel == BATT_LOW_EYES && blinking != _pBattBlink)) {
    _draw(fx, fy, _topH, _botH, _blinkH, _mood);
    _frames = _frames + 1;   // non ++: su un volatile e' deprecato
    _neverDrawn = false;
    _pX = fx; _pY = fy; _pTop = _topH; _pBot = _botH;
    _pBlink = _blinkH; _pMood = _mood;
    _pBattLevel = _battLevel; _pBattPerc = _battPerc;
    _pBattShow = _battShow; _pBattBlink = blinking;
  }
}

void Eyes::_scheduleBlink() {
  _tBlink = millis() + random(BLINK_MIN_MS, BLINK_MAX_MS);
}

int Eyes::_approach(int value, int target, int step) {
  if (value < target) {
    value += step;
    if (value > target) value = target;
  } else if (value > target) {
    value -= step;
    if (value < target) value = target;
  }
  return value;
}

// ==================================================
// === GRAPHICS ENGINE ===
// ==================================================

// mood:
// 0=Neutral, 1=Angry, 2=Happy, 3=Sleepy,
// 4=Puzzled (asymmetric), 5=Scared (small pupils)
// blinkH: the blink lid, drawn on top of everything else.

// A 22x11 icon bottom left, inside the panel's yellow band.
void Eyes::_drawBatteryIcon() {
  const int bx = 2, by = SCREEN_H - 14, bw = 20, bh = 11;

  _display.drawRect(bx, by, bw, bh, SSD1306_WHITE);            // shell
  _display.fillRect(bx + bw, by + 3, 2, bh - 6, SSD1306_WHITE); // terminal

  int pieno = ((bw - 4) * _battPerc) / 100;
  if (pieno > 0) _display.fillRect(bx + 2, by + 2, pieno, bh - 4, SSD1306_WHITE);
}

// A plug with its lead: it means the board is running on the cable and
// there is no battery. Better that than an invented percentage.
void Eyes::_drawPlugIcon() {
  const int bx = 2, by = SCREEN_H - 14;

  _display.fillRect(bx + 6, by + 2, 8, 7, SSD1306_WHITE);      // body
  _display.drawFastHLine(bx + 2, by + 3, 4, SSD1306_WHITE);    // prong
  _display.drawFastHLine(bx + 2, by + 7, 4, SSD1306_WHITE);    // prong
  _display.drawFastHLine(bx + 14, by + 5, 7, SSD1306_WHITE);   // lead
}

// Full screen: if the pack is at its limit, nobody should miss it.
void Eyes::_drawBigBattery() {
  const int bx = 14, by = 12, bw = 86, bh = 34;

  _display.drawRect(bx, by, bw, bh, SSD1306_WHITE);
  _display.drawRect(bx + 1, by + 1, bw - 2, bh - 2, SSD1306_WHITE);
  _display.fillRect(bx + bw, by + 11, 5, 12, SSD1306_WHITE);   // terminal

  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.setCursor(bx + 10, by + 8);
  _display.print("BATTERY");
  _display.setCursor(bx + 14, by + 19);
  _display.printf("%d.%02d V", _battMilliVolt / 1000, (_battMilliVolt % 1000) / 10);

}

void Eyes::_draw(int xOff, int yOff, int topH, int botH,
                         int blinkH, uint8_t mood) {
  _display.clearDisplay();

  // Battery at its limit: it takes the whole screen and the eyes vanish.
  if (_battLevel == BATT_CRITICAL_EYES) {
    _drawBigBattery();
    _display.display();
    return;
  }

  // Pupil size varies with the mood
  int pupilNow = pSize;
  if (mood == EYES_SCARED) pupilNow = 6; // Pupilla piccolissima se impaurito

  // 1. Work out the pupils
  int pLX = xL + w/2 - pupilNow/2 + xOff;
  int pRX = xR + w/2 - pupilNow/2 + xOff;
  int pY  = y + h/2 - pupilNow/2 + yOff;

  // Keep them inside the eye
  pLX = constrain(pLX, xL+2, xL+w-pupilNow-2);
  pRX = constrain(pRX, xR+2, xR+w-pupilNow-2);

  // 2. Draw the eye backgrounds (white)
  _display.fillRoundRect(xL, y, w, h, r, SSD1306_WHITE);
  _display.fillRoundRect(xR, y, w, h, r, SSD1306_WHITE);

  // 3. Draw the pupils (black and SQUARE)
  // Asleep with the eyes shut: no pupils
  if (!(mood == EYES_SLEEPY && topH >= h/2)) {
    _display.fillRect(pLX, pY, pupilNow, pupilNow, SSD1306_BLACK);
    _display.fillRect(pRX, pY, pupilNow, pupilNow, SSD1306_BLACK);
  }

  // 4. Lids and expressions

  // --- NEUTRAL, SLEEPY, SCARED ---
  if (mood == EYES_NEUTRAL || mood == EYES_SLEEPY || mood == EYES_SCARED) {
    if (topH > 0) {
      _display.fillRect(xL, y, w, topH, SSD1306_BLACK);
      _display.fillRect(xR, y, w, topH, SSD1306_BLACK);
    }
    if (botH > 0) {
      _display.fillRect(xL, y+h-botH, w, botH, SSD1306_BLACK);
      _display.fillRect(xR, y+h-botH, w, botH, SSD1306_BLACK);
    }
    // White line for sleep
    if (mood == EYES_SLEEPY && topH >= h/2) {
      _display.drawLine(xL, y+h/2, xL+w, y+h/2, SSD1306_WHITE);
      _display.drawLine(xR, y+h/2, xR+w, y+h/2, SSD1306_WHITE);
    }
  }

  // --- ANGRY (triangles on top) ---
  else if (mood == EYES_ANGRY) {
    _display.fillRect(xL, y, w, topH-8, SSD1306_BLACK);
    _display.fillRect(xR, y, w, topH-8, SSD1306_BLACK);
    // Slanted triangles
    _display.fillTriangle(xL, y+topH-8, xL+w, y+topH-8, xL+w, y+topH+12, SSD1306_BLACK);
    _display.fillTriangle(xR, y+topH-8, xR+w, y+topH-8, xR, y+topH+12, SSD1306_BLACK);
  }

  // --- HAPPY (round cheeks) ---
  else if (mood == EYES_HAPPY) {
    // Instead of triangles, black circles underneath give the cheek effect
    // Two filled black circles rising from below
    int radius = w; // Raggio grande quanto l'occhio
    int cheekY = y + h + radius - botH - 5; // Calcolo posizione

    _display.fillCircle(xL + w/2, cheekY, radius, SSD1306_BLACK);
    _display.fillCircle(xR + w/2, cheekY, radius, SSD1306_BLACK);
  }

  // --- PUZZLED (asymmetric) ---
  else if (mood == EYES_PUZZLED) {
    // Left eye: normal (open)
    // Right eye: lid halfway down (sceptical)
    int puzzledLid = h/2 - 5;
    _display.fillRect(xR, y, w, puzzledLid, SSD1306_BLACK);
  }

  // 5. Blink: a black lid over everything, so it works in any expression
  if (blinkH > 0) {
    _display.fillRect(xL, y, w, blinkH, SSD1306_BLACK);
    _display.fillRect(xR, y, w, blinkH, SSD1306_BLACK);
  }

  // 6. Battery. Shown if you asked for it, and always when it is low:
  //    in that case it blinks, so it cannot go unnoticed.
  if (_battLevel == BATT_LOW_EYES) {
    if (((millis() / BATT_BLINK_MS) % 2) == 0) _drawBatteryIcon();
  } else if (_battShow) {
    if (_battLevel == BATT_USB_EYES) _drawPlugIcon();
    else                                _drawBatteryIcon();
  }

  _display.display();
}

// ==================================================
// === DIAGNOSTICS (handy when bringing up the PCB) ===
// ==================================================

void Eyes::scanBus() {
  Serial.printf("EYES: scanning I2C bus (GPIO%d = SDA, GPIO%d = SCL)...\n",
                EYES_SDA, EYES_SCL);
  byte trovati = 0;
  for (byte addr = 1; addr < 127; addr++) {
    _bus->beginTransmission(addr);
    if (_bus->endTransmission() == 0) {
      Serial.printf("  -> device found at 0x%02X\n", addr);
      trovati++;
    }
  }
  if (trovati == 0) {
    Serial.println("  -> bus silent: check J3 (GND, 3V3, SCL, SDA) and the solder joints.");
  }
}
