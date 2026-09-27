#ifndef REMOTE_H
#define REMOTE_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ==================================================
// === REMOTE - WEB PAGE SERVED BY THE SPIDER ===
// === ESP32-S3-WROOM-1 ===
// ==================================================
//
// The spider brings up its own WiFi network. The phone joins it, you open
// 192.168.4.1 and the interface appears: no app to install, no App Store,
// and it works the same from an iPhone, an Android or a computer.
//
// Why WiFi and not Bluetooth: Safari on iOS does not support Web Bluetooth
// and never has, and every browser on the iPhone is required to use WebKit.
// So a web page simply cannot speak BLE on an iPhone. With WiFi the problem
// does not exist.
//
// No library to install: WiFi.h and WebServer.h already ship with the
// esp32 core.
//
// --- PROTOCOL ---
// One request carries the command and brings the telemetry back, so a
// single round trip does both:
//   GET /c?b=w                  single key (the letters of runCommand)
//   GET /c?j=lx,ly,rx,ry        the two joysticks, -100 to 100
//   GET /c?p=stride:70          one parameter
//   GET /c                      telemetry only
// The reply is always the telemetry JSON.

// Page marker. Bump it whenever the interface changes: the firmware sends
// it in the telemetry and the page compares its own against the one it
// receives, so an old copy cached on the phone flags itself instead of
// making the firmware look broken.
#ifndef PAGE_VERSION
#define PAGE_VERSION "2026-09-20b"
#endif

#ifndef TELE_SSID
#define TELE_SSID "Spider"
#endif
#ifndef TELE_PASSWORD
#define TELE_PASSWORD "spider1234"   // at least 8 characters, or the AP will not start
#endif

// If nothing arrives for this long the remote zeroes the joysticks. It
// matters when the phone locks or leaves the page with a stick held down:
// without it, the spider would wander off on its own.
#ifndef TELE_TIMEOUT_MS
#define TELE_TIMEOUT_MS 1200
#endif

class Remote {
public:
  Remote();

  bool begin();
  void update();                       // call it on every pass of loop()

  bool anyoneConnected() const { return WiFi.softAPgetStationNum() > 0; }
  IPAddress address() const { return WiFi.softAPIP(); }

  // --- HOOKS INTO THE SKETCH ---
  // The remote knows nothing about the spider: it just forwards whatever
  // arrives. That keeps it a module of its own, and tomorrow the same
  // arrangement can serve BLE just as well.
  void setCommand(void (*fn)(char));
  void setJoystick(void (*fn)(int lx, int ly, int rx, int ry));
  void setParameter(void (*fn)(const char *name, float value));
  void setTelemetry(String (*fn)());

private:
  WebServer _srv;
  unsigned long _tLastCommand;
  bool _joystickActive;

  void (*_fnCommand)(char);
  void (*_fnJoystick)(int, int, int, int);
  void (*_fnParameter)(const char *, float);
  String (*_fnTelemetry)();

  void _servePage();
  void _serveCommand();
};

#endif // REMOTE_H
