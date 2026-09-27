#include "Remote.h"
#include "page.h"

Remote::Remote()
  : _srv(80),
    _tLastCommand(0),
    _joystickActive(false),
    _fnCommand(nullptr),
    _fnJoystick(nullptr),
    _fnParameter(nullptr),
    _fnTelemetry(nullptr) {}

void Remote::setCommand(void (*fn)(char))                        { _fnCommand = fn; }
void Remote::setJoystick(void (*fn)(int, int, int, int))         { _fnJoystick = fn; }
void Remote::setParameter(void (*fn)(const char *, float))       { _fnParameter = fn; }
void Remote::setTelemetry(String (*fn)())                       { _fnTelemetry = fn; }

bool Remote::begin() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(TELE_SSID, TELE_PASSWORD)) {
    Serial.println("REMOTE: cannot bring up the WiFi network.");
    return false;
  }

  // Without this the WiFi goes into power save and response times bounce
  // between 10 and 200 ms, which you can feel on a joystick.
  WiFi.setSleep(false);

  _srv.on("/", HTTP_GET, [this]() { _servePage(); });
  _srv.on("/c", HTTP_GET, [this]() { _serveCommand(); });

  // iPhones and Androids keep knocking on certain addresses to work out
  // whether the network has Internet. We used to answer with the whole
  // page: ten kilobytes per probe, on a server that handles one request at
  // a time, and the phone kept retrying because it never got the reply it
  // expected. That was one of the causes of the sluggishness. Now it gets
  // the short answer it wants and stops asking.
  _srv.onNotFound([this]() {
    _srv.send(200, "text/html",
              F("<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"));
  });
  _srv.begin();

  Serial.printf("REMOTE: network \"%s\", password \"%s\"\n", TELE_SSID, TELE_PASSWORD);
  Serial.printf("REMOTE: connect and open http://%s\n",
                WiFi.softAPIP().toString().c_str());
  _tLastCommand = millis();
  return true;
}

void Remote::update() {
  _srv.handleClient();

  // If the phone goes quiet with a stick held down (screen locked, page
  // closed, walked out of range) the spider must not carry on by itself.
  if (_joystickActive && millis() - _tLastCommand > TELE_TIMEOUT_MS) {
    _joystickActive = false;
    if (_fnJoystick) _fnJoystick(0, 0, 0, 0);
    Serial.println("REMOTE: no commands arriving, stopping.");
  }
}

void Remote::_servePage() {
  _srv.sendHeader("Cache-Control", "no-store");
  _srv.send_P(200, "text/html", PAGE);
}

void Remote::_serveCommand() {
  _tLastCommand = millis();

  if (_srv.hasArg("b")) {
    String b = _srv.arg("b");
    if (b.length() > 0 && _fnCommand) _fnCommand(b.charAt(0));
  }

  if (_srv.hasArg("j")) {
    // format "lx,ly,rx,ry", each from -100 to 100
    String j = _srv.arg("j");
    int v[4] = {0, 0, 0, 0};
    int start = 0;
    for (int i = 0; i < 4; i++) {
      int comma = j.indexOf(',', start);
      String piece = (comma < 0) ? j.substring(start) : j.substring(start, comma);
      v[i] = constrain(piece.toInt(), -100, 100);
      if (comma < 0) break;
      start = comma + 1;
    }
    _joystickActive = (v[0] || v[1] || v[2] || v[3]);
    if (_fnJoystick) _fnJoystick(v[0], v[1], v[2], v[3]);
  }

  if (_srv.hasArg("p")) {
    // format "name:value"
    String p = _srv.arg("p");
    int duepunti = p.indexOf(':');
    if (duepunti > 0 && _fnParameter) {
      String name = p.substring(0, duepunti);
      float val = p.substring(duepunti + 1).toFloat();
      _fnParameter(name.c_str(), val);
    }
  }

  String t = _fnTelemetry ? _fnTelemetry() : String("{}");
  _srv.sendHeader("Cache-Control", "no-store");
  _srv.send(200, "application/json", t);
}
