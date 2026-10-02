// Tuning mode: lets the web tuner (tuner/index.html) read sensors, change PID gains /
// color thresholds / distance offsets live, calibrate, and drive the motors over USB serial,
// so none of that needs a re-flash.
//
// HOW TO ENTER: flip the pause switch to PAUSE, open the tuner page and press Connect. The page
// sends "TUNE" repeatedly; the PAUSE state in loop() calls tuningPollHandshake() and hands control
// to tuningMode(). Flip the switch back to RUN (or press Exit in the page) to leave. The physical
// switch always wins: in tuning mode, switch -> RUN stops the motors and leaves tuning mode.
//
// PROTOCOL (one ASCII line each way, 115200 baud):
//   page -> robot:  TUNE | PING | PARAMS | SET <name> <value> | STREAM <hz> [dgce] | MOT <A-D> <-255..255>
//                   DRIVE <fw|bk|tl|tr> <0..255> | STOP | ENCRESET | CALDIST <sensor> <mm> | CLEARREF
//                   TEST TURN <deg> | TEST FWD <mm> [wall|gyro] [realtiming 0|1] | EXIT
//   robot -> page:  JSON objects, one per line, {"t":"<type>",...}. Any line not starting with '{'
//                   is ordinary debug output from the rest of the firmware and is just shown in a log.
//
// Safety: manual motion (MOT/DRIVE) stops by itself if the page stops refreshing it for
// TUNE_DEADMAN_MS; tests stop on STOP, on the switch, or if the page goes silent.

#define TUNE_FW_VERSION "tune-1"
#define TUNE_LINE_MAX 96
#define TUNE_DEADMAN_MS 400      // manual drive stops if not refreshed within this time
#define TUNE_HOST_TIMEOUT_MS 3000 // tests abort if the page sends nothing (not even PING) this long
#define TUNE_MAX_PARAMS 48

#define TG_DIST  1
#define TG_GYRO  2
#define TG_COLOR 4
#define TG_ENC   8

// ---------------------------------------------------------------------------------------------
// Parameter table. Everything the page can edit. Defaults are snapshotted from the live values
// the first time tuning mode is entered (nothing else writes these), so they always equal the
// compiled-in defaults and are never duplicated here.
// ---------------------------------------------------------------------------------------------
TuneParam tnParams[] = {
  {"center.kp", "PID - wall follower (fwd)", "Steers to keep the right-wall gap. Used every tick of fwd().", &tune.center.kp, TT_DOUBLE, 0, 20, 0.1},
  {"center.ki", "PID - wall follower (fwd)", "Integral term (clamped internally).", &tune.center.ki, TT_DOUBLE, 0, 1, 0.001},
  {"center.kd", "PID - wall follower (fwd)", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.center.kd, TT_DOUBLE, 0, 1e6, 100},
  {"scale.kp", "PID - distance scale (fwd)", "Slows the robot as it nears the target encoder count.", &tune.scale.kp, TT_DOUBLE, 0, 0.05, 0.0001},
  {"scale.ki", "PID - distance scale (fwd)", "Integral term.", &tune.scale.ki, TT_DOUBLE, 0, 0.01, 0.00001},
  {"scale.kd", "PID - distance scale (fwd)", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.scale.kd, TT_DOUBLE, 0, 1e5, 10},
  {"gyro.kp", "PID - heading hold (gyro)", "Currently unused by fwd() (gyro hold is commented out). TEST FWD gyro uses it.", &tune.gyro.kp, TT_DOUBLE, 0, 20, 0.1},
  {"gyro.ki", "PID - heading hold (gyro)", "Integral term.", &tune.gyro.ki, TT_DOUBLE, 0, 0.1, 0.0005},
  {"gyro.kd", "PID - heading hold (gyro)", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.gyro.kd, TT_DOUBLE, 0, 1e6, 100},
  {"climb.kp", "PID - ramp climb", "Heading hold while the robot is on a ramp.", &tune.climb.kp, TT_DOUBLE, 0, 20, 0.1},
  {"climb.ki", "PID - ramp climb", "Integral term.", &tune.climb.ki, TT_DOUBLE, 0, 1, 0.001},
  {"climb.kd", "PID - ramp climb", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.climb.kd, TT_DOUBLE, 0, 1e6, 100},
  {"turn.kp", "PID - turn (absoluteturn)", "Turn speed from heading error in degrees.", &tune.turn.kp, TT_DOUBLE, 0, 20, 0.1},
  {"turn.ki", "PID - turn (absoluteturn)", "Integral term.", &tune.turn.ki, TT_DOUBLE, 0, 1, 0.001},
  {"turn.kd", "PID - turn (absoluteturn)", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.turn.kd, TT_DOUBLE, 0, 1e6, 100},
  {"par.kp", "PID - obstacle parallel", "Squares up to the wall during obstacle avoidance.", &tune.parallel.kp, TT_DOUBLE, 0, 20, 0.1},
  {"par.ki", "PID - obstacle parallel", "Integral term.", &tune.parallel.ki, TT_DOUBLE, 0, 1, 0.001},
  {"par.kd", "PID - obstacle parallel", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.parallel.kd, TT_DOUBLE, 0, 1e6, 100},
  {"wig.kp", "PID - obstacle wiggle", "Side-sensor balance during the wiggle step.", &tune.wiggle.kp, TT_DOUBLE, 0, 50, 0.5},
  {"wig.ki", "PID - obstacle wiggle", "Integral term.", &tune.wiggle.ki, TT_DOUBLE, 0, 1, 0.001},
  {"wig.kd", "PID - obstacle wiggle", "Derivative term. PID.cpp divides by elapsed MICROseconds, so useful values are large (1e3 - 1e5); below ~10 it does almost nothing.", &tune.wiggle.kd, TT_DOUBLE, 0, 1e6, 100},
  {"col.black", "Color thresholds", "Black tile if clear/baseline is below this.", &tune.colBlack, TT_FLOAT, 0, 1, 0.005},
  {"col.silver", "Color thresholds", "Silver tile if the red channel is above this.", &tune.colSilver, TT_INT, 0, 65535, 10},
  {"col.white", "Color thresholds", "White tile if clear/baseline is above this.", &tune.colWhite, TT_FLOAT, 0, 2, 0.005},
  {"col.clear", "Color thresholds", "Baseline clear value everything is divided by (set it over a white tile).", &clear, TT_FLOAT, 1, 65535, 10},
  {"off.1", "Distance offsets (mm)", "Sensor 1 (front right): subtracted from every reading.", &SENSOR_OFFSET_MM[1], TT_INT, -200, 200, 1},
  {"off.2", "Distance offsets (mm)", "Sensor 2 (right front): subtracted from every reading.", &SENSOR_OFFSET_MM[2], TT_INT, -200, 200, 1},
  {"off.3", "Distance offsets (mm)", "Sensor 3 (right back): subtracted from every reading.", &SENSOR_OFFSET_MM[3], TT_INT, -200, 200, 1},
  {"off.4", "Distance offsets (mm)", "Sensor 4 (back): subtracted from every reading.", &SENSOR_OFFSET_MM[4], TT_INT, -200, 200, 1},
  {"off.5", "Distance offsets (mm)", "Sensor 5 (left back): subtracted from every reading.", &SENSOR_OFFSET_MM[5], TT_INT, -200, 200, 1},
  {"off.6", "Distance offsets (mm)", "Sensor 6 (left front): subtracted from every reading.", &SENSOR_OFFSET_MM[6], TT_INT, -200, 200, 1},
  {"off.7", "Distance offsets (mm)", "Sensor 7 (front left): subtracted from every reading.", &SENSOR_OFFSET_MM[7], TT_INT, -200, 200, 1},
};
const int TN_NPARAMS = sizeof(tnParams) / sizeof(tnParams[0]);
double tnDefaults[TUNE_MAX_PARAMS];
bool tnDefaultsSaved = false;

// ---------------------------------------------------------------------------------------------
// Tuning-mode state
// ---------------------------------------------------------------------------------------------
char tnLine[TUNE_LINE_MAX];
int tnLineLen = 0;
bool tnLineOverflow = false;
unsigned long tnLastHost = 0;      // last time a complete line arrived from the page
bool tnExitReq = false;
uint8_t tnStreamMask = 0;
unsigned long tnStreamPeriodMs = 0;
unsigned long tnLastStream = 0;
bool tnMotionOn = false;
unsigned long tnLastMotion = 0;
String tnJ;                        // JSON line being built

// ---------------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------------
bool tnEq(const char* a, const char* b) { // case-insensitive equality
  while (*a && *b) {
    char x = *a++, y = *b++;
    if (x >= 'A' && x <= 'Z') x += 32;
    if (y >= 'A' && y <= 'Z') y += 32;
    if (x != y) return false;
  }
  return *a == 0 && *b == 0;
}

bool tnParseNum(const char* s, double* out) {
  if (s == NULL || *s == 0) return false;
  char* end;
  double v = strtod(s, &end);
  if (*end != 0) return false;
  if (isnan(v) || isinf(v)) return false;
  *out = v;
  return true;
}

double tnWrap180(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void tnBegin(const char* type) {
  tnJ = "{\"t\":\"";
  tnJ += type;
  tnJ += "\"";
}
void tnKey(const char* k) {
  tnJ += ",\"";
  tnJ += k;
  tnJ += "\":";
}
void tnNum(const char* k, double v, int dec) {
  tnKey(k);
  if (isnan(v) || isinf(v)) tnJ += "null";
  else tnJ += String(v, dec);
}
void tnInt(const char* k, long v) {
  tnKey(k);
  tnJ += String(v);
}
void tnStr(const char* k, const char* v) { // characters that would break JSON are replaced
  tnKey(k);
  tnJ += '"';
  for (; *v; v++) tnJ += (*v == '"' || *v == '\\' || (unsigned char)*v < 32) ? '_' : *v;
  tnJ += '"';
}
void tnEnd() {
  tnJ += "}";
  Serial.println(tnJ);
}
void tnOk(const char* cmd) {
  tnBegin("ok");
  tnStr("c", cmd);
  tnEnd();
}
void tnErr(const char* cmd, const char* msg) {
  tnBegin("err");
  tnStr("c", cmd);
  tnStr("m", msg);
  tnEnd();
}

double tnGetParam(const TuneParam* p) {
  switch (p->type) {
    case TT_DOUBLE: return *(double*)p->ptr;
    case TT_FLOAT:  return *(float*)p->ptr;
    default:        return *(int*)p->ptr;
  }
}
void tnPutParam(const TuneParam* p, double v) {
  switch (p->type) {
    case TT_DOUBLE: *(double*)p->ptr = v; break;
    case TT_FLOAT:  *(float*)p->ptr = (float)v; break;
    default:        *(int*)p->ptr = (int)lround(v); break;
  }
}
int tnFindParam(const char* name) {
  for (int i = 0; i < TN_NPARAMS; i++) if (strcmp(tnParams[i].name, name) == 0) return i;
  return -1;
}

void tnSendParam(int i) {
  const TuneParam* p = &tnParams[i];
  tnBegin("param");
  tnStr("n", p->name);
  tnStr("g", p->group);
  tnStr("desc", p->desc);
  tnStr("ty", p->type == TT_INT ? "i" : "f");
  tnNum("v", tnGetParam(p), p->type == TT_INT ? 0 : 6);
  tnNum("d", tnDefaults[i], p->type == TT_INT ? 0 : 6);
  tnNum("lo", p->lo, 6);
  tnNum("hi", p->hi, 6);
  tnNum("st", p->step, 6);
  tnEnd();
}

void tnSendHello() {
  tnBegin("hello");
  tnStr("fw", TUNE_FW_VERSION);
  tnInt("ms", millis());
  tnInt("nparams", TN_NPARAMS);
  tnEnd();
}

// ---------------------------------------------------------------------------------------------
// Serial line reader (non-blocking). Returns true when a full line is waiting in tnLine.
// ---------------------------------------------------------------------------------------------
bool tnReadLine() {
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      bool ok = !tnLineOverflow;
      tnLine[tnLineLen] = 0;
      tnLineLen = 0;
      tnLineOverflow = false;
      tnLastHost = millis();
      if (ok) return true;
      continue; // an over-long line is dropped, keep reading
    }
    if (tnLineLen < TUNE_LINE_MAX - 1) tnLine[tnLineLen++] = (char)ch;
    else tnLineOverflow = true;
  }
  return false;
}

// Called from the PAUSE state in loop(). Non-blocking; true when the page asked for tuning mode.
bool tuningPollHandshake() {
  while (tnReadLine()) {
    if (tnEq(tnLine, "TUNE")) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------------------------
void tnSendTelemetry() {
  tnBegin("s");
  tnInt("ms", millis());
  if (tnStreamMask & TG_DIST) {
    int raw[8];
    for (int i = 1; i <= 7; i++) raw[i] = measureRaw(i);
    tnJ += ",\"r\":[";
    for (int i = 1; i <= 7; i++) { if (i > 1) tnJ += ','; tnJ += String(raw[i]); }
    tnJ += "],\"d\":[";
    for (int i = 1; i <= 7; i++) {
      if (i > 1) tnJ += ',';
      int corrected = (raw[i] == -1) ? -1 : max(0, raw[i] - SENSOR_OFFSET_MM[i]); // same rule as measure()
      tnJ += String(corrected);
    }
    tnJ += "]";
  }
  if (tnStreamMask & TG_GYRO) {
    sensors_event_t ev;
    uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
    i2cMutex.lock();
    bno.getEvent(&ev);
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
    i2cMutex.unlock();
    tnNum("hdg", ev.orientation.x, 1);
    tnNum("rol", ev.orientation.y, 1);
    tnNum("pit", ev.orientation.z, 1);
    tnJ += ",\"cal\":[";
    tnJ += String(calSys); tnJ += ','; tnJ += String(calGyro); tnJ += ',';
    tnJ += String(calAccel); tnJ += ','; tnJ += String(calMag); tnJ += ']';
  }
  if (tnStreamMask & TG_COLOR) {
    uint16_t r = 0, g = 0, b = 0, c = 0;
    i2cMutex.lock();
    myMux.setPort(TCS_PORT);
    tcs.getRawData(&r, &g, &b, &c);
    i2cMutex.unlock();
    tnJ += ",\"col\":[";
    tnJ += String(r); tnJ += ','; tnJ += String(g); tnJ += ',';
    tnJ += String(b); tnJ += ','; tnJ += String(c); tnJ += ']';
    tnNum("ratio", (double)c / clear, 3);
    tnInt("cls", classifyColor(r, g, b, c));
  }
  if (tnStreamMask & TG_ENC) {
    tnJ += ",\"enc\":[";
    tnJ += String(drivetrain.encoderCountA); tnJ += ',';
    tnJ += String(drivetrain.encoderCountB); tnJ += ',';
    tnJ += String(drivetrain.encoderCountD); tnJ += ']';
  }
  tnEnd();
}

// Average RAW readings of one distance sensor against a known true distance and report the
// offset that would make it read true. Nothing is applied: the page decides (SET off.N).
void tnCalDist(int sensor, int trueMm) {
  const int samples = 100;
  long sum = 0;
  int valid = 0, mn = 99999, mx = -1;
  for (int i = 0; i < samples; i++) {
    int v = measureRaw(sensor);
    if (v != -1) {
      sum += v;
      valid++;
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
    delay(10); // let a fresh continuous-ranging sample arrive between reads
  }
  tnBegin("caldist");
  tnInt("n", sensor);
  tnInt("true", trueMm);
  tnInt("valid", valid);
  tnInt("samples", samples);
  if (valid > 0) {
    double mean = (double)sum / valid;
    tnNum("mean", mean, 2);
    tnInt("min", mn);
    tnInt("max", mx);
    tnInt("offset", lround(mean - trueMm));
  }
  tnInt("current", SENSOR_OFFSET_MM[sensor]);
  tnEnd();
}

// Re-baseline `clear` (the value every color ratio is divided by) from the current reading.
// Do this with the sensor over a white tile.
void tnClearRef() {
  const int samples = 20;
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < samples; i++) {
    uint16_t r = 0, g = 0, b = 0, c = 0;
    i2cMutex.lock();
    myMux.setPort(TCS_PORT);
    tcs.getRawData(&r, &g, &b, &c);
    i2cMutex.unlock();
    if (c > 0) { sum += c; valid++; }
  }
  if (valid == 0) { tnErr("CLEARREF", "color sensor returned no light"); return; }
  clear = (float)sum / valid;
  tnBegin("clearref");
  tnNum("clear", clear, 1);
  tnEnd();
}

// ---------------------------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------------------------
// Positive speed = that wheel drives the robot forward (motor D is mounted mirrored, like in fw()).
void tnMotor(char id, int speed) {
  Adafruit_DCMotor* m = (id == 'A') ? motorA : (id == 'B') ? motorB : (id == 'C') ? motorC : motorD;
  bool inverted = (id == 'D');
  uint8_t dir = ((speed >= 0) != inverted) ? FORWARD : BACKWARD;
  i2cMutex.lock();
  m->run(dir);
  m->setSpeed(abs(speed));
  i2cMutex.unlock();
}

void tnStopMotion() {
  drivetrain.fullstop();
  tnMotionOn = false;
}

// ---------------------------------------------------------------------------------------------
// PID tests. These mirror the control laws in absoluteturn() and fwd() (same speed limits, same
// time/scale exits) but without the map / camera / victim / ramp / obstacle handling, so they can run
// on the bench. Gains are re-read every tick, so SET commands during a test act immediately.
// If you change the control law in movement.ino, change it here too.
// ---------------------------------------------------------------------------------------------
bool tnHandleLine(char* line, bool busy);

bool tnTestShouldAbort(const char** why) {
  while (tnReadLine()) {
    if (tnHandleLine(tnLine, true)) { *why = tnExitReq ? "exit" : "stopped"; return true; }
  }
  if (digitalRead(logicswitch) == LOW) { *why = "switch"; return true; }
  if (millis() - tnLastHost > TUNE_HOST_TIMEOUT_MS) { *why = "host-silent"; return true; }
  return false;
}

void tnTrace(const double* v, int n) {
  tnBegin("tr");
  tnJ += ",\"v\":[";
  for (int i = 0; i < n; i++) { if (i) tnJ += ','; tnJ += isnan(v[i]) ? String("null") : String(v[i], 3); }
  tnJ += "]";
  tnEnd();
}

void tnTestHeader(const char* name, const char* cols, const char* info) {
  tnBegin("test");
  tnStr("name", name);
  tnKey("cols");
  tnJ += cols; // pre-formatted JSON array of column names
  tnStr("info", info);
  tnEnd();
}

void tnTestDone(const char* name, const char* why, double finalErr, double extra, unsigned long ms) {
  tnBegin("test_done");
  tnStr("name", name);
  tnStr("why", why);
  tnNum("final_err", finalErr, 2);
  tnNum("extra", extra, 1);
  tnInt("ms", ms);
  tnEnd();
}

// Turn by relDeg (+ = clockwise/right) using the turn PID, like absoluteturn().
void tnTestTurn(double relDeg) {
  tnStopMotion();
  PID pid(tune.turn.kp, tune.turn.ki, tune.turn.kd);
  double startHdg = myGyro.heading();
  double target = startHdg + relDeg;
  while (target >= 360.0) target -= 360.0;
  while (target < 0.0) target += 360.0;
  double diff = tnWrap180(target - startHdg);
  bool right = (diff > 0);
  double initAbs = fabs(diff);
  unsigned long limitMs = (unsigned long)(2.0 * initAbs / 90.0 * 1000.0); // same cut-off as absoluteturn()

  char info[48];
  snprintf(info, sizeof(info), "start %d deg, target %d deg, limit %lu ms", (int)startHdg, (int)target, limitMs);
  tnTestHeader("turn", "[\"time_ms\",\"error_deg\",\"pid_out\",\"speed\",\"heading\"]", info);

  const char* why = "limit";
  unsigned long t0 = millis(), lastTrace = 0;
  while (true) {
    if (tnTestShouldAbort(&why)) break;
    unsigned long el = millis() - t0;
    if (el > limitMs) { why = "limit"; break; }
    pid.kp = tune.turn.kp; pid.ki = tune.turn.ki; pid.kd = tune.turn.kd;
    double h = myGyro.heading();
    double d = tnWrap180(target - h);
    double out = pid.getPID(fabs(d));
    int spd = (int)constrain(out, 20, 150);
    if (right) drivetrain.turnright(spd); else drivetrain.turnleft(spd);
    if (el - lastTrace >= 15) {
      lastTrace = el;
      double v[5] = {(double)el, d, out, (double)spd, h};
      tnTrace(v, 5);
    }
  }
  unsigned long total = millis() - t0;
  tnStopMotion();
  delay(300); // let it coast to a stop, then report where it really ended up
  double finalErr = tnWrap180(target - myGyro.heading());
  tnTestDone("turn", why, finalErr, 0, total);
}

// Drive distMm using the scale PID for speed and either the wall follower (center()) or a gyro
// heading hold for steering, like fwd(). realTiming reads the color sensor every tick as the real
// fwd() does, so the loop rate (and therefore the derivative term) matches competition.
void tnTestFwd(double distMm, bool useGyro, bool realTiming) {
  tnStopMotion();
  drivetrain.reset_encoderCount(true, true, true);
  double pulses = distMm / (wheel_diameter * M_PI) * wheel_cpr * gear_ratio;
  PID centerPID(tune.center.kp, tune.center.ki, tune.center.kd);
  PID gyroPID(tune.gyro.kp, tune.gyro.ki, tune.gyro.kd);
  PID scalePID(tune.scale.kp, tune.scale.ki, tune.scale.kd);
  double initYaw = myGyro.heading();

  char info[48];
  snprintf(info, sizeof(info), "%d mm, steer by %s, %lu pulses", (int)distMm, useGyro ? "gyro" : "wall", (unsigned long)pulses);
  tnTestHeader("fwd", "[\"time_ms\",\"error\",\"pid_out\",\"scale\",\"speed_AC\",\"speed_BD\",\"mm\"]", info);

  const char* why = "distance";
  unsigned long t0 = millis(), lastTrace = 0;
  double mm = 0;
  while (true) {
    if (tnTestShouldAbort(&why)) break;
    unsigned long el = millis() - t0;
    if (el > 15000) { why = "timeout"; break; }
    long enc = ((long)drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3;
    mm = ((double)enc / wheel_cpr) / gear_ratio * wheel_diameter * M_PI;
    if (enc > pulses) { why = "distance"; break; }

    if (realTiming) {
      uint16_t r, g, b, c;
      i2cMutex.lock();
      myMux.setPort(TCS_PORT);
      tcs.getRawData(&r, &g, &b, &c); // only for the loop timing; the value is unused
      i2cMutex.unlock();
    }
    centerPID.kp = tune.center.kp; centerPID.ki = tune.center.ki; centerPID.kd = tune.center.kd;
    gyroPID.kp = tune.gyro.kp; gyroPID.ki = tune.gyro.ki; gyroPID.kd = tune.gyro.kd;
    scalePID.kp = tune.scale.kp; scalePID.ki = tune.scale.ki; scalePID.kd = tune.scale.kd;

    double err, adj;
    if (useGyro) {
      err = tnWrap180(myGyro.heading() - initYaw);
      adj = gyroPID.getPID(err);
    } else {
      err = center();
      adj = centerPID.getPID(err);
    }
    double scale = scalePID.getPID(pulses - enc);

    int fl = measure(7), fr = measure(1); // same emergency stop as fwd()
    if (fl <= 50 && fl != -1 && fr <= 50 && fr != -1) { why = "front-wall"; break; }
    if (scale * 120 < 25) { why = "scale-exit"; break; } // same exit as fwd()

    double spdAC = constrain(scale * (120 - adj), 20, 150);
    double spdBD = constrain(scale * (120 + adj), 20, 150);
    drivetrain.drive(spdAC, spdAC, spdBD, spdBD); // drive(A, C, B, D)
    if (el - lastTrace >= 15) {
      lastTrace = el;
      double v[7] = {(double)el, err, adj, scale, spdAC, spdBD, mm};
      tnTrace(v, 7);
    }
  }
  unsigned long total = millis() - t0;
  tnStopMotion();
  delay(300);
  long encEnd = ((long)drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3;
  double mmEnd = ((double)encEnd / wheel_cpr) / gear_ratio * wheel_diameter * M_PI;
  tnTestDone("fwd", why, mmEnd - distMm, mmEnd, total); // final_err = overshoot (+) / undershoot (-) in mm
}

// ---------------------------------------------------------------------------------------------
// Command handling. Returns true if the command asks to abort a running test / leave tuning mode.
// `busy` = a test is running: only PING, SET, STOP and EXIT are accepted.
// ---------------------------------------------------------------------------------------------
bool tnHandleLine(char* line, bool busy) {
  char* argv[6];
  int argc = 0;
  char* tok = strtok(line, " \t");
  while (tok && argc < 6) { argv[argc++] = tok; tok = strtok(NULL, " \t"); }
  if (argc == 0) return false;
  const char* cmd = argv[0];

  if (tnEq(cmd, "PING")) { tnBegin("pong"); tnInt("ms", millis()); tnEnd(); return false; }
  if (tnEq(cmd, "STOP")) { tnStopMotion(); tnOk("STOP"); return true; }
  if (tnEq(cmd, "EXIT")) { tnStopMotion(); tnExitReq = true; return true; }
  if (tnEq(cmd, "TUNE")) { tnSendHello(); return false; }

  if (tnEq(cmd, "SET")) {
    double v;
    if (argc < 3 || !tnParseNum(argv[2], &v)) { tnErr("SET", "usage: SET <name> <number>"); return false; }
    int i = tnFindParam(argv[1]);
    if (i < 0) { tnErr("SET", "unknown parameter"); return false; }
    if (v < tnParams[i].lo || v > tnParams[i].hi) { tnErr("SET", "value out of range"); return false; }
    tnPutParam(&tnParams[i], v);
    tnBegin("set");
    tnStr("n", tnParams[i].name);
    tnNum("v", tnGetParam(&tnParams[i]), tnParams[i].type == TT_INT ? 0 : 6);
    tnEnd();
    return false;
  }

  if (busy) { tnErr(cmd, "busy: send STOP first"); return false; }

  if (tnEq(cmd, "PARAMS")) {
    for (int i = 0; i < TN_NPARAMS; i++) tnSendParam(i);
    tnBegin("params_end");
    tnInt("n", TN_NPARAMS);
    tnEnd();
    return false;
  }

  if (tnEq(cmd, "STREAM")) {
    double hz;
    if (argc < 2 || !tnParseNum(argv[1], &hz) || hz < 0 || hz > 30) { tnErr("STREAM", "usage: STREAM <0..30> [dgce]"); return false; }
    uint8_t mask = 0;
    const char* groups = (argc >= 3) ? argv[2] : "dgce";
    for (const char* p = groups; *p; p++) {
      if (*p == 'd') mask |= TG_DIST;
      else if (*p == 'g') mask |= TG_GYRO;
      else if (*p == 'c') mask |= TG_COLOR;
      else if (*p == 'e') mask |= TG_ENC;
    }
    tnStreamMask = (hz == 0) ? 0 : mask;
    tnStreamPeriodMs = (hz == 0) ? 0 : (unsigned long)(1000.0 / hz);
    tnLastStream = 0;
    tnOk("STREAM");
    return false;
  }

  if (tnEq(cmd, "MOT")) {
    double sp;
    char id = (argc >= 2) ? (char)toupper(argv[1][0]) : 0;
    if (argc < 3 || argv[1][1] != 0 || id < 'A' || id > 'D' || !tnParseNum(argv[2], &sp) || sp < -255 || sp > 255) {
      tnErr("MOT", "usage: MOT <A|B|C|D> <-255..255>");
      return false;
    }
    tnMotor(id, (int)sp);
    tnMotionOn = true;
    tnLastMotion = millis();
    return false; // no reply: the page re-sends this every ~100 ms as a keep-alive
  }

  if (tnEq(cmd, "DRIVE")) {
    double sp;
    if (argc < 3 || !tnParseNum(argv[2], &sp) || sp < 0 || sp > 255) { tnErr("DRIVE", "usage: DRIVE <fw|bk|tl|tr> <0..255>"); return false; }
    int s = (int)sp;
    if (tnEq(argv[1], "fw")) drivetrain.fw(s);
    else if (tnEq(argv[1], "bk")) drivetrain.backward(s);
    else if (tnEq(argv[1], "tl")) drivetrain.turnleft(s);
    else if (tnEq(argv[1], "tr")) drivetrain.turnright(s);
    else { tnErr("DRIVE", "direction must be fw, bk, tl or tr"); return false; }
    tnMotionOn = true;
    tnLastMotion = millis();
    return false; // keep-alive command, no reply
  }

  if (tnEq(cmd, "ENCRESET")) {
    drivetrain.reset_encoderCount(true, true, true);
    tnOk("ENCRESET");
    return false;
  }

  if (tnEq(cmd, "CALDIST")) {
    double n, mm;
    if (argc < 3 || !tnParseNum(argv[1], &n) || !tnParseNum(argv[2], &mm) || n < 1 || n > 7 || mm < 20 || mm > 1000) {
      tnErr("CALDIST", "usage: CALDIST <1..7> <20..1000 mm>");
      return false;
    }
    tnCalDist((int)n, (int)mm);
    return false;
  }

  if (tnEq(cmd, "CLEARREF")) { tnClearRef(); return false; }

  if (tnEq(cmd, "TEST")) {
    if (argc >= 3 && tnEq(argv[1], "TURN")) {
      double deg;
      if (!tnParseNum(argv[2], &deg) || fabs(deg) < 2 || fabs(deg) > 360) { tnErr("TEST", "usage: TEST TURN <2..360 deg, - for left>"); return false; }
      tnTestTurn(deg); // all arguments are parsed before the test starts: it reuses the line buffer
      return tnExitReq;
    }
    if (argc >= 3 && tnEq(argv[1], "FWD")) {
      double mm;
      if (!tnParseNum(argv[2], &mm) || mm < 50 || mm > 1200) { tnErr("TEST", "usage: TEST FWD <50..1200 mm> [wall|gyro] [realtiming 0|1]"); return false; }
      bool useGyro = (argc >= 4 && tnEq(argv[3], "gyro"));
      bool realTiming = !(argc >= 5 && tnEq(argv[4], "0"));
      tnTestFwd(mm, useGyro, realTiming);
      return tnExitReq;
    }
    tnErr("TEST", "usage: TEST TURN <deg> | TEST FWD <mm> [wall|gyro] [realtiming 0|1]");
    return false;
  }

  tnErr(cmd, "unknown command");
  return false;
}

// ---------------------------------------------------------------------------------------------
// Entry point, called from the PAUSE state in loop(). Blocks until the page sends EXIT or the
// pause switch is flipped back to RUN.
// ---------------------------------------------------------------------------------------------
void tuningMode() {
  tnStopMotion();
  if (!tnDefaultsSaved) { // nothing else writes these, so right now they are the compiled-in defaults
    for (int i = 0; i < TN_NPARAMS; i++) tnDefaults[i] = tnGetParam(&tnParams[i]);
    tnDefaultsSaved = true;
  }
  tnExitReq = false;
  tnStreamMask = 0;
  tnLastHost = millis();
  tnLineLen = 0;
  tnLineOverflow = false;
  tnSendHello();
  lcdPrint("TUNE MODE"); // blocks ~1 s; the hello above is already out

  const char* why = "exit";
  while (true) {
    if (digitalRead(logicswitch) == LOW) { why = "switch"; break; }
    if (tnExitReq) { why = "exit"; break; }
    while (tnReadLine()) {
      tnHandleLine(tnLine, false);
      if (tnExitReq) break;
    }
    if (tnMotionOn && millis() - tnLastMotion > TUNE_DEADMAN_MS) tnStopMotion(); // page stopped refreshing
    if (tnStreamMask && millis() - tnLastStream >= tnStreamPeriodMs) {
      tnLastStream = millis();
      tnSendTelemetry();
    }
    delay(1);
  }
  tnStopMotion();
  tnStreamMask = 0;
  tnBegin("exit");
  tnStr("why", why);
  tnEnd();
}
