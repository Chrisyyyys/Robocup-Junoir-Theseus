// 2026 RCJ Rescue Maze SuperTeam - Robot B (chef) mission logic.
//
// Flow per order (x3), from the strategy doc:
//   1. waitForHandoff()   - sit on the black handoff tile >= 5 s, receive SUM over WiFi
//   2. collect loop       - drive the station route; at each REQUIRED colour stop 3 s
//                           blinking (500/500), then executeResupply() pushes that
//                           station's grey box onto its delivery tile (+10, once each)
//   3. prepareDish()      - fully stationary 10 s anywhere in the Kitchen Area
//   4. dishHandoff()      - back on the black tile, stationary >= 5 s next to Robot A
// After all 3 orders: exitToRedTile() for the exit bonus (stop >= 5 s).
//
// Movement here deliberately does NOT reuse the maze fwd(): fwd() treats black
// floor as a hole and backs off, but in SuperTeam the black tiles are our
// handoff tiles. missionFwd() is a stripped-down straight drive instead
// (encoder distance + gyro heading hold, front-wall safety stop only).
//
// Enable by setting SUPERTEAM_MISSION = true in Main.ino. The WiFi AP itself
// is brought up by initSuperteamComms() in setup() (superteam_comms.ino).
//
// !! COORDINATE WITH ROBOT A'S TEAM: Robot A should start sending the order
// (repeating every ~500 ms until ACKed) only once it is parked on its blue
// handoff tile, so the 5 s handoff windows of the two robots overlap.

// ---------------- timing (rulebook) ----------------
const unsigned long HANDOFF_HOLD_MS = 5000 + 500;  // >= 5 s on adjacent tiles (+margin)
const unsigned long COLLECT_BLINK_MS = 3000 + 500; // >= 3 s continuous 500/500 blink
const unsigned long COOK_HOLD_MS = 10000 + 500;    // >= 10 s fully stationary
const unsigned long EXIT_HOLD_MS = 5000 + 1000;    // >= 5 s on the red tile
const int NUM_ORDERS = 3;
const int NUM_STATIONS = 5;

// ---------------- route tables (MEASURE ON THE REAL FIELD) ----------------
// The SuperTeam field is fixed (rules: layout does not change between runs,
// only order-target SUMs and the ingredient colour order can differ), so the
// kitchen is driven from a hand-written route table instead of the maze
// explorer. Ops: 'F' = forward val mm, 'L'/'R'/'B' = snap-turn left/right/180.
struct RouteStep { char op; int val; };

// TODO(field): all routes below are PLACEHOLDERS - walk the real field with a
// tape measure and rewrite them. Distances are tile centre to tile centre
// (1 tile = 300 mm).
const RouteStep ROUTE_START_TO_HANDOFF[] = { {'F', 300} };
const RouteStep ROUTE_HANDOFF_TO_ST1[]   = { {'B', 0}, {'F', 300} };
const RouteStep ROUTE_BETWEEN_STATIONS[] = { {'F', 300} };  // reused for st1->2, 2->3, 3->4, 4->5
const RouteStep ROUTE_ST5_TO_HANDOFF[]   = { {'B', 0}, {'F', 300} };
const RouteStep ROUTE_HANDOFF_TO_RED[]   = { {'B', 0}, {'F', 300} };
#define ROUTE_LEN(r) (int)(sizeof(r) / sizeof((r)[0]))

// Per-station resupply geometry: which way the grey box sits relative to the
// robot's route heading when parked at that station, and how far to push.
// turn: 'L', 'R', 'B' (or 'N' = no box push at this station).
struct StationCfg { char boxTurn; int pushMm; };
// TODO(field): set the real turn directions/distances per station.
StationCfg stationCfg[NUM_STATIONS] = {
  {'R', 300}, {'R', 300}, {'R', 300}, {'R', 300}, {'R', 300},
};
bool boxPushed[NUM_STATIONS] = {false, false, false, false, false};

// Ingredient colour at each station, in the physical order they are met on
// the route. The rules allow this left-to-right order to differ per run, so:
// TODO(hardware): replace assumedStationColors with a real reading in
// identifyStationColor() (side camera / colour sensor at the target), OR
// update this array by hand during pre-run calibration once the field is
// visible. Uses the ING_* bits from superteam_comms.ino.
uint8_t assumedStationColors[NUM_STATIONS] = {
  ING_BLACK, ING_BLUE, ING_GREEN, ING_RED, ING_YELLOW,
};

Direction missionDir = NORTH; // route-frame heading, tracked like currentDir

// ---------------- low-level helpers ----------------

// stationary hold; keeps calling fullstop so a bumped motor can't creep
void holdStill(unsigned long ms) {
  drivetrain.fullstop();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    drivetrain.fullstop();
    delay(10);
  }
}

// identification LED, 500 ms ON / 500 ms OFF for the whole duration
void blinkIdLed(unsigned long totalMs) {
  drivetrain.fullstop();
  unsigned long t0 = millis();
  while (millis() - t0 < totalMs) {
    digitalWrite(LEDPIN, (((millis() - t0) / 500) % 2 == 0) ? HIGH : LOW);
    delay(5);
  }
  digitalWrite(LEDPIN, LOW);
}

// snap-turn relative to the current route heading and remember the new one.
// quarters: +1 = right, -1 = left, +2 = 180. Uses the same gyro-cardinal
// system as the maze code so headings can't drift over a run.
void missionTurnRel(int quarters) {
  missionDir = (Direction)((((int)missionDir + quarters) % 4 + 4) % 4);
  absoluteturn(turnNeededDeg(missionDir));
  delay(150);
}

// straight drive: encoder distance + gyro heading hold. No black-tile abort,
// no camera servicing, no ramp logic - the SuperTeam field has none of that.
void missionFwd(double dist) {
  double pulses = dist / (wheel_diameter * M_PI) * wheel_cpr * gear_ratio;
  PID gyroPID(1, 0.001, 0.03);
  PID Scale_PID(0.0045, 0, 0.0008);
  double targetHeading = turnNeededDeg(missionDir);
  drivetrain.reset_encoderCount(true, true, true);

  while ((drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3 <= pulses) {
    // front-wall safety stop (same sensors/threshold as fwd())
    int fl = measure(7);
    int fr = measure(1);
    if (fl <= 50 && fl != -1 && fr <= 50 && fr != -1) break;

    double yaw = myGyro.heading() - targetHeading;
    while (yaw > 180.0) yaw -= 360.0;
    while (yaw < -180.0) yaw += 360.0;
    double adjustment = gyroPID.getPID(yaw);
    double Scale = Scale_PID.getPID(pulses - (drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3);
    if (Scale * 120 < 25) break;
    drivetrain.drive(constrain(Scale * (120 - adjustment), 20, 150),
                     constrain(Scale * (120 - adjustment), 20, 150),
                     constrain(Scale * (120 + adjustment), 20, 150),
                     constrain(Scale * (120 + adjustment), 20, 150));
  }
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true, true, true);
}

// straight reverse by encoder distance (quadrature counts go negative)
void missionBackward(double dist) {
  double pulses = dist / (wheel_diameter * M_PI) * wheel_cpr * gear_ratio;
  drivetrain.reset_encoderCount(true, true, true);
  while ((drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3 >= -pulses) {
    drivetrain.backward(150);
  }
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true, true, true);
}

void runRoute(const RouteStep* route, int len) {
  for (int i = 0; i < len; i++) {
    switch (route[i].op) {
      case 'F': missionFwd(route[i].val); break;
      case 'L': missionTurnRel(-1); break;
      case 'R': missionTurnRel(+1); break;
      case 'B': missionTurnRel(+2); break;
    }
    delay(150);
  }
}

// ---------------- mission steps ----------------

// Step 1: order handoff. Already parked on the black tile when called.
// Blocks until an order arrives over WiFi, then keeps holding so the shared
// stationary window with Robot A is >= 5 s. Returns the SUM value.
int waitForHandoff() {
  lcdPrint("waiting order");
  drivetrain.fullstop();
  // flush any stale order captured before we reached the tile
  if (superteamOrderAvailable()) superteamTakeOrder();
  while (!superteamOrderAvailable()) {
    drivetrain.fullstop();
    delay(20);
  }
  int sum = superteamTakeOrder();
  Serial.print("superteam: handoff order SUM=");
  Serial.println(sum);
  holdStill(HANDOFF_HOLD_MS); // A holds its 5 s in parallel
  return sum;
}

// Step 1b: decode SUM -> required-ingredient bitmask (ingredientsForSum lives
// in superteam_comms.ino next to the protocol it belongs to)
uint8_t decodeOrder(int sum) {
  uint8_t needed = ingredientsForSum(sum);
  const char* name =
      (sum == -2) ? "Tteokbokki" :
      (sum == -1) ? "Sujebi" :
      (sum ==  0) ? "Bibimbap" :
      (sum ==  1) ? "Doenjang" :
      (sum ==  2) ? "Galbitang" : "???";
  lcdPrint(name);
  Serial.print("superteam: dish=");
  Serial.print(name);
  Serial.print(" mask=0x");
  Serial.println(needed, HEX);
  return needed;
}

// Which colour is at station i on THIS run.
// TODO(hardware): read the target for real (camera / side colour sensor);
// until then this returns the calibrated assumption table.
uint8_t identifyStationColor(int station) {
  return assumedStationColors[station];
}

// Step 2a: validate a collection - full stop >= 3 s, blinking the whole time
void collectIngredient(int station) {
  lcdPrint("collecting");
  Serial.print("superteam: collecting at station ");
  Serial.println(station);
  blinkIdLed(COLLECT_BLINK_MS);
}

// Step 2b: push this station's grey box onto its delivery tile, then return
// to the route position/heading. Scored once per box, so skipped if done.
void executeResupply(int station) {
  StationCfg& cfg = stationCfg[station];
  if (cfg.boxTurn == 'N' || boxPushed[station]) return;
  lcdPrint("box push");
  int quarters = (cfg.boxTurn == 'L') ? -1 : (cfg.boxTurn == 'B') ? +2 : +1;
  missionTurnRel(quarters);        // face the box
  missionFwd(cfg.pushMm);          // push it over the delivery tile (>half counts)
  missionBackward(cfg.pushMm);     // back out to the route tile
  missionTurnRel(-quarters);       // restore route heading
  boxPushed[station] = true;
}

// Step 3: cook - fully stationary >= 10 s anywhere in the Kitchen Area
void prepareDish() {
  lcdPrint("cooking 10s");
  Serial.println("superteam: preparing dish");
  holdStill(COOK_HOLD_MS);
}

// Step 4: dish handoff - stationary >= 5 s on the black tile next to A.
// No data is sent: Robot A detects our presence with its distance sensors.
void dishHandoff() {
  lcdPrint("dish handoff");
  Serial.println("superteam: dish handoff");
  holdStill(HANDOFF_HOLD_MS);
}

void exitToRedTile() {
  lcdPrint("exiting");
  runRoute(ROUTE_HANDOFF_TO_RED, ROUTE_LEN(ROUTE_HANDOFF_TO_RED));
  holdStill(EXIT_HOLD_MS);
  lcdPrint("run complete");
}

// ---------------- top-level mission ----------------
// Called from loop() when SUPERTEAM_MISSION is true. Runs the whole game,
// then parks. TODO(strategy): Lack of Progress recovery - if the referee
// resets us to the silver start tile mid-run, restart this flow via the
// pause switch (both robots go back to their start tiles per the rules).
void runSuperteamMission() {
  Serial.println("superteam: mission start");
  runRoute(ROUTE_START_TO_HANDOFF, ROUTE_LEN(ROUTE_START_TO_HANDOFF));

  for (int order = 0; order < NUM_ORDERS; order++) {
    int sum = waitForHandoff();
    uint8_t needed = decodeOrder(sum);
    int collected = 0;

    // drive the station loop in physical order; stop only where needed
    // (stopping+blinking at a wrong target is -10, driving past is free)
    for (int st = 0; st < NUM_STATIONS; st++) {
      if (st == 0) runRoute(ROUTE_HANDOFF_TO_ST1, ROUTE_LEN(ROUTE_HANDOFF_TO_ST1));
      else         runRoute(ROUTE_BETWEEN_STATIONS, ROUTE_LEN(ROUTE_BETWEEN_STATIONS));

      uint8_t colorHere = identifyStationColor(st);
      if (needed & colorHere) {
        collectIngredient(st);
        collected++;
        executeResupply(st); // +10 box bonus while we're already parked here
      }
    }

    if (collected > 0) prepareDish();
    else Serial.println("superteam: WARNING no ingredients collected, skipping cook");

    runRoute(ROUTE_ST5_TO_HANDOFF, ROUTE_LEN(ROUTE_ST5_TO_HANDOFF));
    dishHandoff();
  }

  exitToRedTile();
}
