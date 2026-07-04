#include <mbed.h> // access arduino mbed OS (rtos::Thread)
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Stepper.h>
#include <LiquidCrystal.h> // lcd screen
#include <array> // std::array (Grid type for multi-floor maps)
#include <deque>
#include <vector>
#include <utility>

#include <ArduinoQueue.h> // queue
#include <Vector.h> // vector
#include "PID.h"
#include "timer.h"
#include "gyro.h"
#include "dispenser.h"
#include "motors.h"
// movement constants
#define MIN_DIST 120         // mm (tune this)
#define OBSTACLE_DIST 90
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define ROBOT_LENGTH_MM 170                                      // mm, robot front-to-back length
#define TARGET_GAP_MM (((double)TILE_MM - ROBOT_LENGTH_MM) / 2.0) // mm, ideal front/back clearance when centered (52.5)
#define CENTER_TOL_MM 10                                          // mm, front-back centering tolerance
#define MAX_CENTER_CORRECTION_MM 300.0                            // mm, one tile — offset this large means an unreliable reading or the robot isn't really in-tile; skip/abort centering
#define ROBOT_WIDTH_MM 140                                          // mm, robot left-right width
#define TARGET_SIDE_GAP_MM (((double)TILE_MM - ROBOT_WIDTH_MM) / 2.0) // mm, ideal side-wall clearance when centered (80)
#define SIDE_WALL_MAX_MM 200                                        // mm; a side reading beyond this is the next tile through a gap, not this tile's wall
#define LATERAL_TOL_MM 15                                            // mm, lateral correction tolerance (looser than CENTER_TOL_MM)
#define MAX_LATERAL_OFFSET_MM 90.0                                   // mm, sanity cap — offset this large means an unreliable reading; skip
#define LATERAL_CORRECTION_GAIN 1                                // multiplier on the computed turn angle; bench-tune upward since fwd() partially fights the pre-turn (pulls back toward cardinal)
#define BLACK_THRESHOLD 0.1f // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 800 // use red value
#define WHITE_THRESHOLD 0.85f
#define MULTIPLER 1.1
#define WALL_MISMATCH_THRESHOLD 2 // >= this many of the 4 absolute walls disagreeing with the stored tile flags a position mismatch

#define TARGET_WALL_DISTANCE 80
float clear; 

#include "MazeTile.h"

// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// shut down allcall
#define PCA_ADDR 0x60
#define MODE1    0x00
#define ALLCALL_BIT 0x01  // MODE1 bit0

// set up color sensor
#define TCS_PORT 7
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
// set up gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
gyro myGyro;
// set up motorshield and motors.
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorA = AFMS.getMotor(1);
Adafruit_DCMotor *motorB = AFMS.getMotor(2);
Adafruit_DCMotor *motorC = AFMS.getMotor(3);
Adafruit_DCMotor *motorD = AFMS.getMotor(4);

// set up encoder pins
const int encoderPin_A_A = 3;
const int encoderPin_A_B = 5; 
const int encoderPin_B_A = 2;
const int encoderPin_B_B = 4; 
const int encoderPin_D_A = 18;
const int encoderPin_D_B = 19;


//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B,encoderPin_D_A,encoderPin_D_B);
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 80; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};

// create stepper object
const int steps_per_revolution = 2048;
Stepper myStepper = Stepper(steps_per_revolution, 8, 9,10,11); 
// create lcd object
int en = 25; int rs = 27; int d4 = 23; int d5 = 53; int d6 = 29; int d7 = 31;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// map grids 
// MAP_SIZE and grid are defined here
const int MAP_SIZE = 40;
using Grid = std::array<std::array<Tile, MAP_SIZE>, MAP_SIZE>;
Grid mapGrid; // active floor's tiles
Grid m1;      // floor storage ("basement"/floor 0)
Grid m2;      // floor 1
Grid m3;      // floor 2

int currentFloor = 0; // current floor (0..2) for elevation()/descend()
int LEDPIN = 51;


//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  CENTERING,
  UPDATE_MAP,
  PLAN_NEXT,
  VICTIM_DETECT,
  EXECUTE_MOVE,
  BOTCHED_TURN_RECOVERY,
  BOTCHED_FWD_RECOVERY,
  BACKPEDAL,
  PAUSE,
  RETURN
};
enum Steps {
  TURN,
  PARALLEL,
  BACKTRACK,
  WIGGLE,
  FWD
};
// coord struct
struct coord {
  int x;
  int y;
};
Steps steps = TURN;

// initialize 

Direction currentDir = NORTH;     // robot heading in map coords (0..3)
int plannedTurnDeg = 0;           // -90,0,+90,180
Direction plannedMoveDir = NORTH; // absolute direction robot will move next
bool turnCompletedForMove = false;
// bound BOTCHED_TURN_RECOVERY so a persistently un-completable turn can't cycle forever
int botchedTurnAttempts = 0;
const int MAX_BOTCHED_TURN_ATTEMPTS = 3;
int x_pos = MAP_SIZE/2;
int y_pos = MAP_SIZE/2;
RobotState state = SENSE_TILE;
// maze return to start condition variables
int medkits = 8;
timer mazeTime;
// black blue toggles
bool blacktoggle = false;
bool bluetoggle = false;
bool stairtoggle = false;
// obstacle toggle
bool obstacle = false;
// victim toggles
bool victimtoggle = false;
bool victimAtCurrent = false;
// camera GPIOs
const int gpio1 = 13;
const int gpio2 = 12;
// stepper variables
const int angle_offset = 44;
const int angle_increment = 22;
dispenser disp(angle_increment,angle_offset,steps_per_revolution);
// logic switch pin
const int logicswitch = 22;
volatile bool Pausemaze = false; // set by pauseThread, read by loop()
volatile bool moveInterrupted = false; // fwd() sets true when a pause aborts the move before the tile is completed
int x_checkpoint = MAP_SIZE/2, y_checkpoint = MAP_SIZE/2;
int floor_checkpoint = 0; // floor the last checkpoint was recorded on (0..2)
bool tilecheck = false;

// Forward declaration: Arduino can't auto-prototype template return types.
std::deque<std::pair<int, std::pair<int,int>>> BFS(std::pair<int, std::pair<int,int>> currentpos, Grid& m1, Grid& m2, Grid& m3, std::pair<int, std::pair<int,int>> endpos, bool allowBlue = false, bool allowObstacle = false);

double headingErrorDeg(double targetDeg, double actualDeg) {
  double err = targetDeg - actualDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return abs(err);
}

// ===== camera victim-detection RTOS thread =====
// The thread only checks the camera UARTs (Serial3 = left, Serial2 = right).
// -> It never touches the I2C bus (mux/distance/color) so it cannot interfere w/ the main context's measure()/detectWall() calls. 

// When a camera reports a letter while the robot is moving, the thread raises victimPending; fwd()/absoluteturn()
// then stop the drivetrain, pause their PID + timer, run detectCam(), and use markVictimAtEncoderPosition() to label the correct tile before resuming.

volatile bool fwdActive = false; // true only while inside fwd()
volatile bool turnActive = false;
volatile bool victimPending = false; // a camera reported -> movement must service it
volatile int  victimSide = 0;        // 1 = left (Serial3), 2 = right (Serial2)
volatile bool isVictim = false;      // a victim already handled during current move

rtos::Thread cameraThread;
rtos::Mutex i2cMutex;
rtos::Mutex lcdMutex; // lcd mutex to prevent conflict
void cameraTask(){
  while(true){
    int encoderCount = (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
    int nx = x_pos; int ny=y_pos;
    if((fwdActive||turnActive) && !victimPending && !isVictim){
      
      //if(encoderCount>=0.3*pulsesForDistanceMm(TILE_MM)||encoderCount<=0.7*pulsesForDistanceMm(TILE_MM)){
        if(readSerial1() != -1){        // left camera (Serial4)
          if(fwdActive) victimTileFromEncoder(TILE_MM,encoderCount,nx,ny);
          Serial.println("nx, ny");
          Serial.println(nx);
          Serial.println(ny);
          Serial.println(mapGrid[nx][ny].getVictim());
          if(mapGrid[nx][ny].getVictim() == false){
            i2cMutex.lock();
            victimSide = 1;
            drivetrain.fullstop();
            victimPending = true;
            i2cMutex.unlock();
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
            i2cMutex.lock();
            serviceCameraVictim();
            i2cMutex.unlock();
          }
        }
        else if(readSerial2() != -1){   // right camera (Serial3)
          if(fwdActive) victimTileFromEncoder(TILE_MM,encoderCount,nx,ny);
          Serial.println("nx, ny");
          Serial.println(nx);
          Serial.println(ny);
          Serial.println(mapGrid[nx][ny].getVictim());
          if(mapGrid[nx][ny].getVictim() == false){
            i2cMutex.lock();
            victimSide = 2;
            drivetrain.fullstop();
            victimPending = true;
            i2cMutex.unlock();
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
            i2cMutex.lock();
            serviceCameraVictim();
            i2cMutex.unlock();
          }
        }
      }
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
  }
}


// pause maze thread: watches the logic switch and requests a stop.
rtos::Thread pauseThread;
void pauseTask(){
  while(true){
    
    if(digitalRead(logicswitch)==HIGH){
      
      Pausemaze = true;
    }
    else{
      
      Pausemaze = false;
    }
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool turnCompletedSuccessfully(Direction intendedDir) {
  const double TURN_SUCCESS_TOLERANCE_DEG = 20.0;
  double targetHeading = turnNeededDeg(intendedDir);
  double actualHeading = myGyro.heading();
  double err = headingErrorDeg(targetHeading, actualHeading);
  Serial.print("turn target=");
  Serial.print(targetHeading);
  Serial.print(", actual=");
  Serial.print(actualHeading);
  Serial.print(", err=");
  Serial.println(err);
  return err <= TURN_SUCCESS_TOLERANCE_DEG;
}
void setup(){
  // initialize camera gpio pins
  pinMode(gpio1, INPUT);
  pinMode(gpio2, INPUT);
  // initialize logic switch pin
  pinMode(logicswitch, INPUT);
  pinMode(LEDPIN,OUTPUT);
  // begin UART communication.
  Serial.begin(115200);
  Serial3.begin(115200); // switch to 9600 for reliability
  Serial4.begin(115200);
  
  
  Wire.begin();
  disableAllCall();
  myMux.begin();
  init_dist(); // initialize mux before distance sensors.
  scanAllPorts();
  init_color();
  init_drive();
  //detect();
  //initialize map
  initializeMap(); // initialize mapgrid
  // every floor starts as a copy of the freshly initialized (empty) grid.
  m1 = mapGrid;
  m2 = mapGrid;
  m3 = mapGrid;
  currentFloor = 0;
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].setDiscovered(true);
  currentDir = NORTH;
  state = SENSE_TILE;
  // start lcd
  lcd.begin(16, 2);
  // SuperTeam: bring up the WiFi access point + UDP listener for Robot A's
  // order messages (starts its own RTOS thread; non-fatal if WiFi fails).
  initSuperteamComms();
  // start RTOS threads: camera victim detection + pause-switch watcher.
  cameraThread.start(cameraTask);
  cameraThread.set_priority(osPriorityAboveNormal);
  pauseThread.start(pauseTask);
  //Serial.println("starting");
  
  
}
int iterator = 0;

// SuperTeam mode: true = run the Robot B chef mission (superteam_mission.ino)
// instead of the maze state machine. Must stay ABOVE the AP-test block below,
// which would otherwise consume the order before waitForHandoff() sees it.
const bool SUPERTEAM_MISSION = false;

void loop(){
  if (SUPERTEAM_MISSION) {
    runSuperteamMission();          // runs the whole game (3 orders + exit)
    while (true) {                  // park forever when done
      drivetrain.fullstop();
      delay(1000);
    }
  }

  // ---- SuperTeam AP/UDP test: print any order received over WiFi ----
  if (superteamOrderAvailable()) {
    int sum = superteamTakeOrder();
    Serial.print("AP TEST: received order SUM=");
    Serial.println(sum);
  }
  // ---- end AP test ----
for(int i=1;i<=7;i++){
  Serial.print("sensor"+i);
  Serial.print(measure(i));
}
  //diagPrintStackUsage();

  /*
  for(int i = 1;i<=7;i++){
    Serial.print("sensor ");
    Serial.println(i);
    Serial.println(measure(i));
    delay(500);
  }
  
  */
  
  
  //lcdPrint("working");
  //delay(500);
  //drivetrain.drive(150,150*1.25,150*1.25,150);
  //drivetrain.drive(150,150,150,150);
  
  
  static bool wallF, wallR, wallB, wallL;
  switch (state) {
    case SENSE_TILE: {
      // reset per-tile toggles
      blacktoggle = false; bluetoggle = false; victimtoggle = false; obstacle = false;
      // Read for walls
      Serial.println("reading walls");
      readWallsRel(wallF, wallR, wallB, wallL);
      // re-sense: does this tile actually match what the map already recorded for it?
      tilecheck = checkTileMismatch(wallF, wallR, wallB, wallL);

      delay(200);
      state = UPDATE_MAP; // next state.
      // Auto-trigger front-back centering >> only when a front wall is present, off-center beyond CENTER_TOL_MM, and the offset isn't too large (>= one tile) that the reading is unreliable. 
      // Back-wall centering isn't implemented yet, so wallB is not checked here.
      
      if(wallF == true){
        int front1 = measure(1);
        int front7 = measure(7);
        if(front1 != -1 && front7 != -1){
          double frontGap = (front1 + front7) / 2.0;
          double offset = frontGap - TARGET_GAP_MM;
          if(abs(offset) > CENTER_TOL_MM && abs(offset) < MAX_CENTER_CORRECTION_MM) state = CENTERING;
        }
      }
      
      if(Pausemaze == true){
        Serial.println("pause");
        state=PAUSE;
        break;
      }
      break;
    }
    case CENTERING: {
      Serial.println("front/back centering in tile");
      centerFrontBack();
      state = UPDATE_MAP;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case UPDATE_MAP: {
      Serial.println("updating tile");
      // skip the write on a mismatch: preserve the already-trusted wall data for
      // this cell rather than overwriting it with a reading taken while the
      // robot's position belief may be wrong.
      if(!tilecheck) writeWallsToCurrentTile(wallF, wallR, wallB, wallL);
      else Serial.println("tile mismatch detected - preserving existing map data for this tile");
      updateFullyExploredAt(x_pos, y_pos);
      state = VICTIM_DETECT; // poll cameras while stopped before planning.
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case VICTIM_DETECT: {
      
      Serial.println("victim detect");
      state = PLAN_NEXT;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case PLAN_NEXT: {
      Serial.println("plan next");
      plannedMoveDir = pickNextDirection();
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      Serial.println(plannedTurnDeg);
      state = EXECUTE_MOVE;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case EXECUTE_MOVE: {
      if (turnCompletedForMove == false) {
        if(plannedMoveDir != currentDir){
          absoluteturn(plannedTurnDeg);
        }
        delay(200);
        parallel();
        delay(100);

        if (turnCompletedSuccessfully(plannedMoveDir) == false) {
          state = BOTCHED_TURN_RECOVERY;
          break;
        }
        currentDir = plannedMoveDir;
        turnCompletedForMove = true;
        botchedTurnAttempts = 0; // clean turn -> reset the recovery counter
      }
      fwd(TILE_MM);
      // A pause aborted the move before the tile was completed: don't advance
      // position or write walls/edges (the robot didn't actually traverse the tile).
      if(moveInterrupted == true){
        if(Pausemaze == true) state = PAUSE;
        break;
      }
      // update map + robot position only on a successful (non-black) move
      if(blacktoggle == false){
        markEdgeBothWays(x_pos, y_pos, currentDir);
        stepForward(currentDir, x_pos, y_pos); // x_pos/y_pos now = new tile
        // read blue only after the move completes, on the tile just entered
        int color = read_color();
        if(color == 1) bluetoggle = true;
        if(bluetoggle == true){
          delay(5000);
          mapGrid[x_pos][y_pos].setType(BLUE);
        }
        
        if(obstacle == true){
          //set obstacle type
          //make sure to prevent return to the tile with obstacle in the future.
          int nx = x_pos; int ny = y_pos;
          stepForward(currentDir, nx, ny);
          mapGrid[x_pos][y_pos].setObstacle(currentDir, true); // connected
          mapGrid[nx][ny].setObstacle(opposite(currentDir), true); // update both sides.
        }
        
      }
      else{
        
        state = BACKPEDAL; // black tile ahead (marked BLACK by fwd) -> back off
        turnCompletedForMove = false;
        break;
      }

      delay(200);
      parallel();
      delay(100);
      iterator += 1;

      isVictim = false;
      turnCompletedForMove = false;
      tilecheck = false;
      state = SENSE_TILE;
      if(Pausemaze == true) state = PAUSE;
      //if(mazeTime.getTime() >= 1000000*60*6) state = RETURN;
      //if(medkits <= 0) state = RETURN;
      if(iterator >= 25) state = RETURN;
      break;
    }
    case BACKPEDAL: {
      plannedMoveDir = pickNextDirection();
      Serial.println("next direction picked");
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      state = EXECUTE_MOVE;
      blacktoggle = false;
      if(Pausemaze == true) state = PAUSE;
      delay(200);
      break;
    }
    case BOTCHED_TURN_RECOVERY: {
      if(Pausemaze == true){
        state = PAUSE;
        break;
      }
      botchedTurnAttempts += 1;
      Direction snappedDir = (Direction)myGyro.headingToCardinal(myGyro.heading());
      int snappedHeading = turnNeededDeg(snappedDir);
      Serial.println("botched turn detected, snapping to cardinal");
      absoluteturn(snappedHeading);
      delay(150);
      parallel();
      delay(100);
      currentDir = snappedDir;
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;

      // Repeated failures on the same planned turn (wall, gyro drift, motor slip):
      // stop retrying it. Re-plan a fresh direction from the now-clean cardinal
      // heading instead of bouncing between EXECUTE_MOVE and recovery forever.
      if(botchedTurnAttempts >= MAX_BOTCHED_TURN_ATTEMPTS){
        Serial.println("max botched-turn retries reached, re-planning");
        botchedTurnAttempts = 0;
        state = PLAN_NEXT;
        break;
      }

      state = EXECUTE_MOVE;
      break;
    }
    case RETURN: {
      // in case of no elevation used, m1,m2,m3 are all blank grids.
      // let the current floor grid be mapgrid.
      if(currentFloor == 0)      m1 = mapGrid;
      else if(currentFloor == 1) m2 = mapGrid;
      else if(currentFloor == 2) m3 = mapGrid;
      // currentFloor is already 0-indexed (0..2), matching BFS's floor arrays.
      std::pair<int, std::pair<int, int>> currentpos = {currentFloor, {x_pos, y_pos}};
      std::pair<int, std::pair<int, int>> endpos     = {0, {MAP_SIZE/2, MAP_SIZE/2}};

      lcdPrint("starting bfs");
      
      Serial.println("starting bfs");
      std::deque<std::pair<int, std::pair<int,int>>> path = BFS(currentpos, m1, m2, m3, endpos, false);
      if(path.empty()){
        lcdPrint("blue allowed");
        path = BFS(currentpos, m1, m2, m3, endpos, true);
      }
      if(path.empty()){
        lcdPrint("no path found");
        while(true) drivetrain.fullstop();
      }
      Serial.println("path calculated");
      // path[0]=currentpos, path[last]=endpos >> iterate forward toward home
      for(int i = 0; i < (int)path.size() - 1; i++){
        Direction moveDir;
        int dx = path[i+1].second.first  - path[i].second.first;
        int dy = path[i+1].second.second - path[i].second.second;
        if(dy == 0) moveDir = (dx == 1) ? EAST : WEST;
        else        moveDir = (dy == 1) ? NORTH : SOUTH;

        plannedTurnDeg = turnNeededDeg(moveDir);
        absoluteturn(plannedTurnDeg);
        delay(200);
        parallel();
        delay(100);
        currentDir = moveDir;
        fwd(TILE_MM);

        // track floor changes: update currentFloor and swap the active grid
        int dz = path[i+1].first - path[i].first;
        if(dz > 0){
          currentFloor++;
          mapGrid = (currentFloor == 1) ? m2 : m3;
        }
        else if(dz < 0){
          currentFloor--;
          mapGrid = (currentFloor == 0) ? m1 : m2;
        }
      }
      
      while(true){
        drivetrain.fullstop();
        lcdPrint("back to start");
        for(int i = 0;i<5;i++){
          digitalWrite(LEDPIN,HIGH);
          delay(1000);
          digitalWrite(LEDPIN,LOW);
          delay(1000);
        }
      }
    }
    case PAUSE: {
      drivetrain.fullstop();
      delay(200);
      if(digitalRead(logicswitch)==LOW){
        Pausemaze = false;
        // Restore the checkpoint's FLOOR as well as its tile. Save the grid we were working on back into its floor slot (m1=floor0, m2=floor1, m3=floor2), then load the checkpoint floor's grid as the active grid
        // -> so victim flags / walls are looked up on the correct floor.
        if(currentFloor == 0)      m1 = mapGrid;
        else if(currentFloor == 1) m2 = mapGrid;
        else if(currentFloor == 2) m3 = mapGrid;
        currentFloor = floor_checkpoint;
        if(currentFloor == 0)      mapGrid = m1;
        else if(currentFloor == 1) mapGrid = m2;
        else if(currentFloor == 2) mapGrid = m3;
        x_pos = x_checkpoint; y_pos = y_checkpoint; // resume from last checkpoint
        Direction snapped = (Direction)myGyro.headingToCardinal(myGyro.heading()); // snap to cardinal
        absoluteturn(turnNeededDeg(snapped));
        currentDir = snapped;
        
        Serial.println("checkpoint coordinates");
        Serial.println(x_checkpoint);
        Serial.println(y_checkpoint);
        Serial.println(currentDir);
        steps = TURN; // reset avoidance steps
        state = PLAN_NEXT;
      }
      break;
    }
 }



}

// ============================================================================
// 2026 RCJ Rescue Maze SuperTeam - Robot B (chef) mission logic.
//
// Flow per order (x3):
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
// Enable by setting SUPERTEAM_MISSION = true (above loop()). The WiFi AP is
// brought up by initSuperteamComms() in setup() (superteam_comms.ino).
//
// !! COORDINATE WITH ROBOT A'S TEAM: Robot A should start sending the order
// (repeating every ~500 ms until ACKed) only once it is parked on its blue
// handoff tile, so the 5 s handoff windows of the two robots overlap.
// ============================================================================

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
// Push = 2 tiles: the robot crosses its own tile onto the box tile and keeps
// going so the box ends up past the delivery-tile boundary (>half over = +10).
struct StationCfg { char boxTurn; int pushMm; };
// TODO(field): set the real turn directions per station.
StationCfg stationCfg[NUM_STATIONS] = {
  {'R', 2 * TILE_MM}, {'R', 2 * TILE_MM}, {'R', 2 * TILE_MM}, {'R', 2 * TILE_MM}, {'R', 2 * TILE_MM},
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
  missionFwd(cfg.pushMm);          // 2-tile push so the box fully crosses onto its delivery tile
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
