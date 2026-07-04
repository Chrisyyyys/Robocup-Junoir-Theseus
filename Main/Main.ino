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
#include "superteam.h" // ING_* ingredient bits for the SuperTeam mission

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

// ===== SuperTeam (Robot B chef) reactive mission state =====
// true = kitchen mission behavior: black tiles are handoff points (not
// holes), the cameras report ingredient targets (not victims) and trigger
// the box push. The normal explore state machine keeps running either way.
// Full mission logic lives at the bottom of this file.
const bool SUPERTEAM_MISSION = true;
volatile bool stIngredientPending = false; // camera flagged a required ingredient mid-move
volatile int  stIngredientSide = 0;        // 1 = left camera, 2 = right camera
volatile uint8_t stIngredientBit = 0;      // ING_* bit the camera reported
uint8_t stNeeded = 0;        // ingredient set of the active order
uint8_t stCollected = 0;     // ingredients collected so far this order
uint8_t stPushedMask = 0;    // boxes already pushed, by colour bit (scored once each)
bool stHaveOrder = false;    // holding an order not yet handed back as a dish
bool stDishReady = false;    // cooked: next black-tile visit is the dish handoff
int  stOrdersDone = 0;
// where we stood when the colour sensor found the black handoff tile, and the
// heading from there onto it - used to navigate back for the dish handoff
int stBlackApproachX = -1, stBlackApproachY = -1;
Direction stBlackApproachDir = NORTH;

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
    if(SUPERTEAM_MISSION){
      // kitchen mode: the cameras look for ingredient targets, not victims
      superteamCameraPoll();
      rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
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
  calibrateSensor(2,80);
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

void loop(){
  // ---- SuperTeam AP/UDP test: print any order received over WiFi.
  // Disabled in mission mode: it would consume the order before
  // waitForHandoff() sees it. ----
  if (!SUPERTEAM_MISSION && superteamOrderAvailable()) {
    int sum = superteamTakeOrder();
    Serial.print("AP TEST: received order SUM=");
    Serial.println(sum);
  }
  // ---- end AP test ----
// for(int i=1;i<=7;i++){
//   Serial.print("sensor"+i);
//   Serial.print(measure(i));
// }
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
      // SuperTeam: dish cooked -> stop exploring, BFS back to the recorded
      // handoff tile, deliver, take the next order, then resume exploring.
      if(SUPERTEAM_MISSION && stDishReady && stBlackApproachX >= 0){
        // only hand off if we actually made it onto the black tile
        if(superteamReturnToHandoff()) superteamHandoffAtBlackTile();
        state = SENSE_TILE;
        break;
      }
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
        if(SUPERTEAM_MISSION){
          // SuperTeam: the black tile is OUR handoff point, not a hole.
          // fwd() completed the move ONTO the tile (no back-off), so advance
          // the position first. If we owe Robot A a meeting (need an order /
          // dish ready), run it right here, stopped on the tile. Then go
          // sense THIS tile's walls (they were never read) so the planner
          // can route away from it safely.
          markEdgeBothWays(x_pos, y_pos, currentDir);
          stepForward(currentDir, x_pos, y_pos); // now standing on the black tile
          if(!stHaveOrder || stDishReady) superteamHandoffAtBlackTile();
          turnCompletedForMove = false;
          tilecheck = false;
          state = SENSE_TILE;
          break;
        }
        // maze mode: fwd() backed us off, position unchanged -> just re-plan
        state = BACKPEDAL;
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
        //Direction snapped = (Direction)myGyro.headingToCardinal(myGyro.heading()); // snap to cardinal
        //absoluteturn(turnNeededDeg(snapped));
        //currentDir = snapped;
         // Deterministic reset: rotate to the gyro's zero and declare it NORTH.
        // Removes the ambiguous headingToCardinal snap (which could bucket a near-45 deg
        // reading into the wrong cardinal and leave the robot diagonal).
        absoluteturn(0);        // turnNeededDeg(NORTH) == 0
        currentDir = NORTH;
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
// 2026 RCJ Rescue Maze SuperTeam - Robot B (chef) REACTIVE mission logic.
//
// There is NO pre-coded arena route: the robot explores the kitchen with the
// normal maze state machine and reacts to what it senses.
//
//  * black tile (colour sensor)  -> that's our handoff point. NO back-off in
//    SuperTeam mode: fwd() completes the move ONTO the tile; EXECUTE_MOVE
//    advances the position and calls superteamHandoffAtBlackTile() in place:
//    receive the WiFi order (and/or hand over the finished dish), hold the
//    >= 5 s window, then the planner routes away. The approach position/
//    heading is recorded so the robot can navigate back for the dish
//    handoff (the planner otherwise avoids black tiles).
//  * camera reports an ingredient -> superteamCameraPoll() (camera thread)
//    raises stIngredientPending; fwd() stops and services it in place:
//    3 s continuous blink to collect, then the box push - ONLY because the
//    camera detected the target here, never a blind push.
//  * all required ingredients in -> cook 10 s on the spot; PLAN_NEXT then
//    navigates back to the recorded handoff tile via BFS for the dish
//    handoff, takes the next order there, and exploration resumes.
//
// The st* state globals live near the top of this file because cameraTask
// and fwd() reference them before this section.
//
// !! CAMERA TEAM: in SuperTeam mode the cameras must stream the letters
//    'R','Y','G','b','B' when they see an ingredient target
//    ('b' LOWERCASE = blue, 'B' UPPERCASE = black).
// !! ROBOT A TEAM: send the SUM number (resend every ~500 ms until our ACK)
//    while parked on the blue handoff tile; B holds its 5 s window as soon
//    as the number is in.
// ============================================================================

// ---------------- timing (rulebook) ----------------
const unsigned long HANDOFF_HOLD_MS = 5000 + 500;  // >= 5 s on adjacent tiles (+margin)
const unsigned long COLLECT_BLINK_MS = 3000 + 500; // >= 3 s continuous 500/500 blink
const unsigned long COOK_HOLD_MS = 10000 + 500;    // >= 10 s fully stationary
const int NUM_ORDERS = 3;

Direction missionDir = NORTH; // heading frame for the push/handoff maneuvers

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

// snap-turn relative to the current mission heading and remember the new one.
// quarters: +1 = right, -1 = left, +2 = 180. Uses the same gyro-cardinal
// system as the maze code so headings can't drift.
void missionTurnRel(int quarters) {
  missionDir = (Direction)((((int)missionDir + quarters) % 4 + 4) % 4);
  absoluteturn(turnNeededDeg(missionDir));
  delay(150);
}

// straight drive: encoder distance + gyro heading hold. No black-tile abort
// (black tiles are handoff tiles here) and no camera/ramp logic. Keeps the
// front-wall safety stop, which also protects against ramming Robot A.
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

// like missionFwd but WITHOUT the front safety stop and with a higher PWM
// floor: during a box push the box sits right against the robot's nose (the
// front sensors would trip instantly) and the extra friction needs torque.
void missionPush(double dist) {
  double pulses = dist / (wheel_diameter * M_PI) * wheel_cpr * gear_ratio;
  PID gyroPID(1, 0.001, 0.03);
  PID Scale_PID(0.0045, 0, 0.0008);
  double targetHeading = turnNeededDeg(missionDir);
  drivetrain.reset_encoderCount(true, true, true);

  while ((drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3 <= pulses) {
    double yaw = myGyro.heading() - targetHeading;
    while (yaw > 180.0) yaw -= 360.0;
    while (yaw < -180.0) yaw += 360.0;
    double adjustment = gyroPID.getPID(yaw);
    double Scale = Scale_PID.getPID(pulses - (drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3);
    drivetrain.drive(constrain(Scale * (120 - adjustment), 60, 150),
                     constrain(Scale * (120 - adjustment), 60, 150),
                     constrain(Scale * (120 + adjustment), 60, 150),
                     constrain(Scale * (120 + adjustment), 60, 150));
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

// camera byte -> ingredient bit. Camera protocol: 'R' red, 'G' green,
// 'Y' yellow, 'b' (LOWERCASE) blue, 'B' (UPPERCASE) black; anything else 0.
uint8_t ingBitFromCamByte(char c) {
  if (c == 'R') return ING_RED;
  if (c == 'Y') return ING_YELLOW;
  if (c == 'G') return ING_GREEN;
  if (c == 'b') return ING_BLUE;
  if (c == 'B') return ING_BLACK;
  return 0;
}

// Runs on the camera RTOS thread (cameraTask) in SuperTeam mode, replacing
// victim detection. While moving with an active order, watch both cameras
// for a NEEDED, still-missing ingredient colour; on a hit stop the
// drivetrain and hand the service to fwd() via stIngredientPending.
// Colours the dish doesn't need are ignored on purpose: stopping at a wrong
// target is -10, driving past is free.
void superteamCameraPoll() {
  if (!fwdActive || stIngredientPending || !stHaveOrder || stDishReady) return;
  uint8_t bit = ingBitFromCamByte(readCamRaw1());
  int side = 1; // left camera
  if (bit == 0) {
    bit = ingBitFromCamByte(readCamRaw2());
    side = 2;   // right camera
  }
  if (bit == 0) return;
  if (!(stNeeded & bit) || (stCollected & bit)) return;
  drivetrain.fullstop();
  stIngredientSide = side;
  stIngredientBit = bit;
  stIngredientPending = true; // fwd() services it and clears the flag
}

// ---------------- mission steps ----------------

// Order handoff, already ON the black tile. The WiFi number may have arrived
// at any earlier moment (it is kept, never flushed); once it is in, hold the
// shared >= 5 s window (A holds its own 5 s on the blue tile). Returns SUM.
int waitForHandoff() {
  drivetrain.fullstop();
  while (!superteamOrderAvailable()) {
    drivetrain.fullstop();
    delay(50);
  }
  int sum = superteamTakeOrder();
  Serial.print("superteam: handoff order SUM=");
  Serial.println(sum);
  holdStill(HANDOFF_HOLD_MS);
  return sum;
}

// decode SUM -> required-ingredient bitmask (ingredientsForSum lives in
// superteam_comms.ino next to the protocol it belongs to)
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

// cook - fully stationary >= 10 s, any Kitchen tile counts
void prepareDish() {
  Serial.println("superteam: preparing dish");
  holdStill(COOK_HOLD_MS);
}

// Dish handoff - stationary >= 5 s on the black tile next to A. DISH_READY
// is a courtesy heads-up (A's team detects us with their distance sensors);
// the extra 3 s margin helps the two 5 s windows overlap.
void dishHandoff() {
  Serial.println("superteam: dish handoff");
  superteamSend("DISH_READY");
  holdStill(HANDOFF_HOLD_MS + 3000);
}

// Called from inside fwd() (main context) when the camera thread raised
// stIngredientPending; the robot is already stopped beside the target.
// Collect (3 s blink), push the box - ONLY because the camera detected the
// target right here - cook if the order is complete, then let fwd() resume.
void serviceIngredientTarget() {
  missionDir = currentDir; // heading frame for the push maneuver
  Serial.print("superteam: collecting ingredient bit 0x");
  Serial.println(stIngredientBit, HEX);
  blinkIdLed(COLLECT_BLINK_MS);
  stCollected |= stIngredientBit;

  // the target is wall-mounted on the camera's side, so its grey box sits on
  // the neighbouring tile on the OPPOSITE side. Push it 2 tiles so it fully
  // crosses onto its delivery tile, then return to the same spot/heading.
  // Each box scores only once (stPushedMask).
  if (!(stPushedMask & stIngredientBit)) {
    int quarters = (stIngredientSide == 1) ? +1 : -1; // left cam -> box on the right
    missionTurnRel(quarters);
    missionPush(2 * TILE_MM);
    missionBackward(2 * TILE_MM);
    missionTurnRel(-quarters);
    stPushedMask |= stIngredientBit;
  }

  if (stCollected == stNeeded) {
    prepareDish();      // cook right here
    stDishReady = true; // next black-tile visit is the dish handoff
  }
  stIngredientPending = false; // re-enable the camera thread
}

// A black tile was found and we owe Robot A a meeting. Precondition: the
// robot is standing ON the black tile (fwd() drives fully onto it in
// SuperTeam mode - no back-off; superteamReturnToHandoff() reproduces it).
// Hand over the dish and/or take the next order right here; the planner
// routes away afterwards.
void superteamHandoffAtBlackTile() {
  missionDir = currentDir;
  drivetrain.fullstop();

  if (stDishReady) {
    dishHandoff();
    stDishReady = false;
    stHaveOrder = false;
    stOrdersDone++;
    if (stOrdersDone >= NUM_ORDERS) {
      // all dishes served. TODO(field): red-tile exit bonus needs red floor
      // detection in read_color(); park for now.
      lcdPrint("orders done");
      while (true) { drivetrain.fullstop(); delay(1000); }
    }
  }

  if (!stHaveOrder) {
    // take the (next) order right here - A may already be waiting with it
    int sum = waitForHandoff();
    stNeeded = decodeOrder(sum);
    stCollected = 0;
    stHaveOrder = true;
  }
}

// Navigate back to the recorded handoff approach tile with BFS over the
// explored map (mirrors the RETURN state), then face the black tile and
// drive ONTO it (no back-off in SuperTeam). Returns false if no path was
// found, so the caller must NOT run the handoff.
bool superteamReturnToHandoff() {
  lcdPrint("to handoff");
  Serial.println("superteam: returning to handoff tile");
  if (currentFloor == 0)      m1 = mapGrid;
  else if (currentFloor == 1) m2 = mapGrid;
  else                        m3 = mapGrid;
  std::pair<int, std::pair<int,int>> cur = {currentFloor, {x_pos, y_pos}};
  std::pair<int, std::pair<int,int>> end = {currentFloor, {stBlackApproachX, stBlackApproachY}};
  std::deque<std::pair<int, std::pair<int,int>>> path = BFS(cur, m1, m2, m3, end, false);
  if (path.empty()) path = BFS(cur, m1, m2, m3, end, true);
  if (path.empty()) {
    Serial.println("superteam: NO PATH back to handoff tile!");
    return false; // exploration continues; we'll retry from PLAN_NEXT
  }
  for (int i = 0; i < (int)path.size() - 1; i++) {
    Direction moveDir;
    int dx = path[i+1].second.first  - path[i].second.first;
    int dy = path[i+1].second.second - path[i].second.second;
    if (dy == 0) moveDir = (dx == 1) ? EAST : WEST;
    else         moveDir = (dy == 1) ? NORTH : SOUTH;
    absoluteturn(turnNeededDeg(moveDir));
    delay(200);
    parallel();
    delay(100);
    currentDir = moveDir;
    fwd(TILE_MM);
    x_pos = path[i+1].second.first;
    y_pos = path[i+1].second.second;
  }
  // face the black tile the same way we originally found it, then drive
  // fully onto it (fwd() completes onto black tiles in SuperTeam mode)
  absoluteturn(turnNeededDeg(stBlackApproachDir));
  currentDir = stBlackApproachDir;
  delay(150);
  fwd(TILE_MM);
  stepForward(currentDir, x_pos, y_pos); // now standing on the black tile
  return true;
}
