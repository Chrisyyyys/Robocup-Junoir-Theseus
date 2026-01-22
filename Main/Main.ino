
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"

#include "timer.h"
#define MIN_DIST 120          // mm (tune this)
#define TILE_MM 300.0         // one tile = 300mm (RCJ tile)
#define BLACK_C_THRESHOLD 120 // color clear-channel threshold (tune)

#include "MazeTile.h"

// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// shut down allcall
#define PCA_ADDR 0x60
#define MODE1    0x00
#define ALLCALL_BIT 0x01  // MODE1 bit0
#define BLACK_THRESHOLD BLACK_C_THRESHOLD   // so your read_color() compiles
// set up color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
// set up gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
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
// encoder counters
int encoderCountB;
int encoderCountA;
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 68.7; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};


// map size variables
const int MAP_SIZE=20;
Tile mapGrid[MAP_SIZE][MAP_SIZE];

//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  UPDATE_MAP,
  VICTIM_SIGNAL,
  PLAN_NEXT,
  EXECUTE_MOVE
};

// initialize 

Direction currentDir = NORTH;     // robot heading in map coords (0..3)
int plannedTurnDeg = 0;           // -90,0,+90,180
Direction plannedMoveDir = NORTH; // absolute direction robot will move next
int x_pos = MAP_SIZE/2;
int y_pos = MAP_SIZE/2;
RobotState state = SENSE_TILE;




void setup(){
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Wire.begin();
  disableAllCall();
  myMux.begin();
  
  
  init_dist(); // initialize mux before distance sensors.
  scanAllPorts();
  
  init_color();
  init_drive();
  fwd(300);
  /*
  delay(500);
  delay(200);
  fwd(300);
  delay(200);
  turn(90);
  delay(200);
  fwd(300);
  delay(200);
  turn(180);
  delay(200);
  fwd(300);
  delay(200);
  turn(-90);
  delay(200);
  fwd(300);
  */
  //detect();
  //initialize map
  initializeMap();
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].discovered = true; 
  currentDir = NORTH;
  state = SENSE_TILE;

 
}
void loop(){
   static bool wallF, wallR, wallB, wallL;

  switch (state) {

    case SENSE_TILE: {
      // Read sensors
      readWallsRel(wallF, wallR, wallB, wallL);

      // Tile type
      
      // Victim quick check (optional)
      int vL = readSerial1();
      int vR = readSerial2();
      if (vL != -1 || vR != -1) {
        state = VICTIM_SIGNAL;
        break;
      }

      state = UPDATE_MAP;
      break;
    }

    case UPDATE_MAP: {
      writeWallsToCurrentTile(wallF, wallR, wallB, wallL);
      updateFullyExploredAt(x_pos, y_pos);
      state = PLAN_NEXT;
      break;
    }

    case VICTIM_SIGNAL: {
      // store + do your actual signaling / camera confirm
      mapGrid[x_pos][y_pos].victim = true;

      // If you want the full routines:
      detectCam1();
      detectCam2();

      state = UPDATE_MAP;  // continue
      break;
    }

    case PLAN_NEXT: {
      plannedMoveDir = pickNextDirection();
      plannedTurnDeg = turnNeededDeg(currentDir, plannedMoveDir);
      state = EXECUTE_MOVE;
      break;
    }

    case EXECUTE_MOVE: {
      Serial.print(plannedTurnDeg);//figure out motors stuff
      

      // 2) mark edges on map BEFORE moving
      markEdgeBothWays(x_pos, y_pos, currentDir);

      // 3) drive one tile
      fwd(TILE_MM);

      // 4) update robot position
      stepForward(currentDir, x_pos, y_pos);

      // safety clamp
      if (!inBounds(x_pos, y_pos)) {
        //stop motors or do something
      }
      state = SENSE_TILE;
      break;
      
    }
  }
  
  //delay(1000);
  
  /*
  while(encoderCountB<wheel_cpr*gear_ratio){
    motorB->setSpeed(100);
    Serial.println(encoderCountB);
  }
  motorB->setSpeed(0);
  delay(1000);
  encoderCountB = 0;
  */
}


Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}

void stepForward(Direction d, int &x, int &y) {
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
}

bool inBounds(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].discovered = false;
      mapGrid[x][y].fullyExplored = false;
      for (int d = 0; d < 4; d++) {
        mapGrid[x][y].wall[d] = false;
        mapGrid[x][y].edge[d] = false;
      }
      mapGrid[x][y].tileType = BLANK;
      mapGrid[x][y].victim = false;
    }
  }
}

// mark traveled edge in BOTH tiles (current and next)
void markEdgeBothWays(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  if (!inBounds(nx, ny)) return;

  mapGrid[x][y].edge[d] = true;
  mapGrid[nx][ny].edge[opposite(d)] = true;
}

// update fullyExplored = all OPEN dirs have been traveled at least once
void updateFullyExploredAt(int x, int y) {
  Tile &t = mapGrid[x][y];
  bool allDone = true;

  for (int d = 0; d < 4; d++) {
    if (t.wall[d] == false) {     // open
      if (t.edge[d] == false) {   // not traveled yet
        allDone = false;
        break;
      }
    }
  }
  t.fullyExplored = allDone;
}
// 0=front, 1=right, 2=back, 3=left
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL) {
  wallF = (detectWall(0) == 0);
  wallR = (detectWall(1) == 0);
  wallB = (detectWall(2) == 0);
  wallL = (detectWall(3) == 0);
}
//get the wall from L,R into N W
void writeWallsToCurrentTile(bool wallF, bool wallR, bool wallB, bool wallL) {
  Tile &t = mapGrid[x_pos][y_pos];
  t.discovered = true;

  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);

  t.wall[absF] = wallF;
  t.wall[absR] = wallR;
  t.wall[absB] = wallB;
  t.wall[absL] = wallL;
}
Direction pickNextDirection() {
  Tile &t = mapGrid[x_pos][y_pos];

  Direction absL = rotateDir(currentDir, -1);
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);

  auto open  = [&](Direction d){ return t.wall[d] == false; };
  auto untr  = [&](Direction d){ return t.edge[d] == false; };

  // 1) try open + untraveled first
  if (open(absF) && untr(absF)) return absF;
  if (open(absL) && untr(absL)) return absL;
  if (open(absR) && untr(absR)) return absR;
  if (open(absB) && untr(absB)) return absB;

  // 2) else any open (front-priority fallback)
  if (open(absF)) return absF;
  if (open(absL)) return absL;
  if (open(absR)) return absR;
  if (open(absB)) return absB;
  // figure out BFS later
  // 3) trapped
  return absB;
}

int turnNeededDeg(Direction from, Direction to) {
  int diff = ((int)to - (int)from + 4) % 4;
  if (diff == 0) return 0;
  if (diff == 1) return +90;
  if (diff == 2) return 180;
  return -90; // diff==3
}
