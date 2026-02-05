
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
#include "gyro.h"
#define MIN_DIST 150         // mm (tune this)
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define BLACK_C_THRESHOLD 120 // color clear-channel threshold (tune)

#include "MazeTile.h"

// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// shut down allcall
#define PCA_ADDR 0x60
#define MODE1    0x00
#define ALLCALL_BIT 0x01  // MODE1 bit0
#define BLACK_THRESHOLD BLACK_C_THRESHOLD   
// set up color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
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
Tile mapGrid[MAP_SIZE][MAP_SIZE]; // array of tiles

//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  UPDATE_MAP,
  VICTIM_SIGNAL,
  PLAN_NEXT,
  EXECUTE_MOVE,
  BACKPEDAL
};

// initialize 

Direction currentDir = NORTH;     // robot heading in map coords (0..3)
int plannedTurnDeg = 0;           // -90,0,+90,180
Direction plannedMoveDir = NORTH; // absolute direction robot will move next
int x_pos = MAP_SIZE/2;
int y_pos = MAP_SIZE/2;
RobotState state = SENSE_TILE;
bool blacktoggle = false;



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
      // Read for walls
      readWallsRel(wallF, wallR, wallB, wallL);
      delay(500);


      // Tile type
     




      // Victim quick check (optional)
      /*
      int vL =
      int vR = readSerial2();
      if (vL != -1 || vR != -1) {
        state = VICTIM_SIGNAL;
        break;
      }




      state = UPDATE_MAP;
      break;
      */
      state = UPDATE_MAP; // next state.
    }




    case UPDATE_MAP: {
      writeWallsToCurrentTile(wallF, wallR, wallB, wallL);
      updateFullyExploredAt(x_pos, y_pos);
      state = PLAN_NEXT;
      break;
    }
    case VICTIM_SIGNAL: {
      /*
      // store + do your actual signaling / camera confirm
      mapGrid[x_pos][y_pos].victim = true;




      // If you want the full routines:
      detectCam1();
      detectCam2();
      */
      state = UPDATE_MAP;  // continue
      break;
    }




    case PLAN_NEXT: {
      plannedMoveDir = pickNextDirection();
      Serial.println(plannedMoveDir);
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      Serial.println(plannedTurnDeg);
      state = EXECUTE_MOVE;
      break;
    }




    case EXECUTE_MOVE: {
     
      absoluteturn(plannedTurnDeg);
      delay(500);
      //update currentDir
      currentDir = plannedMoveDir;
      // 2) mark edges on map BEFORE moving
      markEdgeBothWays(x_pos, y_pos, currentDir);
      // 3) drive one tile
      fwd(TILE_MM);
      // 4) update robot position
      if(blacktoggle = false){
        stepForward(currentDir, x_pos, y_pos);
      }
      else{
        state = BACKPEDAL;
        break;
      }
      
      delay(200);
      // safety clamp
      /*
      if (!inBounds(x_pos, y_pos)) {
        //stop motors or do something
      }
      */
      //parallel();
      state = SENSE_TILE;
      break;
     
    }
    case BACKPEDAL: {
      plannedMoveDir = pickNextDirection();
      Serial.println(plannedMoveDir);
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      Serial.println(plannedTurnDeg);
      state = EXECUTE_MOVE;
      break;
    }
  }
 
  //delay(1000);

 
  
  
  
}


