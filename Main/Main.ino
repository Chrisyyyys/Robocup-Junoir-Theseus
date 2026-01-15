
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"
#include "MazeTile.h"
// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// shut down allcall
#define PCA_ADDR 0x60
#define MODE1    0x00
#define ALLCALL_BIT 0x01  // MODE1 bit0
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
int x_pos,y_pos;
//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  UPDATE_MAP,
  VICTIM_SIGNAL,
  PLAN_NEXT,
  EXECUTE_MOVE
};
RobotState state;
// initialize 


// when something is seen, it would wait for 5 detections.
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
  
  delay(500);
  
  //initialize map
  initializeMap();
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].discovered = true; 

 
}
void loop(){
  detectCam2();
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
Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}
//update tile position
void stepForward(Direction d, int &x, int &y) {
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
}
//mark the edges
void markEdgeBothWays(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);

  // bounds check
  if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE) return;

  mapGrid[x][y].edge[d] = true;
  mapGrid[nx][ny].edge[opposite(d)] = true;
}
void senseTile(bool wallOut[4], TileTypes &tileTypeOut, bool &victimOut) {
  // TODO: fill using your sensors
  // - set wallOut[NORTH/EAST/SOUTH/WEST]
  // - tileTypeOut = BLANK/BLUE/BLACK 
  // - victimOut = true if victim detected here

  // placeholder:
  for (int i=0;i<4;i++) wallOut[i]=false;
  tileTypeOut = BLANK;
  victimOut = false;
}
// if tile is good 
bool isOpenAndUnvisited(int x, int y, Direction d) {
  if (mapGrid[x][y].wall[d]) return false;

  int nx = x, ny = y;
  stepForward(d, nx, ny);
  if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE) return false;

  return !mapGrid[nx][ny].discovered;
}
