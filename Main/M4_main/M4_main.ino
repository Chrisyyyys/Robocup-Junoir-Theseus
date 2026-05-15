// libraries
#include <mbed.h> // access arduino mbed OS
#include <RPC.h> // RPC
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Stepper.h>
#include "PID.h"
#include "timer.h"
#include "gyro.h"
#include "dispenser.h"
#include "motors.h"
#define MIN_DIST 120         // mm (tune this)
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define BLACK_THRESHOLD 0.10 // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 1.5f // ratio threshold — calibrate on real silver tile (typical normal~0.8, silver~2.0+)
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
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
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
const int encoderPin_A_A = 2;
const int encoderPin_A_B = 4; 
const int encoderPin_B_A = 3;
const int encoderPin_B_B = 5; 
// encoder counters
volatile int encoderCountB;
volatile int encoderCountA;
//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B);
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 68.7; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};

// create stepper object
const int steps_per_revolution = 2048;
Stepper myStepper = Stepper(steps_per_revolution, 8, 9,10,11); 
// map size variables
const int MAP_SIZE=20;
Tile mapGrid[MAP_SIZE][MAP_SIZE]; // array of tiles
// queue




// initialize 

Direction currentDir = NORTH;     // robot heading in map coords (0..3)
int plannedTurnDeg = 0;           // -90,0,+90,180
Direction plannedMoveDir = NORTH; // absolute direction robot will move next
bool turnCompletedForMove = false;
int x_pos = MAP_SIZE/2;
int y_pos = MAP_SIZE/2;
RobotState state = SENSE_TILE;
// maze return to start condition variables
int medkits = 8;
timer mazeTime;
// black blue toggles
bool blacktoggle = false;
bool bluetoggle = false;
bool climbtoggle = false;
int rampCount;
// victim toggles
bool victimtoggle = false;
bool victimAtCurrent = false;
// LED pins
const int pinHarmed = 41;
const int pinStable = 37;
const int pinUnharmed = 29;
// camera GPIOs
const int gpio1 = 13;
const int gpio2 = 12;
// de-activate color sensor while climbing
int use_color = 0;
// stepper variables
const int angle_offset = 44;
const int angle_increment = 22;
dispenser disp(angle_increment,angle_offset,steps_per_revolution);
// logic switch pin
const int logicswitch = 31;
bool Pausemaze = false;
int x_checkpoint, y_checkpoint;
bool tilecheck = false;
// create color thread
Thread colorThread;
// color enum
enum Color = {
  WHITE = 0,
  BLACK = -1,
  BLUE = 1,
  RED = 2,
  SILVER = 3
}
Color latestColor;
void colorTask{
  while(true){
    latestColor = read_color();
    ThisThread.sleep(5ms);
  }
}
// camera task
isVictim = false;
stopToggle = false;
void setstopToggle(bool val){
  stopToggle = val;
}
// movement flag variables
int turn_angle = 0; 
int fwd_angle = 0;
bool fwd_flag = false;
bool turn_flag = false;
double headingErrorDeg(double targetDeg, double actualDeg) {
  double err = targetDeg - actualDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return abs(err);
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
  // initialize LED puns
  pinMode(pinHarmed,OUTPUT);
  pinMode(pinStable,OUTPUT);
  pinMode(pinUnharmed,OUTPUT);
  // initialize camera gpio pins
  pinMode(gpio1, INPUT);
  pinMode(gpio2, INPUT);
  // initialize logic switch pin

  // begin UART communication.
  Serial.begin(115200);
  Serial2.begin(115200); // switch to 9600 for reliability
  Serial3.begin(115200);
  flashLED('S');
  uint8_t cause = MCUSR;
  MCUSR = 0;
  Wire.begin();
  disableAllCall();
  myMux.begin();
  init_dist(); // initialize mux before distance sensors.
  scanAllPorts();
  init_color();
  init_drive();
  //detect();
  //initialize map
  initializeMap();
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].setDiscovered(true); 
  currentDir = NORTH;
  state = SENSE_TILE;
  // bind RPC for basic movement
  RPC.bind("fwd",fwdTask);
  RPC.bind("absoluteturn",turnTask);
  RPC.bind("parallel",parallel);
  RPC.bind("fullstop",[](){drivetrain.fullstop()});
  RPC.bind("togglestop",setstopToggle);
  // bind RPC for tile types: black, blue, ramp
  RPC.bind("queryBlack",blacktoggle);
  RPC.bind("queryBlue",bluetoggle);
  RPC.bind("queryClimb",climbtoggle);
  RPC.bind("queryRamps",rampCount);
  // read encoders
  RPC.bind("readEncoderA",encoderCountA);
  RPC.bind("readEncoderB",encoderCountB);
  // distance
  RPC.bind("measure",measure);
  RPC.bind("detectWall",detectWall);
  // Start color thread
  colorThread.start(colorTask);
  delay(2000); // wait for camera to start.
  
  
}
int iterator = 0;
void loop(){
  if(turn_flag == true){
     absoluteturn(turn_angle);
    if(turnCompletedSuccessfully(turn_angle) == false){
      delay(200);
      absoluteturn(turn_angle);
    }
    turn_flag = false;
  }
  if(fwd_flag == true){
    fwd(fwd_distance);
    fwd_flag = false;
  }
 
}
