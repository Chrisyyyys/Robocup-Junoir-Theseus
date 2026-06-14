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
//wifi



#include "motors.h"
#define MIN_DIST 120         // mm (tune this)
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define BLACK_THRESHOLD 0.10 // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 1.5f // ratio threshold — calibrate on real silver tile (typical normal~0.8, silver~2.0+)
float clear; 



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

//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B);
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 80.0; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};
// obstacle avoidance steps
enum Steps {
  TURN,
  PARALLEL,
  BACKTRACK,
  FWD,
};
Steps steps = TURN;



// black blue toggles
bool blacktoggle = false;
bool bluetoggle = false;
bool climbtoggle = false;
int rampCount;




// create color thread
rtos::Thread colorThread;
// color enum
enum Color {
  WHITE = 0,
  BLACK = -1,
  BLUE = 1,
  RED = 2,
  SILVER = 3
};
volatile Color latestColor = WHITE;
void colorTask(){
  while(true){
    latestColor = read_color();
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(5));
  }
}
// camera task
bool isVictim = false;
bool stopToggle = false;
void setstopToggle(bool val){
  stopToggle = val;
}
// movement flag variables
int turn_angle = 0; 
int fwd_distance = 0;
bool fwd_flag = false;
bool turn_flag = false;
// rpc call functions
bool queryBlack(){ return blacktoggle; }
bool queryBlue(){ return bluetoggle; }
bool queryClimb(){ return climbtoggle; }
int queryRamps(){ return rampCount; }
int cardinalTask(){return myGyro.headingToCardinal(myGyro.heading());}
int readEncoderA(){ return encoderCountA; }
int readEncoderB(){ return encoderCountB; }
void rpcFullStop(){ drivetrain.fullstop(); }
void isFwdComplete(){ return fwd_flag;}
void isTurnComplete(){ return turn_flag;}
double headingErrorDeg(double targetDeg, double actualDeg) {
  double err = targetDeg - actualDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return abs(err);
}

bool turnCompletedSuccessfully(double targetHeading) {
  const double TURN_SUCCESS_TOLERANCE_DEG = 20.0;
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

  // init wifi


  // begin UART communication.
  Serial.begin(115200);
  Serial2.begin(115200); // switch to 9600 for reliability
  Serial3.begin(115200);

  Wire.begin();
  disableAllCall();
  myMux.begin();
  init_dist(); // initialize mux before distance sensors.
  scanAllPorts();
  init_color();
  init_drive();
  RPC.begin(); // begin rpc
  // bind RPC for basic movement
  RPC.bind("fwd",fwdTask);
  RPC.bind("absoluteturn",turnTask);
  RPC.bind("parallel",parallel);
  RPC.bind("fullstop",rpcFullStop);
  RPC.bind("togglestop",setstopToggle);
  // bind RPC for cardinal snapping
  RPC.bind("headingToCardinal",cardinalTask);
  // query movement completion
  RPC.bind("isFwdComplete",isFwdComplete);
  RPC.bind("isTurnComplete",isTurnComplete);
  // bind RPC for tile types: black, blue, ramp
  RPC.bind("queryBlack",queryBlack);
  RPC.bind("queryBlue",queryBlue);
  RPC.bind("queryClimb",queryClimb);
  RPC.bind("queryRamps",queryRamps);
  // read encoders
  RPC.bind("readEncoderA",readEncoderA);
  RPC.bind("readEncoderB",readEncoderB);
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
    if(turnCompletedSuccessfully(turn_angle) == false&&stopToggle==false){
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
