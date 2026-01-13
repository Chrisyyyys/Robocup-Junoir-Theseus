
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"
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
// camera communication code
// clear serial buffers.
void clearSerialBuffer1() {
  while (Serial2.available() > 0) {
    Serial2.read();  // Read and discard one byte from the buffer
  }
}
void clearSerialBuffer2() {
  while (Serial3.available() > 0) {
    Serial3.read();  // Read and discard one byte from the buffer
  }
}
int readSerial1(){ // 
  char sample;
  int output;
  
  if(Serial2.available()>0){
    sample = Serial2.read();
    
    if(sample == 'H'){
      output = 0;
    }
    else if(sample == 'S'){
      output = 1;
    }
    else if(sample == 'U'){
      output = 2;
    }
    else if(sample == 'R'){
      output = 3;
    }
    else if(sample == 'Y'){
      output = 4;
    }
    else{
      output = 5; // green
    }
    return output; // output inside or it keeps outputting
  }
  else{
    // nothing right now
    return -1;
  }
  
}
int readSerial2(){
  char sample;
  int output;
  
  if(Serial3.available()>0){
    sample = Serial3.read();
    
    if(sample == 'H'){
      output = 0;
    }
    else if(sample == 'S'){
      output = 1;
    }
    else if(sample == 'U'){
      output = 2;
    }
    else if(sample == 'R'){
      output = 3;
    }
    else if(sample == 'Y'){
      output = 4;
    }
    else{
      output = 5; // green
    }
    return output; // output inside or it keeps outputting
    
  }
  else{
    // nothing right now
    return -1;
  }
}
void detectCam1(){
  // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int maxCount = 0;
  char res = 0;
  char output;
  if(Serial2.available()>=5){
    Serial.println("letters incoming");
    
    for(int i =0; i<5;i++){
      int value = readSerial1();
      samples[i] = value;
      Serial.println(value);
      
    }
    
    for(int i=0;i<6;i++){
      int count = 0;
      for(int j = 0;j<5;j++){
        if(samples[j] == i) count++;
      }
      if(count > maxCount){
        maxCount = count;
        res = classes[i];
      }
    }
    Serial.println(res);
  }
  else{
    //nothing right now.
    return;
  }

}
void detectCam2(){
  // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int maxCount = 0;
  char res = 0;
  char output;
  if(Serial3.available()>=5){
    Serial.println("letters incoming");
    
    for(int i =0; i<5;i++){
      int value = readSerial2();
      samples[i] = value;
      Serial.println(value);
      
    }
    
    for(int i=0;i<6;i++){
      int count = 0;
      for(int j = 0;j<5;j++){
        if(samples[j] == i) count++;
      }
      if(count > maxCount){
        maxCount = count;
        res = classes[i];
      }
    }
    Serial.println(res);
  }
  else{
    //nothing right now.
    return;
  }

}
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