#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "motors.h"
extern Adafruit_DCMotor motorA;
extern Adafruit_DCMotor motorB;
extern Adafruit_DCMotor motorC;
extern Adafruit_DCMotor motorD;
motors::motors(encoderPin_A_A,encoderPinB_A){
  _encoderPinA_A = encoderPinA_A;
  _encoderPinB_A = encoderPinB_A;
  pinMode(_encoderPin_A_A, INPUT);
  pinMode(_encoderPin_A_B,INPUT);
  pinMode(_encoderPin_B_A, INPUT);
  pinMode(_encoderPin_B_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(_encoderPin_A_A), encoder_update_A, RISING);
  attachInterrupt(digitalPinToInterrupt(_encoderPin_B_A), encoder_update_B, RISING); // only counting rising of Pin: (20/2)/2
  if(!AFMS.begin()){
    Serial.println("could not find motor shield");
  }
  else{
    Serial.println("motor shield found");
  }
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  // initialize gyro
  myGyro.init_Gyro();
}
motors::encoder_update_A(){
  int encoderState = digitalRead(_encoderPin_A_B);
 
  if(encoderState == LOW){
    encoderCountA--;
  }
  else{
    encoderCountA++;
  }
}
motors::encoder_update_B(){
  int encoderState = digitalRead(_encoderPin_B_B);
  
  if(encoderState == HIGH){
    encoderCountB--;
  }
  else{
    encoderCountB++;
  }
}
fullstop(){
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
}
fw(int speed){
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}
backward(int speed){
  motorA->run(BACKWARD);
  motorB->run(BACKWARD);
  motorC->run(BACKWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}
turnright(int speed){
  motorA->run(FORWARD);
  motorB->run(BACKWARD);
  motorC->run(FORWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}
turnleft(int speed){
  motorA->run(BACKWARD);
  motorB->run(FORWARD);
  motorC->run(BACKWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}