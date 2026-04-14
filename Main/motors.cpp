#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "motors.h"
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *motorA;
extern Adafruit_DCMotor *motorB;
extern Adafruit_DCMotor *motorC;
extern Adafruit_DCMotor *motorD;
motors* motorsInstance = nullptr; // pointer to motor class

motors::motors(int encoderPin_A_A,int encoderPin_A_B,int encoderPin_B_A,int encoderPin_B_B){
  _encoderPin_A_A = encoderPin_A_A;
  _encoderPin_A_B = encoderPin_A_B;
  _encoderPin_B_A = encoderPin_B_A;
  _encoderPin_B_B = encoderPin_B_B;
  motorsInstance = this;
 
}
void encoder_update_A_ISR() {
  if (motorsInstance != nullptr) {
    motorsInstance->encoder_update_A();
  }
}

void encoder_update_B_ISR() {
  if (motorsInstance != nullptr) {
    motorsInstance->encoder_update_B();
  }
}
void motors::encoder_update_A(){
  int encoderState = digitalRead(_encoderPin_A_B);
 
  if(encoderState == LOW){
    encoderCountA--;
  }
  else{
    encoderCountA++;
  }
}
void motors::encoder_update_B(){
  int encoderState = digitalRead(_encoderPin_B_B);
  
  if(encoderState == HIGH){
    encoderCountB--;
  }
  else{
    encoderCountB++;
  }
}
void motors::init_drive(){
   pinMode(_encoderPin_A_A, INPUT);
  pinMode(_encoderPin_A_B,INPUT);
  pinMode(_encoderPin_B_A, INPUT);
  pinMode(_encoderPin_B_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(_encoderPin_A_A), encoder_update_A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(_encoderPin_B_A), encoder_update_B_ISR, RISING); // only counting rising of Pin: (20/2)/2
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
  
}
void motors::fullstop(){
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
}
void motors::reset_encoderCount(bool A, bool B){
  if(A == true) encoderCountA = 0;
  if(B == true) encoderCountB = 0;
}
void motors::set_encoderCountA(int A){
  encoderCountA = A;
}
void motors::set_encoderCountB(int B){
  encoderCountB = B;
}
void motors::set_interrupt(bool A, bool B){
  if(A == true){
    attachInterrupt(digitalPinToInterrupt(_encoderPin_A_A), encoder_update_A_ISR, RISING);
  }
  else{
    detachInterrupt(digitalPinToInterrupt(_encoderPin_A_A));
  }
  if(B == true){
    attachInterrupt(digitalPinToInterrupt(_encoderPin_B_A), encoder_update_B_ISR, RISING);
    
  }
  else{
    detachInterrupt(digitalPinToInterrupt(_encoderPin_B_A));
  }
}
void motors::drive(int A, int C, int B, int D){
  motorA->setSpeed(A);
  motorB->setSpeed(B);
  motorC->setSpeed(C);
  motorD->setSpeed(D);
}
void motors::fw(int speed){
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}
void motors::backward(int speed){
  motorA->run(BACKWARD);
  motorB->run(BACKWARD);
  motorC->run(BACKWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}
void motors::turnright(int speed){
  motorA->run(FORWARD);
  motorB->run(BACKWARD);
  motorC->run(FORWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}
void motors::turnleft(int speed){
  motorA->run(BACKWARD);
  motorB->run(FORWARD);
  motorC->run(BACKWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
}