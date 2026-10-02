// Simulator version of the Adafruit Motor Shield V2 library (PCA9685 + TB6612).
// setSpeed() takes a uint8_t like the real library, so values outside 0..255
// wrap around exactly as they do on the robot.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

class Adafruit_MotorShield;

class Adafruit_DCMotor {
 public:
  Adafruit_DCMotor() {}
  void run(uint8_t cmd);
  void setSpeed(uint8_t speed);
  void setSpeed(int speed);  // simulator only: same result as the uint8_t version, but notices wrap-around
  void setSpeedFine(uint16_t speed) { setSpeed((uint8_t)(speed >> 4)); }
  void fullOn() { setSpeed((uint8_t)255); }
  void fullOff() { setSpeed((uint8_t)0); }
  int sim_index = 0;
  Adafruit_MotorShield* sim_shield = nullptr;
};

class Adafruit_StepperMotor {
 public:
  void setSpeed(uint16_t rpm) { (void)rpm; }
  void step(uint16_t steps, uint8_t dir, uint8_t style = SINGLE) { (void)steps; (void)dir; (void)style; }
  uint8_t onestep(uint8_t dir, uint8_t style) { (void)dir; (void)style; return 0; }
  void release() {}
};

class Adafruit_MotorShield {
 public:
  Adafruit_MotorShield(uint8_t addr = 0x60);
  bool begin(uint16_t freq = 1600, TwoWire* theWire = &Wire);
  Adafruit_DCMotor* getMotor(uint8_t n);
  Adafruit_StepperMotor* getStepper(uint16_t steps, uint8_t n) { (void)steps; (void)n; return &steppers_[0]; }
  void setPWM(uint8_t pin, uint16_t val) { (void)pin; (void)val; }
  void setPin(uint8_t pin, bool val) { (void)pin; (void)val; }

 private:
  uint8_t addr_;
  Adafruit_DCMotor motors_[4];
  Adafruit_StepperMotor steppers_[2];
};
