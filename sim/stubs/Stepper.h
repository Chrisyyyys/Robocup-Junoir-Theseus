// Simulator version of the Arduino Stepper library. step() blocks for as long as
// the real motor would take at the set speed.
#pragma once
#include <Arduino.h>

class Stepper {
 public:
  Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2);
  Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4);
  void setSpeed(long whatSpeed);
  void step(int number_of_steps);
  int version() { return 5; }

 private:
  int steps_per_rev_;
  unsigned long step_delay_us_ = 0;
};
