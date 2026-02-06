
#include <Arduino.h>
#include "PID.h"

PID::PID(double _kp, double _ki, double _kd){
  kp = _kp; // public kp = inputted _kp
  ki = _ki;
  kd = _kd;
  previousTime = micros();
  prevError = 0;
  cumError = 0;

}
double PID::getPID(double _error){
  error = _error;
  currentTime = micros(); // functions are slower! you need to use micros
  unsigned long dt = currentTime - previousTime;
  if (dt == 0) dt = 1;
  delta = (error-prevError)/dt;
  cumError += error;
  double output = kp*error + ki*cumError + kd*delta;
  previousTime = currentTime;
  prevError = error; // update previousTime and prevError
  return output;
  
}

