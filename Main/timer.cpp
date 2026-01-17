#include<Arduino.h>
#include "timer.h"

timer::timer(){
  startTime = micros();
}
double timer::getTime(){
  currentTime = micros();
  return currentTime - startTime;
}