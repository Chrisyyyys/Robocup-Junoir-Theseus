#include<Arduino.h>
#include "timer.h"

timer::timer(){
  startTime = micros();
}
double timer::getTime(){
  currentTime = micros();
  return currentTime - startTime-(endTimeStamp-startTimeStamp);
}
void timer::pause(int on){
  if(on == 1) startTimeStamp = micros();
  if(on == 2) endTimeStamp = micros();
  
}