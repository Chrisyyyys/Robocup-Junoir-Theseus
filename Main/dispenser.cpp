#include <Arduino.h>
#include "dispenser.h"
#include <Stepper.h>
extern Stepper myStepper;
dispenser::dispenser(int _incr, int _offset,int _steps_per_rev){
  incr = _incr;
  offset = _offset;
  steps_per_rev = _steps_per_rev;
  leftDispensed = 0; rightDispensed = 0;
  myStepper.setSpeed(10);
}
void dispenser::rotate(int degrees){
  myStepper.step((degrees/360)*steps_per_rev);
}
void dispenser::dispenseLeft(char victim){ // clockwise
  switch(victim){
    case 'H':
      rotate(-incr*2+offset+leftDispensed*incr);
      leftDispensed += 2;
      rotate(offset+leftDispensed*incr);
    case 'S':
      rotate(-incr*1+offset+leftDispensed*incr);
      leftDispensed += 1;
      rotate(offset+leftDispensed*incr);
    case 'U':
      break;
  }
}
void dispenser::dispenseRight(char victim){ // clockwise
  switch(victim){
    case 'H':
      rotate(incr*2+offset+leftDispensed*incr);
      leftDispensed += 2;
      rotate(-offset+leftDispensed*incr);
    case 'S':
      rotate(incr*1+offset+leftDispensed*incr);
      leftDispensed += 1;
      rotate(-offset+leftDispensed*incr);
    case 'U':
      break;
  }
}