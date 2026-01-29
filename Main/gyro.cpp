#include "gyro.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;

gyro::gyro(){
  double headingOffset = 0;
}
void gyro::init_Gyro(){
  if(!bno.begin()){
    Serial.println("can't find gyro");
  }
  else{
    Serial.println("gyro found");
  }
}
void gyro::resetHeading(){
  sensors_event_t event; bno.getEvent(&event);
  headingOffset = (float)event.orientation.x;
}
double gyro::heading(){
  sensors_event_t event; bno.getEvent(&event);
  float heading = (double)event.orientation.x-headingOffset;
  Serial.println("heading");
  Serial.println(heading);
  heading = (heading+ 360)%360;
  return heading;
}