#include "gyro.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;

gyro::gyro(){
  
}
void gyro::init_Gyro(){
  if(!bno.begin()){
    Serial.println("can't find gyro");
  }
  else{
    Serial.println("gyro found");
  }
}

double gyro::heading(){
  sensors_event_t event; bno.getEvent(&event);
  float heading = (double)event.orientation.x;
  
  return heading;
}