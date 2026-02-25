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
int gyro::modulus(int val){
  if(val> 180) {val = val - 360;}
  else{val = val%360;}
  return val;
}
double gyro::heading(){
  sensors_event_t event; bno.getEvent(&event);
  float heading = (double)event.orientation.x;
  if (abs(360-heading)< 5) heading = 0; // wraparound
  return heading;
}
double gyro::yaw_heading(){
  sensors_event_t event; bno.getEvent(&event);
  float yaw_heading = (double)event.orientation.y;
  if (abs(360-yaw_heading)< 5) yaw_heading = 0;
  return yaw_heading;
}