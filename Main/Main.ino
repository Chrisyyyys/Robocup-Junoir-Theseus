#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include <Adafruit_MotorShield.h>
#include "sensor_manager.h"
#include "DistanceSensor.h"
// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// set up motorshield and motors.
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorB = AFMS.getMotor(2);
// set up encoder pins
const int encoderPin_B_A = 2;
const int encoderPin_B_B = 3; 
// encoder counters
int encoderCountB;
// wheel cpr
const int wheel_cpr = 5; // 20/4
//gear ratio
const int gear_ratio = 195;
void setup(){
  Serial.begin(115200);
  Wire.begin();
  init_drive();
  delay(500);
 
}
void loop(){
  while(encoderCountB<wheel_cpr*gear_ratio){
    motorB->setSpeed(100);
    Serial.println(encoderCountB);
  }
  motorB->setSpeed(0);
  delay(1000);
  encoderCountB = 0;
  
}
void initializeMap() {
    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {

            mapGrid[x][y].discovered = false;
            mapGrid[x][y].fullyExplored = false;

            for (int d = 0; d < 4; d++) {
                mapGrid[x][y].wall[d] = false;
                mapGrid[x][y].edge[d] = false;
            }

            mapGrid[x][y].tileType = false;
            mapGrid[x][y].victim = false;
        }
    }
}