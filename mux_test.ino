#include <Wire.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include <VL53L0X.h>
QWIICMUX myMux;
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if(!myMux.begin()){ // initialize mux
    Serial.println("could not find multiplexer");
  }
  else{
    Serial.println("multiplexer found");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
