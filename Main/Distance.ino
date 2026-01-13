// distance sensor code
// blue for SDA, yellow for SCL
// the motor shield takes up the I2C address at 0x70 so chatgpt made some code that prevents conflict.



void disableAllCall() {
  // Point register to MODE1
  Wire.beginTransmission((uint8_t)PCA_ADDR);
  Wire.write((uint8_t)MODE1);
  Wire.endTransmission(false);

  // Read MODE1
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (Wire.available() < 1) return; // couldn't read
  uint8_t mode1 = Wire.read();

  // Clear ALLCALL bit
  mode1 &= (uint8_t)~ALLCALL_BIT;

  // Write MODE1 back
  Wire.beginTransmission((uint8_t)PCA_ADDR);
  Wire.write((uint8_t)MODE1);
  Wire.write(mode1);
  Wire.endTransmission(true);
}

void init_dist() {
  // put your setup code here, to run once:
  
  if(!myMux.begin()){
    Serial.println("can't find the Mux");
  }
  else{
    Serial.println("Mux initialized");
  }
 
  for(int i = 0; i<7; i++){
    myMux.setPort(i);
    sensors[i].setAddress(0x30); // conflict with TCS34725 for some reason.
    delay(10);
    if(!sensors[i].init()){
      Serial.println("Sensor "+String(i)+" failed to initialize");
    }
    else{
      Serial.println("Sensor "+String(i)+" is able to initialize");
    }
    sensors[i].startContinuous(); // start continuous ranging.
  }
  

}

uint8_t scanI2COnCurrentBus() {
  uint8_t count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" ");
      count++;
    }
  }
  return count;
}

void scanAllPorts() {
  for (uint8_t port = 0; port < 8; port++) {
    bool ok = myMux.setPort(port);

    Serial.print("Port ");
    Serial.print(port);
    Serial.print(ok ? ": " : ": (setPort FAILED) ");

    delay(20);

    uint8_t found = scanI2COnCurrentBus();
    if (found == 0) Serial.print("(none)");
    Serial.println();
  }
}
// measure distance
int measure(int sensor){
  
  if(sensor ==1){
    myMux.setPort(2);
    return sensors[2].readRangeContinuousMillimeters();
      
  }
  if(sensor == 2){
    myMux.setPort(1);
    return sensors[1].readRangeContinuousMillimeters();
      
  }
  if(sensor==3){
    myMux.setPort(0);
    return sensors[0].readRangeContinuousMillimeters();
    
  }
  if(sensor==4){
    myMux.setPort(3);
    return sensors[3].readRangeContinuousMillimeters();
    
  }
  if(sensor==5){
    myMux.setPort(6);
    return sensors[6].readRangeContinuousMillimeters();
    
  }
  if(sensor==6){
    myMux.setPort(5);
    return sensors[5].readRangeContinuousMillimeters();
    
  }
  if(sensor==7){
    myMux.setPort(4);
    return sensors[4].readRangeContinuousMillimeters();
    
  }
}




