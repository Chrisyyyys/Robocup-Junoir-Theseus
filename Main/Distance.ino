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
    int value = sensors[2].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
      
  }
  if(sensor == 2){
    myMux.setPort(1);
    int value = sensors[1].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
      
  }
  if(sensor==3){
    myMux.setPort(0);
    int value = sensors[0].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==4){
    myMux.setPort(3);
    int value = sensors[3].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==5){
    myMux.setPort(6);
    int value = sensors[6].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==6){
    myMux.setPort(5);
    int value = sensors[5].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==7){
    myMux.setPort(4);
    int value = sensors[4].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }

  return -1;
}
// detects wall in a direction( 0 is north, 1 is east, etc..) If output = 0, there is a wall.
// realtive directions(local).
int detectWall(int dir){
  if(dir == 0){ // check if there is a wall at north
    int a = measure(1);
    int b = measure(7);
    if((a<MIN_DIST&&a!=-1&&a!=8191)||(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0; // there is a wall.
    }
    else{
      return 1; // no wall
    }
  }
  if(dir == 1){
    int a = measure(2);
    int b = measure(3);
    if((a<MIN_DIST&&a!=-1&&a!=8191)||(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
  }
  if(dir == 2){
    int a = measure(4);
    if(a<MIN_DIST&&a!=-1&&a!=8191){
      return 0;
    }
    else{
      return 1;
    }
  }
  if(dir == 3){
    int a = measure(5);
    int b = measure(6);
    
    if((a<MIN_DIST&&a!=-1&&a!=8191)||(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
    
  }

  return 1;
}

void parallel(){
  if(detectWall(1) == 0){
    if(measure(2)-measure(3)>0){
      while(abs(measure(2)-measure(3))>3){
        
        motorB->run(BACKWARD);
        motorD->run(BACKWARD);
        motorA->setSpeed(100);
        motorB->setSpeed(100);
        motorC->setSpeed(100);
        motorD->setSpeed(100);
      }
      Serial.println("paralleled");
    }
    else if(measure(2)-measure(3)<0) {
      while(abs(measure(2)-measure(3))>3){
        
        motorA->run(BACKWARD);
        motorC->run(BACKWARD);
        motorA->setSpeed(100);
        motorB->setSpeed(100);
        motorC->setSpeed(100);
        motorD->setSpeed(100);
      }
    }
  }
  else if(detectWall(3) == 0){
    if(measure(6)-measure(5)>0){
      while(abs(measure(6)-measure(5))>3){
        motorA->run(BACKWARD);
        motorC->run(BACKWARD);
        motorA->setSpeed(100);
        motorB->setSpeed(100);
        motorC->setSpeed(100);
        motorD->setSpeed(100);
      }
    }
    else if(measure(6)-measure(5)<0){
      while(abs(measure(6)-measure(5))>3){
        motorB->run(BACKWARD);
        motorD->run(BACKWARD);
        motorA->setSpeed(100);
        motorB->setSpeed(100);
        motorC->setSpeed(100);
        motorD->setSpeed(100);
      }
      
    }
  }
  fullstop();
}
int leftright = 0;
void obstacleavoidance(int leftright){
  while(true){
    switch (steps){
      case TURN:{
        if(leftright == 1){ // obstacle at left
          while(measure(7) < MIN_DIST){
            motorB->run(BACKWARD);
            motorD->run(BACKWARD);
            motorA->setSpeed(255);
            motorB->setSpeed(255);
            motorC->setSpeed(255);
            motorD->setSpeed(255);
          }
        }
        else if(leftright == 0){ // obstacle at right
          while(measure(1)<MIN_DIST){
            motorA->run(BACKWARD);
            motorC->run(BACKWARD);
            motorA->setSpeed(255);
            motorB->setSpeed(255);
            motorC->setSpeed(255);
            motorD->setSpeed(255);
          }
          
        }
        fullstop();
        delay(200);
        motorA->setSpeed(255);
        motorB->setSpeed(255);
        motorC->setSpeed(255);
        motorD->setSpeed(255);
        delay(300);
        fullstop();
        delay(200);
        steps = PARALLEL;
        break;
      }
      case PARALLEL:{
        PID pid(1,0,0.1);
        if(leftright == 1){
          while(measure(2)>=25){
            double increment = pid.getPID(abs(measure(3)-measure(2))); // get close to the edge as possible.
            motorA->setSpeed(constrain(100-increment,30,255));
            motorB->setSpeed(constrain(100+increment,30,225));
            motorC->setSpeed(constrain(100-increment,30,255));
            motorD->setSpeed(constrain(100+increment,30,225));
            if(abs(measure(3)-measure(2))<=5){
              fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        else if(leftright == 0){
          while(measure(6)>=25){
            double increment = pid.getPID(abs(measure(6)-measure(5))); // get close to the edge as possible.
            motorA->setSpeed(constrain(100-increment,30,255));
            motorB->setSpeed(constrain(100+increment,30,225));
            motorC->setSpeed(constrain(100-increment,30,225));
            motorD->setSpeed(constrain(100+increment,30,225));
            if(abs(measure(6)-measure(5))<=5){
              fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        steps = BACKTRACK; // put switch step in front of end( always meet it)
        break;
        end:
          break;
        
      }
      case BACKTRACK:{
        Serial.println("backtracking");
        if(leftright == 0){
          while(measure(6)<=25){
            motorA->run(BACKWARD);
            motorB->run(BACKWARD);
            motorC->run(BACKWARD);
            motorD->run(BACKWARD);
            motorA->setSpeed(100);
            motorB->setSpeed(100);
            motorC->setSpeed(100);
            motorD->setSpeed(100);
          }
        }
        else if(leftright == 1){
          while(measure(2)<=40){
            motorA->run(BACKWARD);
            motorB->run(BACKWARD);
            motorC->run(BACKWARD);
            motorD->run(BACKWARD);
            motorA->setSpeed(100);
            motorB->setSpeed(100);
            motorC->setSpeed(100);
            motorD->setSpeed(100);
          }
        }
        fullstop();
        delay(200);
        steps = PARALLEL;
        break;
      }
      case FWD:{
        fwd(300);
      }
    }
  }
}




