// distance sensor code
// blue for SDA, yellow for SCL
// the motor shield takes up the I2C address at 0x70.

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
/*
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
old robot settings
*/
int measure(int sensor){
  // sensor→mux port mapping
  const int portMap[] = {-1, 1, 0, 6, 4, 5, 3, 2};
  if(sensor < 1 || sensor > 7) return -1;
  int port = portMap[sensor];
  int sensorIdx = port; // sensor index matches port number

  i2cMutex.lock();
  myMux.setPort(port);
  int value = sensors[sensorIdx].readRangeContinuousMillimeters();
  i2cMutex.unlock();

  return (value != -1 && value != 8191) ? value : -1;
}
// detects wall in a direction( 0 is north, 1 is east, etc..) If output = 0, there is a wall.
// realtive directions(local).
int detectWall(int dir){
  if(dir == 0){ // check if there is a wall at north
    int a = measure(1);
    int b = measure(7);
    if((a<MIN_DIST&&a!=-1&&a!=8191)&&(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0; // there is a wall.
    }
    else{
      return 1; // no wall
    }
  }
  if(dir == 1){
    int a = measure(2);
    int b = measure(3);
    if((a<MIN_DIST&&a!=-1&&a!=8191)&&(b<MIN_DIST&&b!=-1&&b!=8191)){
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
    
    if((a<MIN_DIST&&a!=-1&&a!=8191)&&(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
    
  }

  return 1;
}

void parallel(){
  const int PARALLEL_TOL_MM = 3;
  const int PARALLEL_SPEED = 90;
  const unsigned long PARALLEL_TIMEOUT_MS = 500;
  const double MAX_PARALLEL_ROTATION_DEG = 45.0;
  const int PARALLEL_MAX_WALL_MM = TILE_MM; // engage even when the wall is up to one tile away

  int sensorA = -1;
  int sensorB = -1;
  int wallDir;
  Serial.println("paralleling");
  
  
  // Prefer aligning to the right wall; otherwise use left wall.
  if (detectWall(1)==0) {
    sensorA = 2;
    sensorB = 3;
    wallDir=1;
  } else if (detectWall(3)==0) {
    sensorA = 6;
    sensorB = 5;
    wallDir=3;
  } else {
    drivetrain.fullstop();
    return;
  }

  unsigned long startMs = millis();
  double startHeading = myGyro.heading();

  while (true) {
    int a = measure(sensorA);
    int b = measure(sensorB);

    // Invalid reading: stop correction to avoid runaway spinning.
    if (a < 0 || b < 0) {
      Serial.println("parallel: invalid sensor reading, aborting correction");
      break;
    }
    // If either sensor no longer sees the side wall within range, stop correcting
    // (the wall ended / robot isn't beside one) to avoid spinning on a phantom reading.
    if(a>MIN_DIST||b>MIN_DIST){
      break;
    }

    int diff = a - b;
    if (abs(diff) <= PARALLEL_TOL_MM) {
      Serial.println("paralleled");
      break;
    }
    // break out after rotation.

    double headingDelta = myGyro.heading() - startHeading;
    while (headingDelta > 180.0) headingDelta -= 360.0;
    while (headingDelta < -180.0) headingDelta += 360.0;

    if (abs(headingDelta) >= MAX_PARALLEL_ROTATION_DEG) {
      Serial.println("parallel: rotation limit hit, aborting correction");
      break;
    }

    if ((millis() - startMs) >= PARALLEL_TIMEOUT_MS) {
      Serial.println("parallel: timeout, aborting correction");
      break;
    }

    // Reset wheel directions then apply correction turn.
    i2cMutex.lock();
    motorA->run(FORWARD);
    motorB->run(FORWARD);
    motorC->run(FORWARD);
    motorD->run(BACKWARD);
    if ((diff > 0 && wallDir == 1)||(diff < 0 && wallDir==3)) {
      motorB->run(BACKWARD);
      motorD->run(FORWARD);
    } else {
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
    }
    i2cMutex.unlock();
    drivetrain.drive(PARALLEL_SPEED,PARALLEL_SPEED,PARALLEL_SPEED,PARALLEL_SPEED);
    
  }
  drivetrain.reset_encoderCount(true,true,true);
  drivetrain.fullstop();
}

// Self-centers the robot front-to-back within a tile using the front wall (avg of sensors 1+7).
// Only acts when a front wall is present (back-wall-only centering is not implemented yet)
// parallel() runs first so the robot is squared to a side wall before the front reading is trusted.

void centerFrontBack(){
  const int CENTERING_SPEED = 50;                   // mirrors PARALLEL_SPEED
  const unsigned long CENTERING_TIMEOUT_MS = 2000;
  // MAX_CENTER_CORRECTION_MM is a file-scope #define (Main.ino), shared with the SENSE_TILE trigger gate >> redundant safety abort 
  // -> in case conditions changed between the trigger check and this function actually running.

  Serial.println("centering front-back (front wall)");
  parallel();

  if(detectWall(0) != 0){ // 0 == wall present, matches detectWall's convention
    Serial.println("centerFrontBack: no front wall, nothing to center against");
    return;
  }

  int front1 = measure(1);
  int front7 = measure(7);
  if(front1 == -1 || front7 == -1){
    Serial.println("centerFrontBack: invalid initial reading, aborting");
    return;
  }

  double frontGap = (front1 + front7) / 2.0;
  double offset = frontGap - TARGET_GAP_MM; // +ve => too far from ront wall, drive forward; -ve => drive backward

  if(abs(offset) >= MAX_CENTER_CORRECTION_MM){
    Serial.println("centerFrontBack: offset exceeds sanity cap, aborting");
    return;
  }
  if(abs(offset) <= CENTER_TOL_MM){
    Serial.println("already centered");
    return;
  }

  bool driveForward = offset > 0;
  unsigned long startMs = millis();

  while(true){
    front1 = measure(1);
    front7 = measure(7);
    if(front1 == -1 || front7 == -1){
      Serial.println("centerFrontBack: invalid sensor reading mid-correction, aborting");
      break;
    }

    frontGap = (front1 + front7) / 2.0;
    offset = frontGap - TARGET_GAP_MM;

    if(abs(offset) <= CENTER_TOL_MM){
      Serial.println("centered");
      break;
    }
    // If the live offset flips sign vs. our initial decision >> overshot, stop rather than reversing (avoids oscillation).
    if((offset > 0) != driveForward){
      Serial.println("centerFrontBack: overshot target, stopping to avoid oscillation");
      break;
    }
    if((millis() - startMs) >= CENTERING_TIMEOUT_MS){
      Serial.println("centerFrontBack: timeout, aborting correction");
      break;
    }

    if(driveForward) drivetrain.fw(CENTERING_SPEED);
    else drivetrain.backward(CENTERING_SPEED);
  }

  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true);
}

int center(){
  int a = measure(2);
  int b = measure(6);
  if(b != 8191 && a != -1 && b!=8191 && b != -1) return (b%30-a%30); // mod 30 to find centering
  else return 0;
}


int obstacleavoidance(int leftright){ // leftright determines to manuver left or right.
// return distance to wall at front
  Serial.println("obstacle avoidance");
  int _ = -1;
  while(true){
    // Single authoritative pause guard: gates every step boundary and transition
    // burst, not just the innermost drive loops. Reset steps so a resume after the
    // pause starts a fresh maneuver instead of re-entering mid-sequence.
    if(Pausemaze == true){
      drivetrain.fullstop();
      steps = TURN;
      return -2;
    }
    switch (steps){
      case TURN:{
        if(leftright == 1){ // obstacle at left
          _ = measure(1);
          _ = (_!=-1&&_!=8191) ? _ : -1;
          Serial.println("turn step");
          while(measure(7) < MIN_DIST){
            motorB->run(BACKWARD);
            motorD->run(FORWARD);
            drivetrain.drive(150,150,150,150);
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
          }
          
        }
        else if(leftright == 0){ // obstacle at right
          _ = measure(7);
          _ = (_!=-1&&_!=8191) ? _ : -1;
          while(measure(1)<MIN_DIST){
            motorA->run(BACKWARD);
            motorC->run(BACKWARD);
            motorB->run(FORWARD);
            motorD->run(BACKWARD); // D is mounted reversed; BACKWARD raw = physically FORWARD, matching motorB
            drivetrain.drive(150,150,150,150);
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
          }
          
        }
        drivetrain.fullstop();
        delay(200);
        drivetrain.fw(150);
        delay(500);
        drivetrain.fullstop();
        delay(200);
        steps = PARALLEL;
        break;
      }
      case PARALLEL:{
        PID pid(1,0,0.1);
        if(leftright == 1){
          int a = measure(2); int b = measure(3);
          while(true){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            a=measure(2); b = measure(3);
            if(a<=30) break;
            Serial.println("paralleling step");
            double increment = pid.getPID(a-b); // signed error: positive turns one way, negative the other
            drivetrain.drive(constrain(100+increment,50,170),constrain(100+increment,50,170),constrain(100-increment,50,170),constrain(100-increment,50,170));
            Serial.println(a-b);
            
            
            if(abs(b-a)<=15){
              //fwd
              drivetrain.fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        else if(leftright == 0){
          while(true){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            int a = measure(6); int b = measure(5);
            if(a<=30) break;
            double increment = pid.getPID(a-b); // signed error: positive turns one way, negative the other
            drivetrain.drive(constrain(100-increment,50,170),constrain(100-increment,50,170),constrain(100+increment,50,170),constrain(100+increment,50,170));
            
            if(abs(a-b)<=15){
              drivetrain.fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        Serial.println("too close, backing up");
        Serial.println(measure(2));
        steps = BACKTRACK; // put switch step in front of end( always meet it)
        break;
        end:
          break;
        
      }
      case BACKTRACK:{
        Serial.println("backtracking step");
        // put a timer on to prevent it from taking too long
        timer myTime;
        if(leftright == 0){
          while(measure(6)<=40&&myTime.getTime()<800000){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            drivetrain.backward(120);
          }
        }
        else if(leftright == 1){
          while(measure(2)<=40&&myTime.getTime()<800000){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            drivetrain.backward(120);
          }
        }
        drivetrain.fullstop();
        delay(200);
        Serial.println("sensor 2, now reading");
        Serial.println(measure(2));
        steps = PARALLEL;
        break;
      }
      
      case FWD:{
        
        if(measure(6)<=35&&measure(2)<=35){
          steps = WIGGLE;
          return -2;
        }
        
        Serial.println("fwd step");
        parallel();
        drivetrain.reset_encoderCount(true,true,true);
        delay(200);
        
        // Drive the rest of the tile, subtracting distance already travelled.
        // Read the front sensor ONCE (a second read can differ and overshoot) and
        // clamp to [0, TILE_MM]: if either front reading is invalid the front is
        // open/garbage, so fall back to one tile instead of a runaway distance.
        int frontNow = measure(1);
        int travelled = (_ != -1 && frontNow != -1) ? (_ - frontNow) : 0;
        int remaining = constrain(TILE_MM - travelled, 0, TILE_MM);
        fwd(remaining);
        steps = TURN;
        return _;
      }
      case WIGGLE:{
        PID pid(8,0,0.1);
        Serial.println("wiggle step");
        delay(2000);
        timer myTime;
        while(abs(measure(2)-measure(6))>=15&&myTime.getTime()<1000000){
          if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
          double diff = pid.getPID(measure(2)-measure(6));
          drivetrain.drive(70+diff,70+diff,70-diff,70-diff);
        }
        drivetrain.fullstop();
        delay(200);
        steps = FWD;
        break;
      }
    }
  }
}



