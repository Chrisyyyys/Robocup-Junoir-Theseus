void encoder_update_B(){ // why the fuck is it high when going backwards???
  int encoderState = digitalRead(encoderPin_B_B);
  
  if(encoderState == HIGH){
    encoderCountB--;
  }
  else{
    encoderCountB++;
  }
}
void encoder_update_A(){ // every pulse consists for once going up and once going down.
  int encoderState = digitalRead(encoderPin_A_B);
 
  if(encoderState == LOW){
    encoderCountA--;
  }
  else{
    encoderCountA++;
  }
}
void init_drive(){
  pinMode(encoderPin_A_A, INPUT);
  pinMode(encoderPin_A_B,INPUT);
  pinMode(encoderPin_B_A, INPUT);
  pinMode(encoderPin_B_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin_A_A), encoder_update_A, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin_B_A), encoder_update_B, RISING); // only counting rising of Pin: (20/2)/2
  if(!AFMS.begin()){
    Serial.println("could not find motor shield");
  }
  else{
    Serial.println("motor shield found");
  }
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  // initialize gyro
  myGyro.init_Gyro();
}
// full stop
void fullstop(){
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
}
void fwd(double dist){ // in mm
  double pulses = dist/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  bool black = false; // toggle for black tile
  bool climbtoggle = false; // toggle for climbing
  int cnt = 0; // tiles traversed while climbing.
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  double init_heading = myGyro.heading();
  PID myPID(0.30,0,0.2); // 0.28 for 125
  Serial.println("forwarding");
  int init_yaw = myGyro.modulus((int)myGyro.yaw_heading());
  while((encoderCountA<= pulses && (encoderCountB <= pulses||climbtoggle == true))&&black!=true){
    //if(digitalRead(logicswitch)==true) Pausemaze = true;
    // check cameras
    
    /*
    if(readSerial1()!=-1&&victimtoggle == false&&t.getVictim()==false){ // check serial
      fullstop();
      
      if(detectWall(3)==0){
        Serial.println(measure(6));
        Serial.println("victim at left");
        //Serial.println((char)Serial2.read());
        myPID.pausePID(1);
        clearSerialBuffer1();
        detectCam1();
        myPID.pausePID(2);
        victimtoggle = true;
      }
    }
    else if(readSerial2()!=-1&&victimtoggle == false&&t.getVictim()==false){
      fullstop();
      if(detectWall(1)==0){
        Serial.println(measure(2));
        Serial.println("victim at right");
        //Serial.println((char)Serial3.read());
        myPID.pausePID(1);
        clearSerialBuffer2();
        detectCam2();
        myPID.pausePID(2);
        victimtoggle = true;
      }
    }
    
    if(victimtoggle == true){
      if(encoderCountA<2*pulses/3){
        victimAtCurrent = true;
        mapGrid[x_pos][y_pos].setVictim(true);
      }
      else victimAtCurrent = false;
    }
    */
    // color
    
    int color = read_color(); // read color
    if(color == 1&&use_color%2==0){
      bluetoggle = true;
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(1);
    }
    if(color == -1&&use_color%2==0){
      fullstop();
      delay(100);
      
      //Do you mean  t.setWall(plannedMoveDir, true)
      // find next tile, set it to black
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(3);
      blacktoggle = true;
      
      while(encoderCountA >= 0 && encoderCountB >= 0){
        motorA->run(BACKWARD);
        motorB->run(BACKWARD);
        motorC->run(BACKWARD);
        motorD->run(BACKWARD);
       
        motorA->setSpeed(200);
        motorB->setSpeed(200);
        motorC->setSpeed(200);
        motorD->setSpeed(200);
      }
      black = true;
    }
    
    // PID centering
    difference = center();
    //Serial.println(center());
    double adjustment = myPID.getPID(difference);
    // emergency stop
    int a = measure(1);
    int b = measure(7);
    if((a<=50&&a!=-1)||(b<=50&&b!=-1)){
      Serial.println("stopping");
      Serial.println(a);
      Serial.println(b);
      fullstop();
      delay(50);
      break;
    }
    // self correction
    // if acceleration is greater than a certain value and it is not just a stop then do something.
    /*
    double d = myGyro.get_filtered_acceleration();
    Serial.println(d);
    if(abs(d)>0.5&&encoderCountA>pulses/30&&encoderCountA<pulses*29/30&&victimtoggle==false){
      Serial.println("botched fwd");
      Serial.println(d);
      // step 1: go back
      motorA->run(BACKWARD);
      motorB->run(BACKWARD);
      motorC->run(BACKWARD);
      motorD->run(BACKWARD);
      motorA->setSpeed(150);
      motorB->setSpeed(150);
      motorC->setSpeed(150);
      motorD->setSpeed(150);
      delay(500);
      fullstop();
      delay(200);
      // step 2: turn( with slight offset)
      absoluteturn(init_heading+(myGyro.modulus((int)myGyro.heading())-myGyro.modulus((int)init_heading)>0 ? -10: 10)); // boundary condition kinda broken but we can fix it in the future.
      // step 3: reset encoders
      encoderCountA = 0; encoderCountB = 0;
    }
    */
    // check for steps( stop)
    
    double acceleration = myGyro.get_acceleration();
    Serial.println(acceleration);
    if(acceleration>0.3&&encoderCountA>pulses/30&&encoderCountA<pulses*29/30){
      if(measure(1)>=MIN_DIST&&measure(7)>=MIN_DIST&&abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw)<5){
        Serial.println(acceleration);
        int stairX = x_pos;
        int stairY = y_pos;
        stepForward(currentDir, stairX, stairY);
        updateFullyExploredAt(x_pos, y_pos);
        if (inBounds(stairX, stairY)) {
          mapGrid[stairX][stairY].setDiscovered(true);
          mapGrid[stairX][stairY].setType(STAIR);
        }
        stairtoggle = true;
        delay(1000);
        detachInterrupt(digitalPinToInterrupt(encoderPin_A_A));
        detachInterrupt(digitalPinToInterrupt(encoderPin_B_A));
        motorA->run(BACKWARD);
        motorB->run(BACKWARD);
        motorC->run(BACKWARD);
        motorD->run(BACKWARD);
        motorA->setSpeed(150);
        motorB->setSpeed(150);
        motorC->setSpeed(150);
        motorD->setSpeed(150);
        delay(250);
        fullstop();
        delay(200);
        absoluteturn(myGyro.opposite_heading(plannedTurnDeg)); // turn 180
        delay(200);
        motorA->run(BACKWARD);
        motorB->run(BACKWARD);
        motorC->run(BACKWARD);
        motorD->run(BACKWARD);
        attachInterrupt(digitalPinToInterrupt(encoderPin_A_A), encoder_update_A, RISING); // turn encoders back on
        attachInterrupt(digitalPinToInterrupt(encoderPin_B_A), encoder_update_B, RISING);
        while(encoderCountA>-pulses*1.3&&encoderCountB>-pulses*1.3){
          motorA->setSpeed(150);
          motorB->setSpeed(150);
          motorC->setSpeed(150);
          motorD->setSpeed(150);
        }
        fullstop();
        delay(200);
        absoluteturn(plannedTurnDeg);
        delay(100);
        break;
      }
    }
    // check yaw heading
    // if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
     if(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 15){
      int _encoderCountB = encoderCountB;
      climbtoggle = true;
      encoderCountB = 0;
      Serial.println(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw));
      Serial.println("climbing");
      
      while(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 15){
        Serial.println(encoderCountB);
        detachInterrupt(digitalPinToInterrupt(encoderPin_A_A)); // stop encoders
         // PID centering
        difference = center();
        //Serial.println(center());
        double adjustment = myPID.getPID(difference);
        motorA->setSpeed(200+adjustment); // center during climbing
        motorC->setSpeed(200+adjustment);
        motorB->setSpeed(200-adjustment);
        motorD->setSpeed(200-adjustment);
        if(encoderCountB >= pulses/cos(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw)*(M_PI/180))){
          //Serial.println(pulses/cos(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw)));
          Serial.println(encoderCountB);
          Serial.println("1 section of the ramp climbed");
          cnt++;
          encoderCountB=0;
        } // track tiles
      }
      use_color++;
      attachInterrupt(digitalPinToInterrupt(encoderPin_A_A), encoder_update_A, RISING);
      attachInterrupt(digitalPinToInterrupt(encoderPin_B_A), encoder_update_B, RISING);
      encoderCountB = _encoderCountB;
    }
    motorA->setSpeed(150+adjustment);
    motorC->setSpeed(150+adjustment);
    motorB->setSpeed(150-adjustment);
    motorD->setSpeed(150-adjustment);
  }
  // sometimes it barely makes it over the slope
  if(climbtoggle == true){
    for(int i = 0; i<cnt;i++){
      Serial.println("adding ramp to map");
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
    }
    Serial.println("compensating");
    motorA->setSpeed(200);
    motorB->setSpeed(200);
    motorC->setSpeed(200);
    motorD->setSpeed(200);
    delay(100);
  }
  
  fullstop();
  encoderCountA = 0; encoderCountB = 0;

}
// absolute turning
void absoluteturn(double angle){ 
  // create PID instance.
  PID myPID(10,0,0.3); 
  double MOTORSPEED = 0;
  double current_angle=myGyro.heading();
  bool fasterway = false;
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  if(abs(angle-current_angle)> abs(angle-(360-current_angle))){
    current_angle = myGyro.inverse(current_angle,true); // make sure the robot turns the least amount
    fasterway = true;
  } 
  double init_angle = current_angle;
  Serial.println("current angle");
  Serial.println(current_angle);
   // create timer to cut of turning
  timer myTimer;

  if(myGyro.inverse(angle,fasterway) - current_angle > 0){
    while(true){
      motorB->run(BACKWARD);
      motorD->run(BACKWARD);
      if(myGyro.inverse(angle,fasterway)-current_angle<=0 && current_angle < 190) break;
      
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break; // turning limit
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      
      MOTORSPEED = myPID.getPID(myGyro.inverse(angle,fasterway)-current_angle);
      if(readSerial1()!=-1&&victimtoggle == false&&t.getVictim()==false){
        fullstop();
        if(detectWall(3)==0){
          Serial.println("victim at left");
          myPID.pausePID(1);
          myTimer.pause(1);
          
          clearSerialBuffer1();
          detectCam1();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
      }
      else if(readSerial2()!=-1&&victimtoggle == false&&t.getVictim()==false){
        fullstop();
        if(detectWall(1)==0){
          Serial.println("victim at right");
          myPID.pausePID(1);
          myTimer.pause(1);
          
          clearSerialBuffer2();
          detectCam2();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
      }
      motorA->setSpeed(constrain(MOTORSPEED,20,200));
      motorB->setSpeed(constrain(MOTORSPEED,20,200));
      motorC->setSpeed(constrain(MOTORSPEED,20,200));
      motorD->setSpeed(constrain(MOTORSPEED,20,200));
    }
  }

  else if(myGyro.inverse(angle,fasterway)-current_angle<0) {
    while(true){
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
      if(myGyro.inverse(angle,fasterway)-current_angle>=0 && current_angle > 170) break;
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break;
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      MOTORSPEED = myPID.getPID(current_angle-myGyro.inverse(angle,fasterway));
      if(readSerial1()!=-1&&victimtoggle == false&&t.getVictim()==false){
        fullstop();
        if(detectWall(3)==1){
          Serial.println("victim at left");
          myPID.pausePID(1);
          myTimer.pause(1);
          
          clearSerialBuffer1();
          detectCam1();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
        
      }
      else if(readSerial2()!=-1&&victimtoggle == false&&t.getVictim()==false){
        fullstop();
        if(detectWall(1)==1){
          Serial.println("victim at right");
          myPID.pausePID(1);
          myTimer.pause(1);
          clearSerialBuffer2();
          detectCam2();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
      }
      motorA->setSpeed(constrain(MOTORSPEED,20,200));
      motorB->setSpeed(constrain(MOTORSPEED,20,200));
      motorC->setSpeed(constrain(MOTORSPEED,20,200));
      motorD->setSpeed(constrain(MOTORSPEED,20,200));
    }
  }
  
  if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
  Serial.println("finished turning");
  fullstop();
  encoderCountA = 0; encoderCountB = 0; // reset encoder counters.
}
// full stop function
