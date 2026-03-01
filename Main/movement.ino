void encoder_update_B(){ // why the fuck is it high when going backwards???
  int encoderState = digitalRead(encoderPin_B_B);
  
  if(encoderState == LOW){
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
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  PID myPID(0.4,0,0.2);
  int init_yaw = myGyro.modulus((int)myGyro.yaw_heading());
  while((encoderCountA<= pulses && encoderCountB <= pulses)&&black!=true){
    // color
    int color = read_color(); // read color
    if(color == 1&&use_color%2==0){
      bluetoggle = true;
    }
    if(color == -1&&use_color%2==0){
      fullstop();
      delay(100);
      
      //Do you mean  t.setWall(plannedMoveDir, true)
      t.setWall(plannedMoveDir, true);
      blacktoggle = true;
      
      while(encoderCountA >= 0 && encoderCountB >= 0){
        motorA->run(BACKWARD);
        motorB->run(BACKWARD);
        motorC->run(BACKWARD);
        motorD->run(BACKWARD);
       
        motorA->setSpeed(225);
        motorB->setSpeed(225);
        motorC->setSpeed(225);
        motorD->setSpeed(225);
      }
      black = true;
    }
    // PID centering
    difference = center();
    //Serial.println(center());
    double adjustment = myPID.getPID(difference);
    // emergency stop
    
    if((measure(1)<=50&&measure(1)!=-1)||(measure(7)<=50&&measure(7)!=-1)){
      Serial.println("stopping");
      Serial.println(measure(1));
      Serial.println(measure(7));
      fullstop();
      delay(50);
      break;
    }
    
    // check yaw heading
    // if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
    if(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 20){
      while(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 20){
        climbtoggle = true;
        Serial.println("climbing");
        detachInterrupt(digitalPinToInterrupt(encoderPin_A_A));
        detachInterrupt(digitalPinToInterrupt(encoderPin_B_A));
        motorA->setSpeed(255);
        motorB->setSpeed(255);
        motorC->setSpeed(255);
        motorD->setSpeed(255);
      }
      use_color++;
      attachInterrupt(digitalPinToInterrupt(encoderPin_A_A), encoder_update_A, RISING);
      attachInterrupt(digitalPinToInterrupt(encoderPin_B_A), encoder_update_B, RISING);
    }
    // check cameras

    if(Serial2.available()&&victimtoggle == false){
      Serial.println("victim at left");
      pausePID(1);
      fullstop();
      clearSerialBuffer1();
      detectCam1();
      pausePID2(2);
      victimtoggle = true;
    }
    else if(Serial3.available()&&victimtoggle == false){
      Serial.println("victim at right");
      pausePID(1);
      fullstop();
      clearSerialBuffer2();
      detectCam2();
      pausePID(2);
      victimtoggle = true;
    }
    
    motorA->setSpeed(150+adjustment);
    motorC->setSpeed(150+adjustment);
    motorB->setSpeed(150-adjustment);
    motorD->setSpeed(150-adjustment);
    Serial.println("cycle");
    
  }
  // sometimes it barely makes it over the slope
  if(climbtoggle == true){
    /*
    writeWallsToCurrentTile(0, 1, 0, 1);
    updateFullyExploredAt(x_pos, y_pos);
    markEdgeBothWays(x_pos, y_pos, currentDir);
    stepForward(currentDir, x_pos, y_pos);
    */
    Serial.println("compensating");
    motorA->setSpeed(255);
    motorB->setSpeed(255);
    motorC->setSpeed(255);
    motorD->setSpeed(255);
    delay(200);
  }
  if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
  fullstop();
  encoderCountA = 0; encoderCountB = 0;

}
// absolute turning
void absoluteturn(double angle){ 
  // create PID instance.
  PID myPID(10,0,0.3); 
  double MOTORSPEED = 0;
  double current_angle=myGyro.heading();
  double init_angle = current_angle;
  Serial.println(current_angle);
   // create timer to cut of turning
  timer myTimer;
  if(angle - current_angle > 0){
    while(true){
      motorB->run(BACKWARD);
      motorD->run(BACKWARD);
      if(angle-current_angle<=0 && current_angle < 190) break;
      
      if(myTimer.getTime() > 2*abs(angle-init_angle)/90*1000000) break; // turning limit
      current_angle = myGyro.heading();
      
      MOTORSPEED = myPID.getPID(angle-current_angle);
      if(Serial2.available()&&victimtoggle == false){
        Serial.println("victim at left");
        myPID.pausePID(1);
        myTimer.pause(1);
        fullstop();
        clearSerialBuffer1();
        detectCam1();
        myTimer.pause(2);
        myPID.pausePID(2);
        victimtoggle = true;
        
      }
      else if(Serial3.available()&&victimtoggle == false){
        Serial.println("victim at right");
        myPID.pausePID(1);
        myTimer.pause(1);
        fullstop();
        clearSerialBuffer2();
        detectCam2();
        myTimer.pause(2);
        myPID.pausePID(2);
        victimtoggle = true;
      }
      motorA->setSpeed(constrain(MOTORSPEED,20,255));
      motorB->setSpeed(constrain(MOTORSPEED,20,255));
      motorC->setSpeed(constrain(MOTORSPEED,20,255));
      motorD->setSpeed(constrain(MOTORSPEED,20,255));
    }
  } 
  else if(angle-current_angle<0) {
    while(true){
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
      if(angle-current_angle>=0 && current_angle > 170) break;
      if(myTimer.getTime() > 2*abs(angle-init_angle)/90*1000000) break;
      current_angle = myGyro.heading();
      MOTORSPEED = myPID.getPID(current_angle-angle);
      if(Serial2.available()&&victimtoggle == false){
        Serial.println("victim at left");
        myPID.pausePID(1);
        myTimer.pause(1);
        fullstop();
        clearSerialBuffer1();
        detectCam1();
        myTimer.pause(2);
        myPID.pausePID(2);
        victimtoggle = true;
      }
      else if(Serial3.available()&&victimtoggle == false){
        Serial.println("victim at right");
        myPID.pausePID(1);
        myTimer.pause(1);
        fullstop();
        clearSerialBuffer2();
        detectCam2();
        myTimer.pause(2);
        myPID.pausePID(2);
        victimtoggle = true;
      }
      motorA->setSpeed(constrain(MOTORSPEED,20,255));
      motorB->setSpeed(constrain(MOTORSPEED,20,255));
      motorC->setSpeed(constrain(MOTORSPEED,20,255));
      motorD->setSpeed(constrain(MOTORSPEED,20,255));
    }
  }
  if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
  Serial.println("finished turning");
  fullstop();
  encoderCountA = 0; encoderCountB = 0; // reset encoder counters.
}
// full stop function
