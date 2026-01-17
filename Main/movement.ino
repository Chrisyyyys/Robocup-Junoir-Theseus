void encoder_update_B(){ // why the fuck is it high when going backwards???
  int encoderState = digitalRead(encoderPin_B_B);
  
  if(encoderState == LOW){
    encoderCountB++;
  }
  else{
    encoderCountB--;
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
  if(!bno.begin()){
    Serial.println("can't find gyro");
  }
  else{
    Serial.println("gyro found");
  }
}
void fwd(double dist){
  double pulses = dist/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  while(encoderCountA<= pulses & encoderCountB <= pulses){
    motorA->setSpeed(255);
    motorB->setSpeed(255);
    motorC->setSpeed(255);
    motorD->setSpeed(255);
  }
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  encoderCountA = 0; encoderCountB = 0;

}
void turn(double angle){
  // create PID instance.
  PID myPID(10,0,0.3); double MOTORSPEED = 0;
  // create timer to cut of turning
  timer myTimer;
  double correction = 0;
  sensors_event_t event;
  bno.getEvent(&event);
  double current_angle=(float)event.orientation.x;
  if(angle > 0){ // right turn
    motorB->run(BACKWARD);
    motorD->run(BACKWARD);
    while(true){
      if(current_angle>=angle&& current_angle<190) break; // anti wraparound
      if(myTimer.getTime() > 2*1000000) break;
      bno.getEvent(&event);
      current_angle = (float)event.orientation.x; // get the angle
      MOTORSPEED = myPID.getPID(angle-current_angle);
      motorA->setSpeed(constrain(MOTORSPEED,0,255));
      motorB->setSpeed(constrain(MOTORSPEED,0,255));
      motorC->setSpeed(constrain(MOTORSPEED,0,255));
      motorD->setSpeed(constrain(MOTORSPEED,0,255));
    }
  }
  else if(angle<0){
    motorA->run(BACKWARD);
    motorC->run(BACKWARD);
    while(true){
      if(current_angle<360+angle&&current_angle>190) break; // anti wraparound
      if(myTimer.getTime() > 2*1000000) break;
      bno.getEvent(&event);
      current_angle = (float)event.orientation.x; // get the angle
      MOTORSPEED = myPID.getPID(current_angle-(360+angle));
      motorA->setSpeed(constrain(MOTORSPEED,0,255));
      motorB->setSpeed(constrain(MOTORSPEED,0,255));
      motorC->setSpeed(constrain(MOTORSPEED,0,255));
      motorD->setSpeed(constrain(MOTORSPEED,0,255));
    }
  }
  Serial.println("finished turning");
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  encoderCountA = 0; encoderCountB = 0; // reset encoder counters.
}
// full stop function
