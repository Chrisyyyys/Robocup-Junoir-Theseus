void encoder_update_B(){
  int encoderState = digitalRead(encoderPin_B_B);
 
  if(encoderState == LOW){
    encoderCountB++;
  }
  else{
    encoderCountB--;
  }
}
void init_drive(){
  pinMode(encoderPin_B_A, INPUT);
  pinMode(encoderPin_B_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin_B_A), encoder_update_B, RISING); // only counting rising of Pin: (20/2)/2
  if(!AFMS.begin()){
    Serial.println("could not find motor shield");
  }
  else{
    Serial.println("motor shield found");
  }
  motorB->run(FORWARD);
  // turn on motor
 // motorB->run(RELEASE);
}