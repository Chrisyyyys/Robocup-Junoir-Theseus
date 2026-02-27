// camera communication code
// clear serial buffers.
void clearSerialBuffer1() {
  while (Serial2.available() > 0) {
    Serial2.read();  // Read and discard one byte from the buffer
  }
}
void clearSerialBuffer2() {
  while (Serial3.available() > 0) {
    Serial3.read();  // Read and discard one byte from the buffer
  }
}
// led
void flashLED(char victimState){
  if(victimState == 'H'){
    digitalWrite(pinHarmed,HIGH);
    delay(5000);
  }
  else if(victimState == 'S'){
    digitalWrite(pinStable,HIGH);
    delay(5000);
  }
  else{
    digitalWrite(pinUnharmed,HIGH);
    delay(5000);
  }
  digitalWrite(pinHarmed,LOW);
  digitalWrite(pinStable,LOW);
  digitalWrite(pinUnharmed,LOW);
}
int readSerial1(){ // 
  char sample;
  int output;
  
  if(Serial2.available()>0){
    sample = Serial2.read();
    
    if(sample == 'H'){
      output = 0;
    }
    else if(sample == 'S'){
      output = 1;
    }
    else if(sample == 'U'){
      output = 2;
    }
    else if(sample == 'R'){
      output = 3;
    }
    else if(sample == 'Y'){
      output = 4;
    }
    else{
      output = 5; // green
    }
    return output; // output inside or it keeps outputting
  }
  else{
    // nothing right now
    return -1;
  }
  
}
int readSerial2(){
  char sample;
  int output;
  
  if(Serial3.available()>0){
    sample = Serial3.read();
    
    if(sample == 'H'){
      output = 0;
    }
    else if(sample == 'S'){
      output = 1;
    }
    else if(sample == 'U'){
      output = 2;
    }
    else if(sample == 'R'){
      output = 3;
    }
    else if(sample == 'Y'){
      output = 4;
    }
    else{
      output = 5; // green
    }
    return output; // output inside or it keeps outputting
    
  }
  else{
    // nothing right now
    return -1;
  }
}
void detectCam1(){ // doesn't return anything.
  // read buffer
  // if there is content, take 5 samples and take the most common letter( the model sometime misidentifies)
  int samples[5];
  int maxCount = 0;
  char res = 0;
  char output;
  timer myTimer;
  while(Serial2.available()<5){
    Serial.println(myTimer.getTime());
    if(myTimer.getTime() > 5*1000000){
      Serial.println("out of time");
      return;
    }
  }
  Serial.println("letters incoming");
  
  for(int i =0; i<5;i++){
    int value = readSerial1();
    samples[i] = value;
    Serial.println(value);
    
  }
  
  for(int i=0;i<6;i++){
    int count = 0;
    for(int j = 0;j<5;j++){
      if(samples[j] == i) count++;
    }
    if(count > maxCount){
      maxCount = count;
      res = classes[i];
    }
  }
  flashLED(res);
  
  return;
}
void detectCam2(){
  // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int maxCount = 0;
  char res = 0;
  char output;
  timer myTimer;
  while(Serial3.available()<5){
    if(myTimer.getTime() > 5*1000000) return;
    Serial.println(myTimer.getTime());
  }
  for(int i =0; i<5;i++){
    int value = readSerial2();
    samples[i] = value;
    Serial.println(value);
    
  }
  
  for(int i=0;i<6;i++){
    int count = 0;
    for(int j = 0;j<5;j++){
      if(samples[j] == i) count++;
    }
    if(count > maxCount){
      maxCount = count;
      res = classes[i];
    }
  }
  flashLED(res);
  return;
}
void detect(){ // the robot goes forward until it detects something( does not return)
  fullstop();
  bool victimAtLeft = false; // victim at left
  bool victimAtRight = false; // victim at right
  // clear buffers
  clearSerialBuffer1();
  clearSerialBuffer2();
  // don't detect if there is no wall( prevent misdetection)
  bool wallAtLeft = false;
  bool wallAtRight = false;
  if(measure(3) <MIN_DIST&&measure(3)!=-1) wallAtRight = true;
  if(measure(6)<MIN_DIST&&measure(6)!=-1) wallAtLeft = true;
  timer myTimer;
  while(true){
    if(readSerial1() != -1&&wallAtLeft == true){
      Serial.println("left");
      victimAtLeft = true;
      victimtoggle = true;
      break;
    }
    if(readSerial2() != -1&&wallAtRight == true){
      Serial.println("right");
      victimAtRight = true;
      victimtoggle = true;
      break;
    }
    if(myTimer.getTime() >= 1000000*1) break; // give 1.5 seconds to detect.
    motorA->setSpeed(100);
    motorB->setSpeed(100);
    motorC->setSpeed(100);
    motorD->setSpeed(100);
  }
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  delay(200);
  if(victimAtLeft == true){
    clearSerialBuffer1();
    detectCam1();
  }
  else if(victimAtRight == true){
    clearSerialBuffer2();
    detectCam2();
  }
  
  // backpedal
  motorA->run(BACKWARD);
  motorB->run(BACKWARD);
  motorC->run(BACKWARD);
  motorD->run(BACKWARD);
  while(encoderCountA > 0){
    Serial.println("backtracking");
    motorA->setSpeed(150);
    motorB->setSpeed(150);
    motorC->setSpeed(150);
    motorD->setSpeed(150);
  }
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  encoderCountA = 0; encoderCountB = 0;

}