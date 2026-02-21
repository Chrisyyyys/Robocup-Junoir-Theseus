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
  if(Serial2.available()>=5){
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
    Serial.println(res);
  }
  else{
    //nothing right now.
    while(Serial2.available()<5){
      continue;
    }
    detectCam2();
  }

}
void detectCam2(){
  // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int maxCount = 0;
  char res = 0;
  char output;
  if(Serial3.available()>=5){
    Serial.println("letters incoming");
    
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
    Serial.println(res);
  }
  else{
    //nothing right now.
    while(Serial3.available()<5){
      continue;
    }
    detectCam2();
  }

}
void detect(){ // the robot goes forward until it detects something( does not return)
  bool victimAtLeft = false;
  bool victimAtRight = false;
  clearSerialBuffer1();
  clearSerialBuffer2();
  timer myTimer;
  while(true){
    if(readSerial1() != -1){
      victimAtLeft = true;
      victimtoggle = true;
      break;
    }
    if(readSerial2() != -1){
      victimAtRight = true;
      victimtoggle = true;
      break;
    }
    if(myTimer.getTime() >= 1000000*1.5) break; // give 1.5 seconds to detect.
    motorA->setSpeed(150);
    motorB->setSpeed(150);
    motorC->setSpeed(150);
    motorD->setSpeed(150);
  }
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  delay(500);
  if(victimAtLeft == true){
    detectCam1();
  }
  else if(victimAtRight == true){
    detectCam2();
  }
  // backpedal
  motorA->run(BACKWARD);
  motorB->run(BACKWARD);
  motorC->run(BACKWARD);
  motorD->run(BACKWARD);
  while(encoderCountA > 0 && encoderCountB > 0){
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