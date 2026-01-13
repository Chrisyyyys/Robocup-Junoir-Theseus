void init_color(){
  //tcs.setInterrupt(true);
  
 
  //tcs.enable();
  if (tcs.begin()) {
    Serial.println("Found sensor");
    
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    /*
    digitalWrite(POWERPIN,LOW);
    delay(100);
    digitalWrite(POWERPIN,HIGH);
    init_color();
    */
    //while (1); // halt!
  }
}