
void init_color(){
  //tcs.setInterrupt(true);
  
 
  //tcs.enable();
  if (tcs.begin()) {
    Serial.println("Found tcs34725 sensor");
    
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
int read_color(){
 
  
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  
  tcs.setInterrupt(true);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  
  tcs.setInterrupt(true);  // turn off LED
  //Serial.println(c);
 
  if(c<BLACK_THRESHOLD){
    Serial.println(c);
    return -1; // black
  }
  if(blue>green&&blue>red){
    return 1; // blue
  }
  else{
    return 0; // good
  }
}
  