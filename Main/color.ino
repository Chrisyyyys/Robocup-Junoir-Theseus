
void init_color(){
  //tcs.setInterrupt(true);
  myMux.setPort(TCS_PORT);
  //digitalWrite(POWERPIN,HIGH);
  //tcs.enable();
  if (tcs.begin()) {
    Serial.println("Found tcs34725 sensor");
    
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    
    
    
    //while (1); // halt!
  }
}
int read_color(){
  myMux.setPort(TCS_PORT);          
  
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  
  tcs.setInterrupt(true);  // turn on LED

  delay(50);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  
  
  Serial.println(c);
 
  if(c<BLACK_THRESHOLD){
    Serial.println(c);
    return -1; // black
  }
  if(blue>green&&blue>red){
    return 1; // blue
  }
  if(red>green&&red>blue){
    return 2;
  }
  else{
    return 0; // good
  }
}
  