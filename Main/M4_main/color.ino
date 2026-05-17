
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
  myMux.setPort(TCS_PORT);          
  tcs.setInterrupt(true);  // turn on LED
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  clear = c;
  Serial.println("clear value");
  Serial.println(clear);
  
}
Color read_color(){
  myMux.setPort(TCS_PORT);          
  tcs.setInterrupt(true);  // turn on LED
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
 
  if((float)c/clear<BLACK_THRESHOLD){
    
    
    return BLACK; // black
  }
  if((float)c/clear>SILVER_THRESHOLD){
    
    return SILVER; // SILVER — prevent fall-through into blue/red checks
  }
  if(b>g&&b>r){
    return BLUE; // blue
  }
  if(r>g&&r>b){
    return RED;
  }
  else{
    return SILVER; // good
  }
}
  