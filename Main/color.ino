
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
int read_color(){
  // [DIAG-COLOR] snapshot the global `clear` divisor the instant this call starts,
  // before touching I2C at all. clear is written exactly once (init_color(), pre-thread)
  // and never reassigned anywhere else, so if this ever prints 0 here, its backing memory
  // has been clobbered by something other than this function -- not a bad sensor read.
  Serial.print("[DIAG-COLOR] t=");
  Serial.print(millis());
  Serial.print(" clear_global=");
  Serial.print(clear, 6);
  Serial.print(" &clear=");
  Serial.println((uint32_t)&clear, HEX);

  i2cMutex.lock();
  myMux.setPort(TCS_PORT);
  tcs.setInterrupt(true);  // turn on LED
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  i2cMutex.unlock();

  Serial.print(r);
  Serial.print(" ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  Serial.print("c=");
  Serial.print(c);
  Serial.print(" ratio=");
  Serial.print((float)c/clear);
  Serial.println(" ");

  //Serial.println((float)c/clear);

  if((float)c/clear<BLACK_THRESHOLD){


    return -1; // black
  }
  /*
  if(c>SILVER_THRESHOLD){ // silver reflects more absolute light
    int nx = x_pos; int ny = y_pos;
    stepForward(currentDir,nx,ny);
    //Serial.print("silver at ");
    //Serial.print(nx);
    //Serial.print(", y=");
    //Serial.print(ny);
    mapGrid[nx][ny].setType(CHECKPOINT);
    x_checkpoint = nx; y_checkpoint = ny;
    floor_checkpoint = currentFloor; // remember which floor this checkpoint is on

    return 3; 
  }
  */
  if((float)c/clear>WHITE_THRESHOLD) return 0;


  if(b>g+10&&b>r+10) return 1; //blue

  if(r>g+10&&r>b+10) return 2;

  // [DIAG-COLOR] previously this fell off the end with no return (undefined behavior).
  // Flagged fallthrough so we can see exactly how often/when none of the branches above match,
  // instead of silently returning garbage. -99 is not a real color code anywhere else in the codebase.
  Serial.print("[DIAG-COLOR] FALLTHROUGH t=");
  Serial.print(millis());
  Serial.print(" r=");
  Serial.print(r);
  Serial.print(" g=");
  Serial.print(g);
  Serial.print(" b=");
  Serial.print(b);
  Serial.print(" c=");
  Serial.print(c);
  Serial.print(" clear_global=");
  Serial.println(clear, 6);
  return -99;
}
