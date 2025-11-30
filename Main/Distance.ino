// distance sensor code
// blue for SDA, yellow for SCL

void init_dist() {
  // put your setup code here, to run once:
  
  if(!myMux.begin()){
    Serial.println("can't find the Mux");
  }
  else{
    Serial.println("Mux initialized");
  }
  for(int i = 0; i<7; i++){
    myMux.setPort(i);
    delay(10);
    if(!sensors[i].init()){
      Serial.println("Sensor "+String(i)+" failed to initialize");
    }
    else{
      Serial.println("Sensor "+String(i)+" is able to initialize");
    }
  }
  

}


