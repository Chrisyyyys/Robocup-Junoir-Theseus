
void init_drive(){
  drivetrain.init_drive();
  // initialize gyro
  myGyro.init_Gyro();
}

void fwd(double dist){ // in mm
  double pulses = dist/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  bool black = false; // toggle for black tile
  bool climbtoggle = false; // toggle for climbing
  bool climbed = false; // if climbing occured.
  bool upwards = false; // up/ down for elevation
  int cnt = 0; // tiles traversed while climbing.
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  PID climbPID(2,0,0.1); // pid for centering on ramp
  PID center_PID(1,0,0.2);
  PID gyroPID(1,0.001,0.03);
  PID Scale_PID(0.0045,0,0.0008); // pid for encoder 
  Serial.println("forwarding");
  // allow the camera RTOS thread to flag victims for this move
  fwdActive = true;
  isVictim = false;
  victimPending = false;
  moveInterrupted = false; // becomes true only if a pause aborts this move
  int init_pitch = myGyro.modulus((int)myGyro.pitch_heading());
  int init_yaw = turnNeededDeg(myGyro.headingToCardinal(myGyro.heading()));
  Serial.println("init_yaw");
  Serial.println(init_yaw);
  // [DIAG] round-1 sideswipe instrumentation: show whether init_yaw matches actual heading
  double _entry_hdg = myGyro.heading();
  Serial.print("[FWD] entry hdg=");
  Serial.print(_entry_hdg, 1);
  Serial.print(" init_yaw=");
  Serial.print(init_yaw);
  Serial.print(" offset=");
  Serial.println(_entry_hdg - init_yaw, 1);
  const char* fwdExit = "normal";
  int _fwd_tick = 0;
  int front_left_current=measure(7); int front_right_current=measure(1);
  int front_left_last=measure(7); int front_right_last=measure(1);
  timer myTime;
  myTime.reset_delta_time();
  
  int front_left = measure(7);int front_right = measure(1);
  // outside loop
    if(front_left<=OBSTACLE_DIST&&front_left!=-1&&front_right>=MIN_DIST&&front_right!=-1){ // trigger obstacleavoidance
      Serial.println("obstacle left");
      int prevdist = obstacleavoidance(1);
      drivetrain.fullstop();
      delay(50);
      if(prevdist != -2) obstacle = true;
      else moveInterrupted = true; // avoidance aborted by pause -> tile not completed
      /*
      if(prevdist - (measure(1)+measure(7))/2 > TILE_MM){
        int pulses = pulsesForDistanceMm(prevdist - (measure(1)+measure(7))/2-TILE_MM); // don't "overmove"
        while(drivetrain.encoderCountA >= -pulses && drivetrain.encoderCountB >= -pulses && drivetrain.encoderCountD >= -pulses){ // too far in front, go back
          drivetrain.backward(150);
        }
      }
      else if(prevdist - (measure(1)+measure(7))/2 < TILE_MM){
        int pulses = pulsesForDistanceMm(TILE_MM-(prevdist - (measure(1)+measure(7))/2));
        while(drivetrain.encoderCountA <= pulses && drivetrain.encoderCountB <= pulses && drivetrain.encoderCountD <= pulses){ // too far in front, go back
          drivetrain.fw(150);
        }
      }
      drivetrain.fullstop();
      */
      Serial.println("[FWD] exit=obstacle-left");
      return;
    }
    else if(front_right<=OBSTACLE_DIST&&front_right!=-1&&front_left>=OBSTACLE_DIST&&front_left!=-1){
      Serial.println("obstacle right");
      int prevdist = obstacleavoidance(0);
      drivetrain.fullstop();
      delay(50);
      /*
      if(prevdist - (measure(1)+measure(7))/2 > TILE_MM){
        int pulses = pulsesForDistanceMm(prevdist - (measure(1)+measure(7))/2-TILE_MM);
        while(drivetrain.encoderCountA >= -pulses && drivetrain.encoderCountB >= -pulses && drivetrain.encoderCountD >= -pulses){ // too far in front, go back
          drivetrain.backward(150);
        }
      }
      else if(prevdist - (measure(1)+measure(7))/2 < TILE_MM){
        int pulses = pulsesForDistanceMm(TILE_MM-(prevdist - (measure(1)+measure(7))/2));
        while(drivetrain.encoderCountA <= pulses && drivetrain.encoderCountB <= pulses && drivetrain.encoderCountD <= pulses){ // too far in front, go back
          drivetrain.fw(150);
        }
      }
      
      drivetrain.fullstop();
      */
      if(prevdist != -2) obstacle = true;
      else moveInterrupted = true; // avoidance aborted by pause -> tile not completed
      Serial.println("[FWD] exit=obstacle-right");
      return;
    }
    
  while((climbtoggle==true||(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3<=pulses)&&black!=true){
    Serial.print("distance travelled: ");
    Serial.println((((double)(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3)/5)/195*wheel_diameter*M_PI);
    //Serial.println((drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    if(Pausemaze==true) {drivetrain.fullstop(); moveInterrupted = true; break;}
    // Service a camera victim flagged by the RTOS thread: stop, pause PID +
    
    if(victimPending){
      drivetrain.fullstop(); // does not overide the thread
      climbPID.pausePID(1);
      gyroPID.pausePID(1);
      Scale_PID.pausePID(1);
      myTime.pause(1);
      while(victimPending == true){
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
      }
      gyroPID.pausePID(2);
      climbPID.pausePID(2);
      Scale_PID.pausePID(2);
      myTime.pause(2);
    }
    
    // color: detect black (stop + back off) tiles ahead. Blue is read only
    // after the move completes (in EXECUTE_MOVE), not mid-motion here.
    int color = read_color(); // also marks silver checkpoints internally
    Serial.println("color");
    Serial.println(color);
    if(color == -1){ // black tile ahead -> stop, mark next tile, back off
      drivetrain.fullstop();
      delay(100);
      Serial.println("black");
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(BLACK);
      blacktoggle = true;
      while(drivetrain.encoderCountA >= 0 && drivetrain.encoderCountB >= 0 && drivetrain.encoderCountD >= 0){
        drivetrain.backward(200);
      }
      black = true;
    }
    // PID centering

    double adjustment;

    // only use distance sensor to center when there are walls on both sides.

      // error sign must match the gyro branch: positive adjustment steers the
      // robot the same way for both. wall_right-wall_left is >0 when the robot
      // is closer to the left wall, which correctly steers it back toward center.
    // [DIAG] capture the error fed to PID so it can be logged below
    double _diag_pid_err = center();
    adjustment = center_PID.getPID(_diag_pid_err);
    /*
    double yaw = myGyro.heading()-init_yaw;
    if(yaw>180) yaw = yaw-360;
    if(yaw<-180) yaw+= 360;
    double _diag_pid_err = yaw;
    adjustment = gyroPID.getPID(_diag_pid_err);
    */
 
    /*
=======
    if(wall_left<MIN_DIST && wall_left!=-1 && wall_right<MIN_DIST && wall_right!=-1){
      // Same sign as gyro correction.
      // wall_right - wall_left is > 0 when the robot is closer to the left wall.
      // -> steers back to center.
      adjustment = center_PID.getPID(center());
    }
>>>>>>> Stashed changes
    else{
      double yaw = myGyro.heading()-init_yaw;
      if(yaw>180) yaw = yaw-360;
      if(yaw<-180) yaw+= 360;
      adjustment = gyroPID.getPID(yaw);
    }
    */
    double Scale = Scale_PID.getPID(pulses-(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    
    // emergency stop
    
    front_left_current = measure(7);
    front_right_current = measure(1);
    
    if((front_left_current<=50&&front_left_current!=-1)&&(front_right_current<=50&&front_right_current!=-1)){
      Serial.println("stopping");
      // if the robot doesn't make it halfway across the tile, fwd failed.
      Serial.print("[FWD] emergency-stop fl=");
      Serial.print(front_left_current);
      Serial.print(" fr=");
      Serial.println(front_right_current);
      fwdExit = "emergency-front";
      drivetrain.fullstop();
      delay(50);
      break;
    }
    
    // check pitch: if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
    if(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > 20){
      Serial.println("climbing");
      int _encoderCountA = drivetrain.encoderCountA; // save values before ramp
      int _encoderCountB = drivetrain.encoderCountB;
      int _encoderCountD = drivetrain.encoderCountD;
      climbtoggle = true; // prevent outer loop from exiting on encoder count
      climbed = true;
      Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
      if(myGyro.modulus(myGyro.pitch_heading())-init_pitch>20) upwards = true; // distinguish between moving up and moving down.
      //drivetrain.reset_encoderCount(true,true,true);
      while(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > 20){
        // PID centering
        double yaw = myGyro.heading()-init_yaw;
        if(yaw>180) yaw = yaw-360;
        if(yaw<-180) yaw+= 360;
    
        double adjustment = climbPID.getPID(yaw);
        
        Serial.println("climbing");
        //Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
        Serial.println(adjustment);
        // center during climbing
        if(upwards == true) drivetrain.drive(180-adjustment,180-adjustment,180+adjustment,180+adjustment);
        if(upwards == false) drivetrain.drive(120-adjustment,120-adjustment,120+adjustment,120+adjustment);
        //Serial.println((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3);
        if((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3 >= pulses/cos(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch)*(M_PI/180))){
          Serial.println("1 section of the ramp climbed");
          cnt++;
          drivetrain.reset_encoderCount(true,true,true);
        } // track tiles
      }
      // ramp crested: restore pre-ramp encoder values so outer loop finishes the tile
      drivetrain.set_encoderCountA(_encoderCountA);
      drivetrain.set_encoderCountB(_encoderCountB);
      drivetrain.set_encoderCountD(_encoderCountD);
      climbtoggle = false; // re-enable encoder-based exit in outer loop
    }
    
    
    // [DIAG] throttled per-loop trace (every 5 ticks) — CSV so it can be graphed.
    // wl/wr are re-read here only in the trace block, so the control path is
    // untouched. Fields: t_ms, wl, wr, err, adj, encAvg, fl, fr
    /*
    _fwd_tick++;
    if((_fwd_tick % 5) == 0){
      int _diag_wl = measure(2);
      int _diag_wr = measure(6);
      int _enc_avg = (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
      Serial.print("[FWD] t=");
      Serial.print(millis());
      Serial.print(" wl=");
      Serial.print(_diag_wl);
      Serial.print(" wr=");
      Serial.print(_diag_wr);
      Serial.print(" err=");
      Serial.print(_diag_pid_err, 1);
      Serial.print(" adj=");
      Serial.print(adjustment, 1);
      Serial.print(" enc=");
      Serial.print(_enc_avg);
      Serial.print(" fl=");
      Serial.print(front_left_current);
      Serial.print(" fr=");
      Serial.println(front_right_current);
    }
    */
    if(Scale*120 < 25) break;
    drivetrain.drive(constrain(Scale*(120-adjustment),20,150),constrain(Scale*(120-adjustment),20,150),constrain(Scale*(120+adjustment),20,150),constrain(Scale*(120+adjustment),20,150));
    //drivetrain.drive(150+adjustment,(150+adjustment)*1.25,(150-adjustment)*1.25,150+adjustment);
  }
  Serial.print("[FWD] exit=");
  Serial.println(fwdExit);
  Serial.println("stop- end of fwd");
  // sometimes it barely makes it over the slope
  if(climbed == true){
    for(int i = 0; i<cnt;i++){
      Serial.println("adding ramp to map");
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
      // moving between floors
      // only elevate the first tile of a ramp.
      // transition to another map
      if(i==0){
        if(upwards==true) elevation(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor); // elevate
        else descend(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
      }
    }
    Serial.println("compensating");
    drivetrain.fw(200);
    delay(300);
    drivetrain.fullstop();
    delay(200);
    absoluteturn(turnNeededDeg(currentDir)); // snap direction.
  }
  
  fwdActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true);
  victimtoggle = false;
}
// absolute turning

void absoluteturn(double angle){
  // create PID instance.
  PID myPID(4.5,0,0.3);
  double MOTORSPEED = 0;
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  // allow the camera RTOS thread to flag victims during the turn
  turnActive = true;
  isVictim = false;
  victimPending = false;
  // Shortest signed-path error, wrapped into [-180, 180]:
  //   sign of diff  = direction to turn (+CW/turnright, -CCW/turnleft)
  //   |diff|        = shortest angular distance to target
  // Replaces the old fasterway + inverse() pair, which had a discontinuity at
  // 0/360 that caused left-turns through NORTH to go the 270-degree long way.
  double diff = angle - myGyro.heading();
  while(diff > 180.0)  diff -= 360.0;
  while(diff < -180.0) diff += 360.0;
  bool turn_right = (diff > 0);
  double init_abs = fabs(diff);
  const double TURN_TOL_DEG = 3.0;
  Serial.print("[TURN] target=");
  Serial.print(angle);
  Serial.print(" hdg=");
  Serial.print(myGyro.heading(), 1);
  Serial.print(" init_diff=");
  Serial.println(init_abs, 1);
   // create timer to cut of turning
  timer myTimer;

  if(turn_right){
    while(true){
      if(Pausemaze==true) {drivetrain.fullstop(); break;}
      if(victimPending){ // service camera victim mid-turn
        drivetrain.fullstop();
        myPID.pausePID(1); myTimer.pause(1);
        while(victimPending==true){
          rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
        }
        myPID.pausePID(2); myTimer.pause(2);
      }
      // Recompute the wrapped error every tick.
      double d = angle - myGyro.heading();
      while(d > 180.0)  d -= 360.0;
      while(d < -180.0) d += 360.0;
      
      if(myTimer.getTime() > 2.0 * init_abs / 90.0 * 1000000.0) break; // turning limit

      MOTORSPEED = myPID.getPID(fabs(d));

      drivetrain.turnright(constrain(MOTORSPEED,20,150));
    }
  }

  else if(!turn_right) {
    while(true){
      if(Pausemaze==true) {drivetrain.fullstop(); break;}
      if(victimPending){ // service camera victim mid-turn
        drivetrain.fullstop();
        myPID.pausePID(1); myTimer.pause(1);
        while(victimPending==true){
          rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
        }
        myPID.pausePID(2); myTimer.pause(2);
      }
      double d = angle - myGyro.heading();
      while(d > 180.0)  d -= 360.0;
      while(d < -180.0) d += 360.0;
      
      if(myTimer.getTime() > 2.0 * init_abs / 90.0 * 1000000.0) break;

      MOTORSPEED = myPID.getPID(fabs(d));

      drivetrain.turnleft(constrain(MOTORSPEED,20,150));
    }
  }
  victimtoggle = false;
  Serial.println("finished turning");
  turnActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true); // reset encoder counters.
}

// Corrects left-right position within the tile by turning the robot a small amount before the next forward drive
// -> so fwd()'s heading-hold behavior moves the robot diagonally back towards the center.
// (it locks onto whatever heading it starts at) 
// Must run AFTER turnCompletedSuccessfully() has validated the cardinal turn, so this intentional small heading offset isn't mistaken for a botched turn.

