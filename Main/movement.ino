
void init_drive(){
  drivetrain.init_drive();
  // initialize gyro
  myGyro.init_Gyro();
}

// Drives one move forward. Returns what actually happened: only MOVE_OK means the robot is
// now in the next tile, so the caller must not advance the map position on anything else.
MoveResult fwd(double dist){ // in mm
  double pulses = (dist + FWD_TRIM_MM)/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  bool black = false; // toggle for black tile
  bool climbtoggle = false; // toggle for climbing
  bool climbed = false; // if climbing occured.
  bool upwards = false; // up/ down for elevation
  int cnt = 0; // tiles traversed while climbing.
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  PID climbPID(2,0,0.1); // pid for centering on ramp
  PID center_PID(2,0,0.5);
  PID gyroPID(1,0.001,0.03);
  PID Scale_PID(0.0045,0,0.0008); // pid for encoder 
  MoveResult result = MOVE_OK;
  fwdSoftFail = false;
  if(VERBOSE_DEBUG) Serial.println("forwarding");
  drivetrain.reset_encoderCount(true,true,true); // count this move from zero (the back-offs reverse to 0)
  // allow the camera RTOS thread to flag victims for this move
  fwdActive = true;
  isVictim = false;
  victimPending = false;
  int init_pitch = myGyro.modulus((int)myGyro.pitch_heading());
  int init_yaw = turnNeededDeg((Direction)myGyro.headingToCardinal(myGyro.heading()));
  if(VERBOSE_DEBUG){
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
  }
  const char* fwdExit = "normal";
  int _fwd_tick = 0;
  int front_left_current=measure(7); int front_right_current=measure(1);
  int front_left_last=measure(7); int front_right_last=measure(1);
  timer myTime;
  myTime.reset_delta_time();
  
  // Obstacles are looked for inside the drive loop below. This used to be a single check here, before the move: at
  // OBSTACLE_DIST (90 mm), when an obstacle in the next tile is still ~175 mm away, so it never fired in time.
  int obstacleCount = 0;                           // consecutive loop passes with something close in front
  double mmPerPulse = 1.0 / pulsesForDistanceMm(1.0);

  // Driving time of this move, for the stuck check below. A pass of the loop takes about 30 ms; a much longer pass means the
  // robot was standing still (the camera thread servicing a victim holds the I2C bus, so measure() blocks for seconds), and
  // that time is not counted.
  unsigned long moveStartMs = millis();
  unsigned long lastPassMs = moveStartMs;
  unsigned long stalledMs = 0;

  while((climbtoggle==true||(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3<=pulses)&&black!=true){
    if(VERBOSE_DEBUG){
      Serial.print("distance travelled: ");
      Serial.println((((double)(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3)/5)/195*wheel_diameter*M_PI);
    }
    //Serial.println((drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    unsigned long nowMs = millis();
    if(nowMs - lastPassMs > 1000) stalledMs += nowMs - lastPassMs;
    lastPassMs = nowMs;
    if(Pausemaze==true) {drivetrain.fullstop(); result = MOVE_PAUSED; break;}
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
    if(VERBOSE_DEBUG){
      Serial.println("color");
      Serial.println(color);
    }
    if(color == -1){ // black tile ahead -> stop, mark next tile, back off
      drivetrain.fullstop();
      delay(100);
      Serial.println("black");
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(BLACK);
      backOffToMoveStart();
      result = MOVE_BLACK;
      black = true;
      break; // skip the rest of this pass, which would drive forward again
    }
    // PID centering

    double adjustment;

    // only use distance sensor to center when there are walls on both sides.

      // error sign must match the gyro branch: positive adjustment steers the
      // robot the same way for both. wall_right-wall_left is >0 when the robot
      // is closer to the left wall, which correctly steers it back toward center.
    // [DIAG] capture the error fed to PID so it can be logged below
    double _diag_pid_err = center();
    // Limit how hard the wall follower may steer: unlimited, 30 mm off-centre already gave
    // one side PWM 150 and the other 20 (a ~30 deg/s swerve). Tune on the robot.
    if(centerHasWall){
      adjustment = constrain(center_PID.getPID(_diag_pid_err), -40, 40);
    }
    else{
      // No right wall to follow, so nothing steers the robot: motor mismatch and wheel slip become heading
      // drift (3-5 deg per move in the simulator) and then sideways drift, until a wall stops it. Hold the
      // heading the move started on (the nearest axis) with the gyro instead, with the same steering limit.
      double yawErr = myGyro.heading() - init_yaw;
      if(yawErr > 180) yawErr -= 360;
      if(yawErr < -180) yawErr += 360;
      _diag_pid_err = yawErr; // shows in the [CENTER] trace
      adjustment = constrain(gyroPID.getPID(yawErr), -40, 40);
    }
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
      Serial.print("[FWD] emergency-stop fl=");
      Serial.print(front_left_current);
      Serial.print(" fr=");
      Serial.println(front_right_current);
      fwdExit = "emergency-front";
      drivetrain.fullstop();
      delay(50);
      // If the robot doesn't make it halfway across the tile, fwd failed: it is still in the
      // tile it started from (usually a wall or obstacle the wall check missed), so back off
      // to where the move started. Past halfway it is in the next tile and the far wall is
      // just close, so the move counts. A move that climbed a ramp has left its tile either
      // way (and the encoders were rewound to their pre-ramp values), so it always counts.
      if(!climbed && (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3 < pulses/2){
        fwdExit = "blocked";
        backOffToMoveStart();
        result = MOVE_BLOCKED;
      }
      break;
    }
    
    // Obstacle look-ahead. Something close in front that is not the far wall of the tile we are driving into is an
    // obstacle: stop before touching it, back off to where the move started and report BLOCKED, so the edge is marked
    // blocked and the planner picks another way. (The emergency stop above needs BOTH sensors under 50 mm, but an obstacle
    // against a wall is seen by one sensor only, so it used to be driven into and pushed until the encoders ran out.)
    // Only while flat: a ramp seen from below also reads close.
    if(!climbtoggle && abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) < 6){
      double remainingMm = dist - (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3.0 * mmPerPulse;
      if(remainingMm < 0) remainingMm = 0;
      double farWallMm = remainingMm + FRONT_GAP_AT_CENTER_MM - 45; // the destination tile's own front wall reads about this much
      bool closeAhead = (front_left_current != -1 && front_left_current < OBSTACLE_STOP_MM && front_left_current < farWallMm)
                     || (front_right_current != -1 && front_right_current < OBSTACLE_STOP_MM && front_right_current < farWallMm);
      obstacleCount = closeAhead ? obstacleCount + 1 : 0;
      if(obstacleCount >= 2){
        drivetrain.fullstop();
        delay(100);
        if(obstacleConfirmed()){
          fwdExit = "obstacle";
          backOffToMoveStart();
          result = MOVE_BLOCKED;
          break;
        }
        obstacleCount = 0; // a false alarm: carry on
      }
    }

    // A move whose wheels spin against a wall or an obstacle ends "OK" when the encoders say the tile is done, wherever the robot
    // really is. Such a move takes much longer than a clean one, so stop it when it does. Not on a ramp (it is slower there).
    if(!climbtoggle && !climbed && !stuckCheckOff){
      unsigned long passMs = millis() - lastPassMs; // this pass so far: counts as standing still when it was blocked
      unsigned long drivingMs = millis() - moveStartMs - stalledMs - (passMs > 1000 ? passMs : 0);
      if(drivingMs > MOVE_TIMEOUT_MS){
        drivetrain.fullstop();
        Serial.print("[MOVE] stuck after ");
        Serial.print(drivingMs);
        Serial.println(" ms");
        fwdExit = "stuck";
        fwdSoftFail = true; // the pose may be at fault: the edge gets one more try before it is blocked
        result = MOVE_BLOCKED;
        if(++stuckStreak >= MOVE_STUCK_LIMIT){
          stuckCheckOff = true;
          Serial.println("[MOVE] stuck stops in a row: stuck check switched off, MOVE_TIMEOUT_MS is probably too short");
        }
        break;
      }
    }

    // check pitch: if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
    if(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > 20){
      Serial.println("climbing");
      int _encoderCountA = drivetrain.encoderCountA; // save values before ramp
      int _encoderCountB = drivetrain.encoderCountB;
      int _encoderCountD = drivetrain.encoderCountD;
      climbtoggle = true; // prevent outer loop from exiting on encoder count
      climbed = true;
      if(VERBOSE_DEBUG) Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
      if(myGyro.modulus(myGyro.pitch_heading())-init_pitch>20) upwards = true; // distinguish between moving up and moving down.
      //drivetrain.reset_encoderCount(true,true,true);
      while(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > 20){
        // PID centering
        double yaw = myGyro.heading()-init_yaw;
        if(yaw>180) yaw = yaw-360;
        if(yaw<-180) yaw+= 360;
    
        double adjustment = climbPID.getPID(yaw);
        
        if(VERBOSE_DEBUG){
          Serial.println("climbing");
          //Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
          Serial.println(adjustment);
        }
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
    // [DIAG] right-wall follower trace. front=sensor2, back=sensor3.
    // Watch: are m2/m3 valid (not -1) and <= SIDE_WALL_MAX_MM? is err non-zero when off-center?
    _fwd_tick++;
    if(VERBOSE_DEBUG && (_fwd_tick % 5) == 0){
      int _diag_front = measure(2);
      int _diag_back  = measure(3);
      Serial.print("[CENTER] m2(front)=");
      Serial.print(_diag_front);
      Serial.print(" m3(back)=");
      Serial.print(_diag_back);
      Serial.print(" err=");
      Serial.print(_diag_pid_err, 1);
      Serial.print(" adj=");
      Serial.println(adjustment, 1);
    }
    if(Scale*120 < 25) break;
    drivetrain.drive(constrain(Scale*(120-adjustment),20,150),constrain(Scale*(120-adjustment),20,150),constrain(Scale*(120+adjustment),20,150),constrain(Scale*(120+adjustment),20,150));
    //drivetrain.drive(150+adjustment,(150+adjustment)*1.25,(150-adjustment)*1.25,150+adjustment);
  }
  if(VERBOSE_DEBUG){
    Serial.print("[FWD] exit=");
    Serial.println(fwdExit);
    Serial.println("stop- end of fwd");
  }
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
  if(result == MOVE_OK && strcmp(fwdExit, "normal") == 0) stuckStreak = 0;
  Serial.print("[MOVE] result=");
  Serial.print(moveResultName(result));
  Serial.print(" exit=");
  Serial.print(fwdExit);
  Serial.print(" ms=");
  Serial.println(millis() - moveStartMs - stalledMs);
  return result;
}

// ---- Room to turn on the spot ------------------------------------------------------------------
// The room around the robot before a turn, in mm: the smallest reading of each pair of sensors that is a wall of this tile (within the wall limit), 999 when
// there is none. The sensors measure all the time, so the six are read in three rounds and averaged: the readings are noisy (about 3 mm) and the room to turn is
// only a few mm.
int gapRight = 999, gapLeft = 999, gapFront = 999, gapBack = 999; // result of readTurnGaps()
void readTurnGaps(){
  const int sensors[7] = {2, 3, 6, 5, 1, 7, 4};
  long sum[7] = {0, 0, 0, 0, 0, 0, 0};
  int n[7] = {0, 0, 0, 0, 0, 0, 0};
  for(int round = 0; round < 3; round++){
    for(int i = 0; i < 7; i++){
      int v = measure(sensors[i]);
      if(v != -1 && v < 8000){ sum[i] += v; n[i]++; }
    }
  }
  int avg[7];
  for(int i = 0; i < 7; i++) avg[i] = n[i] ? (int)(sum[i] / n[i]) : -1;
  auto gap = [&](int a, int b, int maxMm){
    int g = 999;
    if(avg[a] != -1 && avg[a] <= maxMm && avg[a] < g) g = avg[a];
    if(b >= 0 && avg[b] != -1 && avg[b] <= maxMm && avg[b] < g) g = avg[b];
    return g;
  };
  gapRight = gap(0, 1, SIDE_WALL_MAX_MM);   // sensors 2, 3
  gapLeft = gap(2, 3, SIDE_WALL_MAX_MM);    // sensors 6, 5
  gapFront = gap(4, 5, FRONT_WALL_MAX_MM);  // sensors 1, 7
  gapBack = gap(6, -1, FRONT_WALL_MAX_MM);  // sensor 4
}

// One straight leg of a shift: drives forward (dir = +1) or in reverse (dir = -1) for up to `pulses`
// encoder counts and returns the counts really driven. Stops early when a front or back sensor reads NUDGE_STOP_GAP_MM.
double nudgeLeg(int dir, double pulses){
  unsigned long startMs = millis();
  drivetrain.reset_encoderCount(true,true,true);
  while(millis() - startMs < 2500){
    if(Pausemaze == true) break;
    double driven = dir * (double)(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD) / 3.0;
    if(driven >= pulses) break;
    if(dir > 0){
      int fl = measure(7), fr = measure(1);
      if((fl != -1 && fl < NUDGE_STOP_GAP_MM) || (fr != -1 && fr < NUDGE_STOP_GAP_MM)) break;
      drivetrain.fw(100);
    }
    else{
      int bk = measure(4);
      if(bk != -1 && bk < NUDGE_STOP_GAP_MM) break;
      drivetrain.backward(100);
    }
  }
  drivetrain.fullstop();
  double driven = dir * (double)(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD) / 3.0;
  return driven < 0 ? 0 : driven;
}

// Shifts the robot sideways by shiftMm (+ = right, - = left) without changing where it is along the tile: point NUDGE_ANGLE_DEG away,
// drive one leg, point the other way, drive the same distance back, square up. A leg of L mm shifts the robot by 2 * L * sin(angle),
// so the leg length is chosen from the shift asked for. When there is no room ahead (a front wall within about 140 mm) the first leg is
// the reverse one. The second leg is exactly as long as the first, so the robot ends beside where it started.
void lateralShift(double shiftMm){
  int dirSign = shiftMm > 0 ? +1 : -1;
  double legMm = constrain(fabs(shiftMm) / (2.0 * sin(NUDGE_ANGLE_DEG * 3.14159265 / 180.0)), NUDGE_MIN_LEG_MM, NUDGE_MAX_LEG_MM);
  double base = turnNeededDeg(currentDir);
  double pulses = pulsesForDistanceMm(legMm);
  int fl = measure(7), fr = measure(1);
  int frontGap = 999;
  if(fl != -1 && fl < frontGap) frontGap = fl;
  if(fr != -1 && fr < frontGap) frontGap = fr;
  int firstDir = (frontGap > 45 + 95) ? +1 : -1; // forward first when there is room ahead, otherwise reverse first
  absoluteturn(base + firstDir * dirSign * NUDGE_ANGLE_DEG);
  double first = nudgeLeg(firstDir, pulses);
  if(first > 0.25 * pulses){ // a leg shorter than this shifts too little to be worth the second leg
    absoluteturn(base - firstDir * dirSign * NUDGE_ANGLE_DEG);
    nudgeLeg(-firstDir, first);
  }
  absoluteturn(base);
  parallel(currentDir);
}

// Call before turning in place. Centres the robot when a wall is closer than the body's swing allows: across the path (the side walls) with a
// sideways shift, along it (a front or back wall) by driving. Up to three tries, each one measured again. Returns true when it moved the robot.
bool ensureTurnClearance(){
  bool acted = false;
  for(int k = 0; k < 3; k++){
    readTurnGaps();
    int right = gapRight, left = gapLeft, front = gapFront, back = gapBack;
    double lat = 0; // + = move right
    if(right < TURN_SIDE_GAP_MIN_MM && left < TURN_SIDE_GAP_MIN_MM) lat = (left - right) / 2.0;  // no room on either side: centre
    else if(right < TURN_SIDE_GAP_MIN_MM) lat = -(TURN_SIDE_GAP_MIN_MM - right + 2);
    else if(left < TURN_SIDE_GAP_MIN_MM) lat = (TURN_SIDE_GAP_MIN_MM - left + 2);
    double lon = 0; // + = forward
    if(front < TURN_END_GAP_MIN_MM && back < TURN_END_GAP_MIN_MM) lon = (front - back) / 2.0;
    else if(front < TURN_END_GAP_MIN_MM) lon = -(TURN_END_GAP_MIN_MM - front + 2);
    else if(back < TURN_END_GAP_MIN_MM) lon = (TURN_END_GAP_MIN_MM - back + 2);
    if(fabs(lat) < 2 && fabs(lon) < 2) return acted;
    acted = true;
    Serial.print("[CLEAR] gaps r=");
    Serial.print(right);
    Serial.print(" l=");
    Serial.print(left);
    Serial.print(" f=");
    Serial.print(front);
    Serial.print(" b=");
    Serial.print(back);
    Serial.print(" -> shift lat=");
    Serial.print(lat, 1);
    Serial.print(" lon=");
    Serial.println(lon, 1);
    if(fabs(lon) >= 2){
      double mm = constrain(lon, -40.0, 40.0);
      nudgeLeg(mm > 0 ? +1 : -1, pulsesForDistanceMm(fabs(mm)));
    }
    if(fabs(lat) >= 2) lateralShift(constrain(lat, -40.0, 40.0));
  }
  return acted;
}

// Reverses until the wheels are back where the current move started (fwd() zeroes the
// encoders when it starts), so an abandoned move leaves the robot in the tile the map
// says it is in.
void backOffToMoveStart(){
  unsigned long startMs = millis();
  while(drivetrain.encoderCountA >= 0 && drivetrain.encoderCountB >= 0 && drivetrain.encoderCountD >= 0){
    if(Pausemaze == true) break;
    if(millis() - startMs > 3000){ Serial.println("[MOVE] back-off timeout"); break; }
    drivetrain.backward(200);
  }
  drivetrain.fullstop();
}

const char* moveResultName(MoveResult r){
  if(r == MOVE_OK) return "OK";
  if(r == MOVE_BLOCKED) return "BLOCKED";
  if(r == MOVE_BLACK) return "BLACK";
  return "PAUSED";
}
// absolute turning
// Turns in place to an absolute maze-frame heading (0 = NORTH, clockwise positive).
// The direction is re-chosen every tick from the signed error, so an overshoot is turned
// back instead of pushed further. Returns true once the heading has stayed within
// TURN_TOL_DEG for TURN_SETTLE_MS; false if a pause or the safety timeout ended the turn.
bool absoluteturn(double angle){
  const double TURN_TOL_DEG = 3.0;
  const unsigned long TURN_SETTLE_MS = 60; // must stay inside the tolerance this long (catches coasting back out)
  const double TURN_KP = 4.5;              // PWM per degree of error (the old PID's gain)
  const int TURN_MIN_PWM = 45;             // lowest PWM that still rotates the robot on the field floor: bench-tune
  const int TURN_MAX_PWM = 150;
  // allow the camera RTOS thread to flag victims during the turn
  turnActive = true;
  isVictim = false;
  victimPending = false;
  // Shortest signed error, wrapped into [-180, 180): the sign is the direction to turn
  // (+ = clockwise/turnright, - = turnleft), the size is the angle still to go.
  double d = wrap180(angle - myGyro.heading());
  // Safety net only: a normal turn ends as soon as it settles.
  const unsigned long budgetMs = 1000 + (unsigned long)(20.0 * fabs(d));
  const double startErr = d;
  if(VERBOSE_DEBUG){
    Serial.print("[TURN] target=");
    Serial.print(angle);
    Serial.print(" start_err=");
    Serial.println(d, 1);
  }

  unsigned long startMs = millis();
  unsigned long pausedMs = 0; // time spent servicing victims, not counted against the budget
  bool inTol = false;
  unsigned long inTolSinceMs = 0;
  bool reached = false;
  while(true){
    if(Pausemaze==true) break;
    if(victimPending){ // service camera victim mid-turn
      drivetrain.fullstop();
      unsigned long pauseStartMs = millis();
      while(victimPending==true){
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
      }
      pausedMs += millis() - pauseStartMs;
      inTol = false;
    }
    d = wrap180(angle - myGyro.heading());
    unsigned long now = millis();
    if(fabs(d) <= TURN_TOL_DEG){
      drivetrain.fullstop();
      if(!inTol){ inTol = true; inTolSinceMs = now; }
      if(now - inTolSinceMs >= TURN_SETTLE_MS){ reached = true; break; }
    }
    else{
      inTol = false;
      int pwm = constrain((int)(TURN_KP * fabs(d)), TURN_MIN_PWM, TURN_MAX_PWM);
      if(d > 0) drivetrain.turnright(pwm);
      else drivetrain.turnleft(pwm);
    }
    if(now - startMs - pausedMs > budgetMs) break; // turning limit
  }
  victimtoggle = false;
  turnActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true); // reset encoder counters.
  Serial.print("[TURN] done target=");
  Serial.print(angle);
  Serial.print(" start_err=");
  Serial.print(startErr, 1);
  Serial.print(" err=");
  Serial.print(wrap180(angle - myGyro.heading()), 1);
  Serial.print(" ms=");
  Serial.print(millis() - startMs);
  Serial.print(" ok=");
  Serial.println(reached ? 1 : 0);
  return reached;
}

// Corrects left-right position within the tile by turning the robot a small amount before the next forward drive
// -> so fwd()'s heading-hold behavior moves the robot diagonally back towards the center.
// (it locks onto whatever heading it starts at) 
// Must run AFTER turnCompletedSuccessfully() has validated the cardinal turn, so this intentional small heading offset isn't mistaken for a botched turn.

