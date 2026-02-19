// I assume is global?
Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}
// at orientation w, conver heading of x to local heading of y.

void stepForward(Direction d, int &x, int &y) {
  // global direction
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
}

bool inBounds(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

bool isFullyExplored(Tile &t) {
  for (int d = 0; d < 4; d++) {
    if (!t.getWall(d) && !t.getEdge(d)) {
      return false;
    }
  }
  return true;
}

bool isFullyExploredAt(int x, int y) {
  if (!inBounds(x, y)) return false;
  return isFullyExplored(mapGrid[x][y]);
}

void recordVisitedTile(int x, int y) {
  if (!inBounds(x, y)) return;

  if (historyLen >= MAX_TILE_HISTORY) {
    for (int i = 1; i < MAX_TILE_HISTORY; i++) {
      historyX[i - 1] = historyX[i];
      historyY[i - 1] = historyY[i];
    }
    historyLen = MAX_TILE_HISTORY - 1;
  }

  historyX[historyLen] = x;
  historyY[historyLen] = y;
  historyLen++;
}

bool findLastNotFullyExploredFromHistory(int &targetX, int &targetY) {
  for (int i = historyLen - 1; i >= 0; i--) {
    int hx = historyX[i];
    int hy = historyY[i];

    if (!inBounds(hx, hy)) continue;
    if (!mapGrid[hx][hy].getDiscovered()) continue;

    if (!isFullyExploredAt(hx, hy)) {
      targetX = hx;
      targetY = hy;
      return true;
    }
  }
  return false;
}

int bfsPath(int startX, int startY, int targetX, int targetY, Direction outPath[], int maxLen) {
  if (!inBounds(startX, startY) || !inBounds(targetX, targetY)) return 0;
  if (!mapGrid[startX][startY].getDiscovered() || !mapGrid[targetX][targetY].getDiscovered()) return 0;

  const int maxNodes = MAP_SIZE * MAP_SIZE;
  int queueX[maxNodes];
  int queueY[maxNodes];
  bool visited[MAP_SIZE][MAP_SIZE];
  int parentX[MAP_SIZE][MAP_SIZE];
  int parentY[MAP_SIZE][MAP_SIZE];
  Direction parentDir[MAP_SIZE][MAP_SIZE];

  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      visited[x][y] = false;
      parentX[x][y] = -1;
      parentY[x][y] = -1;
      parentDir[x][y] = NORTH;
    }
  }

  int head = 0;
  int tail = 0;
  queueX[tail] = startX;
  queueY[tail] = startY;
  tail++;
  visited[startX][startY] = true;

  bool found = false;
  while (head < tail) {
    int cx = queueX[head];
    int cy = queueY[head];
    head++;

    if (cx == targetX && cy == targetY) {
      found = true;
      break;
    }

    Tile &ct = mapGrid[cx][cy];
    for (int d = 0; d < 4; d++) {
      Direction dir = (Direction)d;
      if (ct.getWall(dir)) continue;

      int nx = cx;
      int ny = cy;
      stepForward(dir, nx, ny);
      if (!inBounds(nx, ny)) continue;
      if (!mapGrid[nx][ny].getDiscovered()) continue;
      if (visited[nx][ny]) continue;

      visited[nx][ny] = true;
      parentX[nx][ny] = cx;
      parentY[nx][ny] = cy;
      parentDir[nx][ny] = dir;
      queueX[tail] = nx;
      queueY[tail] = ny;
      tail++;
    }
  }

  if (!found) return 0;

  Direction reversePath[maxNodes];
  int count = 0;
  int cx = targetX;
  int cy = targetY;

  while (!(cx == startX && cy == startY)) {
    if (count >= maxNodes) return 0;
    Direction stepDir = parentDir[cx][cy];
    reversePath[count++] = stepDir;

    int px = parentX[cx][cy];
    int py = parentY[cx][cy];
    if (px < 0 || py < 0) return 0;
    cx = px;
    cy = py;
  }

  if (count > maxLen) return 0;

  for (int i = 0; i < count; i++) {
    outPath[i] = reversePath[count - 1 - i];
  }

  return count;
}

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].setDiscovered(false);
      mapGrid[x][y].setFully(false);
      for (int d = 0; d < 4; d++) {
        mapGrid[x][y].setWall(d, false);
        mapGrid[x][y].setEdge(d, false);
      }
      mapGrid[x][y].setType(BLANK);
    }
  }
}

// mark traveled edge in BOTH tiles (current and next)
void markEdgeBothWays(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  if (!inBounds(nx, ny)) return;

  mapGrid[x][y].setEdge(d, true); // connected
  mapGrid[nx][ny].setEdge(opposite(d), true); // update both sides.
}

// update fullyExplored = all OPEN dirs have been traveled at least once
void updateFullyExploredAt(int x, int y) {
  Tile &t = mapGrid[x][y];
  t.setFully(isFullyExplored(t));
}
// 0=front, 1=right, 2=back, 3=left
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL) { // references needed here to update the variable values
  wallF = (detectWall(0)==0);
  wallR = (detectWall(1)==0);
  wallB = (detectWall(2)==0);
  wallL = (detectWall(3)==0);
  Serial.println(wallF);
  Serial.println(wallR);
  Serial.println(wallB);
  Serial.println(wallL);
}
//get the wall from L,R(local) into N W(global)
// absF is the absolute heading the the robot front is heading.
void writeWallsToCurrentTile(bool wallF, bool wallR, bool wallB, bool wallL) {
  Tile &t = mapGrid[x_pos][y_pos];
  t.setDiscovered(true);
  // shouldn't absolute directions be north south east west?
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);
  t.setWall(absF, wallF);
  t.setWall(absR, wallR);
  t.setWall(absB, wallB);
  t.setWall(absL, wallL);
  // need to mark both ways.
}
Direction pickNextDirection() {
   Tile &t = mapGrid[x_pos][y_pos];

  Direction absL = rotateDir(currentDir, -1);
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);

  auto open  = [&](Direction d){ return t.getWall(d) == false; };
  auto untr  = [&](Direction d){ return t.getEdge(d) == false; };

  // 1) try open + untraveled first
  if (open(absF) && untr(absF)) return absF;
  if (open(absL) && untr(absL)) return absL;
  if (open(absR) && untr(absR)) return absR;
  if (open(absB) && untr(absB)) return absB;

  // 2) else any open (front-priority fallback)
  if (open(absF)) return absF;
  if (open(absL)) return absL;
  if (open(absR)) return absR;
  if (open(absB)) return absB;
  // figure out BFS later
  // 3) trapped
  return absB;
}

int turnNeededDeg(Direction direction) {
  // Compute relative turn from current heading to target heading.
  /*
  int diff = (static_cast<int>(direction) - static_cast<int>(currentDir) + 4) % 4;
  if (diff == 0) return 0;
  if (diff == 1) return 90;
  if (diff == 2) return 180;
  return -90; // diff == 3, turn left
  */
  if (direction == 0) return 0;
  if (direction == 1) return 90;
  if (direction == 2) return 180;
  return 270; // diff==3

}
/*
void returnToStart(x,y){
  Tile &t = mapGrid[x_pos][y_pos];
  // when it reaches beginning, stop.
  if(x == 0 && y == 0){
    return;
  }
  // loop through all edges
  for(int i = 0; i< 4;i++){
    if(t.getEdge(i) == true){
      stepForward(i,x,y); // fwd in direction i 
      plannedTurnDeg = turnNeededDeg(currentDir, i);
      turn(plannedTurnDeg);
      fwd(TILE_MM);
      returnToStart(x,y);
    }
  }
  // check for each.
  
  stepForward();
}
*/
