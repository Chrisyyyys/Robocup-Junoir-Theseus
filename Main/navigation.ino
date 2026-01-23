Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}

void stepForward(Direction d, int &x, int &y) {
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
}

bool inBounds(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].discovered = false;
      mapGrid[x][y].fullyExplored = false;
      for (int d = 0; d < 4; d++) {
        mapGrid[x][y].wall[d] = false;
        mapGrid[x][y].edge[d] = false;
      }
      mapGrid[x][y].tileType = BLANK;
      mapGrid[x][y].victim = false;
    }
  }
}

// mark traveled edge in BOTH tiles (current and next)
void markEdgeBothWays(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  if (!inBounds(nx, ny)) return;

  mapGrid[x][y].edge[d] = true;
  mapGrid[nx][ny].edge[opposite(d)] = true;
}

// update fullyExplored = all OPEN dirs have been traveled at least once
void updateFullyExploredAt(int x, int y) {
  Tile &t = mapGrid[x][y];
  bool allDone = true;

  for (int d = 0; d < 4; d++) {
    if (t.wall[d] == false) {     // open
      if (t.edge[d] == false) {   // not traveled yet
        allDone = false;
        break;
      }
    }
  }
  t.fullyExplored = allDone;
}
// 0=front, 1=right, 2=back, 3=left
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL) {
  wallF = (detectWall(0) == 0);
  wallR = (detectWall(1) == 0);
  wallB = (detectWall(2) == 0);
  wallL = (detectWall(3) == 0);
}
//get the wall from L,R into N W
void writeWallsToCurrentTile(bool wallF, bool wallR, bool wallB, bool wallL) {
  Tile &t = mapGrid[x_pos][y_pos];
  t.discovered = true;
  // absolution
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);

  t.wall[absF] = wallF;
  t.wall[absR] = wallR;
  t.wall[absB] = wallB;
  t.wall[absL] = wallL;
}
Direction pickNextDirection() {
  Tile &t = mapGrid[x_pos][y_pos];

  Direction absL = rotateDir(currentDir, -1);
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);

  auto open  = [&](Direction d){ return t.wall[d] == false; };
  auto untr  = [&](Direction d){ return t.edge[d] == false; };

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

int turnNeededDeg(Direction from, Direction to) {
  int diff = ((int)to - (int)from + 4) % 4;
  if (diff == 0) return 0;
  if (diff == 1) return +90;
  if (diff == 2) return 180;
  return -90; // diff==3
}
