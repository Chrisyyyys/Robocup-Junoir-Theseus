                                                                      
// I assume is global?
Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}
// at orientation w, conver heading of x to local heading of y.

void stepForward(Direction d, int &x, int &y, int &z) {
  // global direction
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
  else if (d == TOP) z++;
  else if (d == BOTTOM) z--;
  //if(victimAtCurrent == false&&victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
}

bool inBounds(int x, int y, int z) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE && z >= 0 && z <;
}

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      for (int z = 0; z < 5; z++){
        mapGrid[x][y][z].setDiscovered(false);
        mapGrid[x][y][z].setFully(false);
        mapGrid[x][y][z].setVisited(false);
        mapGrid[x][y][z].setElevation(false);
        mapGrid[x][y][z].setDescension(false);
        for (int d = 0; d < 4; d++) {
          mapGrid[x][y][z].setWall(d, false);
          mapGrid[x][y][z].setEdge(d, false);
        }
        mapGrid[x][y][z].setType(BLANK);
      }
    }  
  }
}

// mark traveled edge in BOTH tiles (current and next)
void markEdgeBothWays(int x, int y, int z, Direction d) {
  int nx = x, ny = y, nz = z;
  stepForward(d, nx, ny, nz);
  if (!inBounds(nx, ny, nz)) return;

  mapGrid[x][y][z].setEdge(d, true); // connected
  mapGrid[nx][ny][nz].setEdge(opposite(d), true); // update both sides.
}

// update fullyExplored = all OPEN dirs have been traveled at least once
void updateFullyExploredAt(int x, int y, int z) {
  Tile &t = mapGrid[x][y][z];
  bool allDone = true;
  t.setVisited(true);
  for (int d = 0; d < 4; d++) {
    if (t.getWall(d) == false) {     // open
      if (t.getEdge(d) == false) {   // not traveled yet
        allDone = false;
        break;
      }
    }
  }
  t.setFully(allDone);
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
  Tile &t = mapGrid[x_pos][y_pos][z_pos];
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
  Tile &t = mapGrid[x_pos][y_pos][z_pos];

  Direction absL = rotateDir(currentDir, -1);
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  // Plan directly in absolute map directions.
  const Direction priority[3] = {absF,absR, absL};

  auto open  = [&](Direction d){ return t.getWall(d) == false; };
  auto untr  = [&](Direction d){ return t.getEdge(d) == false; };
  auto isBlueTile = [&](int nx, int ny, int nz){
    return mapGrid[nx][ny][nz].getType() == BLUE || mapGrid[nx][ny][nz].getBlue();
  };
  auto blockedForTravel = [&](int nx, int ny, int nz){
    return mapGrid[nx][ny][nz].getType() == BLACK || mapGrid[nx][ny][nz].getType() == STAIR || isBlueTile(nx, ny, nz);
  };
  auto isBlackTile = [&](int nx, int ny, int nz){
    return mapGrid[nx][ny][nz].getType() == BLACK;
  };
  

  //im unsure if this will work? it probably will since TOP/BOTTOM shouldnt be explored
  // 1) try open + untraveled first
  for (int i = 0; i < 3; i++) {
    int nx = x_pos, ny = y_pos, nz = z_pos;
    
    Direction d = priority[i];
    stepForward(d,nx,ny,nz);

    if (inBounds(nx, ny, nz) && open(d) && untr(d) && !mapGrid[nx][ny][nz].getVisited() && !blockedForTravel(nx, ny, nz)) return d;
  }

  // 2) else any open (still avoid black/blue) using the same absolute priority.
  for (int i = 0; i < 3; i++) {
    int nx = x_pos, ny = y_pos, nz = z_pos;
    Direction d = priority[i];
    stepForward(d,nx,ny, nz);
    if (inBounds(nx, ny, nz) && open(d) && !blockedForTravel(nx, ny, nz)) return d;
  }
  for (int i=0;i<3;i++){
    int nx = x_pos, ny = y_pos, nz = z_pos;
    Direction d = priority[i];
    stepForward(d,nx,ny, nz);
    if (inBounds(nx, ny, nz) && open(d)&&!isBlackTile(nx,ny, nz)) return d;
  }

  // figure out BFS later
  // 3) trapped
  return absB;
}

int turnNeededDeg(Direction direction) {
  // Convert an absolute direction enum to an absolute heading angle.
  if (direction == 0) return 0;
  if (direction == 1) return 90;
  if (direction == 2) return 180;
  return 270; // diff==3

}
int dir[6][3] = {
    {0, 1, 0},
    {1, 0, 0},
    {0, -1, 0},
    {-1, 0, 0},
    {0, 0, 1},
    {0, 0, -1}
};

void reallocate(Tile mapgrid[MAP_SIZE][MAP_SIZE], int pos_x = 0, int pos_y = 0, int pos_z = 0) { //input mapgrid, and next tile location
    //cout << "start reallocate" << endl;

    //expand to bottom (remove top)
    if (pos_y >= MAP_SIZE) {
        for (int i = 0; i < MAP_SIZE - 1; i++) {
            for (int j = 0; j < MAP_SIZE; j++) {
              for(int k = 0; k < 5; k++){
                mapgrid[i][j][k] = mapgrid[i + 1][j][k];
              }
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(MAP_SIZE - 1, i);
        }
    }
    //expand to top (remove bottom)
    else if (pos_y < 0) {
        for (int i = MAP_SIZE-1; i > 0; i--) {
            for (int j = 0; j < MAP_SIZE; j++) {
              for(int k = 0; k < 5; k++){
                mapgrid[i][j][k] = mapgrid[i - 1][j][k];
              }
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(0, i);
        }
    }

    //expand to right (remove left)
    else if (pos_x >= MAP_SIZE) {
        for (int j = 0; j < MAP_SIZE - 1; j++) {
            for (int i = 0; i < MAP_SIZE; i++) {
                for(int k = 0; k < 5; k++){
                mapgrid[i][j] = mapgrid[i - 1][j];
                }
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(i, MAP_SIZE-1);
        }
    }

    //expand to left (remove right)
    else if (pos_x < 0) {
        for (int j = MAP_SIZE - 1; j > 0; j--) {
            for (int i = 0; i < MAP_SIZE; i++) {
                mapgrid[i][j] =  mapgrid[i][j - 1];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(i, 0);
        }
    }
}

// pair structure
int BFS(coord currentpos, Tile mapGrid[MAP_SIZE][MAP_SIZE], coord endpos,coord path[MAP_SIZE * MAP_SIZE]) { // auto updates path
    ArduinoQueue<coord> queue = {};
    size_t rows = MAP_SIZE;
    size_t columns = MAP_SIZE;
    bool visited[MAP_SIZE][MAP_SIZE] = {false};
    coord prev[MAP_SIZE][MAP_SIZE];
    queue.enqueue(currentpos); // current tile
    visited[currentpos.x][currentpos.y] = true;
    //search
    while (queue.itemCount() > 0) {
        int x = queue.getHead().x; int y = queue.getHead().y;
        //cout << "visting: " << x << "," << y << endl;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];
            if (nx < rows && ny < columns && nx >= 0 && ny >= 0) {
                if (!visited[nx][ny] &&
                    !mapGrid[x][y].getWall((Direction)i) &&
                    !mapGrid[nx][ny].getWall(opposite((Direction)i)) &&
                    mapGrid[nx][ny].getDiscovered() &&
                    mapGrid[nx][ny].getType() != BLACK &&
                    mapGrid[nx][ny].getType() != STAIR) { //IMPORTANT: ADD MORE CONDITIONALS HERE 
                    queue.enqueue(coord{nx, ny}); // add tile
                    visited[nx][ny] = true;
                    
                    prev[nx][ny] = coord{x, y};
                    
                }
            }
        }
        queue.dequeue();
    }
    //reconstruct path
    // path[0] is (0,0), path[n] is current tile.
    int i = 0;
    coord curr = endpos;
    while (true) {
      path[i] = curr;
      i++;

      if (curr.x == currentpos.x&&curr.y==currentpos.y) {
        break;
      }
      curr = prev[curr.x][curr.y];
    }
    return i;
    
}
