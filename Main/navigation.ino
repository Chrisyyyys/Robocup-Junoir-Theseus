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

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].setDiscovered(false);
      mapGrid[x][y].setFully(false);
      mapGrid[x][y].setVisited(false);
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
  // Plan directly in absolute map directions.
  const Direction priority[4] = {absF, absR, absL, absB};

  auto open  = [&](Direction d){ return t.getWall(d) == false; };
  auto untr  = [&](Direction d){ return t.getEdge(d) == false; };

  // 1) try open + untraveled first
  for (int i = 0; i < 4; i++) {
    int nx = x_pos, ny = y_pos;
    
    Direction d = priority[i];
    stepForward(d,nx,ny);

    if (open(d) && untr(d)&&!mapGrid[nx][ny].getVisited()) return d;
  }

  // 2) else any open using the same absolute priority.
  for (int i = 0; i < 4; i++) {
    Direction d = priority[i];
    if (open(d)) return d;
  }

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
int dir[4][2] = {
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0}
};
/*
deque<pair<int,int>> BFS(pair<int,int> currentpos, Tile mapGrid[MAP_SIZE][MAP_SIZE], pair<int, int> endpos) {
    deque<pair<int, int>> queue = {};
    size_t rows = MAP_SIZE;
    size_t columns = MAP_SIZE;
    vector<vector<bool>> visited(rows, vector<bool>(columns, false));
    deque<pair<int, int>> path = {};
    vector<vector<pair<int, int>>> prev(MAP_SIZE, vector<pair<int,int>>(MAP_SIZE));
    queue.push_back(currentpos);

    //search
    while (queue.size() > 0) {
        int x = queue[0].first; int y = queue[0].second;
        //cout << "visting: " << x << "," << y << endl;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];
            if (nx < rows && ny < columns && nx >= 0 && ny >= 0) {
                if (!visited[nx][ny] && !mapGrid[nx][ny].getWall(i)) { //IMPORTANT: ADD MORE CONDITIONALS HERE 
                    queue.push_back(pair<int, int>(nx, ny));
                    visited[nx][ny] = true;
                    //cout << "queue added: " << nx << "," << ny << endl;
                    prev[nx][ny] = pair<int, int>(x, y);
                    //cout << "at: " << nx << "," << ny << " added: " << x << "," << y << endl;
                }
            }
        }
        queue.pop_front();
    }
    for (int i = 0; i < visited.size(); i++) {
        for (int j = 0; j < visited[0].size(); j++) {
            cout << visited[i][j] << " ";
        }
        cout << endl;
    }
    //reconstruct path
    path.push_front(endpos);
    pair<int, int> curr = prev[endpos.first][endpos.second];
    while (true) {
        //cout << curr.first << "," << curr.second << endl;
        path.push_front(curr);
        curr = prev[curr.first][curr.second];
        if (path[0] == currentpos) {
            break;
        }
        
    }
    //cout << endl << "done search" << endl << endl;
    return path;
}
*/