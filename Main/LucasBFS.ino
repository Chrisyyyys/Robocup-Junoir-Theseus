// BFS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <deque>
#include <utility>

using namespace std;

enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

enum TileTypes {
    BLANK = 0,
    BLUE = 1,
    BLACK = 2
};

struct Tile {
    bool discovered;
    bool fullyExplored;

    bool wall[4];
    bool edge[4];

    TileTypes tileType;
    bool victim;
    // uint8_t wall[4];     // still indexed the same
    // uint8_t edge[4];
    // uint8_t discovered;
    Tile();   // constructor
};

Tile::Tile() {
    discovered = 0;
    fullyExplored = false;
    tileType = BLANK;
    victim = false;
    // for (int i = 0; i < 4; i++) {
    //   wall[i] = 0;
    //   edge[i] = 0;
    // }
    for (int i = 0; i < 4; i++) {
        wall[i] = false;
        edge[i] = false;
    }
}

int dir[4][2] = {
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0}
};

const int MAP_SIZE = 20;
Tile mapGrid[MAP_SIZE][MAP_SIZE];

deque<pair<int,int>> BFS(pair<int,int> currentpos, vector<vector<bool>> map, pair<int, int> endpos) {
    deque<pair<int, int>> queue = {};
    size_t rows = map.size();
    size_t columns = map[0].size();
    vector<vector<bool>> visited(rows, vector<bool>(columns, false));
    deque<pair<int, int>> path = {};
    vector<vector<pair<int, int>>> prev(MAP_SIZE, vector<pair<int,int>>(MAP_SIZE));
    queue.push_back(currentpos);

    //search
    while (queue.size() > 0) {
        visited[queue[0].first][queue[0].second] = true;
        int x = queue[0].first; int y = queue[0].second;
        cout << "visting: " << x << "," << y << endl;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];
            if (nx < map.size() && ny < map[0].size() && nx >= 0 && ny >= 0) {
                if (!visited[nx][ny] && !map[nx][ny]) {
                    queue.push_back(pair<int, int>(nx, ny));
                    //cout << "queue added: " << nx << "," << ny << endl;
                    prev[nx][ny] = pair<int, int>(x, y);
                    //cout << "at: " << nx << "," << ny << " added: " << x << "," << y << endl;
                }
            }
        }
        queue.pop_front();
    }

    //reconstruct
    path.push_front(endpos);
    pair<int, int> curr = prev[endpos.first][endpos.second];
    //cout << endl << "begin path reconstruction" << endl;
    while (true) {
        //cout << curr.first << "," << curr.second << endl;
        path.push_front(curr);
        curr = prev[curr.first][curr.second];
        if (path[0] == currentpos) {
            break;
        }
        
    }
    cout << endl << "done search" << endl << endl;
    //reconstruct
    return path;
}


int main()
{
    vector<vector<bool>> map = {
        {false, false, false},
        { false, true, false },
        { false, false ,true },
        { true, false ,false }
    };

    deque<pair<int, int>> path = BFS(pair<int, int>(0, 0), map, pair<int,int>(3,2));

    cout << "path: " << endl;
    for (int i = 0; i < path.size(); i++) {
        cout << path[i].first << "," << path[i].second << endl;
    }
}