// BFS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <vector>
#include <deque>
#include <utility>

using namespace std;

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

    /*for (int i = 0; i < visited.size(); i++) {
        for (int j = 0; j < visited[0].size(); j++) {
            cout << visited[i][j] << " ";
        }
        cout << endl;
    }*/


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