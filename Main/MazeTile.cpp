#include "bitset"
// Directions: 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST
struct Tile {
    bool discovered;        // robot has visited this tile at least once
    bool fullyExplored;     // no remaining unexplored edges

    bool wall[4];           // true if wall in that direction
    bool edge[4];           // true if robot has traveled through that edge

    bool black;             // true if this tile is a black tile (avoid)
    bool victim;            // true if victim detected here

    // Constructor to initialize everything to false
    Tile() {
        discovered = false;
        fullyExplored = false;
        black = false;
        victim = false;

        for (int i = 0; i < 4; i++) {
            wall[i] = false;
            edge[i] = false;
        }
    }
};
