#ifndef MAZE_TILE_H
#define MAZE_TILE_H

// Directions: 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST

enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

struct Tile {
    bool discovered;        // Tile has been visited at least once
    bool fullyExplored;     // All possible paths from this tile have been checked

    bool wall[4];           // wall[d] = true if wall in direction d
    bool edge[4];           // edge[d] = true if robot has travelled through that direction

    bool black;             // True if this tile is a black tile
    bool victim;            // True if victim detected on this tile

    // Constructor initializes all values
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

// Optional helper: compute opposite direction
static inline Direction opposite(Direction d) {
    return (Direction)((d + 2) % 4);
}

#endif // MAZE_TILE_H
