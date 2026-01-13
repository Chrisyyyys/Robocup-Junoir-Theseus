#ifndef MAZE_TILE_H
#define MAZE_TILE_H

// Directions: 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST

enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

enum TileTypes {
    BLANK = 0,
    BLUE = 1,
    BLACK = 2
};

struct Tile {
    bool discovered;        // Tile has been visited at least once
    bool fullyExplored;     // All possible paths from this tile have been checked

    bool wall[4];           // wall[d] = true if wall in direction d
    bool edge[4];           // edge[d] = true if robot has travelled through that direction

    int tileType;              // accounts for tiletypes (0 for normal, 1 for blue, 2 for black (later can adapt to other tiletypes if needed))
    bool victim;            // True if victim detected on this tile

    // Constructor initializes all values
    Tile() {
        discovered = false;
        fullyExplored = false;
        tileType = 0;
        victim = false;

        wall[4] = {false}; //initializes all to false. no need to loop, saves processing time.
        edge[4] = {false}; 
    }
};

// Optional helper: compute opposite direction
static inline Direction opposite(Direction d) {
    return (Direction)((d + 2) % 4);
}

#endif // MAZE_TILE_H
