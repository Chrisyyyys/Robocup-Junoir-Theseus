/*
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
    BLUE  = 1,
    BLACK = 2
};

struct Tile {
    bool discovered;        // Tile has been visited at least once
    bool fullyExplored;     // All possible paths from this tile have been checked

    bool wall[4];           // wall[d] = true if wall in direction d
    bool edge[4];           // edge[d] = true if robot has travelled through that direction

    TileTypes tileType;     // BLANK, BLUE, BLACK
    bool victim;            // True if victim detected on this tile

    Tile();                 // constructor declaration only
};

// Helper: compute opposite direction (declaration only)
Direction opposite(Direction d);

#endif // MAZE_TILE_H
*/
