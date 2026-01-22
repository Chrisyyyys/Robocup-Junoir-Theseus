#ifndef MAZE_TILE_H
#define MAZE_TILE_H

// Directions: 0=NORTH,1=EAST,2=SOUTH,3=WEST
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
  bool discovered;
  bool fullyExplored;

  bool wall[4];
  bool edge[4];

  TileTypes tileType;
  bool victim;

  Tile();   // constructor
};

Direction opposite(Direction d);

#endif

