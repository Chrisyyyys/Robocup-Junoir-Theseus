#ifndef MAZE_TILE_H
#define MAZE_TILE_H
#include <stdint.h>

//Directions: 0=NORTH,1=EAST,2=SOUTH,3=WEST
enum Direction {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3
};

// enum Direction : uint8_t {
//   NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3,  // old names (main uses these)
//   N = NORTH, E = EAST, S = SOUTH, W = WEST   // new short aliases
// };

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
  // uint8_t wall[4];     // still indexed the same
  // uint8_t edge[4];
  // uint8_t discovered;
  

  Tile();   // constructor
};

Direction opposite(Direction d);

#endif

