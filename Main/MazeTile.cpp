#include "MazeTile.h"

Tile::Tile() {
  discovered = false;
  fullyExplored = false;
  tileType = BLANK;
  victim = false;

  for (int i = 0; i < 4; i++) {
    wall[i] = false;
    edge[i] = false;
  }
}

Direction opposite(Direction d) {
  return (Direction)((d + 2) % 4);
}


