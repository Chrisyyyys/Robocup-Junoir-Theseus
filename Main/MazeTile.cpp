#include "MazeTile.h"

Tile::Tile() {
  tileType = BLANK;
}

Direction opposite(Direction d) {
  return (Direction)((d + 2) % 4); // the opposite global direction.
}


