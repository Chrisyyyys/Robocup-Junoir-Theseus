#include "MazeTile.h"

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



Direction opposite(Direction d) {
  return (Direction)((d + 2) % 4); // the opposite global direction.
}


