#ifndef MAZE_TILE_H
#define MAZE_TILE_H
#include "bitSet.h"

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
  CHECKPOINT =2

};

struct Tile {
  //bool discovered;
  //bool fullyExplored;

  
  private:
  //first 4 bit is wall, last 4 bit is edge
  //set amount of bits
  Bitset<16> bitset;
  TileTypes tileType;
  public:
  //get and set functions
  bool getWall(unsigned dir)
  {
    return bitset.get(dir);
  }
  void setWall(unsigned dir,bool stat){
    bitset.set(dir, stat);
  }
  //+4 is index to the target bit
  bool getEdge(unsigned dir){
    return bitset.get(dir+4);
  }
  void setEdge(unsigned dir,bool stat){
    bitset.set(dir+4, stat);
  }
  bool getDiscovered(){
    return bitset.get(8);
  }

  void setDiscovered(bool stat){
    bitset.set(8,stat);
  }
  bool getFully(){
    return bitset.get(9);
  }
  void setFully(bool stat){
    bitset.set(9,stat);
  }
  TileTypes getType(){
    return tileType;
  }
  void setType(TileTypes type){
    tileType=type;
  }
  bool getVictim(){
    return bitset.get(10);
  }
  void setVictim(bool vic){
    bitset.set(10,vic);
  }
  bool getVisited(){
    return bitset.get(11);
  }
  void setVisited(bool stat){
    bitset.set(11,stat);
  }

  //bool wall[4];
  //bool edge[4];

  
  //bool tileType
  //bool victim;

  Tile();   // constructor
};

Direction opposite(Direction d);

#endif
