#ifndef MAZE_TILE_H
#define MAZE_TILE_H
#include <array>
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
  CHECKPOINT =2,
  BLACK = 3
};

struct Tile {
  //bool discovered;
  //bool fullyExplored;

  
  private:
  //bits 0-3 wall, 4-7 edge, 8 discovered, 9 fully, 10 victim, 11 visited,
  //12 elevate, 13 descend, 16-19 obstacle (one bit per direction)
  //set amount of bits
  Bitset<20> bitset;
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
  // 4 bits for 16 possible tiletypes
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
  // multi-floor elevation flags 
  bool getElevate(){
    return bitset.get(12);
  }
  void setElevate(bool e){
    bitset.set(12,e);
  }
  bool getDescend(){
    return bitset.get(13);
  }
  void setDescend(bool d){
    bitset.set(13,d);
  }
  // obstacle presence, one bit per direction (0=N,1=E,2=S,3=W), stored at 16-19
  bool getObstacle(unsigned dir){
    return bitset.get(dir+16);
  }
  void setObstacle(unsigned dir,bool stat){
    bitset.set(dir+16, stat);
  }

  bool hasObstacle(){
    if(bitset.get(16) || bitset.get(17) || bitset.get(18) || bitset.get(19))
    {
      return true;
    }
    return false;
  }

  //bool wall[4];
  //bool edge[4];

  
  //bool tileType
  //bool victim;

  Tile();   // constructor
};

Direction opposite(Direction d);

#endif
