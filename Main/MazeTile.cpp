

// Directions: 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST
struct Tile {
    bool discovered;        // robot has visited this tile at least once
    bool fullyExplored;     // no remaining unexplored edges

    bool wall[4];           // true if wall in that direction
    bool edge[4];           // true if robot has traveled through that edge

    int tileType;           // accounts for tiletypes (0 for normal, 1 for blue, 2 for black (later can adapt to other tiletypes if needed))
    bool victim;            // true if victim detected here

    // Constructor to initialize everything to false
    Tile() {
        discovered = false;
        fullyExplored = false;
        tileType = 0;
        victim = false;
        
        wall[4] = {false}; //initializes all to false. no need to loop, saves processing time.
        edge[4] = {false}; 
    }
};


