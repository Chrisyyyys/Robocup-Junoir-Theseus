#include "Tunables.h"

// Compile-time defaults. These are the values that were hard-coded in fwd(), absoluteturn(),
// obstacleavoidance() and the color thresholds before the tuner existed.
TuneState tune = {
  /* center   */ {2,      0,     0.5},
  /* scale    */ {0.0045, 0,     0.0008},
  /* gyro     */ {1,      0.001, 0.03},
  /* climb    */ {2,      0,     0.1},
  /* turn     */ {4.5,    0,     0.3},
  /* parallel */ {1,      0,     0.1},
  /* wiggle   */ {8,      0,     0.1},
  /* colBlack  */ 0.1f,
  /* colSilver */ 800,
  /* colWhite  */ 0.85f
};
