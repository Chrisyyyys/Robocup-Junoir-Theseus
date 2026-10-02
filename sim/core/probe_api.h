// Interface between the simulator and the probe compiled into the robot sketch.
#pragma once
#include <stdint.h>

struct SimProbe {
  int state, x, y, dir, floor, iterator;
  int fwd_active, turn_active, victim_pending, pause, black_toggle, blue_toggle, obstacle, victim_toggle;
  int x_checkpoint, y_checkpoint, floor_checkpoint;
  int medkits, map_size, steps;
  long enc_a, enc_b, enc_d;
};

// tile bits returned by sim_probe_tile()
enum {
  SIMT_WALL0 = 0,      // bits 0..3 walls N E S W
  SIMT_EDGE0 = 4,      // bits 4..7 travelled edges
  SIMT_DISCOVERED = 8,
  SIMT_FULLY = 9,
  SIMT_VICTIM = 10,
  SIMT_VISITED = 11,
  SIMT_ELEVATE = 12,
  SIMT_DESCEND = 13,
  SIMT_OBST0 = 16,     // bits 16..19 obstacle per direction
  SIMT_BLOCK0 = 20,    // bits 20..23 blocked edge per direction (newer robot code)
  SIMT_TYPE0 = 24      // bits 24..25 tile type (0 blank, 1 blue, 2 checkpoint, 3 black)
};

void sim_probe_read(SimProbe* p);
const char* sim_probe_state_name(int s);
int sim_probe_state_count();
const char* sim_probe_steps_name(int s);
uint32_t sim_probe_tile(int grid, int x, int y);  // grid: 0 = active map, 1..3 = stored floors m1..m3
int sim_probe_get_iterator();
void sim_probe_set_iterator(int v);
int sim_probe_get_state();
