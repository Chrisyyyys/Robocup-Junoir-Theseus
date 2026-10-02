// ---- Simulator probe ----
// Appended to the merged sketch by sim/tools/sketch.py (it is NOT part of the robot
// firmware). It lets the simulator read the robot's own variables: its state, where
// it thinks it is, and its maze map. If you rename one of these variables in the
// robot code, update the matching line here.
#include "probe_api.h"
#include "sketch_names.h"

void sim_probe_read(SimProbe* p) {
  p->state = (int)state;
  p->x = x_pos;
  p->y = y_pos;
  p->dir = (int)currentDir;
  p->floor = currentFloor;
  p->iterator = iterator;
  p->fwd_active = fwdActive;
  p->turn_active = turnActive;
  p->victim_pending = victimPending;
  p->pause = Pausemaze;
  p->black_toggle = blacktoggle;
  p->blue_toggle = bluetoggle;
  p->obstacle = obstacle;
  p->victim_toggle = victimtoggle;
  p->x_checkpoint = x_checkpoint;
  p->y_checkpoint = y_checkpoint;
  p->floor_checkpoint = floor_checkpoint;
  p->medkits = medkits;
  p->map_size = MAP_SIZE;
  p->steps = (int)steps;
  p->enc_a = drivetrain.encoderCountA;
  p->enc_b = drivetrain.encoderCountB;
  p->enc_d = drivetrain.encoderCountD;
}

int sim_probe_get_state() { return (int)state; }
int sim_probe_get_iterator() { return iterator; }
void sim_probe_set_iterator(int v) { iterator = v; }
int sim_probe_state_count() { return SIM_ROBOTSTATE_COUNT; }

const char* sim_probe_state_name(int s) {
  if (s >= 0 && s < SIM_ROBOTSTATE_COUNT) return SIM_ROBOTSTATE_NAMES[s];
  return "?";
}

const char* sim_probe_steps_name(int s) {
  if (s >= 0 && s < SIM_STEPS_COUNT) return SIM_STEPS_NAMES[s];
  return "?";
}

uint32_t sim_probe_tile(int grid, int x, int y) {
  if (x < 0 || y < 0 || x >= MAP_SIZE || y >= MAP_SIZE) return 0;
  Grid* g = grid == 1 ? &m1 : grid == 2 ? &m2 : grid == 3 ? &m3 : &mapGrid;
  Tile& t = (*g)[x][y];
  uint32_t b = 0;
  for (unsigned d = 0; d < 4; d++) {
    if (t.getWall(d)) b |= 1u << (SIMT_WALL0 + d);
    if (t.getEdge(d)) b |= 1u << (SIMT_EDGE0 + d);
    if (t.getObstacle(d)) b |= 1u << (SIMT_OBST0 + d);
  }
  if (t.getDiscovered()) b |= 1u << SIMT_DISCOVERED;
  if (t.getFully()) b |= 1u << SIMT_FULLY;
  if (t.getVictim()) b |= 1u << SIMT_VICTIM;
  if (t.getVisited()) b |= 1u << SIMT_VISITED;
  if (t.getElevate()) b |= 1u << SIMT_ELEVATE;
  if (t.getDescend()) b |= 1u << SIMT_DESCEND;
  b |= ((uint32_t)t.getType() & 3u) << SIMT_TYPE0;
  return b;
}

namespace rtos {
const char* sim_thread_name_for(void (*fn)()) {
  for (int i = 0; i < SIM_VOID_FN_COUNT; i++)
    if (SIM_VOID_FNS[i] == fn) return SIM_VOID_FN_NAMES[i];
  return "thread";
}
}  // namespace rtos

// The "iterator >= N" move limit found in the sketch (-1 if there is none).
int sim_probe_code_moves_limit() { return SIM_CODE_MOVES_LIMIT; }
