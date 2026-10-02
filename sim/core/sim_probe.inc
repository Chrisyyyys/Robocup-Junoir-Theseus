// ---- Simulator probe ----
// Appended to the merged sketch by sim/tools/sketch.py (it is NOT part of the robot
// firmware). It lets the simulator read the robot's own variables: its state, where
// it thinks it is, and its maze map. If you rename one of these variables in the
// robot code, update the matching line here.
#include "probe_api.h"
#include "sketch_names.h"

// Globals that exist in some versions of the robot code only (SIM_HAVE_* comes from sketch.py).
#define SIM_OPT(name, fallback) SIM_OPT_I(SIM_HAVE_##name, name, fallback)
#define SIM_OPT_I(have, name, fallback) SIM_OPT_II(have, name, fallback)
#define SIM_OPT_II(have, name, fallback) SIM_OPT_##have(name, fallback)
#define SIM_OPT_1(name, fallback) (name)
#define SIM_OPT_0(name, fallback) (fallback)

void sim_probe_read(SimProbe* p) {
  p->state = (int)state;
  p->x = x_pos;
  p->y = y_pos;
  p->dir = (int)currentDir;
  p->floor = SIM_OPT(currentFloor, 0);
  p->iterator = SIM_OPT(iterator, 0);
  p->fwd_active = SIM_OPT(fwdActive, 0);
  p->turn_active = SIM_OPT(turnActive, 0);
  p->victim_pending = SIM_OPT(victimPending, 0);
  p->pause = SIM_OPT(Pausemaze, 0);
  p->black_toggle = SIM_OPT(blacktoggle, 0);
  p->blue_toggle = SIM_OPT(bluetoggle, 0);
  p->obstacle = SIM_OPT(obstacle, 0);
  p->victim_toggle = SIM_OPT(victimtoggle, 0);
  p->x_checkpoint = SIM_OPT(x_checkpoint, MAP_SIZE / 2);
  p->y_checkpoint = SIM_OPT(y_checkpoint, MAP_SIZE / 2);
  p->floor_checkpoint = SIM_OPT(floor_checkpoint, 0);
  p->medkits = SIM_OPT(medkits, 0);
  p->map_size = MAP_SIZE;
  p->steps = (int)SIM_OPT(steps, 0);
#if SIM_HAVE_drivetrain
  p->enc_a = drivetrain.encoderCountA;
  p->enc_b = drivetrain.encoderCountB;
  p->enc_d = drivetrain.encoderCountD;
#else
  p->enc_a = p->enc_b = p->enc_d = 0;
#endif
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

template <class T>
static auto sim_blocked_bits(T& t, int) -> decltype(t.getBlocked(0u), uint32_t()) {
  uint32_t b = 0;
  for (unsigned d = 0; d < 4; d++)
    if (t.getBlocked(d)) b |= 1u << (SIMT_BLOCK0 + d);
  return b;
}
template <class T>
static uint32_t sim_blocked_bits(T&, long) { return 0; }

uint32_t sim_probe_tile(int grid, int x, int y) {
  if (x < 0 || y < 0 || x >= MAP_SIZE || y >= MAP_SIZE) return 0;
#if SIM_HAVE_m1 && SIM_HAVE_m2 && SIM_HAVE_m3
  Grid* g = grid == 1 ? &m1 : grid == 2 ? &m2 : grid == 3 ? &m3 : &mapGrid;
#else
  Grid* g = &mapGrid;
  if (grid != 0) return 0;
#endif
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
  b |= sim_blocked_bits(t, 0);
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
