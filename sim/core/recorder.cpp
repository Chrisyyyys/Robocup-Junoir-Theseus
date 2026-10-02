// Watches the run: records the trace for the viewer, checks what the robot believes
// against the truth, handles lack-of-progress restarts like a referee, decides when
// the run is over and scores it.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <climits>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <sstream>

#include "probe_api.h"
#include "sim.h"

int sim_probe_code_moves_limit();

namespace sim {

static const double DEG = M_PI / 180.0;

namespace {

struct Frame {
  float t, x, y, h, z, p;
  int st, bx, by, bd, bf, it, fl, steps, col, enc;
  int tof[8];
  int mot[4];
};
struct EventRec {
  double t;
  std::string kind, msg;
  double x, y;
};
struct MapSnap {
  double t;
  int floor;
  std::vector<std::array<int, 3>> tiles;
};
struct VictimState {
  bool identified = false;
  bool misidentified = false;
  char claimed = 0;
  int kits = 0;
  double t = 0;
};

std::string g_trace_path, g_result_path;
std::vector<Frame> g_frames;
std::vector<EventRec> g_events;
std::vector<std::pair<double, std::string>> g_serial;
std::string g_serial_cur;
size_t g_serial_cap = 150000;
bool g_serial_truncated = false;
std::vector<std::pair<double, std::string>> g_lcd;
std::string g_lcd_last;
std::map<std::string, std::pair<std::string, int>> g_warnings;
std::vector<std::string> g_warning_order;
std::vector<MapSnap> g_maps;
uint64_t g_map_hash = 0;
int64_t g_phys_t = 0, g_next_frame = 0, g_frame_us = 50000, g_next_lcd_poll = 0;

int g_cur_tile = -2;
std::vector<int> g_visits;
std::set<int> g_cp_visited, g_bump_tiles, g_stair_tiles;
std::map<int, int> g_blue_visits;
std::map<int, double> g_blue_best_stop;
double g_blue_stop_start = -1;
int g_last_cp = -1;
int g_ramp_up = 0, g_ramp_down = 0;
int g_holes = 0;

int g_last_state = -1;
int g_state_return = -1;
int g_checks = 0, g_pos_errors = 0, g_dir_errors = 0;
double g_first_lost = -1;
double g_center_err_sum = 0, g_center_err_max = 0, g_head_err_sum = 0, g_head_err_max = 0;
bool g_was_lost = false;

double g_last_motor = 0;
double g_progress_t = 0, g_progress_x = 0, g_progress_y = 0, g_progress_th = 0;
bool g_last_contact = false;
double g_last_contact_end = -10;

int g_lop_state = 0;  // 0 none, 1 paused, 2 placed
double g_lop_t0 = 0;
int g_lops = 0;
std::string g_lop_reason;
int g_switch_pin = 22;
int g_led_pin = 51;
int g_led_on_count = 0;
double g_led_last_on = -10;

std::vector<VictimState> g_vs;
int g_last_identified = -1;
double g_last_identified_t = -100;
int g_medkits_last = INT_MIN;
int g_kits_total = 0;
int g_stepper_steps = 0;

bool g_claimed_home = false;
std::string g_hang_reason;
double g_setup_done = -1;

int g_iter_seen = 0, g_moves_counted = 0;

std::string jesc(const std::string& s) {
  std::string o;
  o.reserve(s.size() + 8);
  for (char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
      o += c;
    } else if (c == '\n') {
      o += "\\n";
    } else if ((unsigned char)c < 0x20) {
      o += ' ';
    } else if ((unsigned char)c >= 0x80) {
      o += '?';
    } else {
      o += c;
    }
  }
  return o;
}

std::string tile_name(int ti) {
  if (ti < 0) return "outside";
  int x = ti % world.W, y = ti / world.W;
  return "(" + std::to_string(x) + "," + std::to_string(world.H - 1 - y) + ")";
}

double now_s() { return g_phys_t / 1e6; }

const char* state_name(int s) { return sim_probe_state_name(s); }

void map_to_world(int bx, int by, int map_size, int* wx, int* wy) {
  int dx = bx - map_size / 2, dy = by - map_size / 2;
  for (int k = 0; k < world.start_dir; k++) {
    int t = dx;
    dx = dy;
    dy = -t;
  }
  *wx = world.start_x + dx;
  *wy = world.start_y + dy;
}

int actual_dir() {
  double rel = (robot.th - robot.th0) / DEG;
  int d = (int)std::lround(rel / 90.0);
  return ((d % 4) + 4) % 4;
}

void event(const std::string& kind, const std::string& msg) {
  g_events.push_back(EventRec{now_s(), kind, msg, robot.x, robot.y});
}

void start_lop(const std::string& reason, const std::string& outcome_if_off) {
  if (!opt.lop || g_lops >= opt.max_lops) {
    event("end", reason);
    finish(outcome_if_off, reason);
  }
  g_lops++;
  g_lop_reason = reason;
  event("lop", "Lack of progress #" + std::to_string(g_lops) + ": " + reason +
                   ". The referee lifts the robot, flips the pause switch and puts it on the last checkpoint.");
  robot.held = true;
  pin_set_input(g_switch_pin, 1);
  g_lop_state = 1;
  g_lop_t0 = now_s();
}

void lop_update() {
  double t = now_s();
  if (g_lop_state == 1 && t - g_lop_t0 >= 1.0) {
    int target = g_last_cp >= 0 ? g_last_cp : world.idx(world.start_x, world.start_y);
    int tx = target % world.W, ty = target / world.W;
    robot.teleport((tx + 0.5) * world.tile, (ty + 0.5) * world.tile, robot.th0);
    g_cur_tile = target;
    event("lop", "robot placed on " + std::string(g_last_cp >= 0 ? "checkpoint " : "start tile ") + tile_name(target));
    g_lop_state = 2;
  } else if (g_lop_state == 2 && t - g_lop_t0 >= 4.0) {
    robot.held = false;
    pin_set_input(g_switch_pin, 0);
    g_lop_state = 0;
    g_progress_t = t;
    g_last_motor = t;
    event("lop", "pause switch released, robot continues");
  }
}

void belief_check() {
  SimProbe p;
  sim_probe_read(&p);
  int tx, ty;
  if (!world.tile_of(robot.x, robot.y, &tx, &ty)) return;
  int wx, wy;
  map_to_world(p.x, p.y, p.map_size, &wx, &wy);
  int ad = actual_dir();
  g_checks++;
  double cx = (tx + 0.5) * world.tile, cy = (ty + 0.5) * world.tile;
  double ce = std::hypot(robot.x - cx, robot.y - cy);
  double rel = (robot.th - robot.th0) / DEG;
  double he = std::fabs(rel - 90.0 * std::lround(rel / 90.0));
  g_center_err_sum += ce;
  g_center_err_max = std::max(g_center_err_max, ce);
  g_head_err_sum += he;
  g_head_err_max = std::max(g_head_err_max, he);
  bool pos_ok = wx == tx && wy == ty;
  bool dir_ok = ad == ((p.dir % 4) + 4) % 4;
  if (!pos_ok) g_pos_errors++;
  if (!dir_ok) g_dir_errors++;
  if ((!pos_ok || !dir_ok) && !g_was_lost) {
    if (g_first_lost < 0) g_first_lost = now_s();
    static const char* dn = "NESW";
    event("lost", "Robot thinks it is on tile " + tile_name(world.in(wx, wy) ? world.idx(wx, wy) : -1) +
                      " facing " + dn[((p.dir % 4) + 4) % 4] + ", but it is really on " + tile_name(world.idx(tx, ty)) +
                      " facing " + dn[ad] + ".");
  } else if (pos_ok && dir_ok && g_was_lost) {
    event("found", "Robot's position belief matches reality again.");
  }
  g_was_lost = !pos_ok || !dir_ok;
}

void on_enter_tile(int from, int to) {
  if (g_blue_stop_start >= 0 && from >= 0) g_blue_stop_start = -1;
  if (to < 0) return;
  g_visits[to]++;
  const TileInfo& t = world.tiles[to];
  if (t.type == T_BLACK) {
    g_holes++;
    start_lop("drove onto the black tile " + tile_name(to) + " (a hole)", "fell_in_hole");
    return;
  }
  if (t.type == T_SILVER && !g_cp_visited.count(to)) {
    g_cp_visited.insert(to);
    event("checkpoint", "reached checkpoint " + tile_name(to));
  }
  if (t.type == T_SILVER) g_last_cp = to;
  if (t.type == T_BLUE) g_blue_visits[to]++;
  if (t.feature == F_BUMP) g_bump_tiles.insert(to);
  if (t.feature == F_STAIRS) g_stair_tiles.insert(to);
  if (from >= 0 && world.tiles[from].feature == F_RAMP) {
    const Ramp& r = world.ramps[world.tiles[from].ramp];
    if (to == r.high_tile) {
      g_ramp_up++;
      event("ramp", "drove up the ramp to " + tile_name(to));
    } else if (to == r.low_tile) {
      g_ramp_down++;
      event("ramp", "drove down the ramp to " + tile_name(to));
    }
  }
}

void identify_claim(char letter) {
  int best = -1;
  double bd = 1e9;
  for (int i = 0; i < (int)world.victims.size(); i++) {
    const Victim& v = world.victims[i];
    double d = std::hypot(v.wx - robot.x, v.wy - robot.y);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  if (best < 0 || bd > cfg.num("score.identify_distance_mm", 300)) {
    event("victim", std::string("robot reported victim '") + letter + "' but there is no victim within reach");
    return;
  }
  VictimState& s = g_vs[best];
  const Victim& v = world.victims[best];
  bool blinked = g_led_on_count >= 5;
  if (s.identified) {
    event("victim", std::string("robot reported victim '") + letter + "' again at " + tile_name(world.idx(v.x, v.y)) +
                        " (already counted)");
    return;
  }
  s.claimed = letter;
  s.t = now_s();
  bool correct = letter == v.type;
  if (correct && blinked) {
    s.identified = true;
    event("victim", std::string("correctly identified victim ") + v.type + " at " + tile_name(world.idx(v.x, v.y)) +
                        (v.floating ? " (floating wall)" : ""));
  } else {
    s.misidentified = true;
    event("victim", std::string("misidentified victim at ") + tile_name(world.idx(v.x, v.y)) + ": robot said '" +
                        letter + "', it is '" + v.type + "'" + (blinked ? "" : " (LED did not blink 5 times)"));
  }
  g_last_identified = best;
  g_last_identified_t = now_s();
  g_led_on_count = 0;
}

void poll_lcd() {
  std::string s = g_lcd_line[0];
  if (!g_lcd_line[1].empty()) s += " | " + g_lcd_line[1];
  if (s == g_lcd_last) return;
  g_lcd_last = s;
  if (s.empty()) return;
  g_lcd.push_back({now_s(), s});
  event("lcd", "LCD: " + s);
  if (s.rfind("victim: ", 0) == 0 && s.size() >= 9) identify_claim(s[8]);
  if (s.find("back to start") != std::string::npos) g_claimed_home = true;
}

void record_frame() {
  Frame f;
  SimProbe p;
  sim_probe_read(&p);
  f.t = (float)now_s();
  f.x = (float)robot.x;
  f.y = (float)robot.y;
  f.h = (float)((robot.th) / DEG);
  f.z = (float)robot.z;
  f.p = (float)(robot.pitch / DEG);
  f.st = p.state;
  f.bx = p.x;
  f.by = p.y;
  f.bd = p.dir;
  f.bf = p.floor;
  f.it = p.iterator;
  f.steps = p.steps;
  f.enc = (int)((p.enc_a + p.enc_b + p.enc_d) / 3);
  f.fl = (p.fwd_active ? 1 : 0) | (p.turn_active ? 2 : 0) | (p.victim_pending ? 4 : 0) | (p.pause ? 8 : 0) |
         (pin_level(g_led_pin) ? 16 : 0) | (robot.contact ? 32 : 0) | (robot.held ? 64 : 0) | (p.black_toggle ? 128 : 0);
  int tx, ty;
  f.col = robot.tile_under(robot.color_rx, robot.color_ry, &tx, &ty);
  for (int i = 0; i < 8; i++) f.tof[i] = -1;
  for (auto& d : robot.tofs)
    if (d.logical >= 1 && d.logical <= 8) f.tof[d.logical - 1] = d.ranging ? d.latest : -1;
  for (int m = 0; m < 4; m++) f.mot[m] = robot.signed_pwm(m);
  g_frames.push_back(f);

  if (g_medkits_last == INT_MIN) g_medkits_last = p.medkits;
  if (p.medkits < g_medkits_last) {
    int used = g_medkits_last - p.medkits;
    g_kits_total += used;
    if (g_last_identified >= 0 && now_s() - g_last_identified_t < 20) g_vs[g_last_identified].kits += used;
    event("kit", "dropped " + std::to_string(used) + " rescue kit(s)");
  }
  g_medkits_last = p.medkits;

  // robot map snapshot when it changes
  uint64_t h = 1469598103934665603ull ^ (uint64_t)p.floor;
  int n = p.map_size;
  for (int x = 0; x < n; x++)
    for (int y = 0; y < n; y++) {
      uint32_t b = sim_probe_tile(0, x, y);
      if (b) h = (h ^ (b + (uint64_t)(x * n + y) * 0x9E3779B97F4A7C15ull)) * 1099511628211ull;
    }
  if (h != g_map_hash) {
    g_map_hash = h;
    MapSnap s;
    s.t = now_s();
    s.floor = p.floor;
    for (int x = 0; x < n; x++)
      for (int y = 0; y < n; y++) {
        uint32_t b = sim_probe_tile(0, x, y);
        if (b) s.tiles.push_back({x, y, (int)b});
      }
    g_maps.push_back(std::move(s));
  }
}

void apply_moves_limit() {
  if (opt.moves_limit == "code") return;
  int code_limit = sim_probe_code_moves_limit();
  if (code_limit <= 0) return;
  int it = sim_probe_get_iterator();
  if (it == g_iter_seen) return;
  if (it > g_iter_seen) g_moves_counted += it - g_iter_seen;
  g_iter_seen = it;
  if (it >= code_limit) return;  // the robot code already switched to RETURN
  long limit = opt.moves_limit == "off" ? LONG_MAX : atol(opt.moves_limit.c_str());
  int set = (g_moves_counted >= limit - 1) ? code_limit - 1 : 0;
  sim_probe_set_iterator(set);
  g_iter_seen = set;
}

}  // namespace

// ------------------------------------------------------------------ hooks from hal.cpp
void rec_serial_char(char c) {
  if (c == '\r') return;
  if (c != '\n') {
    if (g_serial_cur.size() < 400) g_serial_cur += c;
    return;
  }
  double t = sched_now() / 1e6;
  if (opt.echo_serial) printf("[%8.3f] %s\n", t, g_serial_cur.c_str());
  if (g_serial.size() < g_serial_cap)
    g_serial.push_back({t, g_serial_cur});
  else
    g_serial_truncated = true;
  g_serial_cur.clear();
}

void rec_lcd_changed(const std::string&, const std::string&) {}

void rec_led(int pin, int level) {
  if (pin != g_led_pin || !level) return;
  double t = sched_now() / 1e6;
  if (t - g_led_last_on > 3) g_led_on_count = 0;
  g_led_on_count++;
  g_led_last_on = t;
}

void rec_event(const std::string& kind, const std::string& msg) {
  if (kind == "hang") g_hang_reason = msg;
  g_events.push_back(EventRec{sched_now() / 1e6, kind, msg, robot.x, robot.y});
}

void rec_warning(const std::string& key, const std::string& msg) {
  auto it = g_warnings.find(key);
  if (it == g_warnings.end()) {
    g_warnings[key] = {msg, 1};
    g_warning_order.push_back(key);
    g_events.push_back(EventRec{sched_now() / 1e6, "warning", msg, robot.x, robot.y});
  } else {
    it->second.second++;
  }
}

void rec_stepper(int steps) { g_stepper_steps += std::abs(steps); }

void rec_setup_done() {
  g_setup_done = sched_now() / 1e6;
  g_events.push_back(EventRec{g_setup_done, "setup", "setup() finished", robot.x, robot.y});
  g_last_state = -1;  // the first SENSE_TILE after setup counts as an arrival
}

void rec_init(const std::string& trace_path, const std::string& result_path) {
  g_trace_path = trace_path;
  g_result_path = result_path;
  g_frame_us = (int64_t)(opt.frame_dt * 1e6);
  if (g_frame_us < 1000) g_frame_us = 1000;
  g_visits.assign(world.W * world.H, 0);
  g_vs.assign(world.victims.size(), VictimState());
  g_switch_pin = cfg.integer("pins.pause_switch", 22);
  g_led_pin = cfg.integer("pins.victim_led", 51);
  g_serial_cap = (size_t)cfg.integer("sim.max_serial_lines", 150000);
  g_progress_x = robot.x;
  g_progress_y = robot.y;
  g_progress_th = robot.th;
  for (int i = 0; i < sim_probe_state_count(); i++)
    if (std::string(sim_probe_state_name(i)) == "RETURN") g_state_return = i;
  for (const auto& w : world.warnings) rec_warning("maze:" + w, "maze file: " + w);
}

void rec_on_tick(int64_t t_us) {
  double t = t_us / 1e6;
  apply_moves_limit();
  int st = sim_probe_get_state();
  if (st != g_last_state) {
    if (g_setup_done >= 0 && std::string(state_name(st)) == "SENSE_TILE") belief_check();
    g_last_state = st;
  }
  int tx, ty;
  int ti = world.tile_of(robot.x, robot.y, &tx, &ty) ? world.idx(tx, ty) : -1;
  if (ti != g_cur_tile) {
    int from = g_cur_tile;
    g_cur_tile = ti;
    if (!robot.held) on_enter_tile(from, ti);
  }
  bool motors = robot.moving_motors();
  if (ti >= 0 && world.tiles[ti].type == T_BLUE) {
    bool still = !motors && std::fabs(robot.vL) + std::fabs(robot.vR) < 2;
    if (still && g_blue_stop_start < 0) g_blue_stop_start = t;
    if (!still) g_blue_stop_start = -1;
    if (g_blue_stop_start >= 0) {
      double d = t - g_blue_stop_start;
      if (d > g_blue_best_stop[ti]) g_blue_best_stop[ti] = d;
    }
  }
  if (robot.contact && !g_last_contact && t - g_last_contact_end > 0.5)
    event("contact", "robot touched a wall or obstacle");
  if (!robot.contact && g_last_contact) g_last_contact_end = t;
  g_last_contact = robot.contact;

  if (g_lop_state) lop_update();
  if (t_us >= g_next_lcd_poll) {
    g_next_lcd_poll = t_us + 10000;
    poll_lcd();
  }
  if (t_us >= g_next_frame) {
    g_next_frame += g_frame_us;
    record_frame();
  }

  if (!g_lop_state) {
    if (motors) g_last_motor = t;
    double moved = std::hypot(robot.x - g_progress_x, robot.y - g_progress_y);
    double turned = std::fabs(robot.th - g_progress_th) / DEG;
    if (moved > 30 || turned > 20) {
      g_progress_t = t;
      g_progress_x = robot.x;
      g_progress_y = robot.y;
      g_progress_th = robot.th;
    }
    if (!g_hang_reason.empty() && t - g_last_motor > 5) finish("hung", g_hang_reason);
    if (st == g_state_return && !motors && t - g_last_motor > 8) finish(g_claimed_home ? "returned" : "stopped", "");
    if (!motors && t - g_last_motor > opt.idle_s)
      finish("idle", std::string("motors have been off for ") + std::to_string((int)opt.idle_s) + " s in state " +
                         state_name(st));
    if (motors && t - g_progress_t > opt.stuck_s)
      start_lop("no progress for " + std::to_string((int)opt.stuck_s) + " s (stuck)", "stuck");
  }
  if (t >= opt.time_limit_s) finish("time_limit", "the run time limit was reached");
}

void world_advance(int64_t to_us) {
  while (g_phys_t + 1000 <= to_us) {
    g_phys_t += 1000;
    robot.physics_step(0.001, g_phys_t);
    robot.update_tofs(g_phys_t);
    robot.update_cameras(g_phys_t);
    rec_on_tick(g_phys_t);
  }
}

// ------------------------------------------------------------------ results
namespace {

struct MapScore {
  int tiles = 0, walls_ok = 0, walls_missing = 0, walls_phantom = 0, outside = 0;
  int black_ok = 0, black_wrong = 0, cp_ok = 0, cp_wrong = 0, blue_ok = 0, blue_wrong = 0;
  int victim_marks_ok = 0, victim_marks_wrong = 0;
  std::vector<std::string> notes;
};

MapScore score_map(const SimProbe& p) {
  MapScore s;
  int n = p.map_size;
  for (int f = 0; f < 3; f++) {
    int grid = (f == p.floor) ? 0 : f + 1;
    for (int bx = 0; bx < n; bx++)
      for (int by = 0; by < n; by++) {
        uint32_t b = sim_probe_tile(grid, bx, by);
        if (!b) continue;
        int wx, wy;
        map_to_world(bx, by, n, &wx, &wy);
        bool known = b & ((1u << SIMT_VISITED) | (1u << SIMT_DISCOVERED));
        int type = (b >> SIMT_TYPE0) & 3;
        if (!world.in(wx, wy) || world.at(wx, wy).type == T_VOID) {
          if (known || type) s.outside++;
          continue;
        }
        const TileInfo& t = world.at(wx, wy);
        if (type == 3) (t.type == T_BLACK ? s.black_ok : s.black_wrong)++;
        if (type == 2) (t.type == T_SILVER ? s.cp_ok : s.cp_wrong)++;
        if (type == 1) (t.type == T_BLUE ? s.blue_ok : s.blue_wrong)++;
        if (b & (1u << SIMT_VICTIM)) {
          bool any = false;
          for (auto& v : world.victims)
            if (v.x == wx && v.y == wy) any = true;
          (any ? s.victim_marks_ok : s.victim_marks_wrong)++;
        }
        if (!(b & (1u << SIMT_VISITED)) || t.feature == F_RAMP) continue;
        s.tiles++;
        for (int d = 0; d < 4; d++) {
          bool rw = b & (1u << (SIMT_WALL0 + d));
          int side = (d + world.start_dir) % 4;
          bool tw = world.wall(wx, wy, side);
          if (rw == tw)
            s.walls_ok++;
          else if (tw)
            s.walls_missing++;
          else
            s.walls_phantom++;
          if (rw != tw && s.notes.size() < 12) {
            static const char* dn = "NESW";
            s.notes.push_back(std::string(tw ? "missed wall" : "phantom wall") + " on the " + dn[side] + " side of " +
                              tile_name(world.idx(wx, wy)));
          }
        }
      }
  }
  return s;
}

void write_result_json(FILE* f, const std::string& outcome, const std::string& detail) {
  SimProbe p;
  sim_probe_read(&p);
  double t = now_s();
  int visited = 0;
  for (int i = 0; i < world.W * world.H; i++)
    if (g_visits[i] > 0 && world.tiles[i].reachable && world.tiles[i].feature != F_RAMP) visited++;
  MapScore ms = score_map(p);

  // where the robot ended up
  int tx, ty;
  bool in = world.tile_of(robot.x, robot.y, &tx, &ty);
  bool on_start = in && tx == world.start_x && ty == world.start_y;
  bool fully_inside = false;
  if (on_start) {
    double cx = (tx + 0.5) * world.tile, cy = (ty + 0.5) * world.tile, half = world.tile / 2;
    fully_inside = true;
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2) {
        double px, py;
        robot.sensor_pose(sx * robot.half_wid, sy * robot.half_len, &px, &py);
        if (std::fabs(px - cx) > half || std::fabs(py - cy) > half) fully_inside = false;
      }
  }
  bool returned_home = (outcome == "returned" || outcome == "stopped" || outcome == "idle") && on_start;

  // score (approximate RCJ Rescue Maze rules, values in the config file)
  int victims_ok = 0, misid = 0, vpoints = 0, kitpoints = 0, floating_ok = 0;
  for (size_t i = 0; i < world.victims.size(); i++) {
    const Victim& v = world.victims[i];
    const VictimState& s = g_vs[i];
    if (s.identified) {
      victims_ok++;
      if (v.floating) floating_ok++;
      vpoints += (int)cfg.num(v.floating ? "score.victim_floating" : "score.victim_linear", v.floating ? 30 : 10);
      int need = cfg.integer(std::string("score.kits_") + v.type, v.type == 'H' ? 2 : (v.type == 'S' || v.type == 'R' || v.type == 'Y') ? 1 : 0);
      kitpoints += std::min(need, s.kits) * cfg.integer("score.kit", 10);
    }
    if (s.misidentified) misid++;
  }
  int misid_points = misid * cfg.integer("score.misidentification", -5);
  int cp_points = (int)g_cp_visited.size() * cfg.integer("score.checkpoint", 10);
  int blue_points = 0;
  for (auto& kv : g_blue_visits)
    blue_points += std::max(0, cfg.integer("score.blue_first", 30) - cfg.integer("score.blue_revisit_penalty", 10) * (kv.second - 1));
  int bump_points = (int)g_bump_tiles.size() * cfg.integer("score.speed_bump", 5);
  int stair_points = (int)g_stair_tiles.size() * cfg.integer("score.stairs", 5);
  int ramp_points = (g_ramp_up + g_ramp_down) * cfg.integer("score.ramp", 10);
  int exit_points = (returned_home && fully_inside) ? victims_ok * cfg.integer("score.exit_per_victim", 10) : 0;
  int lop_points = g_lops * cfg.integer("score.lop", 0);
  int total = vpoints + kitpoints + misid_points + cp_points + blue_points + bump_points + stair_points + ramp_points +
              exit_points + lop_points;

  fprintf(f, "{\"outcome\":\"%s\",\"detail\":\"%s\",\"maze\":\"%s\",\"seed\":%llu,\"sim_time_s\":%.2f,",
          outcome.c_str(), jesc(detail).c_str(), jesc(world.name).c_str(), (unsigned long long)opt.seed, t);
  fprintf(f, "\"setup_s\":%.2f,\"tiles_reachable\":%d,\"tiles_visited\":%d,\"coverage\":%.3f,", g_setup_done, world.reachable_count,
          visited, world.reachable_count ? (double)visited / world.reachable_count : 0.0);
  fprintf(f, "\"moves\":%d,\"distance_m\":%.2f,", g_moves_counted ? g_moves_counted : p.iterator, robot.odometer_mm / 1000);
  fprintf(f, "\"position_checks\":%d,\"position_errors\":%d,\"heading_errors\":%d,\"first_lost_s\":%s,", g_checks,
          g_pos_errors, g_dir_errors, g_first_lost < 0 ? "null" : std::to_string(g_first_lost).c_str());
  fprintf(f, "\"center_err_mm\":{\"mean\":%.1f,\"max\":%.1f},\"heading_err_deg\":{\"mean\":%.1f,\"max\":%.1f},",
          g_checks ? g_center_err_sum / g_checks : 0.0, g_center_err_max, g_checks ? g_head_err_sum / g_checks : 0.0,
          g_head_err_max);
  fprintf(f,
          "\"map\":{\"tiles\":%d,\"walls_correct\":%d,\"walls_missed\":%d,\"walls_phantom\":%d,\"tiles_outside_maze\":%d,"
          "\"black_correct\":%d,\"black_wrong\":%d,\"checkpoint_correct\":%d,\"checkpoint_wrong\":%d,\"blue_correct\":%d,"
          "\"blue_wrong\":%d,\"victim_marks_correct\":%d,\"victim_marks_wrong\":%d,\"notes\":[",
          ms.tiles, ms.walls_ok, ms.walls_missing, ms.walls_phantom, ms.outside, ms.black_ok, ms.black_wrong, ms.cp_ok,
          ms.cp_wrong, ms.blue_ok, ms.blue_wrong, ms.victim_marks_ok, ms.victim_marks_wrong);
  for (size_t i = 0; i < ms.notes.size(); i++) fprintf(f, "%s\"%s\"", i ? "," : "", jesc(ms.notes[i]).c_str());
  fprintf(f, "]},");
  fprintf(f, "\"holes_entered\":%d,\"lack_of_progress\":%d,\"contacts\":%d,\"contact_time_s\":%.2f,", g_holes, g_lops,
          robot.contacts, robot.contact_time);
  int total_cp = 0, total_blue = 0;
  for (auto& tt : world.tiles) {
    if (tt.type == T_SILVER && tt.reachable) total_cp++;
    if (tt.type == T_BLUE && tt.reachable) total_blue++;
  }
  fprintf(f, "\"checkpoints\":{\"total\":%d,\"visited\":%d},\"blue_tiles\":{\"total\":%d,\"visited\":%d,\"stops\":[",
          total_cp, (int)g_cp_visited.size(), total_blue, (int)g_blue_visits.size());
  bool first = true;
  for (auto& kv : g_blue_best_stop) {
    fprintf(f, "%s{\"tile\":\"%s\",\"longest_stop_s\":%.1f}", first ? "" : ",", tile_name(kv.first).c_str(), kv.second);
    first = false;
  }
  fprintf(f, "]},\"ramps\":{\"up\":%d,\"down\":%d},", g_ramp_up, g_ramp_down);
  fprintf(f, "\"victims\":{\"total\":%d,\"floating\":%d,\"identified\":%d,\"identified_floating\":%d,\"misidentified\":%d,\"kits_dropped\":%d,\"list\":[",
          (int)world.victims.size(), (int)std::count_if(world.victims.begin(), world.victims.end(), [](const Victim& v) { return v.floating; }),
          victims_ok, floating_ok, misid, g_kits_total);
  for (size_t i = 0; i < world.victims.size(); i++) {
    const Victim& v = world.victims[i];
    const VictimState& s = g_vs[i];
    fprintf(f, "%s{\"tile\":\"%s\",\"side\":\"%c\",\"type\":\"%c\",\"floating\":%s,\"identified\":%s,\"claimed\":\"%s\",\"kits\":%d}",
            i ? "," : "", tile_name(world.idx(v.x, v.y)).c_str(), "NESW"[v.side], v.type, v.floating ? "true" : "false",
            s.identified ? "true" : "false", s.claimed ? std::string(1, s.claimed).c_str() : "", s.kits);
  }
  fprintf(f, "]},");
  fprintf(f, "\"end\":{\"on_start_tile\":%s,\"fully_inside_start_tile\":%s,\"robot_said_home\":%s,\"returned_home\":%s,\"state\":\"%s\"},",
          on_start ? "true" : "false", fully_inside ? "true" : "false", g_claimed_home ? "true" : "false",
          returned_home ? "true" : "false", state_name(p.state));
  fprintf(f,
          "\"score\":{\"total\":%d,\"victims\":%d,\"rescue_kits\":%d,\"misidentification\":%d,\"checkpoints\":%d,"
          "\"blue_tiles\":%d,\"speed_bumps\":%d,\"stairs\":%d,\"ramps\":%d,\"exit_bonus\":%d,\"lack_of_progress\":%d},",
          total, vpoints, kitpoints, misid_points, cp_points, blue_points, bump_points, stair_points, ramp_points,
          exit_points, lop_points);
  fprintf(f, "\"warnings\":[");
  for (size_t i = 0; i < g_warning_order.size(); i++) {
    auto& w = g_warnings[g_warning_order[i]];
    fprintf(f, "%s{\"key\":\"%s\",\"message\":\"%s\",\"count\":%d}", i ? "," : "", jesc(g_warning_order[i]).c_str(),
            jesc(w.first).c_str(), w.second);
  }
  fprintf(f, "],\"options\":{\"boot\":\"%s\",\"moves_limit\":\"%s\",\"lop\":%s,\"time_limit_s\":%.0f}}", opt.boot.c_str(),
          opt.moves_limit.c_str(), opt.lop ? "true" : "false", opt.time_limit_s);
}

void write_trace(FILE* f, const std::string& outcome, const std::string& detail) {
  fprintf(f, "{\"format\":\"theseus-sim-trace\",\"version\":1,\"maze\":%s,", world.to_json().c_str());
  fprintf(f, "\"robot\":{\"length\":%.0f,\"width\":%.0f,\"color\":[%.0f,%.0f],\"tof\":[", robot.half_len * 2,
          robot.half_wid * 2, robot.color_rx, robot.color_ry);
  for (size_t i = 0; i < robot.tofs.size(); i++) {
    const TofDevice& d = robot.tofs[i];
    fprintf(f, "%s{\"n\":%d,\"x\":%.0f,\"y\":%.0f,\"a\":%.0f,\"port\":%d}", i ? "," : "", d.logical, d.rx, d.ry, d.yaw,
            d.port);
  }
  fprintf(f, "],\"cameras\":[");
  for (size_t i = 0; i < robot.cameras.size(); i++) {
    const Camera& c = robot.cameras[i];
    fprintf(f, "%s{\"x\":%.0f,\"y\":%.0f,\"a\":%.0f,\"fov\":%.0f,\"range\":%.0f}", i ? "," : "", c.rx, c.ry, c.yaw,
            cfg.num("camera.fov_deg", 60), cfg.num("camera.range_mm", 260));
  }
  SimProbe p;
  sim_probe_read(&p);
  fprintf(f, "],\"map_size\":%d},\"states\":[", p.map_size);
  for (int i = 0; i < sim_probe_state_count(); i++) fprintf(f, "%s\"%s\"", i ? "," : "", sim_probe_state_name(i));
  fprintf(f, "],\"steps_names\":[");
  for (int i = 0; i < 8; i++) {
    const char* n = sim_probe_steps_name(i);
    if (std::string(n) == "?") break;
    fprintf(f, "%s\"%s\"", i ? "," : "", n);
  }
  fprintf(f,
          "],\"frame_fields\":[\"t\",\"x\",\"y\",\"h\",\"z\",\"pitch\",\"state\",\"bx\",\"by\",\"bdir\",\"bfloor\","
          "\"iter\",\"flags\",\"steps\",\"enc\",\"color\",\"tof1\",\"tof2\",\"tof3\",\"tof4\",\"tof5\",\"tof6\",\"tof7\","
          "\"m1\",\"m2\",\"m3\",\"m4\"],\"frames\":[");
  for (size_t i = 0; i < g_frames.size(); i++) {
    const Frame& r = g_frames[i];
    fprintf(f, "%s[%.2f,%.1f,%.1f,%.1f,%.0f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", i ? ",\n" : "", r.t, r.x, r.y, r.h, r.z,
            r.p, r.st, r.bx, r.by, r.bd, r.bf, r.it, r.fl, r.steps, r.enc, r.col);
    for (int k = 0; k < 7; k++) fprintf(f, ",%d", r.tof[k]);
    for (int k = 0; k < 4; k++) fprintf(f, ",%d", r.mot[k]);
    fprintf(f, "]");
  }
  fprintf(f, "],\"maps\":[");
  for (size_t i = 0; i < g_maps.size(); i++) {
    const MapSnap& s = g_maps[i];
    fprintf(f, "%s{\"t\":%.2f,\"floor\":%d,\"tiles\":[", i ? ",\n" : "", s.t, s.floor);
    for (size_t k = 0; k < s.tiles.size(); k++)
      fprintf(f, "%s[%d,%d,%d]", k ? "," : "", s.tiles[k][0], s.tiles[k][1], s.tiles[k][2]);
    fprintf(f, "]}");
  }
  fprintf(f, "],\"events\":[");
  for (size_t i = 0; i < g_events.size(); i++) {
    const EventRec& e = g_events[i];
    fprintf(f, "%s{\"t\":%.3f,\"k\":\"%s\",\"m\":\"%s\",\"x\":%.0f,\"y\":%.0f}", i ? ",\n" : "", e.t, jesc(e.kind).c_str(),
            jesc(e.msg).c_str(), e.x, e.y);
  }
  fprintf(f, "],\"serial_truncated\":%s,\"serial\":[", g_serial_truncated ? "true" : "false");
  for (size_t i = 0; i < g_serial.size(); i++)
    fprintf(f, "%s[%.3f,\"%s\"]", i ? ",\n" : "", g_serial[i].first, jesc(g_serial[i].second).c_str());
  fprintf(f, "],\"result\":");
  write_result_json(f, outcome, detail);
  fprintf(f, "}\n");
}

}  // namespace

void finish(const std::string& outcome, const std::string& detail) {
  static std::atomic<bool> once{false};
  if (once.exchange(true)) {
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (!g_serial_cur.empty()) rec_serial_char('\n');
  if (!detail.empty()) g_events.push_back(EventRec{now_s(), "end", outcome + ": " + detail, robot.x, robot.y});
  if (!g_result_path.empty()) {
    FILE* f = fopen(g_result_path.c_str(), "w");
    if (f) {
      write_result_json(f, outcome, detail);
      fprintf(f, "\n");
      fclose(f);
    }
  }
  if (!g_trace_path.empty()) {
    FILE* f = fopen(g_trace_path.c_str(), "w");
    if (f) {
      write_trace(f, outcome, detail);
      fclose(f);
    }
  }
  if (!opt.quiet) {
    printf("\n=== run finished: %s after %.1f s (simulated) ===\n", outcome.c_str(), now_s());
    if (!detail.empty()) printf("%s\n", detail.c_str());
    write_result_json(stdout, outcome, detail);
    printf("\n");
  }
  fflush(stdout);
  fflush(stderr);
  std::_Exit(0);
}

}  // namespace sim
