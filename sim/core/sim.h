// Shared declarations of the Theseus maze simulator core.
#pragma once
#include <stdint.h>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace sim {

// ------------------------------------------------------------------ config
class Config {
 public:
  bool load_file(const std::string& path, std::string* err);
  void set(const std::string& key, const std::string& value) { kv_[key] = value; }
  bool has(const std::string& key) const { return kv_.count(key) != 0; }
  double num(const std::string& key, double def) const;
  int integer(const std::string& key, int def) const { return (int)std::lround(num(key, def)); }
  std::string str(const std::string& key, const std::string& def) const;
  std::vector<double> nums(const std::string& key) const;
  bool flag(const std::string& key, bool def) const;
  const std::map<std::string, std::string>& all() const { return kv_; }

 private:
  std::map<std::string, std::string> kv_;
};
extern Config cfg;

// ------------------------------------------------------------------ random numbers
// Small portable generator so a seed gives the same run on every computer.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 1) : s(seed * 0x9E3779B97F4A7C15ull + 0x632BE59BD9B4E019ull) {}
  uint64_t next() {
    uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  double uniform() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }
  double uniform(double a, double b) { return a + (b - a) * uniform(); }
  double normal() {
    double u1 = uniform();
    if (u1 < 1e-300) u1 = 1e-300;
    double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
  }
  bool chance(double p) { return uniform() < p; }
};

// ------------------------------------------------------------------ scheduler (virtual time + RTOS threads)
void sched_init_main(const char* name);
int sched_spawn(void (*fn)(), int prio, const char* name);
void sched_set_priority(int id, int prio);
int sched_get_priority(int id);
int sched_current();
const char* sched_thread_name(int id);
void sched_busy(int64_t us);   // the running thread uses the CPU for `us`
void sched_sleep(int64_t us);  // the running thread sleeps (other threads may run)
void sched_sync();             // commit pending CPU time so the world is up to date
int64_t sched_now();           // virtual time in microseconds
int64_t sched_committed_now();
bool sched_started();
bool sched_in_hook();
int sched_mutex_create(const char* name);
void sched_mutex_lock(int id);
bool sched_mutex_trylock(int id);
void sched_mutex_unlock(int id);
void sched_set_world_hook(void (*advance)(int64_t to_us));
void sched_start_watchdog(double wall_seconds);
std::string sched_describe();

// ------------------------------------------------------------------ maze world
enum TileType { T_WHITE = 0, T_BLACK = 1, T_BLUE = 2, T_SILVER = 3, T_VOID = 4 };
enum Feature { F_NONE = 0, F_RAMP = 1, F_STAIRS = 2, F_BUMP = 3, F_DEBRIS = 4 };
enum Dir { DN = 0, DE = 1, DS = 2, DW = 3 };
inline int dir_dx(int d) { return d == DE ? 1 : d == DW ? -1 : 0; }
inline int dir_dy(int d) { return d == DN ? 1 : d == DS ? -1 : 0; }

struct TileInfo {
  TileType type = T_WHITE;
  Feature feature = F_NONE;
  bool start = false;
  int ramp = -1;       // index into World::ramps
  int axis = 0;        // travel axis for bumps/stairs: 0 = north-south, 1 = east-west
  double z = 0;        // floor height (mm); for ramp tiles the height at the low edge
  double zmin = 0, zmax = 0;
  bool reachable = false;
};

struct Victim {
  int x, y, side;  // tile and the wall of that tile the victim is on
  char type;       // H S U (letters) or R Y G (colours)
  bool floating = false;
  double wx = 0, wy = 0, wz = 0;  // world position of the victim (on the wall face)
};

struct Obstacle {
  int kind = 0;  // 0 = cylinder, 1 = box
  double x = 0, y = 0, r = 40, w = 0, h = 0, height = 200, z = 0;
  int tx = 0, ty = 0;
};

struct Ramp {
  std::vector<int> tiles;  // tile indices from low end to high end
  int dir = 0;             // direction of travel going up
  double angle_deg = 20;
  double z_low = 0, z_high = 0;
  int low_tile = -1, high_tile = -1;  // flat tiles at both ends
  double s0 = 0;                      // world coordinate (along dir) of the low edge
};

struct Box {
  double x0, y0, x1, y1, z0, z1;
};

class World {
 public:
  int W = 0, H = 0;
  double tile = 300, wall_t = 12, wall_h = 150;
  std::string name;
  std::vector<TileInfo> tiles;  // index y*W + x, y = 0 is the south row
  std::vector<uint8_t> hwall;   // (H+1)*W, hwall[j*W+x]: horizontal wall on line y=j*tile
  std::vector<uint8_t> vwall;   // H*(W+1), vwall[y*(W+1)+i]: vertical wall on line x=i*tile
  std::vector<Victim> victims;
  std::vector<Obstacle> obstacles;
  std::vector<Ramp> ramps;
  std::vector<Box> boxes;  // wall + void geometry used for collisions and ray casting
  int start_x = 0, start_y = 0, start_dir = 0;
  int reachable_count = 0;
  std::vector<std::string> warnings;
  std::string source_text;

  bool load(const std::string& path, std::string* err);
  bool parse(const std::string& text, std::string* err);
  bool in(int x, int y) const { return x >= 0 && y >= 0 && x < W && y < H; }
  int idx(int x, int y) const { return y * W + x; }
  const TileInfo& at(int x, int y) const { return tiles[idx(x, y)]; }
  bool wall(int x, int y, int side) const;  // wall on `side` of tile (x,y); outside the grid counts as wall
  bool tile_of(double x, double y, int* tx, int* ty) const;
  double floor_z(double x, double y) const;
  // Horizontal ray from (ox,oy,oz) in direction (dx,dy) (unit vector) that rises `slope` mm per mm.
  // Returns the 3D distance to the first surface hit, or -1 if nothing within max_h horizontal mm.
  // If cos_inc is given it receives the cosine of the angle between the beam and the surface normal.
  double raycast(double ox, double oy, double oz, double dx, double dy, double slope, double max_h,
                 double* cos_inc = nullptr) const;
  bool body_collides(double x, double y, double th, double half_len, double half_wid, double zlo, double zhi) const;
  std::string to_json() const;

 private:
  bool finish_build(std::string* err);
  void build_boxes();
  void compute_heights(std::vector<std::string>* errs);
  void compute_floating();
};
extern World world;

// ------------------------------------------------------------------ robot hardware + physics
struct TofDevice {
  int port = 0;          // mux port
  int logical = 0;       // sensor number used in the code (measure(n))
  uint8_t address = 0x29;
  bool initialized = false, ranging = false;
  double rx = 0, ry = 0, yaw = 0;  // mount in robot frame (mm, deg; 0 = forward, 90 = right)
  double bias = 0;
  int64_t next_ready = 0;
  int64_t period = 33000;
  int latest = 8190;
  bool unread = false;
  int64_t reads = 0;
};

struct EncoderModel {
  int motor = 0;  // motor index 0..3
  int pin_a = 0, pin_b = 0;
  int forward_level = 1;
  double accum = 0;
};

struct Camera {
  int serial = 0;  // Serial index the camera talks to
  double rx = 0, ry = 0, yaw = 0;
  int64_t next_frame = 0;
  int64_t period = 50000;
  int visible_victim = -1;
};

class Robot {
 public:
  // pose (world, mm / rad; heading clockwise from north)
  double x = 0, y = 0, th = 0, z = 0, pitch = 0;
  double vL = 0, vR = 0;
  double th0 = 0;  // heading at power-on (gyro zero)
  bool held = false;  // picked up by the referee (lack of progress)
  bool contact = false;
  double contact_time = 0;
  int contacts = 0;              // separate contact episodes (touches more than 0.5 s apart)
  double last_contact_t = -10;   // seconds
  double odometer_mm = 0;

  // motor shield
  uint8_t mdir[4] = {4, 4, 4, 4};
  uint8_t mspeed[4] = {0, 0, 0, 0};
  bool shield_begun = false;
  int msign[4] = {1, 1, 1, -1};
  int mside[4] = {0, 1, 0, 1};  // 0 = left, 1 = right
  double mgain[4] = {1, 1, 1, 1};
  int64_t speed_wraps = 0;

  // geometry
  double half_len = 85, half_wid = 70, wheelbase = 120, track = 130;

  std::vector<TofDevice> tofs;
  std::vector<EncoderModel> encoders;
  std::vector<Camera> cameras;
  int mux_mask = 0;
  bool mux_present = true;
  uint8_t tcs_address = 0x29;
  int tcs_port = 7;
  double color_rx = 0, color_ry = 70;

  void init();
  void physics_step(double dt, int64_t t_us);
  void update_tofs(int64_t t_us);
  int tof_measure(const TofDevice& d);
  TofDevice* tof_on_bus(uint8_t address);
  bool i2c_present(uint8_t address);
  void sensor_pose(double rx, double ry, double* wx, double* wy) const;
  int tile_under(double rx, double ry, int* tx, int* ty) const;
  double gyro_heading_deg(int64_t t_us);
  double gyro_pitch_deg();
  void color_raw(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c, uint8_t it, int gain);
  void update_cameras(int64_t t_us);
  int victim_visible(const Camera& cam) const;
  void teleport(double nx, double ny, double nth);
  bool moving_motors() const;
  int signed_pwm(int m) const;

 private:
  Rng rng_noise_{11}, rng_tof_{12}, rng_cam_{13};
  double gyro_drift_ = 0, gyro_drift_rate_ = 0;
  double fixed_yaw_bias_ = 0;
  void refresh_height();
};
extern Robot robot;

// pins, serial buffers, isr table (implemented in hal.cpp)
void hal_init();
int pin_level(int pin);
void pin_set_input(int pin, int level);
void serial_rx_push(int serial_index, uint8_t byte);
int serial_rx_size(int serial_index);
void isr_fire(int pin);
bool isr_attached(int pin);
extern bool g_in_isr;
extern std::string g_lcd_line[4];

// ------------------------------------------------------------------ recorder / evaluation
void rec_init(const std::string& trace_path, const std::string& result_path);
void rec_on_tick(int64_t t_us);
void rec_serial_char(char c);
void rec_lcd_changed(const std::string& line0, const std::string& line1);
void rec_led(int pin, int level);
void rec_event(const std::string& kind, const std::string& msg);
void rec_warning(const std::string& key, const std::string& msg);
void rec_stepper(int steps);
void rec_setup_done();
[[noreturn]] void finish(const std::string& outcome, const std::string& detail);

// world hook: advances physics and recorder (called by the scheduler)
void world_advance(int64_t to_us);

struct Options {
  uint64_t seed = 1;
  double time_limit_s = 480;
  std::string boot = "warm";       // warm | cold
  std::string moves_limit = "code";  // code | off | <number>
  bool lop = true;
  int max_lops = 5;
  double frame_dt = 0.05;
  bool echo_serial = false;
  bool quiet = false;
  double stuck_s = 30;
  double idle_s = 60;
};
extern Options opt;

}  // namespace sim
