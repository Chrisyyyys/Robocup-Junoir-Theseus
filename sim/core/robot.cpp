// The simulated robot: drive train physics, encoders, distance sensors, gyro,
// colour sensor and cameras. All numbers come from the config file so they can
// be replaced with measurements from the real robot.
#include <algorithm>
#include <cstdio>
#include <sstream>

#include "sim.h"

namespace sim {

Robot robot;
static const double DEG = M_PI / 180.0;

// config values used every physics step (read once in init)
static struct {
  double slide, vmax, db_straight, db_turn, tau, skid, slip, yaw_noise, slope_k, blocked_spin, counts_per_mm,
      enc_noise, battery, tof_height, tof_noise, tof_noise_frac, tof_max, tof_min, tof_dropout, tof_spike,
      tof_cone, gyro_noise, gyro_offset, pitch_noise, pitch_sign, color_noise, cam_fov, cam_range, cam_height,
      cam_misread, debris_yaw, debris_enc;
  int tof_rays, tof_oor, cam_idle_byte;
  double cam_idle_rate;
} P;

void Robot::init() {
  half_len = cfg.num("robot.length_mm", 195) / 2;
  half_wid = cfg.num("robot.width_mm", 180) / 2;
  wheelbase = cfg.num("drive.wheelbase_mm", 120);
  track = cfg.num("drive.track_mm", 130);
  P.vmax = cfg.num("drive.max_speed_mm_s", 210);
  P.db_straight = cfg.num("drive.deadband_straight_pwm", 25);
  P.db_turn = cfg.num("drive.deadband_turn_pwm", 45);
  P.tau = std::max(0.005, cfg.num("drive.motor_tau_s", 0.08));
  P.skid = cfg.num("drive.skid_factor", 1.5);
  P.slip = cfg.num("drive.slip", 0.02);
  P.yaw_noise = cfg.num("drive.yaw_noise_deg_per_m", 1.5);
  P.slope_k = cfg.num("drive.slope_slowdown", 0.8);
  P.blocked_spin = cfg.num("drive.blocked_wheel_spin", 0.3);
  P.slide = cfg.num("drive.wall_slide_mm", 0);
  P.counts_per_mm = cfg.num("drive.encoder_counts_per_rev", 975) / (M_PI * cfg.num("drive.wheel_diameter_mm", 80));
  P.enc_noise = cfg.num("drive.encoder_noise", 0.01);
  P.battery = cfg.num("drive.battery_factor", 1.0);
  P.debris_yaw = cfg.num("field.debris_yaw_deg", 4);
  P.debris_enc = cfg.num("field.debris_encoder_noise", 0.08);
  P.tof_height = cfg.num("tof.height_mm", 40);
  P.tof_noise = cfg.num("tof.noise_mm", 3);
  P.tof_noise_frac = cfg.num("tof.noise_fraction", 0.01);
  P.tof_max = cfg.num("tof.max_range_mm", 1200);
  P.tof_min = cfg.num("tof.min_range_mm", 30);
  P.tof_dropout = cfg.num("tof.dropout_prob", 0.002);
  P.tof_spike = cfg.num("tof.spike_prob", 0.001);
  P.tof_cone = cfg.num("tof.cone_deg", 20);
  P.tof_rays = std::max(1, cfg.integer("tof.cone_rays", 5));
  P.tof_oor = cfg.integer("tof.out_of_range_value", 8190);
  P.gyro_noise = cfg.num("gyro.noise_deg", 0.1);
  P.gyro_offset = cfg.num("gyro.start_offset_deg", 0);
  P.pitch_noise = cfg.num("gyro.pitch_noise_deg", 0.3);
  P.pitch_sign = cfg.num("gyro.pitch_sign", 1);
  P.color_noise = cfg.num("color.noise", 0.03);
  P.cam_fov = cfg.num("camera.fov_deg", 60);
  P.cam_range = cfg.num("camera.range_mm", 260);
  P.cam_height = cfg.num("camera.height_mm", 70);
  P.cam_misread = cfg.num("camera.misread_prob", 0.03);
  std::string idle = cfg.str("camera.idle_byte", "");
  P.cam_idle_byte = idle.empty() ? -1 : (unsigned char)idle[0];
  P.cam_idle_rate = cfg.num("camera.idle_rate_hz", 0);

  uint64_t seed = opt.seed;
  rng_noise_ = Rng(seed * 7 + 1);
  rng_tof_ = Rng(seed * 7 + 2);
  rng_cam_ = Rng(seed * 7 + 3);
  Rng setup_rng(seed * 7 + 4);

  // motors: shield port -> side and mounting direction
  for (int m = 0; m < 4; m++) {
    std::string v = cfg.str("motor." + std::to_string(m + 1), "");
    std::istringstream ss(v);
    std::string side;
    int sign = 1;
    if (ss >> side >> sign) {
      mside[m] = side == "right" ? 1 : 0;
      msign[m] = sign < 0 ? -1 : 1;
    }
    mgain[m] = 1 + setup_rng.uniform(-1, 1) * cfg.num("drive.gain_spread", 0.04);
  }
  // encoders
  encoders.clear();
  for (const char* name : {"A", "B", "C", "D"}) {
    std::vector<double> v = cfg.nums(std::string("encoder.") + name);
    if (v.size() < 4) continue;
    EncoderModel e;
    e.motor = (int)v[0] - 1;
    e.pin_a = (int)v[1];
    e.pin_b = (int)v[2];
    e.forward_level = (int)v[3];
    encoders.push_back(e);
  }
  // distance sensors
  tofs.clear();
  bool cold = opt.boot == "cold";
  for (int n = 1; n <= 16; n++) {
    std::vector<double> v = cfg.nums("tof." + std::to_string(n));
    if (v.size() < 4) continue;
    TofDevice d;
    d.logical = n;
    d.rx = v[0];
    d.ry = v[1];
    d.yaw = v[2];
    d.port = (int)v[3];
    d.rz = v.size() >= 5 ? v[4] : -1;  // optional 5th number: height of this sensor above the floor
    d.bias = setup_rng.uniform(-1, 1) * cfg.num("tof.bias_spread_mm", 4);
    d.period = (int64_t)(cfg.num("tof.timing_budget_ms", 33) * 1000) + 600;
    if (cold) {
      d.address = 0x29;  // power-on default address, not measuring
    } else {
      // warm reset (reset button / re-upload): the sensors kept power, so they are
      // still at the address the last run gave them and still measuring.
      d.address = (uint8_t)cfg.integer("tof.warm_address", 0x30);
      d.initialized = true;
      d.ranging = true;
      d.next_ready = (int64_t)setup_rng.uniform(0, (double)d.period);
    }
    tofs.push_back(d);
  }
  tcs_port = cfg.integer("color.port", 7);
  std::vector<double> cp = cfg.nums("color.position");
  if (cp.size() >= 2) {
    color_rx = cp[0];
    color_ry = cp[1];
  }
  cameras.clear();
  for (const char* side : {"left", "right"}) {
    std::vector<double> v = cfg.nums(std::string("camera.") + side);
    if (v.size() < 4) continue;
    Camera c;
    c.rx = v[0];
    c.ry = v[1];
    c.yaw = v[2];
    c.serial = (int)v[3];
    c.period = (int64_t)(1e6 / std::max(1.0, cfg.num("camera.fps", 20)));
    c.next_frame = (int64_t)setup_rng.uniform(0, (double)c.period);
    cameras.push_back(c);
  }
  gyro_drift_rate_ = cfg.num("gyro.drift_deg_per_min", 0.5) * (setup_rng.chance(0.5) ? 1 : -1) / 60e6;
  fixed_yaw_bias_ = 0;

  x = (world.start_x + 0.5) * world.tile;
  y = (world.start_y + 0.5) * world.tile;
  th = world.start_dir * 90 * DEG + cfg.num("robot.start_angle_error_deg", 0) * DEG;
  th0 = world.start_dir * 90 * DEG;
  refresh_height();
}

void Robot::refresh_height() {
  double fx = std::sin(th), fy = std::cos(th);
  double zf = world.floor_z(x + fx * wheelbase / 2, y + fy * wheelbase / 2);
  double zr = world.floor_z(x - fx * wheelbase / 2, y - fy * wheelbase / 2);
  pitch = std::atan2(zf - zr, wheelbase);
  z = (zf + zr) / 2;
}

void Robot::teleport(double nx, double ny, double nth) {
  x = nx;
  y = ny;
  th = nth;
  vL = vR = 0;
  refresh_height();
}

int Robot::signed_pwm(int m) const {
  if (!shield_begun) return 0;
  int s = mdir[m] == 1 ? 1 : mdir[m] == 2 ? -1 : 0;
  return s * msign[m] * (int)mspeed[m];
}

bool Robot::moving_motors() const {
  for (int m = 0; m < 4; m++)
    if (signed_pwm(m) != 0) return true;
  return false;
}

void Robot::physics_step(double dt, int64_t t_us) {
  if (held) {
    vL = vR = 0;
    return;
  }
  double sum[2] = {0, 0};
  int cnt[2] = {0, 0};
  for (int m = 0; m < 4; m++) {
    sum[mside[m]] += signed_pwm(m) * mgain[m];
    cnt[mside[m]]++;
  }
  double uL = cnt[0] ? sum[0] / cnt[0] : 0, uR = cnt[1] ? sum[1] / cnt[1] : 0;
  double turn = std::fabs(uL - uR) / std::max(1.0, std::fabs(uL) + std::fabs(uR));
  double db = P.db_straight + (P.db_turn - P.db_straight) * std::min(1.0, turn);
  auto shape = [&](double u) {
    double a = std::fabs(u) - db;
    if (a <= 0) return 0.0;
    return std::copysign(a / (255.0 - db) * P.vmax * P.battery, u);
  };
  double tL = shape(uL), tR = shape(uR);
  double sp = std::sin(pitch);
  auto load = [&](double v) {
    if (v == 0) return 0.0;
    double f = 1 - P.slope_k * sp * (v > 0 ? 1 : -1);
    return v * std::max(0.15, std::min(1.6, f));
  };
  tL = load(tL);
  tR = load(tR);
  double a = std::min(1.0, dt / P.tau);
  vL += (tL - vL) * a;
  vR += (tR - vR) * a;

  int tx, ty;
  bool on_debris = world.tile_of(x, y, &tx, &ty) && world.at(tx, ty).feature == F_DEBRIS;
  double v = 0.5 * (vL + vR) * (1 - P.slip);
  double w = (vL - vR) / (track * P.skid);
  double noise_scale = std::sqrt(std::fabs(v) * dt / 1000.0);  // per sqrt(metre)
  w += rng_noise_.normal() * P.yaw_noise * DEG * noise_scale / dt;
  if (on_debris && std::fabs(v) > 1) w += rng_noise_.normal() * P.debris_yaw * DEG * noise_scale / dt;

  double nth = th + w * dt;
  double cp = std::cos(pitch);
  double hm = th + w * dt / 2;
  double nx = x + v * cp * std::sin(hm) * dt, ny = y + v * cp * std::cos(hm) * dt;
  double zlo = z + 8, zhi = z + 110;
  auto free_at = [&](double X, double Y, double T) { return !world.body_collides(X, Y, T, half_len, half_wid, zlo, zhi); };
  double want = std::hypot(nx - x, ny - y);
  bool hit = false;
  double ox = x, oy = y;
  if (free_at(nx, ny, nth)) {
    x = nx; y = ny; th = nth;
  } else {
    hit = true;
    if (free_at(nx, ny, th)) { x = nx; y = ny; }
    else if (free_at(x, y, nth)) { th = nth; }
    else if (P.slide > 0 && [&]() {
               // soft walls: a turn that would swing a corner into a wall pushes the robot sideways instead (the wheels slip), by up to drive.wall_slide_mm
               for (double r = 1; r <= P.slide; r += 1)
                 for (int k = 0; k < 16; k++) {
                   double a = k * M_PI / 8, px = x + r * std::cos(a), py = y + r * std::sin(a);
                   if (free_at(px, py, nth)) { x = px; y = py; th = nth; return true; }
                 }
               return false;
             }()) {}
    else if (free_at(nx, y, th)) { x = nx; }
    else if (free_at(x, ny, th)) { y = ny; }
  }
  double got = std::hypot(x - ox, y - oy);
  odometer_mm += got;
  double now_s = t_us / 1e6;
  if (hit && !contact && now_s - last_contact_t > 0.5) contacts++;
  if (hit) last_contact_t = now_s;
  contact = hit;
  if (hit) contact_time += dt;
  refresh_height();

  // encoders count wheel turns; when the body is blocked the wheels partly spin
  double wheel_factor = 1;
  if (hit && want > 1e-6 && got < 0.3 * want) wheel_factor = P.blocked_spin;
  for (auto& e : encoders) {
    if (e.motor < 0 || e.motor > 3) continue;
    double vs = mside[e.motor] == 0 ? vL : vR;
    double n = P.enc_noise + (on_debris ? P.debris_enc : 0);
    double d = vs * dt * wheel_factor * (1 + rng_noise_.normal() * n);
    e.accum += d * P.counts_per_mm;
    while (e.accum >= 1 || e.accum <= -1) {
      int dir = e.accum > 0 ? 1 : -1;
      e.accum -= dir;
      pin_set_input(e.pin_b, dir > 0 ? e.forward_level : !e.forward_level);
      isr_fire(e.pin_a);
    }
  }
}

void Robot::sensor_pose(double rx, double ry, double* wx, double* wy) const {
  double s = std::sin(th), c = std::cos(th);
  *wx = x + rx * c + ry * s;
  *wy = y - rx * s + ry * c;
}

int Robot::tile_under(double rx, double ry, int* tx, int* ty) const {
  double wx, wy;
  sensor_pose(rx, ry, &wx, &wy);
  if (!world.tile_of(wx, wy, tx, ty)) return T_VOID;
  return world.at(*tx, *ty).type;
}

int Robot::tof_measure(const TofDevice& d) {
  // The VL53L0X sends a light cone (~25 degrees) and reports a distance from the
  // returned signal. Model: several beams across the cone; each return is weighted
  // by the beam profile, how squarely it hits the surface and 1/d^2. Surfaces hit
  // at a grazing angle or far away count little. The reading is the weighted mean
  // of the strong returns, so a wall edge entering or leaving the cone gives
  // in-between readings, like the real sensor.
  double wx, wy;
  sensor_pose(d.rx, d.ry, &wx, &wy);
  double wz = z + (d.rz >= 0 ? d.rz : P.tof_height) + d.ry * std::sin(pitch);
  double yaw = th + d.yaw * DEG;
  double slope = std::tan(pitch) * std::cos(d.yaw * DEG);
  const int nmax = 15;
  int n = std::min(nmax, P.tof_rays);
  double dist[nmax], sig[nmax];
  double smax = 0;
  for (int k = 0; k < n; k++) {
    double u = n == 1 ? 0 : (double)k / (n - 1) - 0.5;  // -0.5 .. 0.5 across the cone
    double off = u * P.tof_cone * DEG;
    double ci = 0;
    double r = world.raycast(wx, wy, wz, std::sin(yaw + off), std::cos(yaw + off), slope, P.tof_max, &ci);
    dist[k] = r;
    sig[k] = 0;
    if (r < 0) continue;
    double beam = std::exp(-8.0 * u * u);  // brightest in the middle of the cone
    double dd = std::max(r, 20.0);
    sig[k] = beam * std::max(ci, 0.02) / (dd * dd);
    smax = std::max(smax, sig[k]);
  }
  // weakest return the sensor still ranges: a square-on wall at max range in the cone centre
  if (smax <= 0 || smax < 1.0 / (P.tof_max * P.tof_max)) return P.tof_oor;
  double sw = 0, sd = 0;
  for (int k = 0; k < n; k++)
    if (sig[k] >= 0.25 * smax) {
      sw += sig[k];
      sd += sig[k] * dist[k];
    }
  double best = sd / sw;
  if (rng_tof_.chance(P.tof_dropout)) return P.tof_oor;
  if (rng_tof_.chance(P.tof_spike)) return (int)rng_tof_.uniform(P.tof_min, std::max(P.tof_min + 1, best));
  double v = best + d.bias + rng_tof_.normal() * (P.tof_noise + P.tof_noise_frac * best);
  if (v < P.tof_min) v = P.tof_min + std::fabs(rng_tof_.normal()) * 5;
  if (v > P.tof_max) return P.tof_oor;
  return (int)std::lround(v);
}

void Robot::update_tofs(int64_t t_us) {
  for (auto& d : tofs) {
    if (!d.ranging) continue;
    while (d.next_ready <= t_us) {
      d.latest = tof_measure(d);
      d.unread = true;
      d.next_ready += d.period + (int64_t)rng_tof_.uniform(-300, 300);
    }
  }
}

TofDevice* Robot::tof_on_bus(uint8_t address) {
  for (auto& d : tofs)
    if ((mux_mask & (1 << d.port)) && d.address == address) return &d;
  return nullptr;
}

bool Robot::i2c_present(uint8_t address) {
  if (address == 0x28 || address == 0x60) return true;  // BNO055, motor shield
  if (address == 0x70) return mux_present;             // TCA9548A
  if (tof_on_bus(address)) return true;
  if (address == tcs_address && (mux_mask & (1 << tcs_port))) return true;
  return false;
}

double Robot::gyro_heading_deg(int64_t t_us) {
  double h = (th - th0) / DEG + P.gyro_offset + gyro_drift_rate_ * (double)t_us + rng_noise_.normal() * P.gyro_noise;
  h = std::fmod(h, 360.0);
  if (h < 0) h += 360.0;
  h = std::floor(h * 16.0 + 0.5) / 16.0;  // BNO055 resolution: 1/16 degree
  if (h >= 360.0) h -= 360.0;
  return h;
}

double Robot::gyro_pitch_deg() {
  double p = P.pitch_sign * pitch / DEG + rng_noise_.normal() * P.pitch_noise;
  return std::floor(p * 16.0 + 0.5) / 16.0;
}

void Robot::color_raw(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c, uint8_t it, int gain) {
  int tx, ty;
  int type = tile_under(color_rx, color_ry, &tx, &ty);
  const char* key = "color.white";
  switch (type) {
    case T_BLACK: key = "color.black"; break;
    case T_BLUE: key = "color.blue"; break;
    case T_SILVER: key = "color.silver"; break;
    case T_RED: key = "color.red"; break;
    case T_VOID: key = "color.black"; break;
    default: break;
  }
  std::vector<double> v = cfg.nums(key);
  while (v.size() < 4) v.push_back(0);
  double cycles = 256 - it;
  double gains[4] = {1, 4, 16, 60};
  double scale = cycles / 10.0 * gains[gain & 3];
  double maxc = std::min(65535.0, 1024.0 * cycles);
  uint16_t* out[4] = {r, g, b, c};
  for (int i = 0; i < 4; i++) {
    double val = v[i] * scale * (1 + rng_noise_.normal() * P.color_noise);
    *out[i] = (uint16_t)std::max(0.0, std::min(maxc, val));
  }
}

int Robot::victim_visible(const Camera& cam) const {
  double cx, cy;
  sensor_pose(cam.rx, cam.ry, &cx, &cy);
  double yaw = th + cam.yaw * DEG;
  double ax = std::sin(yaw), ay = std::cos(yaw);
  int best = -1;
  double best_d = 1e9;
  for (int i = 0; i < (int)world.victims.size(); i++) {
    const Victim& v = world.victims[i];
    double dx = v.wx - cx, dy = v.wy - cy, dist = std::hypot(dx, dy);
    if (dist > P.cam_range || dist < 1) continue;
    if (std::fabs(v.wz - (z + P.cam_height)) > 120) continue;
    double cosang = (dx * ax + dy * ay) / dist;
    if (cosang < std::cos(P.cam_fov / 2 * DEG)) continue;
    // the camera must be in front of the wall face the victim is on
    double nx = -dir_dx(v.side), ny = -dir_dy(v.side);
    if (dx * nx + dy * ny > 0) continue;
    double r = world.raycast(cx, cy, v.wz, dx / dist, dy / dist, 0, dist + 20);
    if (r >= 0 && r < dist - 6) continue;
    if (dist < best_d) {
      best_d = dist;
      best = i;
    }
  }
  return best;
}

void Robot::update_cameras(int64_t t_us) {
  static const char codes[] = "HSURYG";
  for (auto& cam : cameras) {
    while (cam.next_frame <= t_us) {
      cam.next_frame += cam.period;
      int vi = victim_visible(cam);
      cam.visible_victim = vi;
      if (vi >= 0) {
        char ch = reported_letter(world.victims[vi].type);
        if (rng_cam_.chance(P.cam_misread)) ch = codes[rng_cam_.next() % 3];
        serial_rx_push(cam.serial, (uint8_t)ch);
      } else if (P.cam_idle_byte >= 0 && rng_cam_.chance(P.cam_idle_rate * cam.period / 1e6)) {
        serial_rx_push(cam.serial, (uint8_t)P.cam_idle_byte);
      }
    }
  }
}

}  // namespace sim
