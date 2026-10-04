// The simulated maze: parser for the text maze format, heights (ramps, stairs,
// speed bumps), wall geometry, ray casting for the distance sensors and collision
// checks for the robot body.
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <functional>
#include <numeric>
#include <sstream>

#include "sim.h"

namespace sim {

World world;

static const double DEG = M_PI / 180.0;

static int parse_side(const std::string& s) {
  if (s.empty()) return -1;
  switch (toupper((unsigned char)s[0])) {
    case 'N': return DN;
    case 'E': return DE;
    case 'S': return DS;
    case 'W': return DW;
  }
  return -1;
}

static char parse_victim_type(std::string s) {
  for (auto& c : s) c = (char)tolower((unsigned char)c);
  if (s == "h" || s == "harmed") return 'H';
  if (s == "s" || s == "stable") return 'S';
  if (s == "u" || s == "unharmed") return 'U';
  if (s == "r" || s == "red") return 'R';
  if (s == "y" || s == "yellow") return 'Y';
  if (s == "g" || s == "green") return 'G';
  return 0;
}

bool World::load(const std::string& path, std::string* err) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    if (err) *err = "cannot open maze file " + path;
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  if (name.empty()) {
    size_t s = path.find_last_of("/\\");
    name = path.substr(s == std::string::npos ? 0 : s + 1);
  }
  return parse(ss.str(), err);
}

bool World::wall(int x, int y, int side) const {
  if (!in(x, y)) return true;
  switch (side) {
    case DN: return hwall[(y + 1) * W + x] != 0;
    case DS: return hwall[y * W + x] != 0;
    case DE: return vwall[y * (W + 1) + x + 1] != 0;
    case DW: return vwall[y * (W + 1) + x] != 0;
  }
  return true;
}

bool World::parse(const std::string& text, std::string* err) {
  source_text = text;
  tile = cfg.num("field.tile_mm", 300);
  wall_t = cfg.num("field.wall_thickness_mm", 20);
  wall_h = cfg.num("field.wall_height_mm", 150);
  double ramp_angle = cfg.num("field.ramp_angle_deg", 20);

  std::vector<std::string> lines;
  {
    std::string cur;
    for (char c : text) {
      if (c == '\n') {
        lines.push_back(cur);
        cur.clear();
      } else if (c != '\r') {
        cur += c;
      }
    }
    if (!cur.empty()) lines.push_back(cur);
  }
  int first = -1;
  for (int i = 0; i < (int)lines.size(); i++)
    if (!lines[i].empty() && lines[i][0] == '+') {
      first = i;
      break;
    }
  if (first < 0) {
    if (err) *err = "no maze drawing found (the grid lines must start with '+')";
    return false;
  }
  std::vector<std::string> g;
  int i = first;
  for (; i < (int)lines.size(); i++) {
    const std::string& L = lines[i];
    bool plus = !L.empty() && L[0] == '+';
    bool cell = !L.empty() && (L[0] == '|' || L[0] == ' ');
    bool want_plus = (g.size() % 2) == 0;
    if (want_plus ? !plus : !cell) break;
    g.push_back(L);
  }
  int grid_end = i;
  if (g.size() % 2 == 0) g.pop_back();
  H = ((int)g.size() - 1) / 2;
  W = ((int)g[0].size() - 1) / 4;
  if (W < 1 || H < 1) {
    if (err) *err = "maze drawing is too small";
    return false;
  }
  size_t width = (size_t)W * 4 + 1;
  for (auto& L : g)
    if (L.size() < width) L.resize(width, ' ');

  tiles.assign(W * H, TileInfo());
  hwall.assign((H + 1) * W, 0);
  vwall.assign(H * (W + 1), 0);
  victims.clear();
  obstacles.clear();
  ramps.clear();
  warnings.clear();
  bool have_start = false;
  start_dir = -1;

  std::vector<int> obstacle_tiles;
  for (int r = 0; r <= H; r++) {
    const std::string& L = g[2 * r];
    int j = H - r;
    for (int x = 0; x < W; x++) {
      bool wl = false;
      for (int k = 1; k <= 3; k++)
        if (L[4 * x + k] != ' ') wl = true;
      hwall[j * W + x] = wl;
    }
  }
  for (int r = 0; r < H; r++) {
    const std::string& L = g[2 * r + 1];
    int y = H - 1 - r;
    for (int b = 0; b <= W; b++) vwall[y * (W + 1) + b] = L[4 * b] != ' ';
    for (int x = 0; x < W; x++) {
      TileInfo& t = tiles[idx(x, y)];
      char left = L[4 * x + 1], code = L[4 * x + 2], right = L[4 * x + 3];
      switch (code) {
        case ' ': case '.': break;
        case 'S': t.start = true; start_x = x; start_y = y; have_start = true; break;
        case 'X': t.type = T_BLACK; break;
        case 'B': t.type = T_BLUE; break;
        case 'C': t.type = T_SILVER; break;
        case 'R': t.type = T_RED; break;
        case '#': t.type = T_VOID; break;
        case '^': t.feature = F_RAMP; t.axis = DN; break;
        case '>': t.feature = F_RAMP; t.axis = DE; break;
        case 'v': t.feature = F_RAMP; t.axis = DS; break;
        case '<': t.feature = F_RAMP; t.axis = DW; break;
        case 'T': t.feature = F_STAIRS; break;
        case 'b': t.feature = F_BUMP; break;
        case 'p': t.feature = F_BUMP; t.tall = true; break;  // speed bump inside the dangerous zone (up to 2 cm)
        case 'd': t.feature = F_DEBRIS; break;
        case 'O': obstacle_tiles.push_back(idx(x, y)); break;
        default: warnings.push_back(std::string("unknown tile code '") + code + "' treated as a white tile");
      }
      char sides[2] = {left, right};
      int side_dir[2] = {DW, DE};
      for (int s = 0; s < 2; s++) {
        char vt = parse_victim_type(std::string(1, sides[s]));
        if (sides[s] == ' ' || sides[s] == '.') continue;
        if (!vt) {
          warnings.push_back(std::string("ignored character '") + sides[s] + "' next to a tile code");
          continue;
        }
        Victim v;
        v.x = x;
        v.y = y;
        v.side = side_dir[s];
        v.type = vt;
        victims.push_back(v);
      }
    }
  }

  // commands after (or before) the drawing
  for (int li = 0; li < (int)lines.size(); li++) {
    if (li >= first && li < grid_end) continue;
    std::string L = lines[li];
    size_t hash = L.find('#');
    if (hash != std::string::npos) L = L.substr(0, hash);
    std::istringstream ss(L);
    std::string cmd;
    if (!(ss >> cmd)) continue;
    for (auto& c : cmd) c = (char)tolower((unsigned char)c);
    if (cmd == "name" || cmd == "name:") {
      std::string rest;
      std::getline(ss, rest);
      size_t a = rest.find_first_not_of(" \t");
      name = a == std::string::npos ? "" : rest.substr(a);
    } else if (cmd == "start_dir") {
      std::string s;
      ss >> s;
      start_dir = parse_side(s);
    } else if (cmd == "ramp_angle") {
      ss >> ramp_angle;
    } else if (cmd == "wall_thickness") {
      ss >> wall_t;
    } else if (cmd == "victim") {
      int c, r;
      std::string side, type;
      if (!(ss >> c >> r >> side >> type)) {
        warnings.push_back("bad victim line: " + lines[li]);
        continue;
      }
      Victim v;
      v.x = c;
      v.y = H - 1 - r;
      v.side = parse_side(side);
      v.type = parse_victim_type(type);
      if (!in(v.x, v.y) || v.side < 0 || !v.type) {
        warnings.push_back("bad victim line: " + lines[li]);
        continue;
      }
      victims.push_back(v);
    } else if (cmd == "obstacle" || cmd == "box") {
      int c, r;
      double dx, dy, a, b = 0, hgt = 200;
      bool ok = cmd == "obstacle" ? (bool)(ss >> c >> r >> dx >> dy >> a) : (bool)(ss >> c >> r >> dx >> dy >> a >> b);
      if (!ok) {
        warnings.push_back("bad " + cmd + " line: " + lines[li]);
        continue;
      }
      ss >> hgt;
      Obstacle o;
      o.kind = cmd == "obstacle" ? 0 : 1;
      o.tx = c;
      o.ty = H - 1 - r;
      o.x = (o.tx + 0.5) * tile + dx;
      o.y = (o.ty + 0.5) * tile + dy;
      o.r = a;
      o.w = a;
      o.h = b;
      o.height = hgt;
      if (!in(o.tx, o.ty)) {
        warnings.push_back("obstacle outside the maze: " + lines[li]);
        continue;
      }
      obstacles.push_back(o);
    } else {
      warnings.push_back("unknown line in maze file: " + lines[li]);
    }
  }
  // default obstacles for 'O' tiles: a 15 cm+ tall cylinder pushed to one side of the tile
  for (int ti : obstacle_tiles) {
    Obstacle o;
    o.tx = ti % W;
    o.ty = ti / W;
    uint64_t h = (uint64_t)ti * 2654435761u + 12345;
    int side = (int)(h % 4);
    double off = cfg.num("field.obstacle_offset_mm", 80);
    o.x = (o.tx + 0.5) * tile + dir_dx(side) * off;
    o.y = (o.ty + 0.5) * tile + dir_dy(side) * off;
    o.r = cfg.num("field.obstacle_radius_mm", 40);
    o.height = 200;
    obstacles.push_back(o);
  }
  if (!have_start) {
    if (err) *err = "the maze has no start tile (put an 'S' in one tile)";
    return false;
  }
  for (auto& t : tiles)
    if (t.feature == F_RAMP) t.ramp = -2;  // marker, resolved below
  // group ramp tiles into straight runs
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      TileInfo& t = tiles[idx(x, y)];
      if (t.feature != F_RAMP || t.ramp != -2) continue;
      int d = t.axis;
      int bx = x, by = y;
      while (in(bx - dir_dx(d), by - dir_dy(d))) {
        const TileInfo& p = at(bx - dir_dx(d), by - dir_dy(d));
        if (p.feature != F_RAMP || p.axis != d) break;
        bx -= dir_dx(d);
        by -= dir_dy(d);
      }
      Ramp rp;
      rp.dir = d;
      rp.angle_deg = ramp_angle;
      int cx = bx, cy = by;
      while (in(cx, cy) && at(cx, cy).feature == F_RAMP && at(cx, cy).axis == d) {
        rp.tiles.push_back(idx(cx, cy));
        tiles[idx(cx, cy)].ramp = (int)ramps.size();
        cx += dir_dx(d);
        cy += dir_dy(d);
      }
      rp.high_tile = in(cx, cy) ? idx(cx, cy) : -1;
      int lx = bx - dir_dx(d), ly = by - dir_dy(d);
      rp.low_tile = in(lx, ly) ? idx(lx, ly) : -1;
      if (d == DN) rp.s0 = by * tile;
      if (d == DS) rp.s0 = (by + 1) * tile;
      if (d == DE) rp.s0 = bx * tile;
      if (d == DW) rp.s0 = (bx + 1) * tile;
      for (int k = 0; k < (int)rp.tiles.size(); k++) {
        int tx = rp.tiles[k] % W, ty = rp.tiles[k] / W;
        int l1 = (d + 1) % 4, l2 = (d + 3) % 4;
        if (!wall(tx, ty, l1) || !wall(tx, ty, l2))
          warnings.push_back("ramp tile without walls on both sides (the robot could fall off)");
      }
      ramps.push_back(rp);
    }
  return finish_build(err);
}

bool World::finish_build(std::string* err) {
  std::vector<std::string> errs;
  compute_heights(&errs);
  if (!errs.empty()) {
    if (err) *err = errs[0];
    return false;
  }
  // travel axis for bumps / stairs: across the corridor if there is one
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      TileInfo& t = tiles[idx(x, y)];
      if (t.feature == F_BUMP || t.feature == F_STAIRS) {
        if (wall(x, y, DN) && wall(x, y, DS) && !(wall(x, y, DE) && wall(x, y, DW)))
          t.axis = 1;
        else
          t.axis = 0;
      }
    }
  double bump_h = cfg.num("field.bump_height_mm", 10), dz_bump_h = cfg.num("field.dz_bump_height_mm", 20), step_h = cfg.num("field.step_height_mm", 10);
  for (auto& t : tiles) {
    if (t.feature == F_RAMP) continue;
    t.zmin = t.z;
    t.zmax = t.z + (t.feature == F_BUMP ? (t.tall ? dz_bump_h : bump_h) : t.feature == F_STAIRS ? 2 * step_h : 0);
  }
  for (auto& rp : ramps) {
    double slope = std::tan(rp.angle_deg * DEG);
    for (int k = 0; k < (int)rp.tiles.size(); k++) {
      TileInfo& t = tiles[rp.tiles[k]];
      t.z = rp.z_low + k * tile * slope;
      t.zmin = t.z;
      t.zmax = t.z + tile * slope;
    }
  }
  // start direction: first open side if not given
  if (start_dir < 0) {
    start_dir = 0;
    for (int d = 0; d < 4; d++)
      if (!wall(start_x, start_y, d)) {
        start_dir = d;
        break;
      }
  }
  for (auto& v : victims) {
    if (!wall(v.x, v.y, v.side))
      warnings.push_back("victim at tile (" + std::to_string(v.x) + "," + std::to_string(H - 1 - v.y) +
                         ") is on a side without a wall");
    double cx = (v.x + 0.5) * tile, cy = (v.y + 0.5) * tile, half = tile / 2 - wall_t / 2;
    v.wx = cx + dir_dx(v.side) * half;
    v.wy = cy + dir_dy(v.side) * half;
    v.wz = at(v.x, v.y).z + cfg.num("field.victim_height_mm", 70);
  }
  victims.erase(std::remove_if(victims.begin(), victims.end(), [&](const Victim& v) { return !wall(v.x, v.y, v.side); }),
                victims.end());
  for (auto& o : obstacles) o.z = at(o.tx, o.ty).z;
  compute_floating();
  build_boxes();
  // tiles the robot can reach without crossing a black tile
  reachable_count = 0;
  std::vector<int> q;
  q.push_back(idx(start_x, start_y));
  tiles[q[0]].reachable = true;
  for (size_t h = 0; h < q.size(); h++) {
    int x = q[h] % W, y = q[h] / W;
    for (int d = 0; d < 4; d++) {
      if (wall(x, y, d)) continue;
      int nx = x + dir_dx(d), ny = y + dir_dy(d);
      if (!in(nx, ny)) continue;
      TileInfo& n = tiles[idx(nx, ny)];
      if (n.reachable || n.type == T_BLACK || n.type == T_VOID) continue;
      n.reachable = true;
      q.push_back(idx(nx, ny));
    }
  }
  for (auto& t : tiles)
    if (t.reachable && t.feature != F_RAMP) reachable_count++;
  return true;
}

void World::compute_heights(std::vector<std::string>* errs) {
  std::vector<char> set(W * H, 0);
  std::vector<int> q;
  int s = idx(start_x, start_y);
  tiles[s].z = 0;
  set[s] = 1;
  q.push_back(s);
  std::vector<char> ramp_done(ramps.size(), 0);
  auto assign = [&](int ti, double z) {
    if (set[ti]) {
      if (std::fabs(tiles[ti].z - z) > 1.0 && errs->empty())
        errs->push_back("tile heights do not fit together: two paths reach tile (" + std::to_string(ti % W) + "," +
                        std::to_string(H - 1 - ti / W) + ") at different heights. Floors at different heights "
                        "must only be connected by ramps.");
      return;
    }
    tiles[ti].z = z;
    set[ti] = 1;
    q.push_back(ti);
  };
  for (size_t h = 0; h < q.size(); h++) {
    int ti = q[h];
    int x = ti % W, y = ti / W;
    const TileInfo& t = tiles[ti];
    if (t.feature == F_RAMP) continue;
    for (int d = 0; d < 4; d++) {
      if (wall(x, y, d)) continue;
      int nx = x + dir_dx(d), ny = y + dir_dy(d);
      if (!in(nx, ny)) continue;
      int ni = idx(nx, ny);
      const TileInfo& n = tiles[ni];
      if (n.type == T_VOID) continue;
      if (n.feature == F_RAMP) {
        Ramp& rp = ramps[n.ramp];
        int r = n.ramp;
        double rise = rp.tiles.size() * tile * std::tan(rp.angle_deg * DEG);
        if (ramp_done[r]) continue;
        if (ni == rp.tiles.front() && d == rp.dir) {
          rp.z_low = t.z;
          rp.z_high = t.z + rise;
        } else if (ni == rp.tiles.back() && d == (rp.dir + 2) % 4) {
          rp.z_high = t.z;
          rp.z_low = t.z - rise;
        } else {
          continue;  // entering a ramp from its side: walls are missing, ignore
        }
        ramp_done[r] = 1;
        for (int k = 0; k < (int)rp.tiles.size(); k++) set[rp.tiles[k]] = 1;
        if (rp.high_tile >= 0 && !wall(rp.tiles.back() % W, rp.tiles.back() / W, rp.dir)) assign(rp.high_tile, rp.z_high);
        if (rp.low_tile >= 0 && !wall(rp.tiles.front() % W, rp.tiles.front() / W, (rp.dir + 2) % 4))
          assign(rp.low_tile, rp.z_low);
        continue;
      }
      assign(ni, t.z);
    }
  }
}

void World::compute_floating() {
  // Walls connected (through other walls) to the outer wall are "linear", the rest are "floating".
  int nc = (W + 1) * (H + 1);
  std::vector<int> parent(nc);
  std::iota(parent.begin(), parent.end(), 0);
  std::function<int(int)> find = [&](int a) { return parent[a] == a ? a : parent[a] = find(parent[a]); };
  auto unite = [&](int a, int b) { parent[find(a)] = find(b); };
  auto cid = [&](int i, int j) { return j * (W + 1) + i; };
  for (int j = 0; j <= H; j++)
    for (int x = 0; x < W; x++)
      if (hwall[j * W + x]) unite(cid(x, j), cid(x + 1, j));
  for (int y = 0; y < H; y++)
    for (int i = 0; i <= W; i++)
      if (vwall[y * (W + 1) + i]) unite(cid(i, y), cid(i, y + 1));
  std::vector<char> anchored(nc, 0);
  for (int j = 0; j <= H; j++)
    for (int i = 0; i <= W; i++) {
      bool border = i == 0 || j == 0 || i == W || j == H;
      bool near_void = false;
      for (int dx = -1; dx <= 0; dx++)
        for (int dy = -1; dy <= 0; dy++) {
          int tx = i + dx, ty = j + dy;
          if (in(tx, ty) && at(tx, ty).type == T_VOID) near_void = true;
        }
      if (border || near_void) anchored[find(cid(i, j))] = 1;
    }
  // 2026 rules (3.3): tiles that lead to the start tile by consistently following the leftmost or rightmost wall are
  // 'linear tiles', all others are 'floating tiles'; black tiles count as walls. A wall follower leaves the start tile and
  // keeps one hand on the wall until its state repeats; every tile it passes is linear.
  std::vector<char> linear(W * H, 0);
  auto blocked = [&](int x, int y, int d) {
    if (wall(x, y, d)) return true;
    int nx = x + dir_dx(d), ny = y + dir_dy(d);
    return !in(nx, ny) || at(nx, ny).type == T_VOID || at(nx, ny).type == T_BLACK;
  };
  for (int hand = -1; hand <= 1; hand += 2) {  // -1 = left hand, +1 = right hand
    for (int d0 = 0; d0 < 4; d0++) {
      int x = start_x, y = start_y, d = d0;
      std::vector<char> seen(W * H * 4, 0);
      linear[idx(x, y)] = 1;
      for (int steps = 0; steps < W * H * 8; steps++) {
        int key = (idx(x, y) * 4) + d;
        if (seen[key]) break;
        seen[key] = 1;
        int order[4] = {(d + hand + 4) % 4, d, (d - hand + 4) % 4, (d + 2) % 4};
        int nd = -1;
        for (int k = 0; k < 4; k++)
          if (!blocked(x, y, order[k])) { nd = order[k]; break; }
        if (nd < 0) break;
        d = nd;
        x += dir_dx(d);
        y += dir_dy(d);
        linear[idx(x, y)] = 1;
      }
    }
  }
  for (auto& v : victims) v.floating = !linear[idx(v.x, v.y)];
}

void World::build_boxes() {
  boxes.clear();
  double ht = wall_t / 2;
  auto zr = [&](int x1, int y1, int x2, int y2, double* z0, double* z1) {
    double lo = 1e9, hi = -1e9;
    if (in(x1, y1)) { lo = std::min(lo, at(x1, y1).zmin); hi = std::max(hi, at(x1, y1).zmax); }
    if (in(x2, y2)) { lo = std::min(lo, at(x2, y2).zmin); hi = std::max(hi, at(x2, y2).zmax); }
    if (lo > hi) { lo = 0; hi = 0; }
    *z0 = lo - 30;
    *z1 = hi + wall_h;
  };
  for (int j = 0; j <= H; j++)
    for (int x = 0; x < W; x++)
      if (hwall[j * W + x]) {
        Box b{x * tile - ht, j * tile - ht, (x + 1) * tile + ht, j * tile + ht, 0, 0};
        zr(x, j - 1, x, j, &b.z0, &b.z1);
        boxes.push_back(b);
      }
  for (int y = 0; y < H; y++)
    for (int i = 0; i <= W; i++)
      if (vwall[y * (W + 1) + i]) {
        Box b{i * tile - ht, y * tile - ht, i * tile + ht, (y + 1) * tile + ht, 0, 0};
        zr(i - 1, y, i, y, &b.z0, &b.z1);
        boxes.push_back(b);
      }
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      if (at(x, y).type == T_VOID) boxes.push_back(Box{x * tile, y * tile, (x + 1) * tile, (y + 1) * tile, -5000, 5000});
  double fw = W * tile, fh = H * tile, m = 200;
  boxes.push_back(Box{-m, -m, 0, fh + m, -5000, 5000});
  boxes.push_back(Box{fw, -m, fw + m, fh + m, -5000, 5000});
  boxes.push_back(Box{-m, -m, fw + m, 0, -5000, 5000});
  boxes.push_back(Box{-m, fh, fw + m, fh + m, -5000, 5000});
}

bool World::tile_of(double x, double y, int* tx, int* ty) const {
  int a = (int)std::floor(x / tile), b = (int)std::floor(y / tile);
  if (tx) *tx = a;
  if (ty) *ty = b;
  return in(a, b);
}

double World::floor_z(double x, double y) const {
  int tx, ty;
  if (!tile_of(x, y, &tx, &ty)) return 0;
  const TileInfo& t = at(tx, ty);
  switch (t.feature) {
    case F_RAMP: {
      const Ramp& rp = ramps[t.ramp];
      double along = 0;
      if (rp.dir == DN) along = y - rp.s0;
      if (rp.dir == DS) along = rp.s0 - y;
      if (rp.dir == DE) along = x - rp.s0;
      if (rp.dir == DW) along = rp.s0 - x;
      double len = rp.tiles.size() * tile;
      along = std::max(0.0, std::min(len, along));
      return rp.z_low + along * std::tan(rp.angle_deg * DEG);
    }
    case F_BUMP: {
      double u = t.axis == 0 ? y - (ty + 0.5) * tile : x - (tx + 0.5) * tile;
      double hw = cfg.num("field.bump_width_mm", 60) / 2, hh = t.tall ? cfg.num("field.dz_bump_height_mm", 20) : cfg.num("field.bump_height_mm", 10);
      if (std::fabs(u) < hw) return t.z + hh * std::sqrt(1 - (u / hw) * (u / hw));
      return t.z;
    }
    case F_STAIRS: {
      double u = t.axis == 0 ? y - ty * tile : x - tx * tile;
      double step = cfg.num("field.step_height_mm", 10), z = 0;
      if (u >= 0.10 * tile && u < 0.90 * tile) z = step;
      if (u >= 0.25 * tile && u < 0.75 * tile) z = 2 * step;  // rules 3.4.6: the top of the stairs is at least 15 cm long
      return t.z + z;
    }
    default:
      return t.z;
  }
}

static bool ray_box(double ox, double oy, double oz, double dx, double dy, double dz, const Box& b, double tmax_in,
                    double* thit, int* axis) {
  double tmin = 0, tmax = tmax_in;
  int ax = -1;
  const double o[3] = {ox, oy, oz}, d[3] = {dx, dy, dz};
  const double lo[3] = {b.x0, b.y0, b.z0}, hi[3] = {b.x1, b.y1, b.z1};
  for (int a = 0; a < 3; a++) {
    if (std::fabs(d[a]) < 1e-12) {
      if (o[a] < lo[a] || o[a] > hi[a]) return false;
    } else {
      double t1 = (lo[a] - o[a]) / d[a], t2 = (hi[a] - o[a]) / d[a];
      if (t1 > t2) std::swap(t1, t2);
      if (t1 > tmin) {
        tmin = t1;
        ax = a;
      }
      if (t2 < tmax) tmax = t2;
      if (tmin > tmax) return false;
    }
  }
  *thit = tmin;
  *axis = ax;
  return true;
}

double World::raycast(double ox, double oy, double oz, double dx, double dy, double slope, double max_h,
                      double* cos_inc) const {
  double best = max_h;
  bool hit = false;
  double t;
  int axis;
  double norm = std::sqrt(1 + slope * slope);
  double c = 1;  // cosine of the incidence angle at the best hit
  auto box_cos = [&](int a) { return a == 0 ? std::fabs(dx) / norm : a == 1 ? std::fabs(dy) / norm : std::fabs(slope) / norm; };
  for (const Box& b : boxes)
    if (ray_box(ox, oy, oz, dx, dy, slope, b, best, &t, &axis)) {
      best = t;
      hit = true;
      c = axis < 0 ? 1 : box_cos(axis);
    }
  for (const Obstacle& o : obstacles) {
    if (o.kind == 1) {
      Box b{o.x - o.w / 2, o.y - o.h / 2, o.x + o.w / 2, o.y + o.h / 2, o.z - 10, o.z + o.height};
      if (ray_box(ox, oy, oz, dx, dy, slope, b, best, &t, &axis)) {
        best = t;
        hit = true;
        c = axis < 0 ? 1 : box_cos(axis);
      }
      continue;
    }
    double fx = ox - o.x, fy = oy - o.y;
    double bq = fx * dx + fy * dy, cc = fx * fx + fy * fy - o.r * o.r;
    double disc = bq * bq - cc;
    if (disc < 0) continue;
    double t0 = -bq - std::sqrt(disc);
    if (t0 < 0) {
      if (cc > 0) continue;  // the sensor is outside the cylinder and it is behind the beam: the beam's line passes through it, the beam does not
      t0 = 0;                // the sensor is inside the cylinder
    }
    if (t0 >= best) continue;
    double z = oz + slope * t0;
    if (z < o.z - 10 || z > o.z + o.height) continue;
    best = t0;
    hit = true;
    double nx = (ox + dx * t0 - o.x) / o.r, ny = (oy + dy * t0 - o.y) / o.r;
    c = std::fabs(nx * dx + ny * dy) / norm;
  }
  // floor / ramp surfaces (only matters when the beam points down)
  const double step = 5;
  for (double s = step; s < best; s += step) {
    double z = oz + slope * s;
    if (z <= floor_z(ox + dx * s, oy + dy * s)) {
      double a = s - step, b = s;
      for (int k = 0; k < 12; k++) {
        double m = (a + b) / 2;
        if (oz + slope * m <= floor_z(ox + dx * m, oy + dy * m))
          b = m;
        else
          a = m;
      }
      // surface slope from two nearby floor samples
      double z1 = floor_z(ox + dx * (b - 3), oy + dy * (b - 3)), z2 = floor_z(ox + dx * (b + 3), oy + dy * (b + 3));
      double fs = (z2 - z1) / 6;  // floor rise per mm along the beam
      double nz = 1 / std::sqrt(1 + fs * fs), nh = -fs * nz;  // floor normal in (horizontal-along-beam, up)
      c = std::fabs(nh * 1 + nz * slope) / norm;
      best = b;
      hit = true;
      break;
    }
  }
  if (cos_inc) *cos_inc = hit ? c : 0;
  return hit ? best * norm : -1;
}

static bool obb_hits_box(double cx, double cy, double fx, double fy, double rx, double ry, double hl, double hw,
                         const Box& b) {
  double bx = (b.x0 + b.x1) / 2, by = (b.y0 + b.y1) / 2, bhx = (b.x1 - b.x0) / 2, bhy = (b.y1 - b.y0) / 2;
  double dx = bx - cx, dy = by - cy;
  if (std::fabs(dx) > bhx + hw * std::fabs(rx) + hl * std::fabs(fx)) return false;
  if (std::fabs(dy) > bhy + hw * std::fabs(ry) + hl * std::fabs(fy)) return false;
  if (std::fabs(dx * rx + dy * ry) > hw + bhx * std::fabs(rx) + bhy * std::fabs(ry)) return false;
  if (std::fabs(dx * fx + dy * fy) > hl + bhx * std::fabs(fx) + bhy * std::fabs(fy)) return false;
  return true;
}

bool World::body_collides(double x, double y, double th, double hl, double hw, double zlo, double zhi) const {
  double fx = std::sin(th), fy = std::cos(th), rx = std::cos(th), ry = -std::sin(th);
  double rad = std::sqrt(hl * hl + hw * hw);
  for (const Box& b : boxes) {
    if (b.z1 < zlo || b.z0 > zhi) continue;
    if (x + rad < b.x0 || x - rad > b.x1 || y + rad < b.y0 || y - rad > b.y1) continue;
    if (obb_hits_box(x, y, fx, fy, rx, ry, hl, hw, b)) return true;
  }
  for (const Obstacle& o : obstacles) {
    if (o.z + o.height < zlo || o.z > zhi) continue;
    if (o.kind == 1) {
      Box b{o.x - o.w / 2, o.y - o.h / 2, o.x + o.w / 2, o.y + o.h / 2, 0, 0};
      if (obb_hits_box(x, y, fx, fy, rx, ry, hl, hw, b)) return true;
      continue;
    }
    double dx = o.x - x, dy = o.y - y;
    double lr = dx * rx + dy * ry, lf = dx * fx + dy * fy;
    double cr = std::max(-hw, std::min(hw, lr)), cf = std::max(-hl, std::min(hl, lf));
    double ex = lr - cr, ey = lf - cf;
    if (ex * ex + ey * ey < o.r * o.r) return true;
  }
  return false;
}

static std::string jesc(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
      o += c;
    } else if (c == '\n') {
      o += "\\n";
    } else if ((unsigned char)c < 0x20) {
      o += ' ';
    } else {
      o += c;
    }
  }
  return o;
}

std::string World::to_json() const {
  std::ostringstream o;
  o << "{\"name\":\"" << jesc(name) << "\",\"w\":" << W << ",\"h\":" << H << ",\"tile\":" << tile
    << ",\"wall_t\":" << wall_t << ",\"start\":{\"x\":" << start_x << ",\"y\":" << start_y << ",\"dir\":" << start_dir
    << "},\"type\":[";
  for (int i = 0; i < W * H; i++) o << (i ? "," : "") << (int)tiles[i].type;
  o << "],\"feature\":[";
  for (int i = 0; i < W * H; i++) o << (i ? "," : "") << (int)tiles[i].feature;
  o << "],\"axis\":[";
  for (int i = 0; i < W * H; i++) o << (i ? "," : "") << tiles[i].axis;
  o << "],\"z\":[";
  for (int i = 0; i < W * H; i++) o << (i ? "," : "") << (int)std::lround(tiles[i].z);
  o << "],\"reach\":[";
  for (int i = 0; i < W * H; i++) o << (i ? "," : "") << (tiles[i].reachable ? 1 : 0);
  o << "],\"hwall\":[";
  for (size_t i = 0; i < hwall.size(); i++) o << (i ? "," : "") << (int)hwall[i];
  o << "],\"vwall\":[";
  for (size_t i = 0; i < vwall.size(); i++) o << (i ? "," : "") << (int)vwall[i];
  o << "],\"victims\":[";
  for (size_t i = 0; i < victims.size(); i++) {
    const Victim& v = victims[i];
    o << (i ? "," : "") << "{\"x\":" << v.x << ",\"y\":" << v.y << ",\"side\":" << v.side << ",\"type\":\"" << v.type
      << "\",\"floating\":" << (v.floating ? "true" : "false") << "}";
  }
  o << "],\"obstacles\":[";
  for (size_t i = 0; i < obstacles.size(); i++) {
    const Obstacle& b = obstacles[i];
    o << (i ? "," : "") << "{\"kind\":" << b.kind << ",\"x\":" << (int)b.x << ",\"y\":" << (int)b.y
      << ",\"r\":" << (int)b.r << ",\"w\":" << (int)b.w << ",\"h\":" << (int)b.h << "}";
  }
  o << "],\"ramps\":[";
  for (size_t i = 0; i < ramps.size(); i++) {
    const Ramp& r = ramps[i];
    o << (i ? "," : "") << "{\"dir\":" << r.dir << ",\"angle\":" << r.angle_deg << ",\"z_low\":" << (int)r.z_low
      << ",\"z_high\":" << (int)r.z_high << ",\"tiles\":[";
    for (size_t k = 0; k < r.tiles.size(); k++) o << (k ? "," : "") << r.tiles[k];
    o << "]}";
  }
  o << "],\"warnings\":[";
  for (size_t i = 0; i < warnings.size(); i++) o << (i ? "," : "") << "\"" << jesc(warnings[i]) << "\"";
  o << "]}";
  return o.str();
}

}  // namespace sim
