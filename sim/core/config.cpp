// Config file reader: lines of `key = value`, `#` starts a comment.
#include <fstream>
#include <sstream>

#include "sim.h"

namespace sim {

Config cfg;

static std::string trim(const std::string& s) {
  size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

bool Config::load_file(const std::string& path, std::string* err) {
  std::ifstream in(path);
  if (!in) {
    if (err) *err = "cannot open config file " + path;
    return false;
  }
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    n++;
    size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (line.empty()) continue;
    size_t eq = line.find('=');
    if (eq == std::string::npos) {
      if (err) *err = path + ":" + std::to_string(n) + ": expected key = value";
      return false;
    }
    kv_[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
  }
  return true;
}

double Config::num(const std::string& key, double def) const {
  auto it = kv_.find(key);
  if (it == kv_.end() || it->second.empty()) return def;
  char* end = nullptr;
  double v = strtod(it->second.c_str(), &end);
  if (end == it->second.c_str()) return def;
  return v;
}

std::string Config::str(const std::string& key, const std::string& def) const {
  auto it = kv_.find(key);
  return it == kv_.end() ? def : it->second;
}

std::vector<double> Config::nums(const std::string& key) const {
  std::vector<double> out;
  auto it = kv_.find(key);
  if (it == kv_.end()) return out;
  std::istringstream ss(it->second);
  std::string tok;
  while (ss >> tok) {
    char* end = nullptr;
    double v = strtod(tok.c_str(), &end);
    if (end != tok.c_str()) out.push_back(v);
  }
  return out;
}

bool Config::flag(const std::string& key, bool def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  const std::string& v = it->second;
  return v == "1" || v == "true" || v == "yes" || v == "on";
}

}  // namespace sim
