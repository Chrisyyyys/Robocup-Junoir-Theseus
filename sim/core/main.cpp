// Entry point of the simulator: loads config + maze, then runs the robot's own
// setup() and loop() against the simulated hardware.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "sim.h"

void setup();
void loop();

namespace sim {
Options opt;
void hal_init() {}
}  // namespace sim

using namespace sim;

static void usage() {
  fprintf(stderr,
          "usage: theseus_sim --maze FILE [options]\n"
          "  --config FILE        config file (can be given more than once, later wins)\n"
          "  --set key=value      override one config value\n"
          "  --seed N             random seed (noise, motor mismatch, sensor timing)\n"
          "  --trace FILE         write the full trace (for the viewer)\n"
          "  --result FILE        write the result summary (JSON)\n"
          "  --time-limit S       simulated seconds before the run is stopped (default 480 = 8 min)\n"
          "  --boot warm|cold     warm = sensors kept power (reset button), cold = fresh power-on\n"
          "  --moves-limit code|off|N  keep the code's 'iterator >= 25' return trigger, disable it, or use N moves\n"
          "  --lop on|off         simulate the referee's lack-of-progress restarts (default on)\n"
          "  --echo               print the robot's Serial output while running\n"
          "  --quiet              no summary on stdout\n");
}

int main(int argc, char** argv) {
  std::string maze, trace, result;
  std::vector<std::string> configs;
  std::vector<std::string> sets;
  std::vector<std::pair<std::string, std::string>> flags;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
        exit(2);
      }
      return argv[++i];
    };
    if (a == "--maze") maze = next();
    else if (a == "--config") configs.push_back(next());
    else if (a == "--set") sets.push_back(next());
    else if (a == "--trace") trace = next();
    else if (a == "--result") result = next();
    else if (a == "--seed") flags.push_back({"sim.seed", next()});
    else if (a == "--time-limit") flags.push_back({"sim.time_limit_s", next()});
    else if (a == "--boot") flags.push_back({"sim.boot", next()});
    else if (a == "--moves-limit") flags.push_back({"sim.moves_limit", next()});
    else if (a == "--lop") flags.push_back({"sim.lack_of_progress", next()});
    else if (a == "--echo") flags.push_back({"sim.echo_serial", "1"});
    else if (a == "--quiet") flags.push_back({"sim.quiet", "1"});
    else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else {
      fprintf(stderr, "unknown option %s\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (maze.empty()) {
    usage();
    return 2;
  }
  std::string err;
  for (auto& c : configs)
    if (!cfg.load_file(c, &err)) {
      fprintf(stderr, "%s\n", err.c_str());
      return 2;
    }
  for (auto& s : sets) {
    size_t eq = s.find('=');
    if (eq == std::string::npos) {
      fprintf(stderr, "--set needs key=value, got %s\n", s.c_str());
      return 2;
    }
    cfg.set(s.substr(0, eq), s.substr(eq + 1));
  }
  for (auto& f : flags) cfg.set(f.first, f.second);

  opt.seed = (uint64_t)cfg.num("sim.seed", 1);
  opt.time_limit_s = cfg.num("sim.time_limit_s", 480);
  opt.boot = cfg.str("sim.boot", "warm");
  opt.moves_limit = cfg.str("sim.moves_limit", "code");
  opt.lop = cfg.flag("sim.lack_of_progress", true);
  opt.max_lops = cfg.integer("sim.max_lack_of_progress", 5);
  opt.frame_dt = cfg.num("sim.frame_dt_s", 0.05);
  opt.echo_serial = cfg.flag("sim.echo_serial", false);
  opt.quiet = cfg.flag("sim.quiet", false);
  opt.stuck_s = cfg.num("sim.stuck_s", 30);
  opt.idle_s = cfg.num("sim.idle_s", 60);
  opt.return_idle_s = cfg.num("sim.return_idle_s", 40);

  if (!world.load(maze, &err)) {
    fprintf(stderr, "maze error: %s\n", err.c_str());
    return 3;
  }
  hal_init();
  robot.init();
  rec_init(trace, result);
  sched_set_world_hook(world_advance);
  sched_init_main("loop");
  sched_start_watchdog(cfg.num("sim.watchdog_wall_s", 20));

  setup();
  rec_setup_done();
  for (;;) {
    loop();
    sched_busy(5);  // the Arduino core's main loop between two loop() calls
  }
}
