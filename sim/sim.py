#!/usr/bin/env python3
"""Theseus maze simulator: runs the robot code in Main/ against simulated mazes.

  python sim/sim.py run                       # one run on a random 'basic' maze
  python sim/sim.py run --maze sim/mazes/simple.txt --view
  python sim/sim.py batch --count 30 --profile basic
  python sim/sim.py gen --profile full --seed 4 -o mymaze.txt
  python sim/sim.py view sim/out/run/trace.json

Only needs Python 3 and a C++17 compiler (g++ or clang++). See sim/README.md.
"""

import argparse
import concurrent.futures
import datetime
import glob
import hashlib
import html
import json
import os
import platform
import shutil
import subprocess
import sys

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(SIM_DIR)
sys.path.insert(0, os.path.join(SIM_DIR, "tools"))

import mazegen  # noqa: E402
import sketch  # noqa: E402

BUILD = os.path.join(SIM_DIR, "build")
EXE = os.path.join(BUILD, "theseus_sim" + (".exe" if os.name == "nt" else ""))
DEFAULT_CONFIG = os.path.join(SIM_DIR, "config", "robot.cfg")
VIEWER = os.path.join(SIM_DIR, "viewer", "index.html")

IMPORTANT_WARNINGS = ("Wreturn-type", "Wuninitialized", "Wmaybe-uninitialized", "Wswitch-unreachable",
                      "fpermissive", "Warray-bounds", "Woverflow", "Wsequence-point", "Wimplicit-fallthrough")


# ---------------------------------------------------------------- build
def find_compiler():
    for c in (os.environ.get("CXX"), "g++", "clang++", "c++"):
        if c and shutil.which(c):
            return c
    sys.exit("No C++ compiler found. Install g++ (Linux: build-essential, Mac: Xcode command line tools, "
             "Windows: MSYS2 'pacman -S mingw-w64-ucrt-x86_64-gcc') or set CXX.")


def flag_ok(cxx, flag, cache):
    if flag in cache:
        return cache[flag]
    os.makedirs(BUILD, exist_ok=True)
    src = os.path.join(BUILD, "flagtest.cpp")
    with open(src, "w") as fh:
        fh.write("int main(){return 0;}\n")
    r = subprocess.run([cxx, "-std=gnu++17", "-Werror", flag, "-c", src, "-o", os.path.join(BUILD, "flagtest.o")],
                       capture_output=True, text=True)
    cache[flag] = r.returncode == 0
    return cache[flag]


def newer(target, sources):
    if not os.path.exists(target):
        return True
    t = os.path.getmtime(target)
    return any(os.path.getmtime(s) > t for s in sources if os.path.exists(s))


def run_compiler(cmd, what):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        sys.exit("\nBuild failed while compiling %s." % what)
    return r.stderr


def build(sketch_dir, quiet=False):
    """Compile the simulator core and the robot sketch. Returns the executable path."""
    cxx = find_compiler()
    obj = os.path.join(BUILD, "obj")
    os.makedirs(obj, exist_ok=True)
    cache_path = os.path.join(BUILD, "flags.json")
    try:
        with open(cache_path) as fh:
            cache = json.load(fh)
        if cache.get("_cxx") != cxx:
            cache = {"_cxx": cxx}
    except (OSError, ValueError):
        cache = {"_cxx": cxx}
    base = ["-std=gnu++17", "-g", "-I" + os.path.join(SIM_DIR, "stubs"), "-I" + os.path.join(SIM_DIR, "core")]
    # Robot code: like the Arduino build (-fpermissive), unoptimised so the code behaves like the robot's
    # older compiler; uninitialised locals start at zero so runs repeat exactly.
    robot_flags = base + ["-O0", "-fpermissive", "-I" + BUILD, "-I" + sketch_dir]
    for f in ("-ftrivial-auto-var-init=zero", "-fno-unreachable-traps", "-fno-strict-return"):
        if flag_ok(cxx, f, cache):
            robot_flags.append(f)
    with open(cache_path, "w") as fh:
        json.dump(cache, fh)
    core_flags = base + ["-O2", "-Wall"]

    headers = glob.glob(os.path.join(SIM_DIR, "core", "*.h")) + glob.glob(os.path.join(SIM_DIR, "stubs", "**", "*.h"), recursive=True)
    objects = []
    for src in sorted(glob.glob(os.path.join(SIM_DIR, "core", "*.cpp"))):
        o = os.path.join(obj, "core_" + os.path.basename(src)[:-4] + ".o")
        if newer(o, [src] + headers):
            if not quiet:
                print("compiling", os.path.relpath(src, REPO))
            run_compiler([cxx] + core_flags + ["-c", src, "-o", o], src)
        objects.append(o)

    # merge the .ino files like the Arduino IDE
    merged = os.path.join(BUILD, "sketch_merged.cpp")
    names_h = os.path.join(BUILD, "sketch_names.h")
    tmp_cpp, tmp_h = merged + ".new", names_h + ".new"
    sketch.merge_sketch(sketch_dir, tmp_cpp, "sim_probe.inc", tmp_h, display_root=REPO)
    for tmp, final in ((tmp_cpp, merged), (tmp_h, names_h)):
        if os.path.exists(final) and open(tmp, "rb").read() == open(final, "rb").read():
            os.remove(tmp)
        else:
            os.replace(tmp, final)
    warnings = []
    sketch_headers = glob.glob(os.path.join(sketch_dir, "*.h")) + headers + [os.path.join(SIM_DIR, "core", "sim_probe.inc")]
    robot_srcs = [(merged, "sketch")] + [(p, "main_" + os.path.basename(p).rsplit(".", 1)[0])
                                        for p in sketch.cpp_files(sketch_dir)]
    for src, name in robot_srcs:
        o = os.path.join(obj, name + ".o")
        warn_file = o + ".warnings"
        deps = [src] + sketch_headers + ([names_h] if name == "sketch" else [])
        if newer(o, deps) or not os.path.exists(warn_file):
            if not quiet:
                print("compiling", os.path.relpath(src, REPO))
            err = run_compiler([cxx] + robot_flags + ["-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-variable",
                                                      "-Wno-unused-but-set-variable", "-Wno-sign-compare", "-Wno-unused-function",
                                                      "-c", src, "-o", o], src)
            with open(warn_file, "w") as fh:
                fh.write(err)
        with open(warn_file) as fh:
            warnings.append(fh.read())
        objects.append(o)
    with open(os.path.join(BUILD, "robot_warnings.txt"), "w") as fh:
        fh.write("".join(warnings))
    if newer(EXE, objects):
        if not quiet:
            print("linking", os.path.relpath(EXE, REPO))
        run_compiler([cxx, "-o", EXE] + objects + ["-pthread"], "the simulator")
    return EXE


def important_warnings():
    path = os.path.join(BUILD, "robot_warnings.txt")
    if not os.path.exists(path):
        return []
    out, seen = [], set()
    lines = open(path).read().splitlines()
    for i, line in enumerate(lines):
        if ("warning:" in line or "error:" in line) and any(k in line for k in IMPORTANT_WARNINGS):
            key = line.split(" warning:")[0] if " warning:" in line else line
            if key in seen:
                continue
            seen.add(key)
            out.append(line.strip())
    return out


# ---------------------------------------------------------------- one run
def make_maze(args, seed, out_dir):
    if getattr(args, "maze", None):
        path = os.path.join(out_dir, "maze.txt")
        shutil.copyfile(args.maze, path)
        return path
    m = mazegen.generate(seed, args.profile, args.width, args.height)
    path = os.path.join(out_dir, "maze.txt")
    with open(path, "w") as fh:
        fh.write(m.to_text("generated by sim/sim.py (profile %s, seed %d)" % (args.profile, seed)))
    return path


def sim_command(exe, maze, seed, args, trace, result):
    cmd = [exe, "--maze", maze, "--config", DEFAULT_CONFIG, "--seed", str(seed), "--result", result, "--quiet"]
    for c in args.config or []:
        cmd += ["--config", c]
    for s in args.set or []:
        cmd += ["--set", s]
    if trace:
        cmd += ["--trace", trace]
    if args.time_limit:
        cmd += ["--time-limit", str(args.time_limit)]
    if args.boot:
        cmd += ["--boot", args.boot]
    if args.moves_limit:
        cmd += ["--moves-limit", str(args.moves_limit)]
    if args.no_lop:
        cmd += ["--lop", "off"]
    return cmd


def run_sim(cmd, timeout=600):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return -1, "the simulator took longer than %d s of real time" % timeout


def load_result(path, rc, log):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {"outcome": "crashed", "detail": "simulator exit code %s: %s" % (rc, log.strip()[-800:])}


def write_viewer(trace_path, out_html, title="Theseus run"):
    with open(VIEWER, encoding="utf-8") as fh:
        page = fh.read()
    with open(trace_path, encoding="utf-8") as fh:
        data = fh.read().replace("</", "<\\/")
    tag = '<script type="application/json" id="embedded-trace">'
    page = page.replace(tag + "</script>", tag + data + "</script>")
    page = page.replace("<title>Maze Replay</title>", "<title>%s</title>" % html.escape(title))
    with open(out_html, "w", encoding="utf-8") as fh:
        fh.write(page)
    return out_html


def fmt_summary(r):
    def pct(x):
        return "%d%%" % round(100 * x) if isinstance(x, (int, float)) else "-"
    lines = []
    lines.append("Outcome:      %s%s" % (r.get("outcome"), (" - " + r["detail"]) if r.get("detail") else ""))
    if "sim_time_s" not in r:
        return "\n".join(lines)
    lines.append("Time:         %.1f s simulated (setup took %.1f s)" % (r["sim_time_s"], r.get("setup_s", 0)))
    lines.append("Explored:     %d of %d reachable tiles (%s), %d moves, %.1f m driven" % (
        r["tiles_visited"], r["tiles_reachable"], pct(r["coverage"]), r["moves"], r["distance_m"]))
    lost = r["position_errors"] + r["heading_errors"]
    lines.append("Position:     %d checks, wrong position %d times, wrong heading %d times%s" % (
        r["position_checks"], r["position_errors"], r["heading_errors"],
        (", first lost at %.1f s" % r["first_lost_s"]) if r.get("first_lost_s") is not None else ""))
    lines.append("Alignment:    off-centre %.0f mm average (max %.0f), heading off %.1f deg average (max %.1f) on arrival" % (
        r["center_err_mm"]["mean"], r["center_err_mm"]["max"], r["heading_err_deg"]["mean"], r["heading_err_deg"]["max"]))
    m = r["map"]
    lines.append("Map:          %d tiles mapped, %d walls right, %d missed, %d phantom, %d tiles placed outside the maze" % (
        m["tiles"], m["walls_correct"], m["walls_missed"], m["walls_phantom"], m["tiles_outside_maze"]))
    v = r["victims"]
    lines.append("Victims:      %d of %d identified (%d misidentified), %d rescue kits dropped" % (
        v["identified"], v["total"], v["misidentified"], v["kits_dropped"]))
    lines.append("Field:        checkpoints %d/%d, holes entered %d, lack of progress %d, wall contacts %d" % (
        r["checkpoints"]["visited"], r["checkpoints"]["total"], r["holes_entered"], r["lack_of_progress"], r["contacts"]))
    e = r["end"]
    lines.append("End:          state %s, on start tile: %s, robot said it was home: %s" % (
        e["state"], "yes" if e["on_start_tile"] else "no", "yes" if e["robot_said_home"] else "no"))
    s = r["score"]
    lines.append("Score (est.): %d  (victims %d, kits %d, checkpoints %d, blue %d, exit %d)" % (
        s["total"], s["victims"], s["rescue_kits"], s["checkpoints"], s["blue_tiles"], s["exit_bonus"]))
    for w in r.get("warnings", []):
        lines.append("Warning:      %s%s" % (w["message"], (" (x%d)" % w["count"]) if w["count"] > 1 else ""))
    return "\n".join(lines)


def cmd_run(args):
    exe = build(args.sketch, quiet=args.quiet_build)
    seed = args.seed if args.seed is not None else 1
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = args.out or os.path.join(SIM_DIR, "out", "run-" + stamp)
    os.makedirs(out_dir, exist_ok=True)
    maze = make_maze(args, seed, out_dir)
    trace = os.path.join(out_dir, "trace.json")
    result = os.path.join(out_dir, "result.json")
    cmd = sim_command(exe, maze, seed, args, trace, result)
    if args.echo:
        cmd.append("--echo")
        print("running:", " ".join(cmd))
        rc = subprocess.call(cmd)
        log = ""
    else:
        rc, log = run_sim(cmd)
    r = load_result(result, rc, log)
    print()
    with open(maze) as fh:
        print("".join(l for l in fh if not l.startswith("#")))
    print(fmt_summary(r))
    warn = important_warnings()
    if warn:
        print("\nCompiler warnings worth a look in the robot code (all in sim/build/robot_warnings.txt):")
        for w in warn[:8]:
            print("  " + w)
    if os.path.exists(trace):
        html_path = write_viewer(trace, os.path.join(out_dir, "view.html"), "Theseus run - " + os.path.basename(maze))
        print("\nReplay: open %s in a browser" % html_path)
        if args.view:
            import webbrowser
            webbrowser.open("file://" + os.path.abspath(html_path))
    print("Files:  %s" % out_dir)
    return 0 if r.get("outcome") not in ("crashed", "hang", "deadlock") else 1


# ---------------------------------------------------------------- batch
def classify(r, expect_return):
    """Short list of problems found in a run."""
    p = []
    o = r.get("outcome")
    if o in ("crashed", "hang", "deadlock", "hung"):
        p.append(o)
        return p
    if r.get("position_errors", 0) or r.get("heading_errors", 0):
        p.append("lost")
    if r.get("holes_entered", 0):
        p.append("hole")
    if r.get("lack_of_progress", 0):
        p.append("lack of progress")
    m = r.get("map", {})
    if m.get("walls_missed", 0) or m.get("walls_phantom", 0) or m.get("tiles_outside_maze", 0):
        p.append("map errors")
    if expect_return and not r.get("end", {}).get("returned_home"):
        p.append("not home")
    if o == "stuck":
        p.append("stuck")
    return p


def _batch_job(job):
    exe, maze, seed, args_dict, out_dir, keep = job
    args = argparse.Namespace(**args_dict)
    trace = os.path.join(out_dir, "trace_%d.json" % seed)
    result = os.path.join(out_dir, "result_%d.json" % seed)
    rc, log = run_sim(sim_command(exe, maze, seed, args, trace, result))
    r = load_result(result, rc, log)
    r["_seed"] = seed
    r["_maze"] = os.path.relpath(maze, out_dir)
    return r


def cmd_batch(args):
    exe = build(args.sketch, quiet=args.quiet_build)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = args.out or os.path.join(SIM_DIR, "out", "batch-" + stamp)
    os.makedirs(out_dir, exist_ok=True)
    seeds = list(range(args.seed_start, args.seed_start + args.count))
    jobs = []
    args_dict = vars(args).copy()
    args_dict.pop("func", None)
    for seed in seeds:
        if args.maze:
            maze = os.path.join(out_dir, "maze_%d.txt" % seed)
            shutil.copyfile(args.maze, maze)
        else:
            m = mazegen.generate(seed, args.profile, args.width, args.height)
            maze = os.path.join(out_dir, "maze_%d.txt" % seed)
            with open(maze, "w") as fh:
                fh.write(m.to_text("generated by sim/sim.py batch (profile %s, seed %d)" % (args.profile, seed)))
        jobs.append((exe, maze, seed, args_dict, out_dir, args.keep_traces))
    results = []
    workers = args.jobs or max(1, (os.cpu_count() or 2))
    print("running %d simulations with %d workers..." % (len(jobs), workers))
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        for r in pool.map(_batch_job, jobs):
            results.append(r)
            print("  seed %-4d %-12s coverage %4s  %s" % (
                r["_seed"], r.get("outcome"), ("%d%%" % round(100 * r["coverage"])) if "coverage" in r else "-",
                ", ".join(classify(r, args.moves_limit not in ("off",))) or "ok"))
    expect_return = args.moves_limit not in ("off",)
    for r in results:
        r["_problems"] = classify(r, expect_return)
        trace = os.path.join(out_dir, "trace_%d.json" % r["_seed"])
        keep = args.keep_traces == "all" or (args.keep_traces == "problems" and r["_problems"])
        if os.path.exists(trace):
            if keep:
                write_viewer(trace, os.path.join(out_dir, "view_%d.html" % r["_seed"]), "Theseus seed %d" % r["_seed"])
                r["_view"] = "view_%d.html" % r["_seed"]
            else:
                os.remove(trace)
    with open(os.path.join(out_dir, "summary.json"), "w") as fh:
        json.dump(results, fh, indent=1)
    md = batch_report_md(results, args)
    with open(os.path.join(out_dir, "report.md"), "w") as fh:
        fh.write(md)
    with open(os.path.join(out_dir, "report.html"), "w") as fh:
        fh.write(batch_report_html(results, args))
    print()
    print(md)
    print("Report: %s" % os.path.join(out_dir, "report.html"))
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as fh:
            fh.write(md + "\n")
    crashed = [r for r in results if r.get("outcome") in ("crashed", "hang", "deadlock")]
    return 1 if crashed else 0


def batch_stats(results):
    ok = [r for r in results if "coverage" in r]
    n = len(results)
    def avg(k, f=lambda r, k: r[k]):
        return sum(f(r, k) for r in ok) / len(ok) if ok else 0
    s = {
        "runs": n,
        "no_problems": sum(1 for r in results if not r["_problems"]),
        "lost": sum(1 for r in results if "lost" in r["_problems"]),
        "hole": sum(1 for r in results if "hole" in r["_problems"]),
        "lop": sum(1 for r in results if "lack of progress" in r["_problems"]),
        "map": sum(1 for r in results if "map errors" in r["_problems"]),
        "not_home": sum(1 for r in results if "not home" in r["_problems"]),
        "crashed": sum(1 for r in results if r.get("outcome") in ("crashed", "hang", "deadlock", "hung")),
        "coverage": avg("coverage"),
        "score": avg("score", lambda r, k: r["score"]["total"]),
        "victims": avg("victims", lambda r, k: (r["victims"]["identified"] / r["victims"]["total"]) if r["victims"]["total"] else 1),
        "first_lost": [r["first_lost_s"] for r in ok if r.get("first_lost_s") is not None],
    }
    return s


def batch_report_md(results, args):
    s = batch_stats(results)
    n = s["runs"]
    lines = ["## Theseus simulator batch: %d runs (%s mazes)" % (n, "given" if args.maze else args.profile), ""]
    lines.append("| | runs | share |")
    lines.append("|---|---:|---:|")
    for label, key in (("No problems found", "no_problems"), ("Got lost (position or heading wrong)", "lost"),
                       ("Drove into a black tile", "hole"), ("Needed a lack-of-progress restart", "lop"),
                       ("Wrong walls in its map", "map"), ("Did not end on the start tile", "not_home"),
                       ("Crashed / hung", "crashed")):
        if key == "not_home" and args.moves_limit == "off":
            continue
        lines.append("| %s | %d | %d%% |" % (label, s[key], round(100 * s[key] / n) if n else 0))
    lines.append("")
    lines.append("Average coverage %d%%, victims found %d%%, estimated score %.0f." % (
        round(100 * s["coverage"]), round(100 * s["victims"]), s["score"]))
    if s["first_lost"]:
        fl = sorted(s["first_lost"])
        lines.append("When runs got lost, the first wrong position came after %.0f s (median)." % fl[len(fl) // 2])
    lines.append("")
    lines.append("| seed | outcome | coverage | lost at | map errors | holes | LoP | victims | score | problems |")
    lines.append("|---:|---|---:|---:|---:|---:|---:|---:|---:|---|")
    for r in results:
        if "coverage" not in r:
            lines.append("| %d | %s | - | - | - | - | - | - | - | %s |" % (r["_seed"], r.get("outcome"), r.get("detail", "")[:80]))
            continue
        m = r["map"]
        lines.append("| %d | %s | %d%% | %s | %d | %d | %d | %d/%d | %d | %s |" % (
            r["_seed"], r["outcome"], round(100 * r["coverage"]),
            ("%.0f s" % r["first_lost_s"]) if r.get("first_lost_s") is not None else "-",
            m["walls_missed"] + m["walls_phantom"] + m["tiles_outside_maze"], r["holes_entered"], r["lack_of_progress"],
            r["victims"]["identified"], r["victims"]["total"], r["score"]["total"], ", ".join(r["_problems"]) or "ok"))
    return "\n".join(lines) + "\n"


def batch_report_html(results, args):
    s = batch_stats(results)
    rows = []
    for r in results:
        cov = ("%d%%" % round(100 * r["coverage"])) if "coverage" in r else "-"
        view = ('<a href="%s">replay</a>' % r["_view"]) if r.get("_view") else ""
        lost = ("%.0f s" % r["first_lost_s"]) if r.get("first_lost_s") is not None else "-"
        probs = ", ".join(r["_problems"]) or "ok"
        cls = "ok" if not r["_problems"] else "bad"
        rows.append("<tr class='%s'><td>%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td><a href='%s'>maze</a> %s</td></tr>" % (
            cls, r["_seed"], html.escape(r.get("outcome", "")), cov, lost,
            r["score"]["total"] if "score" in r else "-", html.escape(probs), html.escape(r["_maze"]), view))
    return """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Simulator Batch Report</title><style>
:root{--bg:#fbfaf7;--fg:#1d1d1b;--mut:#6b6a66;--line:#e2dfd8;--ok:#2f7d4a;--bad:#b3412e}
@media (prefers-color-scheme:dark){:root{--bg:#191917;--fg:#ecebe6;--mut:#a3a19b;--line:#36352f;--ok:#6cc08b;--bad:#ff8a73}}
body{background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif;margin:0;padding:24px 16px;max-width:1000px;margin:auto}
h1{font-size:22px;margin:0 0 4px}p{color:var(--mut);margin:0 0 16px}table{border-collapse:collapse;width:100%%;font-variant-numeric:tabular-nums}
td,th{border-bottom:1px solid var(--line);padding:6px 8px;text-align:left}tr.ok td:nth-child(6){color:var(--ok)}tr.bad td:nth-child(6){color:var(--bad)}
.k{display:flex;gap:24px;flex-wrap:wrap;margin:16px 0 24px}.k div b{display:block;font-size:24px}a{color:inherit}
.wrap{overflow-x:auto}</style></head><body>
<h1>Simulator batch report</h1><p>%d runs on %s mazes, generated %s</p>
<div class="k"><div><b>%d%%</b>runs with no problems</div><div><b>%d%%</b>average coverage</div><div><b>%d</b>runs got lost</div>
<div><b>%d</b>runs entered a hole</div><div><b>%.0f</b>average score</div></div>
<div class="wrap"><table><tr><th>seed</th><th>outcome</th><th>coverage</th><th>first lost</th><th>score</th><th>problems</th><th>files</th></tr>%s</table></div>
</body></html>""" % (s["runs"], "given" if args.maze else html.escape(args.profile), datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
                     round(100 * s["no_problems"] / max(1, s["runs"])), round(100 * s["coverage"]), s["lost"], s["hole"],
                     s["score"], "".join(rows))


# ---------------------------------------------------------------- gen / view
def cmd_gen(args):
    m = mazegen.generate(args.seed if args.seed is not None else 1, args.profile, args.width, args.height)
    text = m.to_text("generated by sim/sim.py gen (profile %s, seed %s)" % (args.profile, args.seed))
    if args.output:
        with open(args.output, "w") as fh:
            fh.write(text)
        print("wrote", args.output)
    else:
        print(text)
    return 0


def cmd_view(args):
    out = args.output or os.path.splitext(args.trace)[0] + ".html"
    write_viewer(args.trace, out, "Theseus replay")
    print("wrote", out)
    if not args.no_open:
        import webbrowser
        webbrowser.open("file://" + os.path.abspath(out))
    return 0


def cmd_build(args):
    exe = build(args.sketch)
    print("built", exe)
    warn = important_warnings()
    if warn:
        print("\nCompiler warnings worth a look in the robot code:")
        for w in warn:
            print("  " + w)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, with_maze=True):
        p.add_argument("--sketch", default=os.path.join(REPO, "Main"), help="folder with the robot sketch (default: Main)")
        if with_maze:
            p.add_argument("--maze", help="maze file to use instead of a random one")
            p.add_argument("--profile", default="basic", choices=["walls", "basic", "full"],
                           help="random maze contents: walls only, basic (no ramps/obstacles) or full")
            p.add_argument("--width", type=int)
            p.add_argument("--height", type=int)
        p.add_argument("--config", action="append", help="extra config file (overrides sim/config/robot.cfg)")
        p.add_argument("--set", action="append", help="override one setting, e.g. --set drive.deadband_turn_pwm=60")
        p.add_argument("--time-limit", type=float, help="simulated seconds (default 480)")
        p.add_argument("--boot", choices=["warm", "cold"], help="cold = fresh power-on (default warm)")
        p.add_argument("--moves-limit", default=None, help="code (default), off, or a number of moves before returning")
        p.add_argument("--no-lop", action="store_true", help="end the run instead of a lack-of-progress restart")
        p.add_argument("--out", help="output folder")
        p.add_argument("--quiet-build", action="store_true")

    p = sub.add_parser("run", help="run one simulation")
    common(p)
    p.add_argument("--seed", type=int, help="random seed for the maze and the noise (default 1)")
    p.add_argument("--echo", action="store_true", help="print the robot's Serial output live")
    p.add_argument("--view", action="store_true", help="open the replay in the browser afterwards")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("batch", help="run many simulations and write a report")
    common(p)
    p.add_argument("--count", type=int, default=20)
    p.add_argument("--seed-start", type=int, default=1)
    p.add_argument("--jobs", type=int, help="parallel simulations (default: number of CPUs)")
    p.add_argument("--keep-traces", choices=["none", "problems", "all"], default="problems")
    p.set_defaults(func=cmd_batch)

    p = sub.add_parser("gen", help="write a random RCJ-style maze file")
    p.add_argument("--profile", default="basic", choices=["walls", "basic", "full"])
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--width", type=int)
    p.add_argument("--height", type=int)
    p.add_argument("-o", "--output")
    p.set_defaults(func=cmd_gen)

    p = sub.add_parser("view", help="turn a trace.json into a replay page")
    p.add_argument("trace")
    p.add_argument("-o", "--output")
    p.add_argument("--no-open", action="store_true")
    p.set_defaults(func=cmd_view)

    p = sub.add_parser("build", help="only compile")
    p.add_argument("--sketch", default=os.path.join(REPO, "Main"))
    p.set_defaults(func=cmd_build)

    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
