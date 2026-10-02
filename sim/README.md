# Theseus maze simulator

Runs the robot code in `Main/` **unchanged** against a simulated RoboCupJunior Rescue Maze, so you can
test navigation, mapping, victim handling and the return to start on a laptop or in GitHub Actions,
without the robot.

The simulator compiles the real sketch on your computer. Every call into the hardware (`measure()`'s
VL53L0X reads, the BNO055, the colour sensor, the motor shield, the camera UARTs, the pause switch,
the LED and the LCD) is answered by a simulated robot driving through a simulated maze. Your
`setup()`, `loop()`, state machine, PID loops, camera thread and pause thread all run exactly as
written, in virtual time, so an 8-minute run takes a few seconds.

What you get from a run:

- a replay page (`view.html`) showing the robot, its sensor beams, where it **thinks** it is, the
  map it builds next to the real maze (mistakes in red), its state, motors, sensors and Serial output
- a result summary: tiles explored, when (and why) it first got lost, map mistakes, victims found,
  holes entered, lack-of-progress restarts, an estimated RCJ score, and warnings about hardware misuse
- batch reports over many random RCJ-style mazes, so you can see how often something goes wrong
  and compare before/after a code change

**[FINDINGS.md](FINDINGS.md)** lists what the simulator found in the current robot code, with a patch
to try ([examples/suggested-fixes.patch](examples/suggested-fixes.patch)).

Results are **only as realistic as the numbers in `sim/config/robot.cfg`**. Several of them are
guesses (marked `GUESS`). See [Calibrating the simulator](#calibrating-the-simulator).

## Quick start

You need Python 3.8+ and a C++17 compiler.

- Linux: `sudo apt install g++ python3`
- Mac: `xcode-select --install`
- Windows: install [MSYS2](https://www.msys2.org/), then in the "MSYS2 UCRT64" shell run
  `pacman -S mingw-w64-ucrt-x86_64-gcc python`, or use WSL.

From the repository root:

```sh
python sim/sim.py run --maze sim/mazes/simple.txt --view    # one run, opens the replay
python sim/sim.py run --profile basic --seed 7               # one run on a random maze
python sim/sim.py batch --count 30 --profile basic           # 30 random mazes + report
```

The first run compiles everything (about 20 s); later runs only recompile what changed. Output goes
to `sim/out/` (not committed): `trace.json`, `result.json`, `maze.txt` and `view.html` per run;
`report.html`, `report.md` and replay pages for the problem runs per batch.

## Commands

| Command | What it does |
|---|---|
| `run` | One simulation. `--maze FILE` or a random maze (`--profile`, `--seed`). `--view` opens the replay, `--echo` prints the robot's Serial output live. |
| `batch` | Many simulations in parallel (`--count`, `--seed-start`, `--jobs`), then a report. `--keep-traces none\|problems\|all`. |
| `gen` | Write a random maze file: `python sim/sim.py gen --profile full --seed 3 -o my.txt` |
| `view` | Turn any `trace.json` into a replay page. |
| `build` | Compile only, and list compiler warnings found in the robot code. |

Options shared by `run` and `batch`:

| Option | Meaning |
|---|---|
| `--profile walls\|basic\|full` | Random maze contents. `basic` = walls, black/blue/silver tiles and victims (no ramps, obstacles, bumps or stairs). `full` = everything. |
| `--moves-limit code\|off\|N` | `code` keeps `if(iterator >= 25) state = RETURN;` as written. `off` stops it from ever triggering, so the robot explores until the time limit. `N` sends it home after N moves. The simulator does this by resetting the `iterator` variable while the program runs; your code is not edited. |
| `--time-limit S` | Simulated seconds (default 480 = an 8-minute run). |
| `--boot warm\|cold` | `warm` (default): the sensors kept power, as after the reset button. `cold`: fresh power-on. See finding 1 in [FINDINGS.md](FINDINGS.md). |
| `--no-lop` | End the run instead of simulating a lack-of-progress restart. |
| `--set key=value` | Override any config value, e.g. `--set drive.deadband_turn_pwm=50`. |
| `--config FILE` | Extra config file applied on top of `robot.cfg` (e.g. your measured values). |
| `--sketch DIR` | Simulate another sketch folder (default `Main`), e.g. a copy with a fix you want to try. |

Useful combinations:

```sh
python sim/sim.py batch --count 50 --moves-limit off          # how well does it explore and map?
python sim/sim.py batch --count 50                            # does the return to start work?
python sim/sim.py batch --count 20 --profile full             # ramps, obstacles, stairs, bumps, debris
python sim/sim.py run --maze sim/mazes/simple.txt --boot cold # reproduce the start-up freeze
```

To check that a code change helps, run the same batch (same `--seed-start` and `--count`) before
and after: the mazes and the noise are identical, so differences come from your change. You can
keep your current code and try the change on a copy:

```sh
mkdir -p /tmp/fixed && cp -r Main /tmp/fixed/        # keep the folder name Main (like Main.ino)
patch -d /tmp/fixed -p1 < sim/examples/suggested-fixes.patch   # or edit /tmp/fixed/Main by hand
python sim/sim.py batch --count 40 --moves-limit off                          # current code
python sim/sim.py batch --count 40 --moves-limit off --sketch /tmp/fixed/Main # the changed copy
```

## The replay page

- **What really happened**: the true maze, the robot (orange) with its distance-sensor beams
  (teal), the path driven, and the tile the robot *believes* it is on (purple, dashed red when wrong).
- **What the robot believes**: the robot's own map (`mapGrid`) drawn in maze coordinates. Walls it
  recorded are dark; walls it recorded that do not exist are red; real walls it missed are dashed red.
  Tiles placed outside the maze mean its position belief was off when it mapped them.
- **Timeline**: robot state over time (colours), events (triangles: lost, lack of progress, victims),
  red bars where the position belief was wrong on arrival. Click to jump. Space plays and pauses,
  arrow keys step 1 s (Shift: 10 s), `[` / `]` jump to the previous / next tile arrival.
- **Serial monitor**: the robot's own `Serial` output at the current time, with a filter box.
- **Events** and **Results**: everything the evaluator noticed, and the score breakdown.

## Maze files

Mazes are text drawings, 3 characters per tile between `+` corners (see `sim/mazes/`):

```
name Example
+---+---+---+
| S       . |
+   +---+   +
|H. | X   C |
+---+---+---+
victim 2 0 N S
obstacle 1 0 0 -80 40
start_dir E
ramp_angle 20
```

- Walls: `---` and `|`; spaces mean no wall.
- Middle character of a tile: `S` start, `.` or space white, `X` black (hole), `B` blue, `C` silver
  checkpoint, `^ > v <` ramp going **up** in that direction (consecutive arrows form one ramp; give
  ramp tiles walls on both sides), `T` stairs, `b` speed bump, `d` debris, `O` obstacle, `#` not part
  of the field.
- A victim letter left or right of the middle character sits on that tile's west or east wall:
  `H S U` (letters) or `R Y G` (colour victims). Victims on north/south walls go in a
  `victim COL ROW N|S|E|W TYPE` line. COL and ROW count from the top-left tile, starting at 0.
- `obstacle COL ROW DX DY RADIUS` places a cylinder (DX/DY in mm from the tile centre, +x east,
  +y north); `box COL ROW DX DY W H` places a box.
- `start_dir` is the direction the robot faces at the start (its "NORTH"). Default: the first open side.
- Heights of the floors are worked out from the ramps, so an upper area just needs a ramp leading to it.

`sim/mazes/scenarios/` holds small mazes that each test one behaviour (a checkpoint, a black tile, a blue
tile, a wall that ends, a ramp). They make good quick checks after a change.

## Calibrating the simulator

These numbers decide how realistic the motion is. Put measured values in your own config file and
pass it with `--config`, or edit `sim/config/robot.cfg`.

| Setting | How to measure it on the robot |
|---|---|
| `drive.deadband_turn_pwm` | Call `drivetrain.turnright(x)` with x = 10, 15, 20, … and note the first value that makes the robot rotate. Or look at the robot's Serial output after turns: `turn target=…, actual=…, err=E` with a typical error E means the turn stalls where 4.5 × E ≈ this value. **This decides how `absoluteturn()` behaves**, because that loop only stops on its timer. |
| `drive.deadband_straight_pwm` | Same with `drivetrain.fw(x)`. |
| `drive.max_speed_mm_s` | Run `drivetrain.fw(255)` for 2 s on the field and measure the distance. |
| `drive.track_mm`, `drive.wheelbase_mm` | Measure between wheel centres. |
| `drive.skid_factor` | Turn at `turnright(150)` for 1 s, read the heading change H (deg). Then skid_factor = (2 × speed at PWM 150 × 1 s) / (track × H in radians). |
| `tof.N` | Position (mm right of centre, mm forward of centre) and direction of each distance sensor. |
| `color.white/black/blue/silver` | The `r g b c=` numbers `read_color()` prints while the robot sits on each kind of tile. |
| `color.position` | How far in front of the robot's centre the colour sensor is. |
| `camera.idle_byte` | If the OpenMV cameras send a byte (like `L`) when they see no victim, put it here. `cameraTask` reacts to any byte, so this matters. |
| `gyro.start_offset_deg` | What `myGyro.heading()` reads right after power-on. The code assumes about 0. |

A good check after calibrating: drive the real robot one tile, one 90° turn and one 180° turn with
Serial connected, run the same moves in the simulator (`--echo`), and compare the `[FWD]` and `[TURN]` lines.

## What is simulated, and how faithfully

| Part | Model | Confidence |
|---|---|---|
| Robot program | Your sketch compiled for PC, run with the same thread priorities as on the GIGA (camera thread above `loop()`), mutexes that block like mbed's, and the real time cost of every I2C transfer, library delay and sensor wait. | High |
| Distance sensors | 7 VL53L0X behind the mux on their ports, ~33 ms per reading (`readRangeContinuousMillimeters()` waits for a fresh one, like the Pololu library). Each reading combines 9 beams over a 25° cone, weighted by beam strength, angle to the surface and distance, plus noise, per-sensor bias and occasional dropouts. | Medium |
| Motors and motion | Four motors (left A/C, right B/D, D mounted reversed), PWM deadband (higher when turning on the spot), motor lag, per-motor speed mismatch, skid steering, wheel slip, random heading wander, slowing on ramps, collisions with walls/obstacles (sliding along them), wheels spinning when pushing a wall. | Medium. **Depends on calibration.** |
| Encoders | Pulses on the real pins call your interrupt routines (`encoder_update_A` etc.), so their direction logic runs as written. | High |
| Gyro (BNO055) | Heading 0–360 clockwise from the start direction, 1/16° steps, noise and slow drift; pitch on ramps, bumps and stairs. | Medium |
| Colour sensor | Raw r g b c of the floor under the sensor, scaled for the integration time and gain, with noise; includes the 25 ms the Adafruit library waits per read. | Medium. Replace the colour values with real readings. |
| Cameras | One byte per frame (20 fps) of the victim letter while a victim on a wall faces the camera within 26 cm and ±30°, with occasional misreads. | Low. Your OpenMV code isn't simulated, only what it sends. |
| Field | 30 cm tiles, walls 12 mm thick on tile edges, black/blue/silver tiles, ramps (heights worked out automatically), stairs and speed bumps (2 cm), debris (extra slip), obstacles, letter and colour victims on linear and floating walls. | High for geometry |
| Referee | Lack of progress after 30 s without progress or on entering a black tile: the robot is lifted, the pause switch is flipped, and it is put back on the last visited checkpoint. | Approximate |
| Score | Approximate RCJ 2026 rules (victims 10/30, kits, checkpoints, blue tiles, bumps, ramps, exit bonus). The point values are in the config. | Approximate |

Not simulated: the dispenser mechanism itself (only the time the stepper takes), the LED strips,
real camera images, electrical problems, I2C errors, battery sag over a run, and sensors seeing the
floor or ramp edges through the vertical part of their cone.

## How it works

- `tools/sketch.py` merges the `.ino` files like the Arduino IDE (main sketch first, then the others
  alphabetically, with generated prototypes) and appends `core/sim_probe.inc`, which gives the
  simulator read access to the sketch's variables (`state`, `x_pos`, `y_pos`, `currentDir`,
  `mapGrid`, …). **If you rename one of those variables, update `core/sim_probe.inc` too.**
- `stubs/` contains PC versions of `Arduino.h`, `Wire.h`, `mbed.h`/`rtos.h` and the libraries
  (VL53L0X, Adafruit TCS34725 / BNO055 / Motor Shield, SparkFun mux, Stepper, LiquidCrystal,
  ArduinoQueue, Vector). If you add a library to the robot, add a stub here.
- `core/sched.cpp` runs each RTOS thread as a real thread but lets only one run at a time, switching
  between them in virtual time. Runs are deterministic: the same seed gives the same run.
- `core/robot.cpp` is the robot model, `core/world.cpp` the maze and ray casting,
  `core/hal.cpp` the Arduino/library functions, `core/recorder.cpp` the evaluation and trace.
- The robot code is compiled with `-fpermissive` (as the Arduino build does), without optimisation, and
  with uninitialised local variables set to zero, so behaviour is repeatable. Compiler warnings
  about the robot code are saved to `sim/build/robot_warnings.txt`; `sim.py run` lists the important ones.
- A watchdog stops a run if the robot code loops forever without calling anything that takes time.

## Continuous testing

`.github/workflows/simulator.yml` builds the simulator and runs batches on every push and pull request
that touches `Main/` or `sim/`. The tables appear on the workflow run's summary page and the full reports
(with replay pages for the problem runs) can be downloaded as the run's artifact. The workflow
only reports; it does not fail because the robot got lost, only if the simulator itself crashes.
