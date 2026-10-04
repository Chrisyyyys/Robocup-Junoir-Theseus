# What the simulator found in the robot code

Findings from running the code in `Main/` (as of commit 423e4e6) through the simulator on random
RCJ-style mazes and on the scenario mazes in `sim/mazes/scenarios/`. The suggested changes are
collected in [`examples/suggested-fixes.patch`](examples/suggested-fixes.patch), which applies to
`Main/` as of that commit.

**Update (2 October 2026):** `main` now includes branch `claude/practical-brahmagupta-y7saan` (merge
`b1acd52`). That branch fixed findings 2, 4 and 9 in its own way and has the other suggested changes
(findings 1, 3, 5, 6 and 10) in commits `7b01ff6` (bug fixes) and `5dbae59` (front-wall range and
steering limit, to tune on the robot). On the same 40 exploration mazes, `main` now gets lost in 5
runs instead of 39. Still open: the turn/return problems (finding 8 and the return trip ignoring
failed turns and moves), ramps (12-13), phantom obstacles (11) and the camera byte check (14).

**Update (2 October 2026, later):** branch `claude/pose-and-obstacle-fixes` closes the turn and return
problems (finding 8 and the return trip), replaces the obstacle avoidance (finding 11, which turned out
to be worse than "phantom") and adds findings 16-18. Still open: ramps (12-13), stairs and the camera
byte check (14). The numbers and the reasoning are in
[the design note](../docs/superpowers/specs/2026-10-02-pose-and-obstacle-fixes-design.md), what to check
on the robot is in [the test plan](../docs/superpowers/specs/2026-10-02-pose-and-obstacle-fixes-test-plan.md).
The simulator itself had a bug that made every full-field result with obstacles wrong (finding 18), and
it stopped a run too early on the way home, so the returning and full-field rows of the table below were
measured again.

The "See it" commands for findings 1-6, 9 and 10 now only show the problem on the older code. To
run that, extract it with `mkdir -p /tmp/old && git archive 423e4e6 Main | tar -x -C /tmp/old` and
add `--sketch /tmp/old/Main` to the command.

**How much to trust this.** The program logic (state machine, map, planner, timing, threads) runs
exactly as written, so logic findings are solid. Anything about motion depends on the motor and
sensor numbers in `sim/config/robot.cfg`, several of which are guesses until they are measured on
the robot. That includes where the distance sensors sit on the body: nobody has measured it, and
[Sensor geometry the simulator assumes](#sensor-geometry-the-simulator-assumes) lists what was assumed and how much
it changes the results. Each finding says which kind it is. Every finding can be reproduced with the command shown.

> **Update 4 Oct 2026.** The simulator now models the V2 robot from its CAD (195 x 180 mm over the wheels, sensors at their CAD positions) and the 2026 rules. Findings 1-18 and the tables below were measured with the earlier assumed robot (170 x 140 mm, side sensors 70 mm from the centre line); the section "Sensor geometry the simulator assumes" describes that earlier assumption. The current numbers and the new findings are in [the V2 note](../docs/superpowers/specs/2026-10-04-v2-robot-and-2026-rules.md).

## Results before and after the suggested changes

Same mazes, same seeds, same noise, so the only difference is the code. 8-minute runs on `basic`
mazes (walls, black / blue / silver tiles, victims) unless noted.

| | Original code (423e4e6) | With the suggested changes | `main` now (b1acd52) | `claude/pose-and-obstacle-fixes` |
|---|---:|---:|---:|---:|
| **Exploring** (40 mazes, move limit off): runs that got lost | 39 of 40 | 13 of 40 | 5 of 40 | 0 of 40 |
| runs with no problem at all | 1 | 11 | 20 | 30 |
| runs with wrong walls in the map | 39 | 28 | 16 | 10 |
| average tiles explored / victims found / estimated score | 71% / 57% / 99 | 79% / 62% / 114 | 77% / 60% / 110 | 74% / 57% / 102 |
| **Returning home** (40 mazes, the code's 25-move limit): ended on the start tile | 19 of 40 | 27 of 40 | 25 of 40 | 37 of 40 |
| runs that got lost | 35 | 10 | 4 | 1 |
| **Full field** (20 mazes with ramps, obstacles, stairs, bumps, debris): runs that got lost | 19 of 20 | 7 of 20 | 8 of 20 | 7 of 20 |
| ended on the start tile | 9 | 13 | 13 | 14 |
| runs where `loop()` used far too much stack (finding 2) | 0 | 0 | 0 | 0 |

The returning and full-field rows were measured again after two simulator fixes (findings 18 and the
`sim.return_idle_s` setting): the simulator used to end a run as "stopped" 8 s after the wheels stood
still in `RETURN`, which also caught a robot handling a victim on the way home, and a cylinder behind a
sensor read as an obstacle in front. That is why the start-tile counts are higher than in the first
version of this table, and why the 3 stack overflows of the original code (the deep `fwd()` /
`obstacleavoidance()` recursion of finding 2) no longer show up in these batches. The exploring rows
did not change: those mazes have no obstacles.

The changes in the patch cover findings 1-6 and 10. `claude/pose-and-obstacle-fixes` adds the turn
clearance (8), the obstacle look-ahead (11), a move length trim, a gyro hold without a right wall, the
camera-thread handshake (16), a stuck-move check and a way home that re-plans every tile. It gets lost in
none of these 40 exploring mazes and ends on the start tile in 37 of 40 returns. Its exploring score and
coverage are not higher than `main`'s (102 against 110 here; 118 against 112 and 106 against 106 on the other
160 mazes in the design note): the gain is reliability. Still open: ramps (12-13), stairs, and obstacles
beside the path (17).

Reproduce: `python sim/sim.py batch --count 40 --moves-limit off --seed-start 1` gives the exploring
rows of the last two columns, for `main` and for the branch; leave out `--moves-limit off` for returning
home, and use `--count 20 --profile full` for the full field. For the first two columns, add `--sketch
/tmp/old/Main` (the older code, see above), or `--sketch` with a copy of it that has the patch applied
(and the `(Direction)` cast of the smaller things below, which clang needs). All the batches here used
`--seed-start 1`; without it, every batch gets new random mazes.

"Lost" means that on arriving at a tile, the robot's belief (`x_pos`, `y_pos`, `currentDir`) did
not match the tile it was really on.

## Will crash or freeze the robot

### 1. Freeze at power-on (logic, confirmed)

`setup()` calls `calibrateSensor(2,80)` (`Main.ino:299`) **before** `init_dist()` has started the
distance sensors. After a cold power-on sensor 2 is not measuring, and because `setTimeout()` is never
called, `readRangeContinuousMillimeters()` waits forever. After the reset button the sensors are still
running from the last start, so it does not freeze. This matches "the robot waits sometimes".

- See it: `python sim/sim.py run --boot cold` ends with `hung` in `setup()`.
- Fix: call `calibrateSensor()` only after `init_dist()` (its result is not used anyway) and call
  `sensors[i].setTimeout(100)` in `init_dist()` so a missing sensor can never block forever.

### 2. Endless recursion between `fwd()` and `obstacleavoidance()` (logic, confirmed)

In the `FWD` step of `obstacleavoidance()` (`Distance.ino:611`), `fwd(remaining)` is called before
`steps` is reset to `TURN`. If that inner `fwd()` sees the "obstacle" again, `obstacleavoidance()`
continues straight in the `FWD` step and calls `fwd()` again, and so on. In the simulation this went
up to 198 nested calls (2 of 40 basic runs, 4 of 8 runs with real obstacles), which would overflow
the GIGA's stack and crash. It was often triggered by a wall seen at an angle (finding 11), not a
real obstacle.

- See it: batch reports show `Thread 'loop' used … KB of stack`; the Serial log shows `fwd step` over and over.
- Fix: set `steps = TURN;` before calling `fwd(remaining)`.

### 3. `timer::reset_delta_time()` has no `return` (logic, confirmed)

It is declared `double` but returns nothing (`timer.cpp:16`). That is undefined behaviour in C++.
With the GIGA's current compiler it probably just returns garbage, but newer GCC versions turn it
into a crash; the simulator's compiler crashed inside `fwd()` on the first move until it was told to
behave like the old one. Fix: return a value or make it `void`.

## Why the robot loses track of where it is

### 4. A move stopped early still counts as a whole tile (logic, confirmed)

After `fwd(TILE_MM)` the position is advanced unless a black tile or a pause stopped it
(`Main.ino:445-447`). But `fwd()` also stops early when both front sensors read 50 mm or less
(`movement.ino:183`). The robot then believes it moved a tile when it moved a few centimetres,
usually into a wall: in the simulation this was the first mistake in **27 of the 39** runs that got
lost.

- See it: `sim/mazes/scenarios/checkpoint_corridor.txt`; in the replay the purple "believed" square
  jumps into the wall.
- Fix: in `fwd()` note when the emergency stop ends the move before half a tile, and in
  `EXECUTE_MOVE` sense again instead of stepping forward (in the patch).

### 5. Moves come up short, and the front wall is then missed (logic + calibration)

`fwd()` slows down in proportion to the distance left and stops when `Scale*120 < 25`
(`movement.ino:254`), which is 46 encoder counts (12 mm) before the target. With a little wheel slip
each tile ends about 15-20 mm short. In a corridor nothing corrects this (`centerFrontBack()` only
uses a wall in front), so the error adds up: in the corridor scenario the robot arrived at 431, 714,
995 and 1276 mm instead of 450, 750, 1050 and 1350 mm.

`detectWall(0)` only reports a front wall when both front sensors read less than `MIN_DIST` (120 mm).
A centred robot sees the wall at about 60 mm, but once it is 60-70 mm short the reading is 130 mm and
the wall is "gone". The robot drives into it and finding 4 takes over.

- Fix: detect the front wall up to 200 mm (a wall in the current tile is never further; the next
  tile's wall is at least 360 mm away). Then `CENTERING` also re-centres the robot against it.
  Re-centring against the back wall (sensor 4) as well would remove the drift in long corridors.

### 6. The right-wall follower steers very hard (calibration: bench-test this)

`center()` returns `(80 - average gap) + (back - front)` and `center_PID` multiplies it by 2
(`movement.ino:18`) with no limit. Entering a corridor 30 mm off-centre gives -60 to -110: one side at
PWM 150 and the other at 20, so the robot yaws at about 30°/s and often ends the move more than 20°
off. Your design note for this code says it was not bench-tested yet. In the simulation, limiting
the correction to ±40 cut the runs that got lost from 28 to 13 (out of 40).

- Fix: `adjustment = constrain(center_PID.getPID(err), -40, 40);` (in the patch), then tune on the
  robot. A wall follower that falls back to gyro heading-hold when the wall reference disappears would
  be more robust still.

### 7. Walls are written down relative to `currentDir` even when the robot is crooked (logic)

`writeWallsToCurrentTile()` assumes the robot faces exactly `currentDir` (`navigation.ino:114-125`).
When a move ended 30-60° off (findings 6 and 8), the walls go into the map in the wrong directions,
and the planner then chooses moves into walls. Re-snapping to the nearest cardinal heading (and
`parallel()`) before `readWallsRel()` when the gyro is more than ~15° off would prevent this.

### 8. Turning on the spot next to a wall (logic + geometry)

A 170 x 140 mm robot (`ROBOT_LENGTH_MM` and `ROBOT_WIDTH_MM` in `Main.ino`; the length was 195 mm until
2 July, which would make the sweep 120 mm) sweeps a circle of 110 mm radius when it turns on the spot, so it must
be within about 34 mm of the tile centre to turn without touching a wall. Nothing re-centres the robot sideways
(the constants `LATERAL_TOL_MM`, `MAX_LATERAL_OFFSET_MM` suggest this existed once). When it is off
centre, the turn hits the wall, `absoluteturn()` gives up when its timer runs out,
`turnCompletedSuccessfully()` fails, and `BOTCHED_TURN_RECOVERY` tries the same turn again from the
same place. After 3 tries it re-plans, picks the same direction and starts over. This loop caused
most of the lack-of-progress restarts in the simulation.

- Fix ideas: back away from the nearer side wall (or drive half a tile back) before retrying a failed
  turn; after `MAX_BOTCHED_TURN_ATTEMPTS`, exclude that direction when re-planning.
- Measured on the original code (5641 turns of 60° or more): of the turns that started with a side wall
  closer than 110 mm to the centre, 99% failed (1139 of 1151); with a side wall further away, 7% (235 of
  3529). 26% of all turns failed, and a geometric test (does the rotated 170 x 140 mm rectangle hit a wall?)
  explains 88% of the failures.
- Fixed on `claude/pose-and-obstacle-fixes`: before a turn in place the robot shifts about 28 mm away from a
  side wall nearer than 46 mm (`ensureTurnClearance()`).
  Turns that start less than 110 mm from a wall fall from 20% of all turns to 0.5%, and failed turns from 26% to 3.6%.

### 9. `absoluteturn()` never stops at the target (logic, confirmed)

`TURN_TOL_DEG` is declared but never used (`movement.ino:311`). The loop always turns in the
direction it started in, with at least PWM 20, until its timer runs out (2 s per 90°). So every turn
takes the full time, where it ends depends on where the motors stall, and an overshoot is never
corrected. Stopping when the error is below `TURN_TOL_DEG`, and allowing the turn to reverse, would
make turns faster and more repeatable.

## Field elements

### 10. Leaving a checkpoint marks the next tile as a checkpoint (logic, confirmed)

`read_color()` marks the tile *ahead* (`x_pos/y_pos + currentDir`) as `CHECKPOINT` whenever it sees
silver (`color.ino:70-79`). At the start of a move off a silver tile the colour sensor is still over
it, so the next tile gets marked, and `x_checkpoint`/`y_checkpoint` are overwritten with it. After a
lack-of-progress restart the referee puts the robot on the real checkpoint, but the code believes it
is one tile further on. In the batches the robots marked 30 wrong checkpoints and 9 right ones.

- See it: `sim/mazes/scenarios/checkpoint_corridor.txt` (one right and one wrong checkpoint).
- Fix: only accept silver for the next tile in the second half of the move (in the patch), or mark
  the tile after the move ends, the way blue tiles are already handled.

### 11. Phantom obstacles at corners (logic + sensors)

`fwd()` starts obstacle avoidance when the front-left sensor reads 90 mm or less while the front-right
reads 120 mm or more (or the other way round). Approaching a wall slightly at an angle, or a wall
corner, does exactly that, so the robot sometimes starts an avoidance manoeuvre in a maze with no
obstacles, which is also what led into finding 2. Requiring the reading to persist for a few cycles,
or checking that the robot is square to the walls (`parallel()` first), would help.

- Measured on 40 full-field mazes that each contain a cylinder: the avoidance routine started 89 times, 52
  of them (58%) with nothing in front of the robot, and in 25 of the 26 runs in which the robot touched a
  cylinder it had not started in the 12 s before the first touch. It looks once, at the start of a move,
  at 90 mm, when an obstacle in the next tile is still 175 mm away. The emergency stop in the drive loop
  needs both front sensors at 50 mm or less, and an obstacle against a wall is seen by one. Replaced on
  `claude/pose-and-obstacle-fixes` by a look-ahead inside the drive loop (`OBSTACLE_STOP_MM`); see
  [the design note](../docs/superpowers/specs/2026-10-02-pose-and-obstacle-fixes-design.md#why-obstacle-avoidance-seemed-not-to-work-at-all).

### 12. Ramps of 20° or less are never noticed (logic, confirmed)

Climbing starts when `abs(modulus((int)pitch) - init_pitch) > 20` (`movement.ino:197`). A 20° ramp
reads 19-20°, so it never counts, and RCJ ramps can be anything up to 25°. Also `init_pitch` is
measured at the start of every move, and the robot stops on the ramp after each tile, so a move that
starts on the ramp never detects it either. Then `elevation()` is never called and the robot does
not know it changed floors.

- See it: `sim/mazes/scenarios/ramp_up.txt` (20°): no "climbing" lines in the Serial log.
- Fix: a lower threshold (10-12°), measured against the pitch on the start tile rather than at the
  start of each move.

### 13. After a ramp the robot is not centred on the tile (logic + calibration)

When climbing is detected (25° ramps), the robot stops just past the top edge plus the 300 ms
"compensating" drive, about 140 mm short of the centre of the first tile at the top. Its next move
then ends near the tile boundary and it loses count. The section counter also counts the first ramp
section early because the encoders still hold the counts from the approach (`movement.ino:222`).

### 14. The camera thread reacts to any byte (logic, depends on your OpenMV code)

`cameraTask` checks `readSerial1() != -1` (`Main.ino:211, 229`), but `classifyCamByte()` returns -2
for anything that is not H, S or U. If the OpenMV sends anything else (a "searching" byte, a colour
letter), the robot stops, blinks and waits up to 4 s for letters that never come. In the simulation,
an idle byte 10 times a second gave 20 false stops and made 10 moves take 168 s instead of 128 s.
Use `>= 0` in `cameraTask` if letters are the only thing that should stop the robot.

- See it: `python sim/sim.py run --set camera.idle_byte=L --set camera.idle_rate_hz=10`

### 15. The end of each move depends on the motors' minimum PWM (calibration)

Near the end of `fwd()` the PWM falls toward 20-25. If your motors need more than that to move,
the robot stops a little short of the target and only finishes when noise or the wall follower
adds a little PWM. With no wall on the right it can sit there until lack of progress. The default
simulator settings assume your motors still move at PWM 20; if the real robot sometimes hums just
before the end of a tile, this is why. Measure `drive.deadband_straight_pwm` (see the README).

## Found later

### 16. The robot drives on while the camera thread handles a victim (logic + timing, confirmed)

`cameraTask()` stops the wheels, raises `victimPending`, sleeps a fixed 10 ms and then holds `i2cMutex` for
the whole victim: `serviceCameraVictim()` reads the wall sensors and the camera and runs the dispenser,
5-9 s. `fwd()` only looks at `victimPending` at the top of its loop pass, and every `drive()`,
`fullstop()` and sensor read needs the same mutex. If `fwd()` is in the middle of a pass when the thread
stops the wheels, that pass ends with one more `drive()`, and the next call blocks on the mutex with the
motors running. The robot drives on for the whole victim, 2-4 tiles (up to 1.25 m in one move), while
the map advances one tile. `absoluteturn()` has the same pattern.

Whether it happens depends on the timing of the loop. On `main` it did not happen once in 6785 moves of
the plain mazes; with the other changes of `claude/pose-and-obstacle-fixes`, which shift the timing, it
happened in 34 of 7235 moves and was the largest single cause of lost robots until it was fixed.

- See it: in the current `Main/`, put the old sequence back in both places in `cameraTask()`
  (`i2cMutex.lock(); victimSide = 1; drivetrain.fullstop(); victimPending = true; i2cMutex.unlock();
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(10)); i2cMutex.lock(); serviceCameraVictim();
  i2cMutex.unlock();`, and the same with `victimSide = 2`), then
  `python sim/sim.py run --seed 727133 --moves-limit off --time-limit 30`. After `victim at left`
  (5 s) the robot drives from y = 1000 mm to y = 1940 mm inside one `[MOVE] result=OK`.
- Fix (in the branch): the thread waits until `fwd()` or `absoluteturn()` has stopped the wheels and says
  so (`victimAck`), and skips the victim if nobody does within 300 ms.

### 17. Obstacles beside the path are invisible to the front sensors (geometry, in the assumed mounting)

In the simulator the two front sensors sit 35 mm either side of the centre line and each sees a cone of about ±12°.
That position is an assumption, not a measurement (see [Sensor geometry the simulator
assumes](#sensor-geometry-the-simulator-assumes)). The body reaches 70 mm either side, and a cylinder of 40 mm radius touches it when its centre is less than 110
mm from the line. A cylinder whose centre is 90-110 mm to one side is in the body's way and outside what
the sensors see: they keep reading the far wall until the front corner touches it. After the look-ahead
of finding 11 this is what is left: in the 24 full-field runs (development and held-out mazes) where the
robot still touched a cylinder, 23 contacts happened while it drove forward, the cylinder mostly 95-126 mm
ahead of the centre and 72-110 mm to one side, and in 19 of the 24 both front sensors still read 250 mm
or more at the moment of contact. No change in the logic fixes it. If the real sensors are mounted as
assumed, two more sensors on the front corners, angled outwards, would.

- See it: `python sim/sim.py run --maze sim/mazes/scenarios/obstacle_open_room.txt --seed 2 --moves-limit off`
  (also seeds 4 and 6): the cylinder is brushed with the front corner, and the front sensors read the far wall.

### 18. Simulator: a cylinder behind a sensor was a hit at distance 0 (fixed)

`World::raycast()` clamped a negative entry distance to 0 for every cylinder the line of the beam passes
through, so a cylinder *behind* a sensor looked like one the sensor was inside and returned the minimum
range (about 30 mm). Whenever the robot turned its back on an obstacle, its front sensors read a phantom
obstacle in front. Contact counts, avoidance triggers and full-field results that involve obstacles were
affected; mazes without cylinders (the `basic` profile) were not. Fixed in `core/world.cpp`.

## Sensor geometry the simulator assumes

**Not measured on the robot.** `sim/config/robot.cfg` puts the seven VL53L0X sensors where the comments in the
code say they are (front pair, right front and back, left front and back, back). The numbers are guesses. The
repository holds no drawing, photo or measurement of how the sensors really sit on the bot (`docs/mechanical` is
an empty file), and the only statement from the team is which sensor is on which side
(`docs/superpowers/specs/2026-07-04-right-wall-follower-center-design.md`, "Sensor layout").

| `measure(n)` | Where | x mm (right of centre) | y mm (ahead of centre) | Points | Mux port |
|---:|---|---:|---:|---:|---:|
| 1 | front right | 35 | 85 | 0° | 1 |
| 7 | front left | -35 | 85 | 0° | 2 |
| 2 | right front | 70 | 50 | 90° | 0 |
| 3 | right back | 70 | -50 | 90° | 6 |
| 6 | left front | -70 | 50 | 270° | 3 |
| 5 | left back | -70 | -50 | 270° | 5 |
| 4 | back | 0 | -85 | 180° | 4 |

The beams are 40 mm above the floor (`tof.height_mm`, a guess) with a 25° cone (the VL53L0X field of view). The body
is 170 x 140 mm from `Main.ino`, but `ROBOT_LENGTH_MM` was 195 until commit `d5a3640` (2 July, "added pausing in
obstacle avoidance"; the comment `(52.5)` next to `TARGET_GAP_MM` still matches 195), so one of the two lengths is wrong.

What depends on these numbers: the blind band beside the front sensors' cones (finding 17); the 60 mm obstacle stop
(`OBSTACLE_STOP_MM`, kept below the reading at the foot of a ramp, which is about the beam height divided by the tangent of
the slope); the 46 mm side clearance before a turn (`TURN_SIDE_GAP_MIN_MM`, from the body size and the side sensors'
offset); the 59 mm front gap (`FRONT_GAP_AT_CENTER_MM`, from the front sensors' offset); and what every wall reading means.
The last two gaps are now worked out in `Main.ino` from `ROBOT_LENGTH_MM`, `ROBOT_WIDTH_MM`, `TOF_FRONT_FWD_MM` and
`TOF_SIDE_OUT_MM`, and come out the same as before at 170 x 140 mm (60 of 60 runs identical).

**How much it matters.** The original code and the fixed code on the same 100 mazes (development sets A1-A3: 40
exploring, 40 returning home, 20 full field), with the mounting changed. Each cell is runs lost / lack-of-progress
restarts / wall contacts:

| Real robot | Original | Fixed |
|---|---:|---:|
| as assumed above | 28 / 73 / 687 | 4 / 9 / 201 |
| front sensors at ±55 mm | 25 / 69 / 873 | 9 / 7 / 377 |
| front sensors at ±20 mm | 26 / 71 / 915 | 11 / 14 / 496 |
| beams 25 mm above the floor | 32 / 86 / 791 | 9 / 19 / 437 |
| beams 60 mm above the floor | 34 / 88 / 812 | 9 / 23 / 475 |
| front sensors toed out 15° | 27 / 82 / 965 | 7 / 9 / 474 |
| side sensors at ±35 mm along the body | 31 / 94 / 947 | 11 / 21 / 710 |
| 195 mm long, code still says 170 | 28 / 96 / 1461 | 14 / 16 / 760 |
| 195 mm long, code constants set to 195 | 25 / 104 / 1420 | 11 / 12 / 562 |

The fixes beat the original in every row, but they were developed and tuned in the first one. With another mounting the
fixed code loses 7-14 runs of 100 instead of 4 and touches walls two to four times as often. The body length matters
most, and the length constants have to match the robot. Measure the robot before relying on the other numbers in this file
([test plan, step 0](../docs/superpowers/specs/2026-10-02-pose-and-obstacle-fixes-test-plan.md#0-measure-the-robot)).

## Smaller things

- `dispenser.cpp:24`: the `Serial.println("sipensing left")` before the first `case` never runs.
- `movement.ino:28` passes an `int` where a `Direction` is expected; it only compiles because the
  Arduino build uses `-fpermissive`. (A cast is added on `claude/pose-and-obstacle-fixes`, because clang,
  which the simulator can be built with, does not accept it.)
- `PID` never initialises `prevError` and `cumError`, so the first output of every new `PID` object
  uses whatever was left on the stack. Initialise them in the constructor.
- The 25-move limit sends the robot home after roughly 200-300 s of an 8-minute run. In the
  simulation the robot managed about 60 moves in 8 minutes. A time-based return (the commented-out
  `mazeTime` check) would use the run better, once the return itself is reliable.
- Blue tiles are avoided by the planner and the robot waits 5 s on them. The 2026 rules, as far as
  the search results show, give points for visiting blue tiles. Check the official rules.
