# What the simulator found in the robot code

Findings from running the code in `Main/` (as of commit 423e4e6) through the simulator on random
RCJ-style mazes and on the scenario mazes in `sim/mazes/scenarios/`. Nothing in `Main/` was changed.
The suggested changes are collected in [`examples/suggested-fixes.patch`](examples/suggested-fixes.patch),
so you can review them and apply them with `git apply sim/examples/suggested-fixes.patch` if you
agree.

**Update:** branch `claude/practical-brahmagupta-y7saan` already fixed findings 2, 4 and 9 in its own
way. The other suggested changes (findings 1, 3, 5, 6 and 10) were added there in commits `7b01ff6`
(bug fixes) and `5dbae59` (front-wall range and steering limit, to tune on the robot). On the same 40
exploration mazes that branch went from 32 runs getting lost to 5. Still open: the turn/return
problems (finding 8 and the return trip ignoring failed turns and moves), ramps (12-13), phantom
obstacles (11) and the camera byte check (14).

**How much to trust this.** The program logic (state machine, map, planner, timing, threads) runs
exactly as written, so logic findings are solid. Anything about motion depends on the motor and
sensor numbers in `sim/config/robot.cfg`, several of which are guesses until they are measured on
the robot. Each finding says which kind it is. Every finding can be reproduced with the command shown.

## Results before and after the suggested changes

Same mazes, same seeds, same noise, so the only difference is the code. 8-minute runs on `basic`
mazes (walls, black / blue / silver tiles, victims) unless noted.

| | Original code | With the suggested changes |
|---|---:|---:|
| **Exploring** (40 mazes, move limit off): runs that got lost | 39 of 40 | 13 of 40 |
| runs with no problem at all | 1 | 11 |
| runs with wrong walls in the map | 39 | 28 |
| average tiles explored / victims found / estimated score | 71% / 55% / 99 | 79% / 61% / 114 |
| **Returning home** (40 mazes, the code's 25-move limit): ended on the start tile | 11 of 40 | 20 of 40 |
| runs that got lost | 35 | 10 |
| **Full field** (20 mazes with ramps, obstacles, stairs, bumps, debris): runs that got lost | 19 of 20 | 16 of 20 |
| ended on the start tile | 4 | 9 |
| runs where `loop()` used far too much stack (finding 2) | 3 | 0 |

The changes in the patch cover findings 1-6 and 10. What is left is mostly findings 8 (turning next to a
wall), 12-13 (ramps) and the tuning of the wall follower (6), which need work on the robot itself.

Reproduce: `python sim/sim.py batch --count 40 --moves-limit off`, then the same with
`--sketch <a copy of Main with the patch applied>`.

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

A 170 x 140 mm robot sweeps a circle of 110 mm radius when it turns on the spot, so it must be within
about 34 mm of the tile centre to turn without touching a wall. Nothing re-centres the robot sideways
(the constants `LATERAL_TOL_MM`, `MAX_LATERAL_OFFSET_MM` suggest this existed once). When it is off
centre, the turn hits the wall, `absoluteturn()` gives up when its timer runs out,
`turnCompletedSuccessfully()` fails, and `BOTCHED_TURN_RECOVERY` tries the same turn again from the
same place. After 3 tries it re-plans, picks the same direction and starts over. This loop caused
most of the lack-of-progress restarts in the simulation.

- Fix ideas: back away from the nearer side wall (or drive half a tile back) before retrying a failed
  turn; after `MAX_BOTCHED_TURN_ATTEMPTS`, exclude that direction when re-planning.

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

## Smaller things

- `dispenser.cpp:24`: the `Serial.println("sipensing left")` before the first `case` never runs.
- `movement.ino:28` passes an `int` where a `Direction` is expected; it only compiles because the
  Arduino build uses `-fpermissive`.
- `PID` never initialises `prevError` and `cumError`, so the first output of every new `PID` object
  uses whatever was left on the stack. Initialise them in the constructor.
- The 25-move limit sends the robot home after roughly 200-300 s of an 8-minute run. In the
  simulation the robot managed about 60 moves in 8 minutes. A time-based return (the commented-out
  `mazeTime` check) would use the run better, once the return itself is reliable.
- Blue tiles are avoided by the planner and the robot waits 5 s on them. The 2026 rules, as far as
  the search results show, give points for visiting blue tiles. Check the official rules.
