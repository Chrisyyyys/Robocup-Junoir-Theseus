# Test plan: pose, victim and obstacle fixes

**Branch:** `claude/pose-and-obstacle-fixes` (from `main` at `8fd4316`)
**Fixes under test:** #1 move length (`FWD_TRIM_MM`), #2 heading hold without a right wall, #3 side clearance before turning in place, #4 camera thread waits for the wheels to stop, #5 obstacle look-ahead, #6 the way home, #7 stuck-move check.
**Why these fixes, and what they did in the simulator:** [the design note](2026-10-02-pose-and-obstacle-fixes-design.md). This plan is only about confirming them on the robot.
**Not fixed on this branch:** the gaps at the end.
**The robot:** the V2 robot as its CAD draws it ([the V2 note](2026-10-04-v2-robot-and-2026-rules.md)). Sections 0 and 3 were rewritten for it; the rest is unchanged.

The simulator's motor, sensor and floor numbers are estimates until measured (`sim/config/robot.cfg`), and the robot's shape and sensor positions come from its CAD, not from the robot. Do [step 0](#0-check-the-v2-numbers) first. Then these have to be tuned on the robot: `FWD_TRIM_MM` (section 1), the geometry numbers (step 0 and section 3), `OBSTACLE_STOP_MM` (section 5) and `MOVE_TIMEOUT_MS` (section 7). Do those sections first, in that order. The other sections only check behaviour.

## Before you start

1. Flash the branch and open the Serial Monitor at 115200 baud. Leave `VERBOSE_DEBUG` at `0` (top of `Main.ino`). Every test is judged from the log lines below.
2. Start the way the [heading test plan](2026-09-25-heading-and-move-fixes-test-plan.md) describes: power on with the logic switch at **PAUSE**, place the robot square on the start tile, keep it still for about 2 s, flip to **RUN**.
3. Where a test says **Baseline**, you can run it once on `main` first to see the old failure.

New log lines (the lines of the earlier plan, `[PLAN]`, `[TURN]`, `[CHECK]`, `[SYNC]`, `[MOVE]`, still appear):

| Log line | Meaning |
|---|---|
| `[MOVE] result=… exit=… ms=…` | `ms` is how long the move took, in milliseconds of driving (standing still for a victim is not counted). A clean tile takes about 3.6 s in the simulator. |
| `[CLEAR] gaps r=… l=… f=… b=… -> shift lat=… lon=…` | Before a turn in place, the right, left, front or back sensors read under the limit that lets the body swing round (`TURN_SIDE_GAP_MIN_MM`, `TURN_END_GAP_MIN_MM`; 999 = no wall there). The robot is about to move to the middle: `lat` mm sideways (+ = right), `lon` mm along the path (+ = forward). About one turn in two shows this line. |
| `[CLEAR] turn failed with room on the sensors, backed off … mm` | A turn failed although the sensors saw room: something they cannot see (an obstacle straight ahead) is in the way. The robot backed off before it tries again. |
| `[OBST] closest front reading …` | The look-ahead stopped the robot because something was close in front, and this is what three confirming readings of the front sensors gave (mm). |
| `[MOVE] result=BLOCKED exit=obstacle` | The move ended in front of an obstacle. The robot backed off to where the move began. The edge is recorded as blocked (`[MOVE] blocked edge recorded …`) and the next `[PLAN]` line picks another way. |
| `[MOVE] stuck after … ms` | A move took longer than `MOVE_TIMEOUT_MS`: the wheels were spinning against something. The robot stopped where it was (`exit=stuck`, `result=BLOCKED`). |
| `[MOVE] move failed, trying that edge once more` | Follows a `stuck`: the edge is not blocked yet. The robot squares up, senses again, and the planner normally tries the same edge again. If that fails too, `blocked edge recorded` follows. |
| `[MOVE] stuck stops in a row: stuck check switched off …` | Three stuck stops in a row: `MOVE_TIMEOUT_MS` is probably too short for this robot, and the check stays off until the next start. |
| `[RETURN] move limit reached, heading for the start tile` | The 25-move limit was hit. From here every `[PLAN]` line is a step toward the start tile. |
| `[RETURN] back on the start tile` | The robot's own position says it is home. `back to start` is shown on the display only now. |

Directions are numbers: 0 = North, 1 = East, 2 = South, 3 = West.

## 0. Check the V2 numbers

The simulator, and every number in the [V2 note](2026-10-04-v2-robot-and-2026-rules.md), uses the robot as the CAD in github.com/Arsur24/Theseus draws it (the Fusion archive `V2.f3z`, read with `sim/tools/cad_sensors.py`). Nobody has measured the built robot against it. Measure with a ruler, from the point the robot turns around (halfway between the front and back wheels and between the left and right wheels), and compare:

| Measure | The CAD says | Where it goes |
|---|---|---|
| Length, front to back, and width over the wheels | 195 × 180 mm (the chassis plates are 138 mm wide) | `ROBOT_LENGTH_MM`, `ROBOT_WIDTH_MM` in `Main.ino`; `robot.length_mm`, `robot.width_mm` in `sim/config/robot.cfg` |
| Which end is the front? | the end with the pair of distance sensors (the code has two front sensors, 1 and 7, and one back sensor, 4) | if it is the other end, renumber the sensors in `portMap[]` |
| Front sensors 1 and 7: sideways offset, how far ahead of the centre, angle, window height above the floor | ±80 mm, 97.5 mm (on the front face), straight ahead, 106 mm | `TOF_FRONT_FWD_MM`; `tof.1`, `tof.7` |
| Side sensors 2, 3 (right front and back), 6, 5 (left front and back): sideways offset, forward or back offset, height | ±90.2 mm (flush with the wheels' outer faces), ±88 mm, 106 mm | `TOF_SIDE_OUT_MM`; `tof.2`, `tof.3`, `tof.6`, `tof.5` |
| Back sensor 4: offset, height | middle of the back face, 43 mm | `tof.4` |
| Walls of your field: free path between two opposite walls | 280 mm (rules 2026, 3.3.3) | `WALL_THICK_MM` (300 minus the path) |

**The side-sensor offset and the body width are the two that matter most.** In the simulator, with the side sensors 5 mm nearer the centre line than `TOF_SIDE_OUT_MM` the final code lost 20 of 80 runs instead of 9, with 10 mm nearer it lost 52 (the original code 26); an offset that is too small (the sensors further out than the number says) cost nothing. Measure the offset to within 3 mm, and if in doubt use the smaller value. A body 10 mm wider than `ROBOT_WIDTH_MM` leaves 3.8 mm of room to turn and brought 10.9 wall contacts per run instead of 3.3.

`TARGET_GAP_MM`, `TARGET_SIDE_GAP_MM`, `FRONT_GAP_AT_CENTER_MM`, `TURN_SIDE_GAP_MIN_MM` and `TURN_END_GAP_MIN_MM` are worked out from these in `Main.ino`, so you do not set them yourself. Put the same numbers in `sim/config/robot.cfg`, rerun the batches in the V2 note (section 8) and compare: with the CAD numbers the final code lost 5 of 120 explore mazes and needed 1 restart; the sensitivity table (V2 note, section 6) shows what a different mounting does. A top and a side photo with a ruler in the frame also work.

A check that needs no ruler: with the robot centred in a corridor tile the side sensors should read about 50 mm and the front and back sensors, facing a wall, about 42 mm (print `measure(2)`, `measure(1)`, `measure(4)`). If they read 20 mm more or less, the offsets above are wrong by that much.

## 1. Move length (#1)

`fwd()` stops 12 mm before its target (`Scale*120 < 25`) and wheel slip costs a few more millimetres, so a move asked for one tile used to end about 19 mm short in the simulator, and in a corridor nothing made it up. `FWD_TRIM_MM` adds a fixed amount to every move.

**1a. Calibrate `FWD_TRIM_MM`.**
1. Take a straight corridor of at least 6 tiles with no front wall nearer than 2 tiles beyond the 5th tile, on the floor you compete on. Tape a line on the floor at the front of the robot's start position.
2. Run the robot through 5 tiles (watch the `[MOVE] result=OK` lines) and measure how far the front of the robot moved.
3. `FWD_TRIM_MM = (1500 - distance in mm) / 5`. Change it in `Main.ino`, flash and repeat once.
- **Pass:** 5 tiles end within ±10 mm of 1500 mm.
- **Baseline:** with `FWD_TRIM_MM` at 0 the robot ends about 95 mm short after 5 tiles in the simulator.

**1b. A tile at a time.** In a corridor with walls on both sides, read the `[WALLS]` line after each tile.
- **Pass:** `F=1` appears on the tile that really has a front wall, and the front-wall tile is reached without the `[FWD] emergency-stop` line.

## 2. Heading hold without a right wall (#2)

With no right wall to follow, nothing steered the robot, so motor mismatch and wheel slip turned into heading drift (3-5° per move in the simulator) and then into sideways drift. Now `center()` tells `fwd()` whether it had a right wall (`centerHasWall`); without one, `fwd()` holds the heading the move started on with the gyro, limited to the same ±40 as the wall follower.

**2a. Open on the right.** Build a 4-tile straight run with a wall on the left only (open on the right, no wall on the far side either for the last 2 tiles).
- **Pass:** the `[SYNC] facing=… err=…` after each move reads under 4° and the robot still ends each move within about 30 mm of the tile centre (check with a ruler).
- **Baseline:** on `main` the heading drifts about 3-5° per move in the simulator, so after a few moves the robot is visibly angled.

**2b. A wall that appears and disappears.** Drive along a right wall that has a 1-tile gap in the middle.
- **Pass:** no swerve when the wall reappears after the gap, and no `[CHECK] … ok=0` on the next turn.

## 3. Room to turn on the spot (#3)

A 195 × 180 mm body turning in place swings its corners around a circle of 132.7 mm radius. In a 280 mm path the middle is 140 mm from a wall face, so there are only 7 mm of room on each side: a turn from more than 7 mm off the middle stalls against a wall at about 45°, which is the "stops at ~45° instead of 90°" in the team's Sprint #2 notes. The same goes along the path in front of a dead end or a corner. Before every turn in place the robot reads the six side, front and back sensors (three rounds, averaged). If a side reading is under `TURN_SIDE_GAP_MIN_MM` (45 mm) or a front or back reading under `TURN_END_GAP_MIN_MM` (38 mm) it first moves to the middle: sideways with a two-leg shift (point 9° away, drive a leg, point back, drive the same distance back, square up; the leg length is chosen from the shift needed), or along the path with a straight leg. Up to three tries. After a turn that failed although the sensors saw room it backs off `BOTCH_BACKOFF_MM` (45 mm) before the next try.

**3a. Check the numbers.** Centred in a corridor tile, the side sensors read about 50 mm (`TARGET_SIDE_GAP_MM`) and, facing a wall, the front and back sensors about 42 mm (`TARGET_GAP_MM`). The VL53L0X cannot measure under 30 mm. If the readings are different, correct `TOF_SIDE_OUT_MM`, `TOF_FRONT_FWD_MM` or `WALL_THICK_MM` (step 0): the limits follow.

**3b. A turn next to a wall.** Put the robot square in a corridor tile facing a dead end, 15 mm off the middle towards the right wall, and let it plan its turn.
- **Pass:**
  - The log shows `[CLEAR] gaps r=… l=… f=… b=… -> shift lat=…` with `r` under 45 and `lat` about -10 to -15.
  - The robot ends within 5 mm of the middle (a second `[CLEAR]` line only if it was well off).
  - The turn finishes with `[CHECK] turn … ok=1`.
- **Baseline:** on `main` the turn stalls at about 45° (`[CHECK] turn … ok=0`), then `botched turn detected`, the same turn from the same place, and after three tries a re-plan that picks the same turn again.

**3c. Too close to the end wall.** Put the robot square in a dead-end tile 15 mm further forward than the middle (the front sensors read about 27 mm) and let it plan its turn.
- **Pass:** `[CLEAR] … f=2x … lon=-1x`, the robot backs up, and the turn finishes with `[CHECK] turn … ok=1`.

**3d. A turn that fails anyway.** Put a cylinder (about 80 mm across) against the end wall of a dead end, straight ahead, and let the robot drive in until it stops (the front sensors read the wall behind it).
- **Pass:** a turn that stalls once, then `[CLEAR] turn failed with room on the sensors, backed off 4x mm` and a turn with `[CHECK] turn … ok=1`. Without the back-off the same turn is retried from the same spot until the 30 s restart.

**3e. Count.** In a full run count the `[CHECK] turn … ok=0` lines (the `ok=` of `[TURN] done` is only the 3° settling test and is often 0 on a good turn) and the `[CLEAR] gaps` lines.
- **Pass:** `[CHECK] … ok=0` on fewer than 1 turn in 15 (in the simulator with this body the original code failed 60% of its turn checks, the final code 3%). `[CLEAR] gaps` appears before about one turn in two; if it appears before almost every turn, the offsets in step 0 are probably wrong.

## 4. Victim stop (#4)

While the robot drives, a camera thread notices a victim, and the robot must stand still for the 5-9 seconds the victim takes. The thread used to wait a fixed 10 ms and then take the I2C bus. Every motor command needs that bus, so if the movement code was in the middle of a loop pass it issued one more drive command, then waited for the bus with the motors running, and the robot drove 2-4 tiles on its own while "handling" the victim. Now the thread waits until `fwd()` or `absoluteturn()` has stopped the wheels and says so (`victimAck`).

**4a. A victim in the middle of a move.** Put a letter victim on a wall about 100 mm after the start of a tile (so the camera sees it half a tile into the move) and let the robot drive past it.
- **Pass:**
  - The robot stops within a few centimetres of where the victim appears and stays there while the kit is dispensed.
  - It then finishes the same move: one `[MOVE] result=OK` for the tile, and the next `[PLAN]` line shows the position one tile further.
- **Baseline:** on `main` the robot can keep driving while it dispenses, pass one or more tiles, and the next `[WALLS]` lines do not match the map (in the simulator it did not happen on `main`, but did as soon as the other changes shifted the loop timing, so check this with all the changes in).

**4b. A victim during a turn.** Place a victim where the camera sees it while the robot turns in place.
- **Pass:** the robot stops turning while it handles the victim and then finishes the turn (`[TURN] … ok=1`).

**4c. A victim just as the move ends.** Put the victim at the end of a tile.
- **Pass:** either the victim is handled, or it is skipped; nothing is dispensed while the wheels are turning, and the move ends normally. If skipped, the camera reports it again on the next move.

## 5. Obstacles (#5)

The old code looked for an obstacle once, before a move began, at 90 mm. In the next tile it is at least 175 mm away, so that check almost never fired for a real obstacle (it fired for walls seen at an angle instead), and the emergency stop needs **both** front sensors at 50 mm or less, which an obstacle pushed against a wall does not give. Now `fwd()` looks every loop pass: when a front sensor reads less than `OBSTACLE_STOP_MM` and less than the far wall of the tile would read (`remaining + FRONT_GAP_AT_CENTER_MM - 45 mm`, which is `remaining - 3 mm` for the V2 robot) for two passes in a row, the robot stops, confirms with three more readings, backs off to where the move began and reports `BLOCKED`.

**5a. Calibrate `OBSTACLE_STOP_MM` against a ramp.** A ramp ahead also reads close, because the beam hits the incline some distance beyond its foot (about beam height / tan(angle)). With the V2 sensors as the CAD draws them (windows about 106 mm above the floor) the smallest reading at the foot of a 25° ramp is 106 / tan 25° = 227 mm, far above `OBSTACLE_STOP_MM`; with the 40 mm beams the simulator assumed before 4 Oct 2026 it was 44-105 mm at 20° and 25°. The height of the windows is one of the numbers the CAD does not settle, so check it. `OBSTACLE_STOP_MM` has to stay below that, or the robot treats ramps as obstacles.
1. Put the steepest ramp you expect (25°) in front of the robot, 300 mm away, square.
2. Print `measure(7)` and `measure(1)` (temporary `Serial.println` in `loop()`) while you push the robot slowly toward the foot of the ramp by hand, until its wheels touch the incline.
3. Note the smallest value either sensor gave, `r_min`. Set `OBSTACLE_STOP_MM` to the smaller of 60 and `r_min - 10` (not under 45). If `r_min` is under 55 mm, the distance alone cannot tell that ramp from an obstacle: the robot will stop for it, or you keep the ramps to a gentler angle.
- **Pass:** the robot drives up the 25° ramp without `[OBST]` or `BLOCKED` in the log.
- **Fail:** `[OBST] closest front reading 7x` right before the ramp: `OBSTACLE_STOP_MM` is too high.

**5b. Obstacle in a corridor (stress test, not a legal field: the rules keep 20 cm of the tile free).** Put a cylinder (about 80 mm across) in the middle of a corridor, 2 tiles ahead of the robot. With the sensors as drawn (80 mm either side of the centre line) a cylinder exactly in the middle is seen from farther than about 18 cm and then lost, see 5d.
- **Pass:**
  - `[OBST] closest front reading N` with N usually 35-55 (always under `OBSTACLE_STOP_MM + 20`), then `[MOVE] result=BLOCKED exit=obstacle`.
  - The robot never touches the cylinder, backs off to where the move began, and the next `[PLAN]` line shows the same `x` and `y` and a different `next=`.
  - It never tries that way again in this run.
- **Baseline:** on `main` the robot drives into it and pushes it, or starts the detour from the wrong place.

**5c. Obstacle against a wall.** Put the cylinder touching the right wall of the corridor (centre 100 mm right of the middle line), then the left wall.
- **Pass:** same as 5b. Only one front sensor sees it here, which is the case the old two-sensor emergency stop missed.

**5d. Obstacle straight ahead against a far wall (known gap).** Put the cylinder against the end wall of a dead end, exactly in the middle.
- **Expected with the sensors as drawn:** the robot touches it. The two front sensors are 80 mm either side of the centre line and each sees a cone of about ±12°, so a cylinder of 40 mm radius straight ahead is outside both cones when closer than about 18 cm: both read the wall behind it. The stuck-move check ends the move after 7 s.
- **Fix:** hardware. Angle the two front sensors inward by about 15° or add a third sensor at the front centre (V2 note, section 7).

**5e. No false stops.** Drive 20 tiles of a maze with T-junctions and wall ends (no obstacles).
- **Pass:** no `[OBST]` lines.
- **If there are some:** note where. A sensor swept across the end of a wall while the robot hugs one side of the tile gives a short reading for a moment; two passes in a row are needed, so a stop here means the robot really was that close to the wall end (see section 3).

## 6. The way home (#6)

The old return plan was computed once, walked blind (`fwd()` and every turn without checking its result), and always ended with `back to start`, wherever the robot really was. Now, after the 25-move limit, the planner steers toward the start tile one tile at a time with the same state machine as exploring (turn, check, move, check, recover), and `back to start` appears only when the robot's own position is the start tile.

**6a. Normal return.** Run until `[RETURN] move limit reached…`.
- **Pass:** every `[PLAN]` line afterwards moves one tile closer to the start tile along known edges (blue and obstacle tiles only when nothing else is left), and `[RETURN] back on the start tile` is logged when the robot stands on the start tile.
- **Baseline:** on `main`, one failed turn on the way home made it drive into a wall, and it said `back to start` anyway.

**6b. A blocked way home.** After `[RETURN] move limit reached…`, put a board across the next edge on its way.
- **Pass:** `[MOVE] result=BLOCKED`, `[MOVE] blocked edge recorded`, and the next `[PLAN]` line takes another route if there is one.

## 7. Stuck moves (#7)

The encoders count wheel spin as distance. A robot whose wheels spin against a wall or an obstacle used to finish its move when the encoders said the tile was done and report `OK`, so the map advanced a tile that the robot never entered. In the simulator that was the first mistake in 21 of 43 lost runs. Such a move takes much longer than a clean one (8 s and more, against about 3.6 s), so `fwd()` now stops a move that has been driving for more than `MOVE_TIMEOUT_MS` (7000) and reports `BLOCKED` with `exit=stuck`. It does not back off (reversing by the encoders would drive it behind the tile it started in); the recovery squares it up and the edge gets one more try before it is blocked.

**7a. Calibrate `MOVE_TIMEOUT_MS`.** Read the `ms=` field of 30 `[MOVE] result=OK exit=normal` lines from a run with a few victims, a bump and a blue tile.
- Set `MOVE_TIMEOUT_MS` to about twice the largest of them (the simulator: typical 3.6 s, 99% under 4.6 s, limit 7 s). Too low and good moves are cut off; too high and a pinned move takes longer to notice.
- A move on a ramp is exempt once the pitch passes 20°, but a gentler ramp is not: check that your slowest ramp tile stays under the limit.
- If `MOVE_TIMEOUT_MS` is too low, three `[MOVE] stuck after …` lines in a row are followed by `[MOVE] stuck stops in a row: stuck check switched off` and the check stays off until the next start; raise the constant.

**7b. Held back.** Start a move and hold the robot back by hand, firmly enough that the wheels slip and it hardly advances, until the log shows a line.
- **Pass:**
  - After `MOVE_TIMEOUT_MS` the log shows `[MOVE] stuck after … ms`, `[MOVE] result=BLOCKED exit=stuck`, `[MOVE] move failed, trying that edge once more`.
  - The robot squares up and senses again; the next `[PLAN]` shows the same `x` and `y`.
  - If the second try is stuck too, `[MOVE] blocked edge recorded` follows and the robot goes another way.
- **Baseline:** on `main` the move ends `OK` and the next `[PLAN]` line is one tile further than the robot is.

**7c. No false stops.** Count `[MOVE] stuck after` lines in a full run.
- **Pass:** none except where you put an obstruction. (The simulator: 4 in 160 plain-maze runs, all on moves that really had not reached the next tile; 14 in 40 full-field runs, half of them 150-250 mm into the move.)

## 8. Full runs

Do three runs on a practice field with a checkpoint, a blue tile, a black tile, a dead end, a ramp and an obstacle. Count in each run:

| Count in the log | Target |
|---|---|
| `[TURN] … ok=0` | 1 in 15 turns or fewer |
| `[CLEAR]` | A few, only next to walls |
| `[MOVE] result=BLOCKED exit=obstacle` | Only where there really is an obstacle |
| `[OBST]` without a real obstacle in front | 0 |
| `[MOVE] stuck after` | Only where the robot was really held |
| Times the robot drives more than one tile in a single move (the position in `[PLAN]` jumps back after the next `[WALLS]`) | 0 |
| `back to start` | Only on the start tile |

## Gaps this branch does not close

- **Obstacle beside the path** (5d): needs sensors that cover the full body width.
- **Stairs** are always treated as an obstacle (the sensors see the first riser as a wall), so the robot never climbs them. The simulator gives 5 points for a stair tile; the original code got them once in 40 runs.
- **Ramps of 20° and above** (findings 12-13 in `sim/FINDINGS.md`): a 20° ramp ahead reads as a front wall (`detectWall(0)`), so the robot often does not try it. Moving the robot 19 mm closer with `FWD_TRIM_MM` makes this more likely.
- **Gyro drift of more than about 20° in an open area** can leave the robot in a loop of failed turns (`botched turn detected` three times, `re-planning`, again) until the referee restarts it. Walls re-zero the gyro; open areas do not.
