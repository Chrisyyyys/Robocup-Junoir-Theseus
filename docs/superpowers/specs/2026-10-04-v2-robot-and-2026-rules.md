# The V2 robot and the 2026 rules in the simulator

**Branch:** `claude/pose-and-obstacle-fixes`
**Status:** tested in the maze simulator only. The robot numbers come from the CAD in [github.com/Arsur24/Theseus](https://github.com/Arsur24/Theseus) (the Fusion 360 archive `V2.f3z` and the sprint notes), not from measuring the robot; [step 0 of the test plan](2026-10-02-pose-and-obstacle-fixes-test-plan.md#0-check-the-v2-numbers) says what to check. The rules are `RCJRescueMaze2026-final.pdf` (last updated 2026-03-29).
**Replaces:** the robot the simulator assumed until now (170 × 140 mm, side sensors 70 mm from the centre line, front sensors ±35 mm). That was a guess, and it was wrong in a way that matters: the results in the [design note](2026-10-02-pose-and-obstacle-fixes-design.md) and in `sim/FINDINGS.md` findings 1-18 were measured with it. The numbers here supersede them.

## What changed

1. **The simulated robot is the V2 robot** (section 1): 195 mm long and 180 mm wide over the wheels, seven VL53L0X sensors at the positions the CAD mounts them, with their heights.
2. **The simulated field follows the 2026 rules** (section 3): a 28 cm path (20 mm walls), 1 cm speed bumps, 2 cm stairs, obstacles that touch a wall, a red tile and a dangerous zone, cognitive targets, linear and floating tiles, and the 2026 scoring.
3. **The robot code needed four changes for the V2 body** (section 4). Turning in place in a 28 cm path leaves 7 mm of room; the original code stalled in almost every maze (106 of 120 explore runs needed at least one lack-of-progress restart). With the changes: 1 of 120.
4. **The camera-thread handshake (change #4, the I2C fix) stays in, as a commit of its own** ("Robot: stop the wheels before the camera thread services a victim"). Without it the robot drove on during victim stops in 28 of 80 runs, up to 1.5 m, and fell into black holes 17 times in 360 mazes (section 5). It was critical for the combination of fixes, not for the original code.

**Not known:** whether the robot was built as the CAD draws it. Two numbers matter most (section 6): the offset of the side sensors (5 mm too large doubles the runs lost, 10 mm too large makes the final code worse than the original) and the width of the body. Measure both before trusting the constants (test plan, step 0).

## 1. The V2 robot, from the CAD

The archive `Fusion 360 designs/V2.f3z` holds the assembly as ASM binary bodies. [`sim/tools/cad_sensors.py`](../../../sim/tools/cad_sensors.py) reads them (`pip install zstandard`, then `python sim/tools/cad_sensors.py V2.f3z`) and prints the plates, the wheel and the VL53L0X mounting holes. The parts of the archive have their own coordinate frames and are not tied together, so heights above the floor and the front of the robot were worked out by hand.

| What | Value | Where it comes from |
|---|---|---|
| Chassis plates | 195 × 138 mm (top and bottom), 195 × 124 mm (inner layers) | Largest horizontal faces of the bodies; `Mechanical/Sprint1.md`: "chassis lengthened to 195" |
| Wheels | 80 mm diameter, 20 mm wide, outer face at x = ±90.2 mm, centre planes at ±80.2 mm | Cylinder of radius 40.0 mm, faces at x = -90.2 and -70.2 mm |
| Footprint where it can touch a wall | **195 mm long × 180 mm wide** | Wheels and side-sensor tabs reach ±90.2 mm; the plates are only ±69 mm |
| Ground clearance | about 20 mm | Battery box 30.5 mm below the plate, the wheel axle 10 mm below it, the floor 50 mm below it |
| Side sensors (2, 3, 5, 6) | x = ±90.2 mm, y = ±88 mm from the centre, facing out, window about 106 mm above the floor | Four VL53L0X hole patterns (12.7 × 20.3 mm) on tabs at the corners, board centre 56.4 mm above the plate frame's origin (floor at -50 mm) |
| Front pair (1, 7) | x = ±80.2 mm on the front face, facing forward, about 106 mm above the floor | Two patterns on the end face of the upper structure |
| Back sensor (4) | x = 0 on the back face, about 43 mm above the floor | One pattern on the lower frame |
| Distance sensor | Adafruit VL53L0X: 25.4 × 17.8 mm board, 25° cone, 30 mm minimum range | CAD model; `electronics/Electronics_Sensor Bill of materials..pdf` |
| Motors | Pololu 195:1, 20D, 12 V; Carobot V3 motor shield on I2C (all drive commands go over the bus) | BOM, block diagram |

Not in the repository, so still assumed: which end of the CAD is the front (the code has two front sensors and one back sensor, the CAD two at one end and one at the other), the exact position of the sensor chip on its board, the colour sensor and camera positions, and whether the robot as built matches the CAD. The first hand-written note of the team about it (Sprint #2) says the V1 robot's wheels were wider than its sensors; the V2 sensors are as wide as the wheels.

## 2. What the geometry does

| Quantity | Value | Consequence |
|---|---|---|
| Corner sweep when turning on the spot | √(195² + 180²) / 2 = **132.7 mm** | The path is 280 mm wide, 140 mm from the middle to a wall face: **7.3 mm of room** on each side. The rotating body touches a wall from the moment it is 7 mm off the middle, at about 47° of the turn. That is the "stops at about 45° instead of 90°" in the team's Sprint #2 notes |
| What the side sensors read when the robot is centred | 49.8 mm | The old target of 80 mm steered the robot 30 mm towards the opposite wall |
| What the front and back sensors read centred, facing a wall | 42.5 mm | 12.5 mm above the sensors' minimum range |
| Smallest ramp-foot reading | 106 / tan 25° = 227 mm | Above `FRONT_WALL_MAX_MM` (200), so a ramp is never taken for a wall (finding 12 was an artefact of low beams) |
| Blind zone of the front pair | an object of 40 mm radius exactly ahead is outside both ±12.5° cones when closer than about 181 mm | A cylinder against the far wall of a tile, straight ahead, is only touched. Finding 22 |

## 3. The 2026 rules in the simulator

| Rule | In the simulator |
|---|---|
| 3.1.2, 3.3.3: 30 cm tiles, a path of 28 cm between opposite walls | `field.wall_thickness_mm = 20` (was 12) |
| 3.1.5: ramps up to 25° | the generator picks 15, 18, 20, 22 or 25° |
| 3.2: black, silver, blue, red tiles; blue = stop 5 s | tile codes `X C B R`; red is only a floor colour (`color.red`) |
| 3.4.1, 3.5.7: speed bumps up to 1 cm, in the dangerous zone up to 2 cm | `b` (10 mm), `p` (20 mm) |
| 3.4.3-3.4.4: obstacles at least 15 cm tall, touching a wall and leaving 20 cm free, or at least 20 cm from any wall | cylinders 200 mm tall touching a wall (offset 100 mm = 140 - 40); a free-standing one is not generated |
| 3.4.6: stairs 2 cm high, top at least 15 cm long | two steps of 10 mm, top 150 mm |
| 3.5: dangerous zone: red entrance, closed by walls, off the main path | a dead end of 2-3 tiles behind a red tile with `p`, `d` or `T` tiles inside (26 of 40 full mazes) |
| 3.3.1: linear and floating tiles (what a left- or right-hand wall follower from the start reaches) | `World::compute_floating` follows both hands from the start; black tiles are walls |
| 3.6.3: no victims on walls facing black, silver, blue or red tiles, obstacles, bumps, stairs, ramps | the generator leaves those tiles out |
| 3.6.6: cognitive targets (ring sum 2, 1, 0 = harmed, stable, unharmed) | victim types `R Y G`; the OpenMV script sends them as `H S U` (`ei_object_detection_PhiOmegaPsi.py`), so the camera model does the same |
| 5.6: victims 5 / 15 (letters) and 10 / 30 (cognitive targets); one kit 10, two kits 30; blue tile 30 (-10 per revisit); bump 5; ramp 10; stairs 10; checkpoint 10; exit bonus; reliability bonus; misidentification -5 | `sim/core/recorder.cpp`, values in `robot.cfg` |
| 5.6.1: stop within 15 cm of the victim | measured from the robot's body to the victim, not from its centre |

Not simulated: bridges (tiles over tiles), floor steps of up to 3 mm, and the blink timing of the victim and exit LEDs (counted, not timed).

## 4. Changes to the robot code

1. **Geometry constants** (`Main.ino`): `ROBOT_LENGTH_MM 195`, `ROBOT_WIDTH_MM 180`, `WALL_THICK_MM 20`, `TOF_FRONT_FWD_MM 97.5`, `TOF_SIDE_OUT_MM 90.2`. `TARGET_GAP_MM`, `TARGET_SIDE_GAP_MM`, `FRONT_GAP_AT_CENTER_MM`, `TURN_SIDE_GAP_MIN_MM` and the new `TURN_END_GAP_MIN_MM` are worked out from them, so a measured robot needs those five numbers.
2. **Room to turn** (`movement.ino`, `ensureTurnClearance()`): before every turn in place the robot reads the six side, front and back sensors (three rounds, averaged) and, when a wall is closer than the swinging body allows (side readings under 45 mm, front or back under 38 mm), moves to the middle: a sideways shift with a leg length chosen from the shift needed (`lateralShift()`), or a straight leg along the path. Up to three tries. The old routine shifted a fixed 28 mm away from the nearer side wall, which with 7 mm of room can push the robot against the other wall. Log: `[CLEAR] gaps r=… l=… f=… b=… -> shift lat=… lon=…`.
3. **Back-off after a failed turn** (`Main.ino`, `BOTCHED_TURN_RECOVERY`): when a turn failed although the sensors saw room (`ensureTurnClearance()` had nothing to do), something they cannot see is in the way, typically an obstacle straight ahead between the two front sensors. The robot backs off 45 mm before it tries again. Log: `[CLEAR] turn failed with room on the sensors, backed off … mm`. It took full-field restarts from 11 to 1 in 40 mazes.
4. **A victim seen during a turn goes on the current tile** (`navigation.ino`, `markVictimAtEncoderPosition()`): during a turn the encoders still hold the last move, a whole tile, so the victim used to be marked on the next tile. No effect on 120 test mazes (identical runs lost, restarts, score and victims identified), but the old line can hide a victim in the next tile.
5. The earlier changes #1-#7 are unchanged, including **#4, the camera-thread handshake**.

## 5. Results

120 mazes per kind (explore, return, full field), 360 in all, in nine sets of 40 that were each run once with the final code: fresh seeds `1101001` / `1102001` / `1103001` (explore, return, full field), held-out seeds `1201001` / `1202001` / `1203001`, and `1301001` / `1302001` / `1303001`. Same seeds, same simulator: the only difference is the robot code. "Original" is `Main/` from `origin/main` (8fd4316).

| | Original | Final code |
|---|---:|---:|
| **Exploring** (no move limit): runs that lost their position | 13 | 5 |
| lack-of-progress restarts (runs with at least one) | 274 (106) | 1 (1) |
| wall contacts per run, seconds in contact per run | 41.5, 160 s | 2.9, 6 s |
| tiles explored, victims identified | 51%, 43% | 68%, 62% |
| estimated score | 106 | 182 |
| **Returning** (25 moves, then home): ended on the start tile, said home elsewhere | 35, 28 | 81, 0 |
| runs lost, restarts (runs) | 17, 234 (106) | 1, 1 (1) |
| estimated score | 104 | 169 |
| **Full field**: runs lost | 45 | 32 |
| restarts (runs) | 341 (116) | 9 (8) |
| ended on the start tile, said home elsewhere | 28, 20 | 57, 5 |
| wall contacts per run, seconds in contact per run | 33.4, 156 s | 4.9, 25 s |
| estimated score | 92 | 161 |

**Why the original code fails on this robot.** Of its restarts, 98 of 101 (explore), 76 of 80 (return) and 92 of 130 (full field, plus 29 near an obstacle) came after turns that stalled against a wall: in the traces of the 40 explore mazes of the first set 1744 of 2906 turn checks failed (`[CHECK] turn ... ok=0`, 60%), against 71 of 2186 (3%) for the final code. The original code is in contact with a wall for 160 s of every 480 s explore run.

**The camera-thread handshake is critical.** The original code never drives while a victim is pending (0 of 80 runs). With the other changes in, but without the handshake, it did in 28 of 80 runs, 8.2 m in total in the explore set (up to 1.5 m in one run), because the camera thread holds the I2C bus for the 5-9 s of a victim while the drive loop is one pass ahead of it:

| Fixed code | Runs that moved while a victim was pending (explore / full field, 40 each) | Distance driven during victim stops | Runs lost (360 mazes) | Black holes entered (360 mazes) | Score (explore / return / full) |
|---|---:|---:|---:|---:|---|
| with the handshake | 1 / 0 | 0.7 m / 0.3 m | 38 | 0 | 182 / 169 / 161 |
| without it | 17 / 11 | 8.2 m / 2.6 m | 73 | 17 | 175 / 167 / 150 |
| original code | 0 / 0 | 0.3 m / 0.2 m | 75 | 11 | 106 / 104 / 92 |

**Firmware build.** The final code builds for the Arduino GIGA R1 with the real toolchain (arduino-cli 1.4.1, `arduino:mbed_giga` 4.6.0, `arm-none-eabi-g++`): 167 168 of 1 966 080 bytes of flash (8%) and 114 888 of 523 624 bytes of RAM for global variables (21%). `ArduinoQueue` and `Vector` are not installed on the machine this was checked on, so the build used the simulator's stand-ins for them (`sim/stubs`); `Stepper` and `LiquidCrystal` were the real libraries. `Main/` from `8fd4316` does not build there: it stops at `movement.ino:32` (`invalid conversion from 'int' to 'Direction'`), the line that has the cast now.

## 6. How much do the CAD assumptions matter

The CAD is not the robot. To see how much that could matter, the original and the final code ran on the same 80 mazes (the explore set `1101001` and the full-field set `1103001`, 40 mazes each) with one assumption changed at a time (`--set` overrides of `sim/config/robot.cfg`, for example `--set tof.2="80 88 90 0 106"`). Each cell is runs that lost their position / lack-of-progress restarts / wall contacts per run; the last column is the estimated score. The last two rows were run for the final code only.

| If the robot is… | Original: lost / restarts / contacts per run | Fixed: lost / restarts / contacts per run | Score original → fixed |
|---|---|---|---|
| as drawn in the CAD | 26 / 231 / 37.2 | 9 / 5 / 3.3 | 102 → 184 |
| soft walls, 10 mm | 45 / 93 / 26.0 | 14 / 0 / 4.0 | 159 → 189 |
| soft walls, 25 mm | 45 / 93 / 26.0 | 14 / 0 / 4.1 | 159 → 189 |
| front pair toed in 15° | 28 / 234 / 36.6 | 4 / 4 / 6.0 | 101 → 201 |
| all sensors lower | 26 / 226 / 37.5 | 11 / 4 / 3.5 | 102 → 186 |
| all sensors higher | 25 / 240 / 36.6 | 8 / 4 / 3.8 | 98 → 186 |
| front pair at ±60 mm | 36 / 230 / 33.9 | 13 / 2 / 6.4 | 107 → 191 |
| front pair at ±100 mm | 53 / 220 / 38.3 | 11 / 1 / 1.3 | 91 → 148 |
| side sensors 10 mm further in | 26 / 228 / 38.0 | 52 / 8 / 18.8 | 100 → 119 |
| robot 170 mm wide | 24 / 207 / 34.1 | 9 / 3 / 1.7 | 114 → 184 |
| robot 190 mm wide | 29 / 257 / 38.2 | 12 / 2 / 10.9 | 85 → 171 |
| side sensors 5 mm further in | not run | 20 / 5 / 7.7 | — → 170 |
| side sensors 10 mm further out | not run | 9 / 3 / 4.7 | — → 170 |

- **Most of what could differ from the CAD costs little.** With rigid or soft walls, the front pair toed in 15° or at ±60 or ±100 mm, all windows 60 mm lower or 130 mm higher, or a body 10 mm narrower, the final code loses 4-14 of 80 runs against 24-53 for the original and needs 0-5 restarts against 93-240.
- **The offset of the side sensors has to be right, and it must not be too large.** If the real sensors sit nearer the centre line than `TOF_SIDE_OUT_MM` says, the wall follower holds the reading the code expects and so rides off the middle by the difference, and the limit for "too close to turn" is off by the same amount, so the code sees room that is not there. A turn in place has 7.3 mm of room. With the side sensors 5 mm too far in the final code loses 20 of 80 runs (9 as drawn), with 10 mm too far in 52 of 80, **worse than the original (26)**. With the sensors 10 mm further out than the code says the error points the other way and costs nothing (9 of 80). If the offset cannot be measured, use the smaller of the possible values.
- **So does the width of the body.** A body 10 mm wider than `ROBOT_WIDTH_MM` leaves 3.8 mm of room to turn: 12 of 80 runs lost against 29 for the original, but 10.9 wall contacts per run against 3.3.
- Centring on both side walls instead of following the right wall alone (tried as a variant) did not remove the dependence: with the side sensors 10 mm too far in it lost 59 of 80 runs. What would remove it is for the robot to measure the offset itself: in a tile with a wall on each side the two side readings add up to 280 mm minus twice the offset. That is not implemented.

## 7. What is still open

- **An obstacle straight ahead against a far wall** (finding 22). In the 120 full-field mazes 19 of the 32 runs that lost their position had a stuck move into one. Nothing tried so far stops the robot reliably. On the dead-end scenario (3 seeds) the front pair toed in by 15° (`--set "tof.1=80 97.5 345 1 106" --set "tof.7=-80 97.5 15 2 106"`), `OBSTACLE_STOP_MM` raised to 100 or 150 mm, and both together stopped the robot in front of the cylinder in at most 1 seed of 3. In the pooled tests the toed-in pair lowered the lost runs from 9 to 4 of 80 and raised the wall contacts per run from 3.3 to 6.0 (section 6). A third sensor at the front centre has not been simulated. A stuck move that ends part-way leaves the robot one tile off, which is how this obstacle costs the position.
- **Passing an obstacle that touches a wall.** The free lane the rules guarantee is 200 mm, the robot is 180 mm wide, and the wall follower would steer it into the far wall: the robot treats such a tile as blocked.
- **Return in time.** At 10-12 s per move (a 90° turn takes 2.8 s, a tile 3.6 s) the 25-move limit plus the way home does not fit into 8 minutes in a third of the mazes (81 of 120 return runs ended on the start tile). A time-based return would use the run better.
- **Cognitive targets.** The code dispenses nothing for `R`, `Y`, `G`, and it does not need to if the camera sends `H`, `S`, `U` for them as the script does. Whether the real script recognises the ring targets reliably is outside the simulator ("find_circles() is highly unstable" in the team's progress notes).
- **Stuck moves that end part-way** still lose the position when the robot is 140-200 mm into a move and the code reports `BLOCKED`: 32 of 120 full-field runs and 5 of 120 explore runs lost their position.
- **The CAD is not the robot.** Every number in section 1 should be checked on the robot with a ruler (test plan, step 0).

## 8. Reproduce

```
python sim/sim.py batch --count 40 --moves-limit off --seed-start 1101001   # explore, 2026 basic mazes
python sim/sim.py batch --count 40 --seed-start 1102001                    # return home
python sim/sim.py batch --count 40 --profile full --seed-start 1103001     # full field
python sim/sim.py batch --count 40 --profile full --seed-start 1103001 --set drive.wall_slide_mm=10   # soft walls
python sim/sim.py batch --count 40 --moves-limit off --seed-start 1101001 --set "tof.2=80 88 90 0 106" --set "tof.3=80 -88 90 6 106" --set "tof.5=-80 -88 270 5 106" --set "tof.6=-80 88 270 3 106"   # side sensors 10 mm further in
python sim/tools/cad_sensors.py V2.f3z                                      # read the CAD again after a change
```
