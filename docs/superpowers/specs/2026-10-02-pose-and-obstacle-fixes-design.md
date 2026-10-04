# Design: pose, victim and obstacle fixes

**Branch:** `claude/pose-and-obstacle-fixes` (from `main` at `8fd4316`)
**Status:** tested in the maze simulator only. The robot in the simulator is the V2 robot as its CAD draws it, not as measured ([Sensor geometry](#sensor-geometry)). The constants in [Constants to calibrate](#constants-to-calibrate-on-the-robot) must be checked or tuned on the robot before the numbers below mean anything on it. The steps for confirming each change are in the [test plan](2026-10-02-pose-and-obstacle-fixes-test-plan.md); the findings behind it are in [`sim/FINDINGS.md`](../../../sim/FINDINGS.md) (8, 11, 16-18).
**Update 4 Oct 2026:** the measurements in the history sections of this note (why obstacle avoidance failed, why the robot loses its position, the tables of changes and the sets below) were made with the robot the simulator assumed until then (170 x 140 mm, walls 12 mm). The simulator now models the V2 robot from its CAD and the 2026 rules. Change 3 (room to turn), the geometry constants and the constants table below describe the code as it is now; the current numbers are in [the V2 note](2026-10-04-v2-robot-and-2026-rules.md), which supersedes the old ones.
**Baseline:** "the original code" below is `main` at `8fd4316`, run on the simulator with the two fixes in this branch (finding 18 and `sim.return_idle_s`).
**The question that started this:** why obstacle avoidance sometimes seems not to work at all. The answer is the next section.

## Why obstacle avoidance seemed not to work at all

On the original code, in 40 full-field mazes that each contain a cylinder, the robot touched a cylinder in **26 runs** (126 touches, 1490 s of contact in total). In 25 of those 26 the avoidance routine had not started in the 12 s before the first touch. It started 89 times in 30 runs, and **52 of the 89 starts (58%) had nothing in front of the robot**: a wall seen at an angle, the foot of a ramp, a stair riser.

So avoidance neither starts when there is an obstacle nor stays quiet when there is none. Five causes, all in `fwd()` and `obstacleavoidance()`:

1. **It looks once, at the wrong moment.** The only trigger is a check at the start of `fwd()`: one front sensor at 90 mm or less and the other at 120 mm or more. A robot standing in the middle of a tile is at least 175 mm from an obstacle in the next tile, so the check cannot fire for it. It fires when the robot starts a move already close to something, which is usually a wall.
2. **Nothing looks while it drives.** The only check in the drive loop is the emergency stop, which needs **both** front sensors at 50 mm or less. An obstacle pushed against a wall (where the maze generator puts them, and where real ones tend to end up) is seen by one front sensor only; the other reads the far wall. The robot drives into it and pushes it until the encoders say the tile is done, and the move is reported `OK`.
3. **The trigger is lopsided.** The left case needs `front_right >= MIN_DIST` (120 mm), the right case `front_left >= OBSTACLE_DIST` (90 mm). Two sensors reading 110 mm trigger neither; 88 and 125 mm, which is what a flat wall approached at a slight angle gives, trigger a detour.
4. **The detour is fragile.** It spins at PWM 255 inside loops that only end when a sensor condition changes, compares readings with the `-1` "no reading" value as if it were a distance, and in the simulator it often left the robot rotated and displaced.
5. **A detour that "succeeds" is remembered wrongly.** `obstacle = true` flagged the tile as an obstacle for good, including tiles where a wall had set the avoidance off.

One more thing made it look worse in the first analysis than it was: the simulator returned a reading of about 30 mm from the front sensors whenever the robot faced *away* from a cylinder, because its ray-cylinder test counted a cylinder behind the sensor as a hit at distance 0 (finding 18, fixed in `sim/core/world.cpp`). The numbers in this note were all measured after that fix.

### What replaced it

`fwd()` now looks in every pass of its drive loop. A front reading under `OBSTACLE_STOP_MM` (60 mm) that is also closer than the far wall of the destination tile would read (`remaining + FRONT_GAP_AT_CENTER_MM - 45` mm) counts; two passes in a row stop the robot, three fresh readings confirm (`obstacleConfirmed()`, log line `[OBST] closest front reading …`), the robot backs off to where the move began and returns `MOVE_BLOCKED`. The existing recovery marks the edge blocked and the planner takes another way. The old detour is no longer called; the function stays in `Distance.ino`, marked as unused, until the look-ahead is confirmed on the robot.

On its own (same mazes and seeds, original code against only this change):

| Full field, 40 mazes with a cylinder | Original | Look-ahead |
|---|---:|---:|
| runs where the robot touched a cylinder | 26 | 13 |
| touches / seconds of contact | 126 / 1490 | 70 / 266 |
| lack-of-progress restarts | 67 | 29 |
| runs that got lost | 19 | 14 |
| ramps climbed | 22 | 30 |
| estimated score (average) | 62 | 67 |
| 5 obstacle scenario mazes × 6 seeds: runs that touched the cylinder | 29 of 30 | 8 of 30 |

On mazes without obstacles (80 runs) the old detour started for walls and left 14 runs lost; without it, 8.

**Threshold 60 mm, not 110 mm.** The first version stopped at 110 mm. A ramp ahead also reads close, because the beam hits the incline some distance beyond its foot (about `beam height / tan(angle)`; the simulator's beams are 40 mm high, a guess), and at 110 mm the robot treated ramps as obstacles: 6 ramps climbed instead of 22. At 60 mm it climbs 30. The smallest front reading before the robot tilted was 117-124 mm for 15° ramps and 44-105 mm for 20° and 25° ramps in the simulator, so 60 mm is a compromise: over 80 full-field mazes the final code stopped for a ramp 3 times. Calibrate it with the steepest ramp you expect (test plan, 5a).

**What is still missed.** With the V2 positions from the CAD (front pair 80 mm either side of the centre line, cones of about ±12.5°) a cylinder of 40 mm radius exactly ahead, against the far wall of a tile, is outside both cones when closer than about 181 mm: both sensors read the wall behind it and the look-ahead never fires (finding 22 in `sim/FINDINGS.md`; V2 note, section 7). With the mounting assumed when this section was written (front pair 70 mm apart) the blind band was beside the path instead (finding 17): in 23 of the 24 development and held-out full-field runs where the robot still touched a cylinder, it did so while driving forward.

## Why the robot loses track of where it is

> The numbers in this section are for the 170 × 140 mm robot assumed until 4 Oct 2026. For the V2 body (195 × 180 mm, 20 mm walls) a turn in place has 7 mm of room on each side, not 34 mm: V2 note, section 2, and finding 19 in `sim/FINDINGS.md`.

A 170 × 140 mm body turning in place sweeps a circle of 110 mm radius. In a 288 mm corridor (144 mm from the middle to a wall face) the centre has 34 mm of slack, and a turn from closer to a wall than 110 mm stalls against it. In the original code's 5641 turns of 60° or more (six sets of mazes):

| Side wall nearest the centre when the turn started | Turns | Turns that failed (`[CHECK] … ok=0`) |
|---|---:|---:|
| under 100 mm | 612 | 612 (100%) |
| 100-110 mm | 539 | 527 (98%) |
| 110-120 mm | 234 | 48 (21%) |
| 120-130 mm | 361 | 17 (5%) |
| 130 mm and more | 2934 | 170 (6%) |

(Another 961 turns had no side wall.) A geometric test (does the rotated rectangle hit a wall box?) explains 88% of all failed turns, and when it says "blocked" the turn fails 99% of the time. 26% of all turns failed. Nothing kept the robot near the middle of the tile: moves ended about 19 mm short with nothing to make it up, there was no steering at all without a right wall, and a failed turn was retried from the same spot.

The failures then fed each other. The first mistake in the 43 lost runs of 200: 21 were a move pinned against a wall that was still reported `OK`; 12 an obstacle detour (10 of them set off by something that was not an obstacle) that left the robot displaced; 7 distance errors; 2 emergency stops counted as a whole tile; 1 a restart where the referee's placement did not match the map.

## The changes

| # | Change | Where | Problem | On its own, original → this change |
|---|---|---|---|---|
| 1 | `FWD_TRIM_MM` (19 mm) added to every `fwd()` target | `Main.ino`, `fwd()` | Moves ended ~19 mm short, uncorrected | runs lost: explore 14 → 9, return 10 → 4, full field 19 → 14 |
| 2 | Hold the start heading with the gyro when there is no right wall | `Distance.ino` `center()`, `fwd()` | No steering without a right wall: 3-5° of drift per move | lost 14 → 9, 10 → 8; full field 19 → 24 |
| 3 | Move to the middle before a turn in place if a wall is closer than the swinging body allows (first version: a fixed 28 mm shift away from a side wall nearer than 46 mm) | `movement.ino` `ensureTurnClearance()` | Turns stalled against the wall (26% of turns) | restarts: 32 → 8, 26 → 17, 67 → 49 (first version, 170 × 140 mm robot) |
| 4 | The camera thread waits until the wheels are stopped before it takes the I2C bus | `Main.ino` `cameraTask()`, `fwd()`, `absoluteturn()` | The robot drove on during a victim stop | lost 14 → 15, 10 → 8 (no effect on `main`'s timing; see below) |
| 5 | Obstacle look-ahead | `movement.ino` `fwd()`, `Distance.ino` `obstacleConfirmed()` | Avoidance (first section) | lost 14 → 8; the table above |
| 6 | The way home planned tile by tile through the normal state machine | `Main.ino`, `navigation.ino` `stepTowardStart()` | The return walked blind and always said "back to start" | on the start tile 56 → 76 of 80; said "home" elsewhere 22 → 1 |
| 7 | A move that drives for more than `MOVE_TIMEOUT_MS` is stopped as stuck; its edge gets one more try | `movement.ino` `fwd()`, `Main.ino` `BOTCHED_FWD_RECOVERY` | A pinned move was reported `OK` | lost 14 → 9, 10 → 5; full field 19 → 24 |

These are counts of runs out of 80 (explore), 80 (return) and 40 (full field); a count of 14 has a spread of about ±4, so single differences of a few runs mean little. The changes were kept for effects that show on several measures at once. The results for the whole set are below.

### 1. Move length

`fwd()` slows down in proportion to the distance left and stops when `Scale*120 < 25`, 46 encoder counts (12 mm) before the target; with a little wheel slip each tile ended 19 mm short, and in a corridor nothing corrects that (`centerFrontBack()` only helps with a wall in front). `FWD_TRIM_MM` adds the missing distance to every target. The value is a measurement of the simulator's motors; measure it on the robot (test plan, 1a).

### 2. Heading hold without a right wall

`center()` returns 0 when there is no right wall on both sensors, so the wall follower adds no steering and motor mismatch and wheel slip turn into heading drift, then sideways drift, until a wall stops the robot. `center()` now also sets `centerHasWall`; when it is false `fwd()` steers with the existing `gyroPID` on the heading the move started on (the nearest axis, `init_yaw`), with the same ±40 limit as the wall follower.

### 3. Room to turn

Before a turn in place (`EXECUTE_MOVE`, and the retry in `BOTCHED_TURN_RECOVERY`) the robot reads the six side, front and back sensors three times and averages (`readTurnGaps()`: the room is a few mm and the readings are noisy). If a side reading is under `TURN_SIDE_GAP_MIN_MM` (45 mm for the V2 body: sweep of 132.7 mm − sensor offset of 90.2 mm + 3 mm of margin) or a front or back reading is under `TURN_END_GAP_MIN_MM` (38 mm), `ensureTurnClearance()` moves the robot to the middle:

- across the path with a two-leg shift (`lateralShift()`): point `NUDGE_ANGLE_DEG` (9°) away from the wall, drive a leg, point the other way, drive the *same distance* back and square up. The leg is `shift / (2 sin 9°)`, between 20 and 130 mm; forward first, or in reverse first when a front wall is within about 140 mm;
- along the path with a straight leg (`nudgeLeg()`), which stops at 32 mm from a wall ahead or behind.

Up to three tries, each measured again. The second leg has to equal the first: a first version with unequal legs drifted along the tile and made position loss worse, and only the mazes that were held out showed it. After a turn that failed although the sensors saw room, the robot backs off `BOTCH_BACKOFF_MM` (45 mm) before the retry: something they cannot see, typically an obstacle straight ahead between the two front sensors, is in the way. Log lines: `[CLEAR] gaps r=… l=… f=… b=… -> shift lat=… lon=…` and `[CLEAR] turn failed with room on the sensors, backed off … mm`.

Result with the first version (a fixed 28 mm shift, 170 × 140 mm robot): turns that start with a wall closer than 110 mm fell from 20% of all turns to 0.5%, and failed turns from 26% to 3.6%. With the V2 body and the version described here the original code failed 1744 of 2906 turn checks (60%) on the 40 explore mazes of the first set, the final code 71 of 2186 (3%) (V2 note, section 5).

### 4. Victim stop

While the robot drives, a camera thread notices a victim and the robot must stand still for the 5-9 s the victim takes. The thread used to raise `victimPending`, sleep a fixed 10 ms and then hold the I2C bus for the whole victim (`serviceCameraVictim()`). Every `drive()`, `fullstop()` and sensor read needs that bus. If `fwd()` was in the middle of a loop pass, it issued one more `drive()` and then blocked on the bus **with the motors running**: the robot drove 2-4 tiles on its own (up to 1.25 m in one move) while the map advanced one tile.

Whether this happens depends on the timing of the loop. On `main` it did not happen once in 6785 moves of the plain mazes, so this change does nothing there. After changes 1-3, 5 and 6 shifted the timing it happened in 34 of 7235 moves, runs lost exploring were 17 of 80 with those changes and 4 of 80 once the handshake was added, and it was the largest single cause of lost robots in that build. The thread now waits for `victimAck`, which `fwd()` and `absoluteturn()` set after their own `fullstop()`, and skips the victim if nobody acknowledges within 300 ms (the camera reports it again). A victim during a turn is handled the same way. How to see the old behaviour: finding 16 in `sim/FINDINGS.md`.

### 5. Obstacle look-ahead

See the first section.

### 6. The way home

The old `RETURN` state computed one BFS path, walked it with `absoluteturn()` and `fwd()` while ignoring their results, and always ended with `back to start`, wherever the robot really was. In the simulator it said so on the wrong tile in 22 of 80 runs. Now, after the 25-move limit, `returning` is set and `PLAN_NEXT` / `BACKPEDAL` ask `planDirection()` for the next step (`stepTowardStart()`: shortest known route, blue tiles and obstacle-flagged tiles allowed only when nothing else is left); every step goes through turn, check, move, check and recovery like exploring. `RETURN` is entered only when `x_pos`, `y_pos` and the floor say the robot is on the start tile. Log lines: `[RETURN] move limit reached…`, `[RETURN] back on the start tile`.

### 7. Stuck moves

The encoders count wheel spin as distance. A robot whose wheels spin against a wall end or an obstacle used to finish its move when the encoders said the tile was done, and report `OK`, so the map advanced a tile that the robot never entered. 21 of the 43 lost runs began that way. Such a move takes much longer than a clean one: clean `OK` moves took 3.6 s of driving on median in the simulator (99% under 4.6 s), pinned ones 8 s and more. `fwd()` now stops a move that has driven for more than `MOVE_TIMEOUT_MS` (7000, not counting time spent standing still, e.g. for a victim) and returns `MOVE_BLOCKED` with `exit=stuck`. It does **not** back off: the encoders count the spin, so reversing by them drove the robot well behind the tile it started in. `BOTCHED_FWD_RECOVERY` squares the robot up and senses again, and the edge is only blocked if the same edge gets stuck a second time (`fwdSoftFail`, `softFailPending`). The move's duration is printed as `ms=` on every `[MOVE] result=` line. A timeout that is too short for the robot would cut off every move, so after `MOVE_STUCK_LIMIT` (3) stuck stops in a row (any normal move resets the count) the check switches itself off and logs `[MOVE] stuck stops in a row: …` (in the simulator it did that once in 400 runs, after three real stuck stops in a row); in the simulator with a timeout of 2 s, about half the duration of a normal move, it did that after the third cut-off and the robot went on exploring (the first cut-off move had already cost it its position in all three mazes: the constant still has to be set properly).

On top of changes 1-6 it took the runs lost from 4 / 6 / 10 to 0 / 0 / 9 (explore / return / full field) and the restarts from 4 / 5 / 14 to 4 / 3 / 10. In the plain mazes it fired 2 times in 160 runs, both on moves that had really not reached the next tile.

## All the changes together

Mazes used while choosing the changes (explore 80, return 80, full field 40) and mazes that were never used until the end (the same numbers; run once):

| | Original, development | All seven, development | Original, held out | All seven, held out |
|---|---:|---:|---:|---:|
| **Exploring** (no move limit): runs lost | 14 | 0 | 9 | 2 |
| lack-of-progress restarts | 32 | 4 | 30 | 3 |
| contact time per run | 34.5 s | 2.5 s | 29.2 s | 9.5 s |
| estimated score / tiles seen | 112 / 71% | 118 / 72% | 106 / 79% | 106 / 77% |
| **Return** (25 moves, then home): ended on the start tile | 56 | 75 | 53 | 72 |
| said "home" while somewhere else | 22 | 0 | 25 | 0 |
| runs lost / restarts | 10 / 26 | 0 / 3 | 6 / 31 | 3 / 5 |
| estimated score | 101 | 114 | 88 | 103 |
| **Full field**: runs lost | 19 | 9 | 21 | 9 |
| restarts | 67 | 10 | 65 | 13 |
| ended on the start tile / said "home" elsewhere | 17 / 14 | 23 / 1 | 20 / 13 | 29 / 0 |
| runs where the robot touched a cylinder | 26 | 15 | 20 | 9 |
| estimated score | 62 | 67 | 63 | 83 |

On the full field the score depends on what the robot happens to find, and its parts move in different directions: over the 40 held-out runs the original code scored 2525 points and this branch 3310 (victims 680 → 830, kits 470 → 630, exit bonus 180 → 400, ramps 110 → 260, but stair tiles 10 → 0 points); over the 40 development runs 2490 and 2670 (blue tiles 690 → 540). Neither the exploring score nor the coverage went up on the held-out mazes: the gain is reliability, not extra exploring.

The simulator also gives a picture of the individual parts: of the original code's 5641 turns, 1489 failed, and 3.6% of 5589 do now; moves that ended `OK` with the robot not even in the next tile: 78 in 7922 before, 0 in 8801 now; moves that covered more than 450 mm in one go: 7 before, 5 now, all of them ramp climbs.

## Tried and not adopted

Each was tested on its own against the same mazes before anything was combined; counts are runs lost over explore (80) / return (80) / full field (40), original 14 / 10 / 19.

| Idea | Why it looked right | Result | Why it was dropped |
|---|---|---|---|
| Obstacle stop at 110 mm | Earlier warning | Full field: ramps climbed 22 → 6 | A ramp ahead also reads close |
| Retry an obstacle stop once from a squared-up pose before blocking the edge | A stop next to a wall end can be the robot's own pose | Cylinder contact 13 → 18 runs, restarts 29 → 47 | The retry approaches a real obstacle again, from closer |
| Move check with the heading limit (stop a move whose heading leaves its axis by more than 20°) | A pinned robot yaws | lost 17 / 9 / 28 | 22 of 46 triggers came on moves that had really driven the whole tile, and the heading check on top of the final code was worse than without it (9 / 8 / 17 against 1 / 0 / 8) |
| Move check by time **with** a back-off to where the move began | The same fix for pinned moves | 11 / 4 / 23 | 35 of 38 time-outs were real pinned moves, but reversing by the encoders (which counted the spin) put the robot behind the start tile |
| Follow the left wall when there is no right wall | A second wall reference for open-sided tiles | 24 / 16 / 26; with the gyro hold on top 19 / 20 / 24 | The map belief got worse |
| Judge a stuck move by the distance sensors: if they say the robot moved half a tile or more, count the move as done | A robot held up past halfway is already in the next tile, and `BLOCKED` leaves the map a tile behind (7 of the 9 lost runs that remain) | On top of the final code 0 / 0 / 8 against 0 / 0 / 9, scores and restarts the same | After a pinned move the sensors see other surfaces (wall ends, the cone's edge): in the six stuck stops that still ended lost the estimate was off by 36-370 mm. More code, no measurable gain |

## What is still open

- **An obstacle straight ahead against a far wall** (above, finding 22) needs the front pair toed in by about 15° or a third sensor at the front centre.
- **Stairs** are always treated as an obstacle: the first riser reads as a wall at 20-50 mm. The simulator gives 5 points for a stair tile; the original code got them once in 40 development runs and twice in 40 held-out runs, this branch never.
- **Ramps** of 20° and more (findings 12-13 in `sim/FINDINGS.md`) are unchanged in the generated mazes (23 ramp traversals in 40 development mazes against 22 for the original, 26 against 11 on the held-out ones). In the 20° ramp scenario the original passed 3 of 6 seeds (and in `ramp_up.txt` it climbed in 1 of 3 and then lost its position) and this branch 0 of 6: from the tile centre the 20° ramp ahead reads 165-190 mm, under `FRONT_WALL_MAX_MM` (200 mm), so `detectWall(0)` sees a wall; the original stood 18 mm short and read 183-202 mm, which is why it passed half of the seeds. A ramp-aware front-wall test is the next step; `FRONT_WALL_MAX_MM` was set to 200 mm because moves ended short, which they no longer do.
- **Stuck moves that end part-way** are what loses position now: 7 of the 9 runs still lost on the development mazes (all on the full field) were held up 140-200 mm into a move, right at the tile boundary. The code reports `BLOCKED`; the robot may already be in the next tile. Nothing on the robot says where it is: the encoders count the wheel spin, and the distance sensors, tried as a judge (above), were not reliable.
- **Gyro drift of more than about 20° in an open area** can leave the robot in a loop of failed turns (`botched turn detected` three times, `re-planning`, again) until the referee restarts it. Walls re-zero the gyro; open areas do not.

## How this was tested

The simulator is deterministic: the same maze and seed give the same run for the same code, so a difference between two versions is the code's. Every change was built on its own branch from the same base and run against the same mazes; only changes that helped were combined, and the combination was run on mazes that had never been used.

| Set | Seeds | Content |
|---|---|---|
| A1, B1 | 727101-727140, 910001-910040 | exploring, `--moves-limit off`, basic mazes |
| A2, B2 | 805204-805243, 920001-920040 | return home (the code's 25-move limit), basic mazes |
| A3, B3 | 664070-664089, 930001-930020 | full field (`--profile full`) |
| C1-C3, D1-D3 | 940001, 950001, 960001; 970001, 980001, 990001 (40, 40, 20 mazes each) | the same three kinds, held out |

The V2 results (V2 note, section 5) were made the same way with new seeds: `1101001`, `1102001`, `1103001` (explore, return, full field; 40 mazes each), held out `1201001`, `1202001`, `1203001` and `1301001`, `1302001`, `1303001`.

For example `python sim/sim.py batch --count 40 --moves-limit off --seed-start 727101`, or `… --count 20 --profile full --seed-start 664070`. "Lost" means that at some arrival the robot's belief (`x_pos`, `y_pos`, `currentDir`) did not match the tile it was on. Obstacle contacts, pinned moves and turn failures are counted from the replay (the robot's body rectangle against the cylinders and walls), not from the robot's own log. The obstacle scenario mazes are in `sim/mazes/scenarios/obstacle_*.txt` and run with the other scenarios in CI; on 5 scenarios × 6 seeds the original touched the cylinder in 29 of 30 runs with 42 restarts, this branch in 9 of 30 with none.

## Sensor geometry

This note was first written with a mounting that was guessed from comments in the code (front pair ±35 mm, side sensors ±70 mm, beams 40 mm high, body 170 × 140 mm). From 4 Oct 2026 the simulator uses the V2 robot as its CAD draws it: 195 × 180 mm over the wheels, front pair ±80 mm on the front face, side sensors ±90 mm flush with the wheels, windows about 106 mm above the floor (back sensor 43 mm). [V2 note, section 1](2026-10-04-v2-robot-and-2026-rules.md#1-the-v2-robot-from-the-cad) has the numbers and where they come from, and [section 6](2026-10-04-v2-robot-and-2026-rules.md#6-how-much-do-the-cad-assumptions-matter) says how much it matters if the built robot differs. It is the CAD, not a measurement of the robot: the side-sensor offset and the body width have to be checked first ([test plan, step 0](2026-10-02-pose-and-obstacle-fixes-test-plan.md#0-check-the-v2-numbers)). The sensitivity table that stood here belonged to the guessed mounting and was replaced by the one in the V2 note.

`TURN_SIDE_GAP_MIN_MM`, `TURN_END_GAP_MIN_MM`, `FRONT_GAP_AT_CENTER_MM` and the two wall-following targets are worked out in `Main.ino` from `ROBOT_LENGTH_MM`, `ROBOT_WIDTH_MM`, `WALL_THICK_MM`, `TOF_FRONT_FWD_MM` and `TOF_SIDE_OUT_MM`; a measured robot needs only those five numbers.

## Constants to calibrate on the robot

| Constant | Default | Meaning | How to set it |
|---|---:|---|---|
| `FWD_TRIM_MM` | 19 | Added to every move target | `(1500 − mm driven in 5 tiles) / 5` (test plan, 1a) |
| `ROBOT_LENGTH_MM`, `ROBOT_WIDTH_MM` | 195, 180 | Body size over the wheels (from the CAD) | Measure the outside of the body (step 0) |
| `WALL_THICK_MM` | 20 | Wall thickness: the path between opposite walls is 300 − 20 = 280 mm (rules 2026, 3.3.3) | Measure the walls of your field |
| `TOF_FRONT_FWD_MM`, `TOF_SIDE_OUT_MM` | 97.5, 90.2 | How far ahead of the body centre the front sensors sit, and how far to the side of it the side sensors sit (from the CAD). **Not measured on the robot. The side offset matters most: too large by 5 mm and the final code loses twice the runs, by 10 mm it is worse than the original (V2 note, section 6).** | Measure with a ruler to within 3 mm; if in doubt use the smaller value (step 0) |
| `TURN_SIDE_GAP_MIN_MM`, `TURN_END_GAP_MIN_MM` | 45, 38 | Side, and front or back, reading under which a turn needs a shift. Worked out: `hypot(length, width) / 2 − sensor offset + 3` | Do not set them; set the numbers above (3a) |
| `BOTCH_BACKOFF_MM` | 45 | Back-off after a turn that failed with room on the sensors | Leave it |
| `NUDGE_ANGLE_DEG`, `NUDGE_MIN_LEG_MM`, `NUDGE_MAX_LEG_MM` | 9, 20, 130 | The sideways shift: a leg of L mm shifts the robot by 2 · L · sin(angle) | If a shift moves the robot more or less than asked, change the angle |
| `OBSTACLE_STOP_MM` | 60 | Front reading that stops the robot | Under the smallest reading at the foot of the steepest ramp (227 mm for 25° with sensors 106 mm high), not under 45 (5a) |
| `FRONT_GAP_AT_CENTER_MM` | 42 | What the front sensors read at the tile centre facing a wall. Worked out: `140 − TOF_FRONT_FWD_MM` | Check it: read both front sensors with the robot centred in front of a wall |
| `MOVE_TIMEOUT_MS` | 7000 | Driving time after which a move counts as stuck | About twice the longest `ms=` of a normal move (7a) |
| `MOVE_STUCK_LIMIT` | 3 | Stuck stops in a row after which the stuck check switches itself off | Leave it |

The simulator's sensor and motor numbers are in `sim/config/robot.cfg`; several are guesses (the README says which); the sensor positions and the body size come from the CAD, not from the robot (V2 note, section 1).
