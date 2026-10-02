# Test plan: heading and move fixes

**Branch:** `claude/practical-brahmagupta-y7saan`
**Fixes under test:** #1 heading frame, #5 turns, #6 wall re-sync, #4 start/resume, #3 move results,
plus the later fixes in section 7: #2 checkpoint tile, start-up freeze, missing `return`, front wall
range and wall-follower steering limit.
**Not fixed on this branch:** the gaps listed at the end.

Work through the sections in order. Section 1 tunes the turns, and every later test depends on turns being right.

## Before you start

1. Flash the branch and open the Serial Monitor at 115200 baud. Every test is judged from the log lines below. Leave `VERBOSE_DEBUG` at `0` (top of `Main.ino`). The log then shows only these lines, the boot messages and rare events such as black tiles, ramps and victims. Set it to `1` only if you need the old per-loop traces.
2. Use the new start procedure (tell the referee, rule 4.2.8):
   1. Power on with the logic switch at **PAUSE**.
   2. Place the robot square on the start tile and keep it still for about 2 s so the gyro can calibrate.
   3. Flip the switch to **RUN**.
3. Where a test says **Baseline**, you can run it once on `main` first to see the old failure, then on this branch.

| Log line | Meaning |
|---|---|
| `[WAIT] armed=… heading=…` | Waiting for the start switch. Heading readout every 0.5 s. |
| `[START] run started…` | Switch went PAUSE → RUN. Heading zeroed to maze NORTH. |
| `[WALLS] F=… R=… B=… L=…` | Walls read on the current tile, relative to the robot (1 = wall). |
| `[PLAN] x=… y=… floor=… facing=… next=…` | Where the robot thinks it is, which way it faces and which way it will move next. |
| `[TURN] done target=… start_err=… err=… ms=… ok=…` | A turn finished: error at the start and at the end in degrees, time taken, and whether it settled (`ok=1`). |
| `[SYNC] facing=… err=… applied` / `skipped` | Robot squared up on a wall. Shows the gyro error found and whether it was corrected. |
| `[CHECK] turn target=…, actual=…, err=…, ok=…` | The heading check before each move. `ok=0` sends the robot into turn recovery. |
| `[MOVE] result=… exit=…` | What a forward move actually did (`OK`, `BLOCKED`, `BLACK` or `PAUSED`) and why it stopped. |
| `[MOVE] blocked edge recorded x=… y=… dir=…` | The robot stopped short. That edge is now treated as a wall. |
| `[RESUME] checkpoint x=… y=… floor=… facing=…` | Resumed after a lack of progress (LoP). |

Directions are numbers: 0 = North, 1 = East, 2 = South, 3 = West.

## 1. Turns settle on the target (#5)

**1a. Tune the minimum turn speed.** `TURN_MIN_PWM` is in `absoluteturn()` in `movement.ino`. Tune it on the floor surface you'll compete on. Run a few tiles and read the `[TURN] done` lines:
- `ok=1` and `err` within ±3: good.
- `ok=0`, `err` 4–10°, `ms` close to the time limit, and the robot stopped short: the robot can't turn at the minimum speed. Raise `TURN_MIN_PWM` by 5.
- `ok=0` and the robot wiggles back and forth around the target: it's too fast. Lower `TURN_MIN_PWM` by 5, or set `TURN_TOL_DEG` to 4.

**1b. Accuracy and time.** Collect 20 turns from a run that includes a dead end, so you get 180° turns.
- **Pass:** at least 18 of 20 have `ok=1`, every |err| ≤ 5°, 90° turns take under about 1.5 s and 180° turns under about 2.5 s.
- **Baseline:** on `main`, every 90° turn takes about 2 s, because it always ran until the timeout.

**1c. Overshoot correction.** During a turn, push the robot about 20° past its target by hand.
- **Pass:** it turns back and settles (`ok=1`).
- **Fail:** it keeps turning the same way.

## 2. Heading is measured from the start, not magnetic north (#1)

**2a. Magnets don't move the heading.** With the switch at PAUSE, the robot waits in `WAIT_START`. Hold a magnet or a steel screwdriver 2–5 cm from the BNO055 and move it around.
- **Pass:** the `[WAIT] heading` value changes by less than 2°.
- **Baseline:** on `main` (magnetometer mode) the heading swings.

**2b. Rotation still registers.** While it's still waiting, turn the robot about 90° clockwise by hand.
- **Pass:** the heading goes up by about 90.

**2c. Start facing each wall.** Start the run four times. Each time, place the robot on the start tile facing a different side.
- **Pass:** at `[START]` the robot doesn't turn toward a fixed compass direction. Its first turns land square to the walls (`[TURN]` err ≤ 3°).
- **Baseline:** on `main`, starting with the switch made the robot turn to gyro 0, which was the same compass direction every time. On a field that isn't lined up with magnetic north, its turns were crooked.

## 3. Walls correct gyro drift (#6)

**3a. Normal run.** Read the `[SYNC]` lines from a 3-minute run.
- **Pass:** nearly all are `applied` with |err| ≤ 5°. Every `skipped` line should match something real, such as a bad turn or an obstacle.

**3b. Injected gyro error (temporary test build).** In the `WAIT_START` case in `Main.ino`, replace:
```cpp
myGyro.setMapHeading(0);
if(parallel(NORTH)) myGyro.setMapHeading(0);
```
with:
```cpp
myGyro.setMapHeading(X);   // TEST ONLY: pretend the gyro is X degrees off
parallel(NORTH);
```
Then start on a tile that has a side wall.
- **X = 10.** **Pass:** at the start you see `[SYNC] facing=0 err=≈10 applied`, and after that the turns are square.
- **X = 30.** **Pass:**
  1. The sync at the start shows `skipped`.
  2. The first move fails the turn check once (`[CHECK] … ok=0`, then `botched turn detected`).
  3. The recovery's `[SYNC]` shows err ≈ 30 `applied`.
  4. From then on, moves are normal, with exactly one recovery.

  If the recovery repeats 3 times and then logs `max botched-turn retries reached, re-planning`, `parallel()` couldn't square the robot in time. Raise `PARALLEL_RECOVERY_TIMEOUT_MS` in `Main.ino`.
- **Baseline for X = 30:** this is the old stuck loop. The robot turns, squares up, fails the check and repeats forever.

**Undo the test change before any other testing.**

## 4. Start and resume read the walls first (#4)

**4a. Wall ahead at the start.** Put the robot on the start tile facing a wall, then start.
- **Pass:** the first `[WALLS]` line shows `F=1` and comes before the first `[PLAN]` line. In that `[PLAN]` line, `next=` is different from `facing=`, so move 1 doesn't go forward. There's no `[FWD] emergency-stop` on move 1.
- **Baseline:** on `main` the first move always went forward, into the wall.

**4b. Powered on at RUN.** Power on with the switch already at RUN.
- **Pass:** the robot doesn't move and the log shows `[WAIT] armed=0`. Flip to PAUSE (`armed=1`), then to RUN, and it starts.

**4c. LoP resume in different directions.** Do this *before* the robot reaches any silver tile, so the checkpoint is still the start tile (section 7b tests silver tiles). During a run:
1. Flip to PAUSE.
2. Carry the robot back to the start tile and put it down facing one of these ways:
   - the same way it faced before;
   - 90° to the side;
   - the opposite way;
   - about 45° between two walls.
3. Flip to RUN.

Repeat for all four ways.
- **Pass, each time:**
  - `[RESUME] … facing=` shows the direction it was put down closest to.
  - The resume turn is at most about 45° (`[TURN] ok=1`).
  - If there's a side wall, a `[SYNC] … applied` line follows.
  - Then a `[WALLS]` line appears, and the next moves are square and don't head into walls.
- **Baseline:** on `main` the robot always turned all the way to gyro 0, then planned without reading the walls.

## 5. Moves report what happened (#3)

**5a. Blocked early.** As the robot starts a move toward an open side, put a board across the opening about 10–15 cm in front of it. The board must cover both front sensors.
- **Pass:**
  - The log shows `[FWD] emergency-stop`, `[MOVE] result=BLOCKED` and `[MOVE] blocked edge recorded`.
  - The robot backs up to where the move started. The next `[PLAN]` line shows the same `x` and `y` as before the move (the position didn't advance) and a different `next=`.
  - It never tries that edge again during this run.
- **Baseline:** on `main`, the map counted this as a move into the next tile.

**5b. Stopping after halfway still counts.** Put the board about 25 cm ahead of where the move starts, so the robot stops more than half a tile in.
- **Pass:** `[MOVE] result=OK`, the next `[PLAN]` line shows the position one tile further on, and the robot centers itself front-to-back against the board.

**5c. Squeezed obstacle.** Set up an obstacle the detour can't squeeze past.
- **Pass:**
  - The robot wiggles at most 2 times.
  - Then the log shows `[MOVE] obstacle detour stuck, giving up` and `result=BLOCKED`.
  - It never starts a new full-tile drive from where it stopped partway.

**5d. Black tile (should behave as before).** Place a black tile ahead.
- **Pass:**
  - The log shows `[MOVE] result=BLACK`.
  - The robot backs up to the tile centre without lurching forward again, and the next `[PLAN]` line picks another direction.
  - No LoP.

## 6. Full runs

Do three runs on a practice field that has a checkpoint, a blue tile, a black tile, a dead end and, if you can, a ramp and an obstacle. Count these in each run:

| Count in the log | Target |
|---|---|
| `[TURN] … ok=0` | 2 or fewer |
| `[SYNC] … skipped` | Only where something really went wrong |
| `botched turn detected` | At most 1, and never repeating in a loop |
| `[MOVE] result=BLOCKED` | Only where you blocked the robot |
| `tile mismatch detected` (runs without a LoP) | 0 |

The changes also touch these paths, so check they still work:
- **Ramps:** the robot climbs, then a `[TURN]` line straightens it up.
- **Victims:** victims seen while driving or turning still make the robot stop, blink for 5 s and drop kits.
- **Blue tiles:** the robot still waits 5 s.
- **Obstacles:** a normal obstacle detour still ends with `result=OK`.

## 7. Later fixes

These were found with the maze simulator (`sim/` on branch `claude/sharp-archimedes-xe6o9b`).

**7a. Cold power-on (start-up freeze).** Switch the robot fully off for 10 s, then on with the logic
switch at PAUSE. Do it 5 times.
- **Pass:** every time, the boot log reaches `Sensor 6 is able to initialize` and then the `[WAIT]` lines.
- **Baseline:** on `main`, `calibrateSensor(2,80)` read sensor 2 before `init_dist()` had started it,
  and without a timeout that read waited forever after a cold power-on: the log stopped before
  `Mux initialized`. A reset with the button did not show it, because the sensors were still running.

**7b. Checkpoint tile (#2).** Run across a silver tile, then flip to PAUSE one tile after it, carry
the robot back to the silver tile (the referee's LoP procedure) and flip to RUN.
- **Pass:** `[RESUME] checkpoint x=… y=…` shows the silver tile's position, and no `tile mismatch
  detected` follows. Silver is only accepted in the second half of a move now, when the colour sensor
  is over the tile ahead.
- **Baseline:** driving off the silver tile marked the *next* tile as the checkpoint.

**7c. Front wall detected from further away.** A front wall now counts up to 200 mm
(`FRONT_WALL_MAX_MM` in `Main.ino`) instead of 120 mm. Put the robot in a tile facing a wall, about
5-7 cm short of the tile centre (front sensors read about 110-150 mm), and start a step.
- **Pass:** `[WALLS] F=1`, the robot centres itself against the wall, and it does not plan a move
  into it.
- Also check a tile with no front wall: `F=0` (the next tile's wall reads 360 mm or more).

**7d. Wall-follower steering limit.** The right-wall correction in `fwd()` is limited to ±40. Start
a straight run along a right-hand wall with the robot 3 cm off-centre.
- **Pass:** it eases back to its distance from the wall without swinging more than about 10°.
- If it corrects too slowly or drifts into the wall, raise the limit (the `-40, 40` in `movement.ino`)
  to 60; if it still swerves, lower it.

## Known gaps: don't count these as failures of this branch

- **Emergency stop:** it still needs **both** front sensors to see the obstruction. If one front sensor has no reading, the robot can still drive into a wall and count the move.
- **Other known issues:**
  - #7 victim race.
  - #8 the return trip has no LoP handling and no 5 s wait on blue.
  - #9 obstacle tiles block the way home.
  - #11 no heading hold without a right wall.
- **Compile check:** the code was only compiled on a PC against stand-in library headers, not with the real board toolchain. If the Arduino IDE reports an error in the changed code, fix that first.
