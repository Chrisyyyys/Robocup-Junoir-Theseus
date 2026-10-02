# Theseus Tuner

A web page for calibrating sensors, tuning PID gains and testing motors **without re-flashing**.
It talks to the Arduino Giga over USB using the browser's Web Serial API.

No install, no build, no internet needed: open `index.html` in **Chrome or Edge** on a laptop
(double-click the file, or serve the folder with `python3 -m http.server`). Firefox and Safari have no
Web Serial. Click **Demo** to try everything against a simulated robot.

## One-time setup

Flash `Main/` to the Giga once. After that, every value below is changed from the page.

## Using it

1. Plug the Giga into the laptop with USB.
2. Flip the robot's **pause switch to PAUSE**. The robot only accepts tuning mode while paused.
3. Open `index.html`, click **Connect robot**, pick the Giga's port. The page keeps asking until the
   robot answers; the badge turns green: *Tuning mode*.
4. Tune. When done, flip the switch back to **RUN** (or press **Exit tuning**) and the robot goes back
   to its normal behaviour (resuming from the last checkpoint, as it always does after a pause).

The physical switch always wins: flipping it to RUN stops the motors and leaves tuning mode.

### Tabs

| Tab | What it does |
|---|---|
| **Dashboard** | Live view of all 7 distance sensors (top-down, to scale, raw and offset-corrected), the right-wall error `center()` would feed the PID, front-gap centering offset, BNO055 heading/pitch and calibration status, color sensor reading and classification, encoder counts. |
| **Calibrate** | *Distance:* hold a target at a known distance, press Measure (100 raw samples), Apply the suggested offset. *Color:* set the white reference, sample each tile type, the page suggests black / silver / white thresholds and checks that every tile classifies correctly with them. |
| **PID** | Sliders for every controller's kp/ki/kd. Run a **turn** test (mirrors `absoluteturn()`) or a **forward** test (mirrors `fwd()`, steering by wall follower or gyro) and see the response plotted. Gains can be changed *while a test runs*. Each run is kept; the previous run is drawn dashed behind the new one so you can see if a change helped. |
| **Motors** | Hold-to-drive pad (also arrow keys / WASD), per-motor sliders to check wiring and encoder direction, encoder counts. |
| **Parameters** | Every tunable value, with firmware default and a reset button. Save a profile in the browser or as a file, and **Copy C++** to paste the tuned values into `Main/Tunables.cpp` and `Main/Distance.ino` as the new defaults. |
| **Log** | Everything sent and received, plus the robot's normal `Serial` debug output. |

### Making values stick

Values live in the robot's RAM, so they reset when it reboots. Either:
- **Save to this browser** on the Parameters tab; next time you connect, the page offers to re-apply it, or
- **Copy C++** and paste it into the code, then flash.

The color baseline (`col.clear`) is measured at boot by `init_color()`, so power the robot on over a white tile.
It is left out of saved profiles and exported code on purpose.

## Safety

- **STOP** button (top right) and the **Space** bar stop all motors at any time.
- Manual driving only lasts while the page keeps refreshing it (every 100 ms). If the page closes, the
  tab is hidden, the cable is pulled or the browser stalls, the robot stops after 0.4 s.
- Tests abort on STOP, on the pause switch, if the page goes silent for 3 s, if both front sensors read
  under 50 mm (same emergency stop as `fwd()`), and after a hard time limit.
- Put the robot on blocks, or keep a hand on it, for the first drive of the day.

## Things worth knowing

- **`kd` is per microsecond.** `PID::getPID` divides the error change by elapsed *microseconds*, so a `kd`
  like 0.3 contributes about 1e-5 of the P term: effectively zero. Useful values are around 1e3 to 1e5, which is
  why the `kd` sliders go up to 1e6. (Changing `PID.cpp` to use seconds would be cleaner, but would change
  how every existing `kd` behaves, so it was left alone.)
- **`gyro` PID is not used by `fwd()`** right now (gyro hold is commented out; wall following is used).
  It is exposed because the *Forward (gyro hold)* test uses it.
- The PID classes in `fwd()`, `absoluteturn()` and `obstacleavoidance()` now read their gains from
  `tune.*` (`Main/Tunables.cpp` holds the defaults, identical to the old hard-coded numbers).
  Gains are read when each PID is created, so a change applies from the next move.
- `PID.cpp` now zero-initializes its state. The PIDs are stack locals, so before this the first
  derivative/integral term started from uninitialized memory and tests were not repeatable.
- The turn/forward **tests are copies of the control laws** in `movement.ino` (same speed limits,
  same exits) without the map/camera/ramp handling, because those need the pause switch to be in RUN. If you change
  the control law in `movement.ino`, change `tnTestTurn` / `tnTestFwd` in `Main/Tuning.ino` too.
- *Mirror real fwd() timing* (on by default) reads the color sensor every tick like `fwd()` does, so the loop
  rate, and therefore the derivative term, matches competition.

## Protocol (for debugging or adding commands)

115200 baud, one ASCII line each way. The page sends commands; the robot answers with one JSON object per line
(`{"t":"<type>",...}`). Any line not starting with `{` is ordinary debug output from the rest of the firmware.

```
TUNE                          handshake (only honoured while the robot is in PAUSE)
PING                          liveness
PARAMS                        dump the parameter table
SET <name> <value>            change a parameter (range-checked)
STREAM <hz> [dgce]            telemetry groups: d=distance g=gyro c=color e=encoders; 0 = off
MOT <A|B|C|D> <-255..255>     one motor (positive = forward); keep-alive needed
DRIVE <fw|bk|tl|tr> <0..255>  whole drivetrain; keep-alive needed
STOP | ENCRESET | EXIT
CALDIST <sensor> <mm>         100 raw samples -> suggested offset (not applied)
CLEARREF                      re-baseline the color sensor over a white tile
TEST TURN <deg>               + = clockwise
TEST FWD <mm> [wall|gyro] [realtiming 0|1]
```

Adding a tunable value: add one row to `tnParams[]` in `Main/Tuning.ino`. The page builds its UI from the table the
robot sends, so it appears in the Parameters tab automatically.

## Files

- `index.html`, `style.css`, `app.js`: the page
- `protocol.js`: Web Serial transport and protocol client
- `charts.js`: small canvas chart (no dependencies)
- `sim.js`: demo robot (made-up physics; it only exercises the UI)
- `../Main/Tuning.ino`: the robot side · `../Main/Tunables.h/.cpp`: tunable values and their defaults
