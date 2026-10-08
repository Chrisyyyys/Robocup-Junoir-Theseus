# Theseus V3 robot: hardware design spec (revision 3, merged)

Date: 2026-10-06. Status: draft for review, not committed. Scope: new robot hardware (chassis, drivetrain, sensors, bumpers, dropper, PCB) and the firmware changes it forces. Previous hardware reference: <https://github.com/Arsur24/Theseus>.

**Base document.** This revision starts from the approved draft, which is kept unchanged in [2026-10-06-theseus-v3-robot-design-v1-approved-draft.md](2026-10-06-theseus-v3-robot-design-v1-approved-draft.md), and changes only what the real code (`Main/` on `origin/main`, d17eee3) or a numeric check forced. Section 0 lists every change with its evidence; anything not listed there is as approved. My earlier revision 2 dropped parts of the approved draft (the PCB, the decisions table, ToF details, the floor-sensor placement and more); all of it is restored here.

**How to read the numbers.** Each number is tagged: **[rule]** from the 2026 Rescue Maze rules (final, 2026-03-29; rule numbers re-checked against the text), **[src]** from a vendor listing or web source (not checked against a datasheet unless stated), **[calc]** my own geometry or statics (assumptions stated, unmeasured), **[check]** a result of the numeric model in [v3-checks/](v3-checks/) (2D/3D geometry with shapely and numpy, run on a PC, not hardware), **[code]** read from `Main/*.ino` or `sim/` on `origin/main`, **[decision]** agreed in design discussion, **[placeholder]** a value I do not know yet (masses, battery size, omni width). Nothing here has been measured on hardware or run in the robot simulator.

## 0. What changed from the approved draft

### 0.1 Restored (revision 2 had dropped them)

- The **Decisions at a glance** table (section 3), including the **PCB** row.
- The **main PCB** section (section 9): power, drivers, encoders, mux, 9 ToF ports, 2 floor ports, 4 bumper inputs, dropper port, SD, LEDs, JST-SH cabling, ordered 2-layer board.
- All **nine ToF sensors** with their purposes and the wall-angle method (Hanafi et al.), the ramp hazard and IMU pitch gating (section 5). The sensor map is Figure 6.
- **Floor sensing as approved:** the silver module (straight plus tilted pair) near the axle, a second port at the front for black, blue and red. Revision 2 had moved both to the front; the colour code does not need that (section 5).
- The approved **14 mm pockets** (revision 2 had shrunk them to 13 mm), the **3 cm omni travel**, the cylinder-versus-octagon argument, the 2WD caveat, the obstacles rule row, the omni push table for 50, 60 and 70 mm, the Sprint 3 numbers, the N20 motor details, all bumper bullets, and the original verification and open-item lists.

### 0.2 Corrections to the approved draft (verified, kept)

| # | Approved draft said | Now | Evidence |
|---|---|---|---|
| 1 | Controller Teensy 4.1, "Serial2/3 unchanged", most firmware unchanged | **Arduino GIGA R1** (the board the V2 code builds for), main PCB as a GIGA-footprint shield | `Main.ino` starts with `#include <mbed.h>` and runs `rtos::Thread cameraThread`, `rtos::Thread pauseThread`, `rtos::Mutex i2cMutex` and `rtos::ThisThread::sleep_for(std::chrono::...)` (mbed OS, not available on a Teensy); the V2 spec and the simulator docs say it builds for the GIGA R1 (`arduino:mbed_giga`); the BOM lists two controller boards installed plus one spare, model not named **[code]**. A Teensy needs the camera and pause threads rewritten |
| 2 | Body radius 9.5 cm (10.5 fallback) | **10.5 cm** | With the wheel track, motors, chute exit and camera re-fitted for each radius, a 1.5 cm GIGA stack (the bare board) fits at R 10.0 and 10.5, the 1.9 cm stack with a shield only at 10.5, and nothing from 1.5 to 2.5 cm fits at 9.5 **[check]** (`fit_stack_only.py`). A smaller main board up to 7.0 x 4.5 x 1.5 cm does fit at 9.5 together with the battery, with the wheel track narrowed to 14.6 cm and the chute exit at x -5.7 **[check]** (`fit_R.py`). So the GIGA is what forces 10.5, not the 9.5 target: item 1 and D1 |
| 3 | Camera UART on `Serial2` / `Serial3` | `Serial3` / `Serial4` | `uart_camera_comms.ino` reads `Serial4` (left, `readSerial1`) and `Serial3` (right, `readSerial2`) **[code]** |
| 4 | `parallel()` threshold 10 mm is about 4.3 degrees | `PARALLEL_TOL_MM = 3` in `squareToWall()`, which is 1.2 degrees at the 14.6 cm side baseline | `Distance.ino:286` **[code]** |
| 5 | "Existing code says 68.7 mm" | `wheel_diameter = 80` | `Main.ino:119` **[code]**; Sprint1.md also says 80 |
| 6 | Plate: 14 mm pockets on a 3.3 cm ring, plate about 8.9 cm, about 3 mm margin parked | **Ring radius 38.6 mm, plate 9.8 cm across**; pockets stay 14 mm, slots 14.5 mm | At 33 mm the pocket squares touch at their inner corners (wall 0.0 mm) and a parked pocket touches its slot (0.0 mm). Walls of 2.8 mm and a parked margin of 2.5 mm need 38.6 mm. All 128 drop sequences still pass (0 violations) **[check]** |
| 7 | Chute exit at x about -5.5 cm and "about 9 cm lateral" | Exit on the wall at **(-6.0, +-8.62)**, nose at z 3.0 | (-5.5, 9) is 10.5 cm from the axle, outside a 9.5 cm body. On the wall at R 10.5 the exit at x -5.5 would be 3 mm from the wheel; -6.0 gives 6.7 mm, which meets the draft's own 5 mm rule **[check]** |
| 8 | Rear skid, belly clearance 2.5 cm | **Rear nub 2.0 cm off the floor plus a rear underside chamfer, belly 3.5 cm** | With the 5 mm skid and belly 2.5 the robot cannot enter a 25 degree ramp: no feasible pose at the foot, the belly line runs 2.4 cm inside the ramp. Raising the belly to 3.5 alone still has no feasible pose (belly 0.4 mm). Nub 2.0 plus chamfer passes with 15 mm of belly clearance **[check]** |
| 9 | Sprint 3 "found kits stayed put from about 3 cm" | Sprint 3's only drop evidence is a **cardboard prototype** with "minimal rolling from about 3 cm" | Sprint 3 write-up in the Theseus repo **[src]**. It is not a measurement against the 15 cm rule |
| 10 | Handle on the lid | **Handle fixed to the chassis** | The captain picks the robot up by it after a lack of progress (rules 4.2.7, 4.2.8, 5.5.3); a handle on a lift-off lid would lift the lid and spill the kits **[rule]** |
| 11 | Front-centre ToF on the front face | **Recessed 1 cm** | On the rim at R 10.5 it would read 37 mm at tile centre next to the 30 mm minimum range; recessed it reads 45 mm **[calc]** |
| 12 | `FRONT_WALL_MAX_MM` implied unchanged | New value about **120** | The cone edge at the foot of a 25 degree ramp reads 183 mm, under V2's 200, so the ramp foot would be taken for a wall; a wall in the current tile reads at most 75 mm **[calc]** |

### 0.3 Consequences of those corrections (not new design choices)

- **ToF ring at z 10.0 cm** (draft 6.5). The belly went up (item 8), so the bumper band moved up with it to z 4.0-6.5; the ToF modules (2.1 cm tall) must clear that band, and 10.0 also keeps the V2 window height of about 10.6 cm that the ramp logic was tuned for **[code]**.
- **Dropper plate at z 9.0-10.2** (draft 6-8): the GIGA stack (1.9 cm) sits under it. Chute drop 4.2 cm over 6.0 cm gives a 35 degree slide, inside the draft's 30-35 degree range.
- **Swept circle 22 cm** (draft 20): 1.6 cm per side in a 25.2 cm corridor (draft 2.6). Side-sensor worst-case reading 33 mm (draft 39).
- **Front load 11.9%** (draft 14%): the omni is 7.95 cm ahead of the axle, not 7.
- **Body about 12 cm tall** (lid 12.1): the cameras sit above the wheels, as you asked in the last round.

### 0.4 New findings and risks (all unmeasured)

1. **Camera standoff.** With the lens flush with the wall it sees only 5.1 cm of wall on a 28 cm path and 3.2 cm on a 25.2 cm path, narrower than a 4 cm letter. Recessing the lens 1.5 cm behind a window gives 7.2 and 5.2 cm (section 8) **[check]**.
2. **Front lift on acceleration.** The omni unloads above g x COM_x / COM_h = 9.81 x 0.95 / 6.4 = 1.5 m/s^2 (0.15 g) **[calc]**; a step to full PWM would exceed that many times over (traction-limited at about 15 m/s^2). Ramp the PWM in firmware or move the centre of mass forward (section 4).
3. **Two corner cases the rules allow.** A 2 cm speed bump on a 25 degree ramp (allowed inside the Dangerous Zone only, rules 3.5.4 and 3.5.7) is **not passed**: belly and bumper clear by under 1 mm. Three 2 cm stairs at the 30 degree DZ incline clear the belly by 1 mm. Both pass at 25 degrees outside the zone (Figure 5, section 4). The Dangerous Zone is optional (rule 3.5.3).
4. **2WD stairs depend on friction** (unchanged from the draft, now quantified in section 4): a rigid 80 mm wheel needs a friction coefficient of about 1.7 for a 2 cm riser. The stair test is the go/no-go.
5. **The packing model and the side-view model disagreed once:** the first packing put a board at z 3.6-4.4 under the rear chamfer. Fixed; the rear bottom corner is not usable below z 4.3-5.2.

### 0.5 Decisions that need your confirmation

| # | Question | My default |
|---|---|---|
| D1 | Controller and body size. **A:** GIGA R1 on a shield-style main PCB, body R 10.5, firmware runs as it is (this spec). **B:** a small controller on a custom main board of at most 7.0 x 4.5 x 1.5 cm, body R 9.5 as you first wanted: that board plus the battery fit at 9.5 **[check]**, but every other check here (terrain, ToF cones, bumpers, plate, camera) was run at 10.5 and would be repeated at 9.5 (omni 7.0 cm ahead, wheel track 14.6 cm, swept circle 20 cm). A Teensy 4.1 needs the camera and pause threads rewritten; a Portenta H7 on the custom carrier would keep the mbed code (same chip family as the GIGA) but I have not checked its pins and UARTs against the code | A |
| D2 | Bumper plate ends: 58 degrees (3.3 cm to the worst-case wall, this spec) or the approved 70 degrees (2.3 cm; geometrically possible, at least 9 mm from every other part) | 58 |
| D3 | Rear nub height: 2.0 cm (14 degree nose-up on a 2 cm step, 15 mm ramp clearance) or 1.5 cm (11 degrees, 8 mm ramp clearance) | 2.0 |
| D4 | Camera lens recessed 1.5 cm behind a window (this spec) or flush | recessed |
| D5 | Centre of mass: accept +0.95 cm and limit acceleration in firmware, or add about 80 g of ballast at the front for +1.5 cm | firmware limit first |
| D6 | Pocket size: 14 mm (approved, plate 9.8 cm) or 13 mm (plate 9.2 cm, 2.7 mm slack) | 14 |
| D7 | Kit printed at 10.3 mm (margin on the 1 cm minimum, rule 3.7.3) or exactly 10 mm as in Sprint 3 | 10.3 |
| D8 | Accept that a 2 cm bump on a 25 degree ramp in the Dangerous Zone is not passed | accept |

## 1. Intent and success criteria

Build a smaller, more agile robot than the current 4WD skid-steer robot, reusing most of the firmware. The user's reasoning for 2WD: less scrub in turns, fewer botched turns. Caveat recorded: turn accuracy comes from the BNO055, so the gain is smoother and faster pivots, not accuracy; the cost is stairs.

Proposed success criteria (adjust):

1. Pivots in place inside a 25.2 cm corridor (28 cm with the rules' 10% tolerance) without touching walls when centred. The swept radius is 11.0 cm, so 1.6 cm per side remain in the 25.2 cm corridor **[calc]**.
2. Climbs a 2 cm stair, a 25 degree ramp and 1-2 cm speed bumps reliably (target: at least 9 of 10 attempts on a test rig).
3. Tells white, black, blue, red and silver floor tiles apart, including the glass-like silver tile that read like white on the TCS34725.
4. Carries 8 kits and drops them within 15 cm of a wall victim without bounce or roll.
5. Most of `Main/` (map, BFS, PID, gyro, camera UART) runs unchanged.

## 2. Rule constraints that drive the design

| Constraint | Value | Rule |
|---|---|---|
| Tile / corridor width | 30 cm / 28 cm, all measurements +-10% | 3.1.2, 3.3.3, 3.8.6 |
| Robot height | at most 25 cm; bridges leave 25 cm | 4.2.9, 3.2.7 |
| Speed bumps / stairs / ramps | at most 1 cm (2 cm in Dangerous Zone; bumps may sit on ramps inside the zone) / at most 2 cm, incline under 25 degrees (30 inside the zone) / at most 25 degrees; floor steps up to 3 mm | 3.4, 3.5, 3.1.5, 3.2.1 |
| Obstacles | at least 15 cm tall, at least 20 cm from walls or touching a wall | 3.4 |
| Victims | about 7 cm above floor, letters 4 cm tall (can be rotated) | 3.6 |
| Kits | at least 1 cm per side and 1 cm^3; at most 8 per robot; must land within 15 cm of the victim and stay (the position is judged when the robot has moved entirely out of the 15 cm boundary) | 3.7.3, 3.7.4, 3.7.2, 5.6.3 |
| No reload mid-run | captain loads at start; modifying the robot mid-run is prohibited; after a lack of progress only the button and one power switch | 3.7.5, 5.4.1, 5.5.3 |
| Required hardware | handle, single start button, dedicated victim LED (blink 500 ms on / 500 ms off for 5 s); class 1 or 2 lasers only, datasheet submitted before the competition (the VL53L0X is class 1: bring its datasheet) | 4.2.7, 4.2.8, 4.2.11, 5.6.1, 4.2.4 |
| Black tiles | holes; visiting one is a lack of progress (-15 reliability points) | 3.2.3, 5.5.1, 5.6.7 |

Scoring context for priorities: stairs 10 points plus 5 exit bonus, ramp 10 plus 5, speed-bump tile 5, checkpoint 10, kit 10 plus 10 reliability, two kits on one victim 30.

## 3. Decisions at a glance

| Area | Decision | Versus the approved draft |
|---|---|---|
| Drive | 2WD differential, axle at the centre of the swept circle, 80 mm cast silicone wheels (baseline), half-recessed in wheel wells | same |
| Front support | single sprung 60 mm omni on the centreline, about 3 cm travel, adjustable preload, hard stops. Rocker bar rejected (no pitch travel on stairs). Smaller omnis rejected: a free wheel cannot climb a step as high as its radius | same |
| Rear support | rear nub 2.0 cm off the floor (adjustable 1.5-2.5) at x -8.0, plus a rear underside chamfer from x -6.5 (24 degrees) | changed: the draft's rear skid cannot enter a 25 degree ramp (0.2 #8) |
| Weight | centre of mass about 1 cm ahead of the axle, front load about 12% (computed +0.95 cm, 11.9%, placeholder masses) | draft 14%: the omni is 7.95 cm ahead, not 7 |
| Body | round base (cylinder), body radius 10.5 cm, swept radius 11.0 cm, belly 3.5 cm, about 12 cm tall (lid 12.1), no taper | changed: R 9.5, belly 2.5, 11-12 cm (0.2 #2, #8; 0.3) |
| Bumpers | two low front-arc plates that stick out 0.5 cm, swept circle 11.0 cm; plates 12 to 58 degrees, band z 4.0-6.5 | ends 58 not 70 (D2); band raised with the belly |
| Distance sensing | 9 x VL53L0X: 3 front, 4 side, 2 rear; two TCA9548A muxes; one printed ring at z 10.0 | ring height 6.5 -> 10.0 (0.3) |
| Floor sensing | two wired, swappable floor-sensor ports; silver module (straight plus tilted pair) near the axle, second port at the front for black, blue and red | same |
| Dropper | low turntable inside the base under the lid; plate 9.8 cm across, 14 mm pockets for 10 mm cubes, one blank arc where both slots park, side chutes exiting behind the wheel wells (x -6.0) at 3 cm height; N20 gearmotor with encoder | plate 8.9 -> 9.8 cm (0.2 #6); plate at z 9.0-10.2 above the controller stack |
| Controller | Arduino GIGA R1 | changed from Teensy 4.1 (0.2 #1, D1) |
| PCB | separate main board (2-layer, ordered), here a GIGA-footprint shield carrying muxes, drivers, ports and connectors, plus off-the-shelf sensor breakouts on short cables | same intent |
| Cameras | two side OpenMV H7 Plus above the wheels (the draft's option A), lens recessed 1.5 cm behind a window, lens z 9.3, tilted 20 degrees down | decided (draft: open) |

## 4. Geometry and drivetrain

Frame: x forward, y left, z up, origin at the axle midpoint on the floor line (axle at z 4.0). Figures 1-3 show the baseline.

**Swept circle.** What matters in a dead-end tile is the farthest body point from the axle midpoint, not body size. With the axle at the body centre the swept radius is the body radius plus whatever sticks out.

- Body radius 10.5 cm; bumper tips and the omni's front edge 11.0 cm, so the swept circle is a 22 cm circle. In the worst-case 25.2 cm corridor that leaves 1.6 cm per side, in a 28 cm path 3.0 cm **[calc]**. (Draft at R 9.5: 20 cm circle, 2.6 cm.)
- A cylinder gives about 11% more area than an octagon at the same swept radius (346 vs 312 cm^2 at R = 10.5; the draft's 283 vs 255 at 9.5) **[calc]**. Sensor pockets are printed, so flat faces are not needed.

**Axle and weight balance.** The user's stated reason for the axle being slightly behind was weight balance. The axle goes at the circle centre. Centre of mass about 1 cm ahead of the axle, so the front omni carries about 12% (`1/7.95`) and the drive wheels about 88%; the packing gives +0.95 cm (11.9%) with a **[placeholder]** mass of 1.07 kg and a COM height of 6.4 cm **[check]**. An earlier draft used 2-3 cm (about 36% front load) to avoid tipping back on a 25 degree ramp; that is only needed without a rear nub. With the nub the robot can rest on both drive wheels and the nub on a ramp (the front omni unloads and the nub drags slightly): on a 25 degree ramp the COM is 2.0 cm behind the drive contact, the nub carries about 25% of the load and the body tilts back about 14 degrees; avoiding that altogether needs the COM at least 3 cm forward (38% front load), which the omni cannot afford on stairs. Make the nub a low-friction part (ball or PTFE) **[calc]**. Descending a 25 degree ramp the omni carries about 49% of the weight (5.2 N), so the arm spring needs at least 1.6 N/cm **[calc]**.

A heavy front is worse for stairs: pushing a free front wheel over a 2 cm step needs a horizontal force of `front_load x tan(alpha)` where `cos(alpha) = (r - h)/r`. With `W` the total weight, push needed (2 cm step):

| Omni | at 11.9% front load (this spec) | at 14% | at 36% |
|---|---|---|---|
| 50 mm | 0.58 W | 0.69 W | 1.76 W |
| 60 mm | 0.34 W | 0.40 W | 1.02 W |
| 70 mm | 0.25 W | 0.29 W | 0.76 W |
| drive traction available at friction 1.5 | 1.32 W | 1.29 W | 0.96 W |

So at 36% a 60 mm omni fails, at 12-14% it passes **[calc]**. Not modelled: the spring force growing as the arm compresses, pitch, roller friction.

**Acceleration limit (new).** The omni unloads when the acceleration exceeds `g x COM_x / COM_h` = 9.81 x 0.95 / 6.4 = 1.5 m/s^2 **[calc]**; at COM +1.5 cm it is 2.3 m/s^2, at +2.0 cm 3.1 m/s^2. A step to full PWM (traction-limited at about 15 m/s^2, ten times the threshold) would lift the front until the nub touches (about 14 degrees nose-up), which tilts the front ToF beams. About 80 g of ballast at x +9 cm moves the COM from +0.95 to +1.5 cm **[calc]**. Either ramp the PWM in the firmware (acceleration under about 1 m/s^2) or add ballast; measure the real centre of mass first.

**Wheels.** 80 mm wheels fit inside the circle: outer face at 9.0 cm laterally, tyre corner at radius 9.85 cm (inside R 10.5), wheel centre planes at +-8.0 cm (track 16 cm), wheel width 2.0 cm **[calc]**. Rigid-wheel statics say a wheel of radius r climbs a step h only if the friction coefficient satisfies `h <= r(1 - cos(atan mu))`:

| Wheel | h = 1 cm | h = 1.5 cm | h = 2 cm |
|---|---|---|---|
| 80 mm | 0.88 | 1.25 | **1.73** |
| 90 mm | 0.81 | 1.12 | 1.50 |
| 100 mm | 0.75 | 1.02 | 1.33 |

Soft cast silicone should do better but this is unmeasured, which is why the stair test comes first; with only two driven wheels nothing pushes from behind (the V2 robot has four). The repo has silicone wheel casting molds (diameter unverified). `Main.ino` has `wheel_diameter = 80` and Sprint1.md says 80 mm; 80 mm is the baseline.

**Front omni.** 60 mm baseline (user suggested smaller; rejected: a 40 mm omni has radius 2 cm equal to the step so it cannot climb a 2 cm riser, 50 mm is workable only at a light front load). The omni must sit inside the swept circle, so axle-to-omni distance is about the swept radius minus the omni radius: 7.95 cm for a 60 mm omni at R_swept 11.0 **[calc]**. A 50 mm omni would add about 0.5 cm of wheelbase; keep it as a parameter for the stair test. Sprung arm: pivot at (3.5, 3.6), 0.6 cm above the rest axle so the omni moves back as it compresses; its front edge stays within 10.99 cm over the whole travel; 3 cm mechanical travel (with nub 2.0: a 2 cm riser uses 0 cm, a 25 degree ramp 1.5 cm, the 30 degree Dangerous Zone stairs 2.1 cm; travel 2.5 and 3.0 give the same results for every required case) **[check]**. A rigid front wheel would tilt the robot nose-up by about 14.6 degrees on a 2 cm step (`asin(2/7.95)`); with the arm the tilt is set by the rear nub (table below). The front-centre ToF sensor sits above and ahead of the omni, recessed 1 cm, with about 0.9 cm between the fully compressed omni and the sensor board (1.1 cm at 2.5 cm travel) **[calc]**.

**Rear support (new).** The rear nub is 2.0 cm off the floor at x -8.0; the rear underside rises from x -6.5 (z 3.5) to z 5.3 at the rear edge, 24 degrees. The nub sets how far the body can tilt nose-up when the omni is on a step, and the chamfer lets the tail clear the ramp surface. Side-view results for a 2 cm riser and a 25 degree ramp, belly 3.5, chamfer on **[check]**:

| Nub height | 2 cm riser up: body tilt / omni used | 25 degree ramp |
|---|---|---|
| 0.5 cm | 7.9 degrees / 1.75 cm | fails: no feasible pose at the foot, belly 0.4 mm |
| 1.0 cm | 9.7 degrees / 1.00 cm | fails: belly 1.5 mm |
| 1.5 cm | 10.8 degrees / 0.50 cm | passes, smallest clearance 8.4 mm (belly), omni 2.0 cm |
| **2.0 cm** | **14.3 degrees / 0 cm** | **passes, smallest clearance 12.6 mm (chute nose), belly 15.1, omni 1.5 cm** |
| 2.5 cm | 14.3 degrees / 0 cm | passes, smallest clearance 8.3 mm (chute nose), belly 14.6, omni 1.0 cm |

Without the chamfer the nub 2.0 cm passes the ramp with only 3.5-4.6 mm of belly clearance; with belly 2.5 and the chamfer, 5.3 mm. Both changes are needed for a 15 mm margin. You dislike the robot tilting back on stairs: 1.5 cm tilts 11 degrees instead of 14 but leaves 8 mm on a ramp (D3).

**Terrain results** (belly 3.5, nub 2.0, rear chamfer, bumper bottom 4.0, chute nose 3.0, omni travel 2.5; smallest clearance over the whole crossing, cm; omni compression is the stiff-spring worst case) **[check]**, poses in Figure 5:

| Case | Result | Belly | Bumper | Chute nose | Max tilt |
|---|---|---|---|---|---|
| 2 cm riser up / down | pass | 1.58 / 1.50 | 2.01 / 2.75 | 1.30 / 1.00 | +14.3 / -14.8 |
| two 1 cm steps up / down | pass | 1.91 / 1.50 | 2.85 / 3.24 | 1.30 / 1.00 | +14.3 / -14.8 |
| 25 degree ramp up / down | pass | 1.51 / 1.52 | 1.94 / 1.95 | 1.26 / 1.28 | +25 / -25 |
| 2 cm / 1 cm speed bump | pass | 1.50 / 2.50 | 2.01 / 3.00 | 1.00 / 2.00 | +14.3 / +7.2 |
| 3 mm tile seam | pass | 3.20 | 3.70 | 2.74 | +2.2 |
| three 2 cm stairs, 25 degree incline, up / down | pass | 1.49 / 2.14 | 1.80 / 2.42 | 1.24 / 3.00 | +28.7 / -30.5 |
| three 2 cm stairs, 30 degree incline (Dangerous Zone), up / down | up: 1 mm belly; down passes | 0.10 / 2.10 | 0.48 / 2.37 | 1.27 / 2.91 | +29.8 / -31.4 |
| 1 cm bump on a 25 degree ramp, up (Dangerous Zone) | pass | 0.94 | 1.13 | 1.26 | +32.2 |
| **2 cm bump on a 25 degree ramp, up (Dangerous Zone)** | **fails** | 0.06 | 0.08 | 1.00 | +39.2 |

Floor sensors: a face 2.5-3.5 cm above the floor at the front port (x 6.5) clears every case by at least 1.15 cm; the silver module (face z 2.8) at the axle line by at least 2.5 cm **[check]**. A three-step 2 cm staircase is not required by my reading of rule 3.4.6 ("the maximum height is 2 cm"; V2's simulator also uses two 1 cm steps, `field.step_height_mm = 10`).

**Motors.** Going from 4 to 2 motors doubles torque per motor. The old robot used Pololu 195:1 20Dx44L 12 V gearmotors (72 rpm no load, stall 10 kg-cm at 1.6 A, gearbox limit 5 kg-cm **[src]**, Pololu item 3493; BOM). Torque needed per wheel: 1.63 kg-cm for a 2 cm riser (rigid wheel), 0.90 kg-cm for a 25 degree ramp; no-load speed 0.30 m/s, so a tile takes at least 1.0 s **[calc]**. Each motor occupies y 1.4-6.1 (44 mm plus 3 mm encoder), leaving a 2.8 cm gap between them. Motor, battery and driver are selected from the torque budget (open item); 12 V motors suggest a 3S pack and a driver rated for at least 14 V and 2 A per channel.

## 5. Sensing

Frame: x forward, y left, origin at axle midpoint, positions on the R = 10.5 cm circle. All ToF at 10.0 cm height (modules 8.95-11.05) on one printed ring. **All nine VL53L0X are present** (Figure 6):

| # | Group | Count | Position (x, y cm) | Aim | Purpose |
|---|---|---|---|---|---|
| 1 | Front centre | 1 | (9.5, 0), recessed 1 cm | straight ahead | front wall, obstacles |
| 2-3 | Front toed-out | 2 | (9.33, +-4.35) | +-25 degrees off forward (tune in sim) | front-wall angle, early side openings |
| 4-7 | Side | 4 | (+-7.28, +-7.28) | straight left / right | wall presence and angle, 14.6 cm baseline |
| 8-9 | Rear | 2 | (-9.26, +-4.52) | straight back | reversing, wall angle when backing to a wall, 9.0 cm baseline |

Every 25 degree cone was traced against the robot's own parts (plate, chutes, cameras, PCB stack, battery, bumpers, wheels): none is blocked **[check]**. Readings with the robot centred and square in a 28 cm path: front centre 45 mm, toed-out 51.5 mm, side 67 mm, rear 47 mm.

**[calc]** unless noted. At the 14.6 cm side baseline, 1 degree of heading error is 2.5 mm of difference; `squareToWall()` stops at `PARALLEL_TOL_MM = 3`, which is 1.2 degrees **[code]** (the draft said a 10 mm threshold of about 4.3 degrees). At the 9.0 cm rear baseline 1 degree is 1.6 mm. Side-sensor wall distance is 6.7 cm nominal and about 3.3 cm worst case (25.2 cm corridor, robot 2 cm off-centre; draft 3.9 cm); the VL53L0X minimum is about 3 cm **[src]**, so the margin is thin. Options: recess the side modules 5 mm (+5 mm of reading), or fall back to VL53L4CD (1 mm minimum **[src]**).

Wall-angle method follows Hanafi et al. 2013 (two same-side sensors, angle from the reading difference over the baseline; PID retained, their fuzzy controller not adopted because the paper is a single straight-corridor ultrasonic test with implausible error figures and untuned PD/PID baselines). Use the angle only when both sensors on that side see the same wall, and use it to re-zero IMU heading drift.

**Addressing.** Two TCA9548A muxes (0x70, 0x71), 16 ports: 9 ToF, 2 floor ports, spare. BNO055 (0x28) on the main bus. A proposed port map is in Figure 6.

**Ramp hazard.** A horizontal front beam on a 25 degree ramp hits the ramp surface and reads it as a wall. With the robot at the tile centre facing the ramp (foot 15 cm ahead, front sensor 9.5 cm forward) the nearest edge of the cone at 10.0 cm height reads 183 mm (beam centre 269 mm); at the draft's 6.5 cm it would read 132 mm **[calc]**. A wall in the current tile reads at most 75 mm (45 mm at tile centre, the robot stops up to 30 mm short), so `FRONT_WALL_MAX_MM` about 120 separates them; V2's 200 would take the ramp foot for a wall. Gate front sensors with IMU pitch as well.

**Floor sensing and silver.** The floor module sits near the axle for silver (a tile is visited when more than half the robot is on it, 5.4.4); a second port sits at the front for black/blue/red so a hole is seen before the wheels reach it (the front port is 6.5 cm ahead of the wheel contact). The colour code (`color.ino`) accepts silver only in the second half of a move (`overNextTile`: encoder average beyond half a tile), that is when the sensor is over the tile ahead; a sensor at the axle line is over the next tile exactly then, so this placement matches the code, and black is accepted at any time, which is why the front port exists **[code]**. Positions: silver module at (0, 0) between the motors (2.0 x 2.2 cm, face z 2.8), front port at (6.5, +3.0) beside the omni arm (face z 2.8) **[calc]**.
Silver is mirror-like; a mirror throws the emitter light away from a tilted sensor. Plan: one sensor straight down plus one tilted (about 20 degrees; at 25 mm height the specular spot lands about 21 mm off to the side **[calc]**). White: high/high. Black: low/low. Silver: high straight, low tilted. A tilted sensor alone would read silver as black, so both are needed.
Candidates for the bench test: TCS34725 raw clear channel (vendor listings give a 2-10 mm working distance **[src]**, which may explain the earlier failure if it was mounted higher), Pololu QTRX-MD (optimal 10 mm, max recommended 40-50 mm **[src]**), VCNL4040 (I2C, qualitative to about 20 cm **[src]**). With the belly at 3.5 cm the module faces at z 2.5-3.5 leave at least 1.15 cm of clearance on every terrain case; short-range optics need a low rounded skid mount or a longer-range part. RCJ documentation found only says the tile is highly reflective and suggests reading raw light intensity **[src]**; no team write-up of a robust method was found.
The main PCB carries two identical wired floor-sensor ports (3V3, GND, SDA, SCL, two analog inputs, one LED-enable line) so the module can be swapped later.

## 6. Bumpers

Two curved plates on the front arcs, 12 to 58 degrees either side of centre (about 8.8 cm long each at the 11.0 cm outer radius; the approved 12 to 70 degrees would be 11.1 cm) **[calc]**, hinged at the outer ends, return spring, 4 mm travel. The bumper is the outermost surface: body behind it is recessed by the travel. Decision: the bumpers stick out for now, giving a swept radius of 11.0 cm.

- Height band 4.0-6.5 cm (draft 2.5-5, raised with the belly) so a 2 cm stair riser or speed bump does not trigger them (smallest clearance 1.94 cm, section 4); the ToF ring sits above at 10.0 cm.
- Sensor: sub-miniature SPDT snap-action lever microswitch (Omron D2F class or generic; part number not verified), normally-open to ground on a pull-up input, 5-10 ms software debounce, one at the inner end of each plate (Figures 1 and 6). A Hall sensor with a magnet is rejected: the BNO055 has a magnetometer and the rules warn of magnetic interference (3.8). Backup: slotted optical interrupter.
- PCB has 4 bumper inputs; populate 2 first (one per plate): left only, right only, both. The outer ends take the two spare inputs.
- Role: safety net for unseen obstacles, being wedged, and the sensor dead zone; triggers the existing `BACKPEDAL` state. Not a mapping input.
- False triggers: the plate ends at 58 degrees reach 9.3 cm sideways, so a robot more than about 3.3 cm off-centre in the worst-case corridor brushes the wall. The approved 70 degrees would reach 10.3 cm (2.3 cm of margin at the larger swept radius; the draft had 3.7 cm); 70 degrees is geometrically possible, at least 9 mm from every other part **[check]** (D2). Software accepts a bumper hit only when it coincides with forward motion and no wall is reported on that side.
- A very thin post exactly dead-centre could pass between the plates; the front-centre ToF covers it.

## 7. Dropper

Low inside the base under a lift-off lid (the captain loads kits by lifting the lid); start button and victim LED on the lid, the **handle on the chassis** (0.2 #10). Earlier drafts put the plate on top of a taper. That was dropped because impact speed depends on the total fall height, not the exit height: Sprint 3 found kits stayed put from about 3 cm (about 0.77 m/s; a cardboard prototype, not a measurement against the 15 cm rule), while a 15 cm fall hits at about 1.7 m/s **[calc]**, and the extra height raised the centre of mass.

- **Kit:** 10 x 10 x 10 mm weighted roly-poly cube from Sprint 3 (rules minimum is 1 cm per side and 1 cm^3, so it cannot shrink). 8 kits, because there is no reload. Recommended: print it at 10.3 +-0.2 mm so tolerance never takes it under 1 cm (D7).
- **Plate:** 14 mm pockets (old design 19 mm; the extra slack was requested), 8 pockets at 30 degree pitch on a **3.86 cm radius** circle covering 30 to 240 degrees, slots 14.5 mm, plate radius 49.1 mm, so **9.8 cm across** (old at least 15 cm). Walls between pockets 2.8 mm, parked margin 2.5 mm **[check]**. The draft's 3.3 cm ring was too small for 14 mm pockets (0.2 #6); 13 mm pockets would allow 3.62 cm and a 9.2 cm plate (D6).
- **Slot logic (replaces an earlier half-pitch-offset requirement, which was wrong: a 14 mm slot half a pitch from a pocket still sits partly under two pockets and leaks, and a full pocket sweeping over the other slot would drop its kit):** two fixed slots in the base at 0 and 270 degrees of the plate frame. The blank arc (90 degrees between the two slots; counting the pocket half-width about 250 to 380 degrees of the plate frame) holds both slots when parked, with 2.5 mm of plate to the nearest pocket each side. Turn the plate CW to bring the next pocket from the pocket-1 end over slot A, CCW to bring the next pocket from the pocket-8 end over slot B. Always use the pocket nearest the target slot first, so a full pocket never passes a slot; any mix of left and right drops totalling 8 works: all 128 states (kits used from each end, side, one or two kits per victim, parked and after each sweep) were simulated with 0 violations **[check]**. A home switch gives the parked reference. The slots are 90 degrees apart and sit symmetric about the robot centre line (A rear-right at robot angle 225 degrees, B rear-left at 135, blank arc facing the rear), centred at (-2.0, 0); the orientation is to be verified in CAD. Figure 4.
- **Tolerance [check]:** the kit footprint stays fully over the opening for +-3 degrees of plate error (94% at 4 degrees); parked, the first pocket-slot contact is at 5-6 degrees (3.4-4.0 mm at the ring). The firmware's `dispenser(incr, offset, steps)` formulas carry over with `incr = 30`, `offset = 0`, the sign of `rotate()` flipped (V2 turned left drops clockwise; here the right slot is served clockwise), and a dwell of about 0.3 s per pocket instead of rotating through at speed.
- **Chutes:** two enclosed channels, 15 mm bore, 1.5 mm wall, at least 5 mm clearance from motors, wheels and PCB (6.7 mm from the wheel **[check]**). Exits go through the side wall at x -6.0 cm (y +-8.62), just behind the wheel wells (wheels span x +-4 cm; bumpers, front side ToF and cameras are in the front half), nose at 3 cm height. Slope 35 degrees, path 7.3 cm. The kit leaves at about 0.64 m/s (assumed friction coefficient 0.35; 0.48-0.73 for 0.5-0.25), lands 2.7 cm further out about 3 cm from a 28 cm wall, at x about -6.6 **[calc]**. Its impact speed is about 1.0 m/s against the roughly 0.77 m/s of the Sprint 3 test, so the last section needs a friction patch or switchback; slopes of about 20 degrees or less may let a kit stick. The printed chute test with real kits decides the geometry.
- **15 cm rule [calc]:** the camera is at x +0.3 and the kit lands at x -6.6, so the kit is 4.7 to 10.8 cm from the victim when the victim is 3 cm behind to 3.6 cm ahead of the camera at the stop; the along-wall slack for the 15 cm rule is about +-14.8 cm (draft: +-14 instead of +-9.5).
- **Motor:** N20 gearmotor with magnetic encoder (10 x 12 x 41.5 mm, about 882 counts per output rev, 0.5 kg-cm stall at 6 V **[src]**), closed-loop with a home switch. At 882 counts per revolution one count is 0.41 degrees. The 28BYJ-48 is dropped as too large. Chosen over a 15 mm stepper (15 x 16.5 mm **[src]**) for torque margin (estimated plate friction a few N-mm **[calc]**), reuse of the drive-motor encoder/PID path, and footprint. Always approach each index from the same direction to cancel backlash. Fallback: 15 mm stepper. The PCB dropper port (DRV8833-class driver, from memory 2.7-10.8 V, about 1.2 A per channel, plus two encoder or home inputs) supports either.
- **Height budget:** plate z 9.0-10.2 (floor 8.7-9.0) above the controller stack and the motors (top at 5.0 cm), inside the ToF ring; lid 12.1 over the camera humps, so about 12 cm against the 25 cm limit **[calc]**.

## 8. Cameras

Two side cameras, OpenMV H7 Plus (45 x 36 mm board, 29 mm with the lens, 17 g, 2.8 mm lens, field of view 65.9 x 51.8 degrees, 3.6-5 V in, 3.3 V I/O **[src]**; the repo has an Edge Impulse script `ei_object_detection_PhiOmegaPsi.py`, not read); UART to `Serial4` (left) and `Serial3` (right), 3.3 V logic **[code]**.

Decision: option A of the draft, **above the wheel wells**, looking straight sideways, tilted 20 degrees down. The wheel top is at 8.0 cm and victims at 7 cm, so beside the wheel the board hits the wheel top: a placement search with the camera beside the wheel at lens heights 6.2-7.4 cm found no collision-free position at R 10.5 for any roof height (an earlier exploration with angled aims found one marginal spot far forward, not re-run) **[check]**. Above the wheel the lens axis is at z 9.3 (the lens block clears the wheel top by 3.6 mm), the board top at 12.05, the lid at 12.1, and the lens x is +0.3, so the chute exits at x -6.0 put the kit about 7 cm behind the camera. The BOM lists 2 cameras required, 4 existing (on the two V2 robots) and none spare, and the OpenMV page says the H7 Plus is sold out or production-run only: the V3 reuses two of the existing four.

**Lens recessed 1.5 cm behind a window (new).** The lens tip is at radius 8.8 cm, not at the wall, behind a window of about 2 x 2 cm in the shell. The reason is the field of view **[calc]**:

| Lens-to-wall distance | Wall seen, height (letter band 5-9) | Width of wall seen (letter 4 cm, may be rotated) |
|---|---|---|
| flush, 28 cm path: 3.7 cm | z 5.5-9.7 (all but 0.5 cm of the band) | 5.1 cm |
| flush, 25.2 cm path: 2.3 cm | z 6.9-9.5 (the bottom 1.9 cm of the letter is cut off) | 3.2 cm (the letter does not fit) |
| **recessed, 28 cm path: 5.2 cm** | **z 3.9-9.8 (the whole band)** | **7.2 cm** |
| **recessed, 25.2 cm path: 3.8 cm** | **z 5.4-9.7** | **5.2 cm** |

The packing is unchanged by the recess (the board moves inboard but stays above the controller stack, 4 mm clear) **[check]**. A recessed camera also sees the victim earlier, which moves the kit slightly further behind it: 10.8 cm worst case, still inside 15. The M12 lens will need refocusing to about 4-5 cm (the datasheet gives no minimum focus distance): test on the bench. A wider-angle M12 lens is the alternative if the letter is still cropped. Tilting 25 degrees instead of 20 would show the whole band even on a 25.2 cm path (z 4.6-9.4) at the cost of 0.2 cm more lid (board top 12.24, lid 12.3) **[check]**. Option B of the draft (ahead of the wheel wells at about +-60 degrees, angled 25-30 degrees forward) is not needed. Figure 3 shows the view.

## 9. Electronics

- **Controller:** Arduino GIGA R1 (101.5 x 53.3 mm; 76 digital I/O, PWM on D2-D13 **[src]**), 3.3 V logic, `Serial3` / `Serial4` for the two cameras (3.3 V, 5 V tolerant I/O on the OpenMV, no level shifter). The draft's Teensy 4.1 (3 I2C, SD slot) is not used (0.2 #1, D1). The GIGA has no SD slot: the main PCB carries a microSD socket on SPI.
- **Main PCB (2-layer, ordered):** a GIGA-footprint shield, stack at most 1.9 cm (1.9 cm fits at R 10.5 only; 1.5 cm at 10.0 and 10.5; none at 9.5; 2.5 cm nowhere) **[check]**, carrying everything below. Its layout is not designed yet; the size is a **[placeholder]**.
  - **Power:** battery input, reverse-polarity protection, fuse, power switch (V2 has a circuit breaker and the Pololu D24V50F5 5 V regulator, BOM), dedicated 3.3 V regulator of at least 1 A (9 ToF at about 19 mA each is about 170 mA **[src]**).
  - **Drive:** two motor channels (12 V, 1.6 A stall: a dual H-bridge rated at least 14 V and 2 A per channel, for example two DRV8871-class parts, driven by direct PWM from D6-D9), two quadrature encoders on interrupt pins (V2 pins A: 3 and 5, B: 2 and 4 stay; D's 18 and 19 become free). This takes motor commands off the I2C bus that the camera victim service holds for seconds.
  - **Dropper port:** DRV8833-class driver, N20 motor, encoder A/B and a home switch (PWM on D10 and D11; the old stepper pins 8-11 are free).
  - **Sensing:** two TCA9548A (0x70, 0x71; ICs, or the SparkFun Qwiic Mux breakouts the BOM lists), 9 ToF ports and 2 floor ports as 4-pin JST-SH (3V3, GND, SDA, SCL; the floor ports also carry two analog inputs and an LED-enable line), BNO055 header (I2C 0x28, main bus).
  - **I/O:** 4 bumper switch inputs (2 populated), start button (V2 `logicswitch` pin 22), power switch, victim LED (pin 51), 2 status LEDs, microSD, two camera UART connectors plus the camera GPIO (V2 pins 13 and 12), optional LCD header (V2 pins 25, 27, 23, 53, 29, 31; keep or drop is open).
- **Sensor boards:** off-the-shelf VL53L0X breakouts (Adafruit PID 3317, 21 x 18 mm; BOM: 14 installed on the two V2 robots plus 1 spare, 9 needed for one V3) in printed pockets, 4-pin JST-SH cables to the main board (dotted lines in Figures 1 and 6).
- **Packing [check]** (R 10.5, roof 11.5, plate floor 8.7, camera recessed, rear chamfer as a constraint; Figures 1-3): GIGA + PCB stack across the right of the centre at z 6.15-8.05 under the plate (3 mm clearance to the nearest part), battery placeholder 7.0 x 3.5 x 2.5 cm in the front-left bay at z 5.1-7.6 (8.6 mm). The battery must stay in the front half: the packing search put it at the rear when left free, which gave a negative front load. Real battery, PCB and wiring sizes can change this; the rear bottom corner is unusable below z 4.3-5.2 because of the chamfer.
- **Fabrication:** ordered 2-layer main PCB; printed chassis (user has a 3D printer); cast silicone tires.

## 10. Firmware impact

| Keep | Adapt | Rewrite or remove |
|---|---|---|
| `MazeTile`, `bitSet`, BFS / navigation, `PID`, `timer`, gyro wrapper (already `OPERATION_MODE_IMUPLUS`, `setMapHeading`), camera UART and the victim handshake (`serviceCameraVictim`, `waitForVictimStop`), `dispenser` interface | `Distance.ino` (9 sensors, two muxes, index map, offset-aware thresholds, pitch gating), `squareToWall()` (angle-based PID with gate, 2 sensors per side as now), `read_color()` into a 5-class floor classifier using the two ports, `dispenser.cpp` (N20, sign flip, dwell), `Main.ino` constants and the encoder average `(A+B+D)/3`, which appears in `Main.ino` and about ten places in `movement.ino` and becomes `(A+B)/2`; the turn and shift routines in `movement.ino` (`absoluteturn`, `nudgeLeg`, `lateralShift`, `ensureTurnClearance`) re-tuned for the round body and 2WD pivots | `init_drive()` and the `motors` class (`drive(A,B,C,D)`, `fw`, `backward`, `turnleft/right`, encoders on A, B, D; four motors through the Adafruit_MotorShield library on the CAROBOT shield over I2C): two motors with direct PWM, two encoders, differential kinematics and BNO055 turns; new: bumper handler, floor ports, dropper routine |

Constants for V3 (R 10.5; the V2 formulas in `Main.ino` otherwise stay) **[calc]**: `TURN_SWEEP_MM` 110 (round body, replaces the length and width formula), `TOF_FRONT_FWD_MM` 95, `TOF_SIDE_OUT_MM` 72.8, hence `TARGET_SIDE_GAP_MM` 67.2 and `FRONT_GAP_AT_CENTER_MM` 45, `TURN_SIDE_GAP_MIN_MM` 40, `TURN_END_GAP_MIN_MM` 18 (below the sensor range, so that check can go), `FRONT_WALL_MAX_MM` about 120 (V2: 200), `OBSTACLE_STOP_MM` 60 to re-check (keep it below the 183 mm ramp-foot reading), `wheel_diameter` 80, `gear_ratio` 195, `wheel_cpr` 5 unchanged with the same encoders.

`MIN_DIST` and the north/east/south/west mapping in `detectWall()` need re-tuning because the sensor offsets from the tile centre change (front-centre sensor about 4.5 cm from the front wall at tile centre). The simulator's `sim/config/robot.cfg` has a rectangular body (`robot.length_mm`, `robot.width_mm`), four motors and seven sensors; it needs a round-body, two-motor, nine-sensor extension before it can test V3.

## 11. Verification before anything is locked

1. 2 cm stair test on the current robot with the front motors free-wheeling (gates wheel size, nub height, spring rate); measure friction with a tilt-plane test of a tyre sample on the stair material. This is the go/no-go for 2WD.
2. Silver bench test: three sensors, two angles, real silver and white tiles, normal and low light, at the real mounting height.
3. Chute test (do first): printed 15 mm bore chute at 35 degrees with a friction patch or switchback exit, 20 weighted cubes; measure where they come to rest (proposed pass: within about 3 cm of the first landing point).
4. ToF cross-talk test with the three front sensors, and the minimum-range behaviour of the side sensors.
5. Bench test of the ported `Distance.ino` with 9 sensors on two muxes on the GIGA.
6. Simulator check of turn clearance and layout after the round-body extension (section 10).
7. Dropper motor: N20 vs 15 mm stepper on a printed plate with 8 cubes; dwell time, backlash.
8. Weigh the real parts, set the centre of mass, measure the front load with a scale under the omni; check the acceleration limit.
9. Camera: focus at 4-5 cm with the real lens and window; coverage of the victim band at 20 degrees tilt.
10. Omni sourcing (60 mm exists in a width of about 2 cm?), spring rate (at least 1.6 N/cm), preload, nub height 1.5 / 2.0 / 2.5.

## 12. Open items

- Camera model confirmed as OpenMV H7 Plus? Focus range, window size.
- Robot mass, battery, drive motor and driver (torque budget).
- Stair test result: wheel size, nub height, omni size.
- Whether the casting molds are 80 mm.
- Slot and pocket geometry in CAD: blank-arc margin 2.5 mm, orientation in the robot frame.
- Battery and PCB location and sizes, now with the controller stack under the plate.
- Whether to keep the LCD.
- Whether the bumper should later be pulled back to the body radius to shrink the swept circle to 10.5.
- Decisions D1-D8 in section 0.5.
- Stair type: single 2 cm riser or several steps (rule 3.4.6 is ambiguous; V2's simulator uses two 1 cm steps).

## 13. Out of scope

Tracked drive, 4WD (fallback only if the stair test fails), mid-run reloading, changes to the mapping or exploration algorithm.

## 14. Figures and check scripts

Figures (generated from the model by [v3-checks/figures.py](v3-checks/figures.py); PNG and SVG in [v3-figures/](v3-figures/)):

1. [Top view](v3-figures/fig1_top.png): body, wheels, motors, omni, bumpers and switches, nine ToF with cones and cables, cameras and their view, plate and chutes, PCB, battery, floor sensors.
2. [Side section](v3-figures/fig2_side.png) at y = 0.
3. [Front section](v3-figures/fig3_front.png) at the axle, with the camera view.
4. [Dropper plate states](v3-figures/fig4_plate.png): parked, right drop, left drop, two-kit drop.
5. [Terrain poses](v3-figures/fig5_terrain.png): riser up and down, ramp, and the Dangerous Zone case that fails.
6. [Sensor map](v3-figures/fig6_sensors.png): nine ToF, two floor ports, mux ports (proposal), what each reads.

Scripts and results are in [v3-checks/](v3-checks/) with a README. They need `pip install shapely numpy matplotlib`.

## Sources

[Theseus repo](https://github.com/Arsur24/Theseus) (including the Sprint 3 write-up), [2026 rules](https://junior.robocup.org/wp-content/uploads/2026/02/RCJRescueMaze2026-final.pdf), Hanafi, Abueejela, Zakaria, "Wall Follower Autonomous Robot Development Applying Fuzzy Incremental Controller", Intelligent Control and Automation 4 (2013) 18-25, [Ctrl+Alt+Defeat](https://community.aisler.net/t/ctrl-alt-defeat-building-an-autonomous-rescue-maze-robot/5733), [Jak&Jonas](https://community.aisler.net/t/jak-jonas-at-the-robocup/4993), [wheel forum thread](https://junior.forum.robocup.org/t/wheel-recommendations-for-robocup-junior-rescue-maze/5384), [VL53L4CD](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html), [OpenMV Cam H7 Plus](https://openmv.io/products/openmv-cam-h7-plus) (fetched for this revision), [Arduino GIGA R1 WiFi pinout](https://docs.arduino.cc/resources/pinouts/ABX00063-full-pinout.pdf), Pololu items [3493](https://www.pololu.com/product/3493) (gearmotor), [3499](https://www.pololu.com/product/3499) (encoders) and [2851](https://www.pololu.com/product/2851) (regulator) as listed in `docs/Electronics_Sensor_BOM.md`, vendor listings for Pololu QTRX-MD, VCNL4040, TCS34725, 15BY25 steppers and SparkFun N20 motors (retailer pages, not datasheets), and the repo files `Main/*.ino`, `Main/motors.*`, `sim/config/robot.cfg`, `docs/Electronics_Sensor_BOM.md` and `docs/superpowers/specs/2026-10-04-v2-robot-and-2026-rules.md`.
