# Theseus V3 robot: hardware design spec

Date: 2026-10-06. Status: draft for review. Scope: new robot hardware (chassis, drivetrain, sensors, bumpers, dropper, PCB) and the firmware changes it forces. Previous hardware reference: <https://github.com/Arsur24/Theseus>.

**How to read the numbers.** Each number is tagged: **[rule]** from the 2026 Rescue Maze rules (final, 2026-03-29), **[src]** from a vendor listing or web source (not checked against a datasheet), **[calc]** my own geometry or statics (assumptions stated, unmeasured), **[code]** read from `Main/*.ino`, **[decision]** agreed in design discussion. Nothing here has been measured on hardware or run in the simulator.

## 1. Intent and success criteria

Build a smaller, more agile robot than the current 4WD skid-steer robot, reusing most of the firmware. The user's reasoning for 2WD: less scrub in turns, fewer botched turns. Caveat recorded: turn accuracy comes from the BNO055, so the gain is smoother and faster pivots, not accuracy; the cost is stairs.

Proposed success criteria (adjust):

1. Pivots in place inside a 25.2 cm corridor (28 cm with the rules' 10% tolerance) without touching walls when centred.
2. Climbs a 2 cm stair, a 25 degree ramp and 1-2 cm speed bumps reliably (target: at least 9 of 10 attempts on a test rig).
3. Tells white, black, blue, red and silver floor tiles apart, including the glass-like silver tile that read like white on the TCS34725.
4. Carries 8 kits and drops them within 15 cm of a wall victim without bounce or roll.
5. Most of `Main/` (map, BFS, PID, gyro, camera UART) runs unchanged.

## 2. Rule constraints that drive the design

| Constraint | Value | Rule |
|---|---|---|
| Tile / corridor width | 30 cm / 28 cm, all measurements +-10% | 3.1.2, 3.3.3, 3.8.6 |
| Robot height | at most 25 cm; bridges leave 25 cm | 4.2.9, 3.2.7 |
| Speed bumps / stairs / ramps | at most 1 cm (2 cm in Dangerous Zone) / at most 2 cm / at most 25 degrees; floor steps up to 3 mm | 3.4, 3.1.5, 3.2.1 |
| Obstacles | at least 15 cm tall, at least 20 cm from walls or touching a wall | 3.4 |
| Victims | about 7 cm above floor, letters 4 cm tall | 3.6 |
| Kits | at least 1 cm per side and 1 cm^3; at most 8 per robot; must land within 15 cm of the victim and stay | 3.7.3, 3.7.4, 3.7.2, 5.6.3 |
| No reload mid-run | captain loads at start; modifying the robot mid-run is prohibited; after a lack of progress only the button and one power switch | 3.7.5, 5.4.1, 5.5.3 |
| Required hardware | handle, single start button, dedicated victim LED (blink 500 ms on / 500 ms off for 5 s) | 4.2.7, 4.2.8, 4.2.11, 5.6.1 |
| Black tiles | holes; visiting one is a lack of progress (-15 reliability points) | 3.2.3, 5.5.1, 5.6.7 |

Scoring context for priorities: stairs 10 points plus 5 exit bonus, ramp 10 plus 5, speed-bump tile 5, checkpoint 10, kit 10 plus 10 reliability, two kits on one victim 30.

## 3. Decisions at a glance

| Area | Decision |
|---|---|
| Drive | 2WD differential, axle at the centre of the swept circle, 80 mm cast silicone wheels (baseline), half-recessed in wheel wells |
| Front support | single sprung 60 mm omni on the centreline, about 3 cm travel, adjustable preload, hard stops; rear skid. Rocker bar rejected (no pitch travel on stairs). Smaller omnis rejected: a free wheel cannot climb a step as high as its radius |
| Weight | centre of mass about 1 cm ahead of the axle, front load about 14% (was 2-3 cm, 36%): a heavy front starves the drive wheels on stairs |
| Body | round base (cylinder), body radius 9.5 cm (10.5 cm fallback if the stair test needs it), about 11-12 cm tall (base 9 cm plus lift-off lid and handle), no taper |
| Bumpers | two low front-arc plates that stick out; swept circle about 10 cm for now |
| Distance sensing | 9 x VL53L0X: 3 front, 4 side, 2 rear; two TCA9548A muxes |
| Floor sensing | two wired, swappable floor-sensor ports; tilted plus straight pair for silver |
| Dropper | low turntable inside the base (about 6-8 cm height) under the lid; about 8.9 cm plate, 14 mm pockets for 10 mm cubes, one blank arc where both slots park, side chutes exiting behind the wheel wells at about 3 cm height; N20 gearmotor with encoder |
| Controller | Teensy 4.1 |
| PCB | separate main board (2-layer, ordered) plus off-the-shelf sensor breakouts on short cables |
| Cameras | two side cameras; placement open until camera dimensions are known |

## 4. Geometry and drivetrain

**Swept circle.** What matters in a dead-end tile is the farthest body point from the axle midpoint, not body size. With the axle at the body centre the swept radius is the body radius.

- Body radius 9.5 cm; bumper tips 10.0 cm, so the swept circle is a 20 cm circle. In the worst-case 25.2 cm corridor that leaves about 2.6 cm per side **[calc]**. Fallback: body 10.5 cm if the stair test demands a bigger omni.
- A cylinder gives about 10% more area than an octagon at the same swept radius (283 vs 255 cm^2 at R = 9.5) **[calc]**. Sensor pockets are printed, so flat faces are not needed.

**Axle and weight balance.** The user's stated reason for the axle being slightly behind was weight balance. The axle goes at the circle centre. Centre of mass about 1 cm ahead of the axle, so the front omni carries about 14% (`1/7`) and the drive wheels about 86% **[calc]**. An earlier draft used 2-3 cm (about 36% front load) to avoid tipping back on a 25 degree ramp; that is only needed without a rear skid. With the rear skid the robot can rest on both drive wheels and the skid on a ramp (the front omni unloads and the skid drags slightly). A heavy front is also worse for stairs: pushing a free front wheel over a 2 cm step needs a horizontal force of `front_load x tan(alpha)` where `cos(alpha) = (r - h)/r`. With `W` the total weight, push needed at 14% / 36% front load: 50 mm omni 0.69 W / 1.76 W, 60 mm 0.40 W / 1.02 W, 70 mm 0.29 W / 0.76 W; drive traction available at an assumed friction coefficient of 1.5 is 1.29 W / 0.96 W **[calc]**. So at 36% a 60 mm omni fails, at 14% it passes. Not modelled: the spring force growing as the arm compresses, pitch, roller friction.

**Wheels.** 80 mm wheels at R = 9.5 fit inside the circle: outer face at most 8.6 cm laterally at x = +-4 cm, track about 15 cm **[calc]**. Rigid-wheel statics say a wheel of radius r climbs a step h only if the friction coefficient satisfies `h <= r(1 - cos(atan mu))`; for h = 2 cm and r = 4 cm that needs mu of about 1.7 **[calc]**. Soft cast silicone should do better but this is unmeasured, which is why the stair test comes first. The repo has silicone wheel casting molds (diameter unverified). Existing code says 68.7 mm and Sprint1.md says 80 mm; 80 mm is the new baseline.

**Front omni.** 60 mm baseline (user suggested smaller; rejected: a 40 mm omni has radius 2 cm equal to the step so it cannot climb a 2 cm riser, 50 mm is workable only at 14% front load). The omni must sit inside the swept circle, so axle-to-omni distance is about the swept radius minus the omni radius: about 7 cm for a 60 mm omni at R_swept = 10 **[calc]**. A 50 mm omni would add about 0.5 cm of wheelbase and ease the clash with the front-centre sensor; keep it as a parameter for the stair test. A rigid front wheel would tilt the robot nose-up by about 17 degrees on a 2 cm step (`asin(2/7)`) **[calc]**; the sprung arm exists to reduce that. The front-centre ToF sensor and the omni both want the front tip; the sensor goes above and ahead of the wheel (height about 6.5 cm).

**Motors.** Going from 4 to 2 motors doubles torque per motor. Motor, battery and driver are selected from a torque budget (open item). The old robot used Pololu 195:1 gearmotors **[code][src]**.

## 5. Sensing

Frame: x forward, y left, origin at axle midpoint, positions on the R = 9.5 cm circle. All ToF at about 6.5 cm height on one printed ring.

| Group | Count | Position (x, y cm) | Aim | Purpose |
|---|---|---|---|---|
| Front centre | 1 | (9.5, 0) | straight ahead | front wall, obstacles |
| Front toed-out | 2 | (8.6, +-4.0) | +-25 deg off forward (tune in sim) | front-wall angle, early side openings |
| Side | 4 | (+-6.7, +-6.7) | straight left / right | wall presence and angle, 13.4 cm baseline |
| Rear | 2 | (-8.5, +-4.2) | straight back | reversing, wall angle when backing to a wall, 8.3 cm baseline |

**[calc]** unless noted. At 13.4 cm baseline, 1 degree of heading error is 2.3 mm of difference and the current 10 mm threshold in `parallel()` is about 4.3 degrees. Side-sensor wall distance is 7.3 cm nominal and about 3.9 cm worst case (25.2 cm corridor, robot 2 cm off-centre); the VL53L0X minimum is about 3 cm **[src]**, so the margin is thin. Fallback: VL53L4CD (1 mm minimum **[src]**).

Wall-angle method follows Hanafi et al. 2013 (two same-side sensors, angle from the reading difference over the baseline; PID retained, their fuzzy controller not adopted because the paper is a single straight-corridor ultrasonic test with implausible error figures and untuned PD/PID baselines). Use the angle only when both sensors on that side see the same wall, and use it to re-zero IMU heading drift.

**Addressing.** Two TCA9548A muxes (0x70, 0x71), 16 ports: 9 ToF, 2 floor ports, spare. BNO055 (0x28) on the main bus.

**Ramp hazard.** A low horizontal front beam on a 25 degree ramp hits the ramp surface about 12-13 cm ahead and reads it as a wall **[calc]**. Gate front sensors with IMU pitch.

**Floor sensing and silver.** The floor module sits near the axle for silver (a tile is visited when more than half the robot is on it, 5.4.4); a second port sits at the front for black/blue/red so a hole is seen before the wheels reach it.
Silver is mirror-like; a mirror throws the emitter light away from a tilted sensor. Plan: one sensor straight down plus one tilted (about 20 degrees; at 25 mm height the specular spot lands about 21 mm off to the side **[calc]**). White: high/high. Black: low/low. Silver: high straight, low tilted. A tilted sensor alone would read silver as black, so both are needed.
Candidates for the bench test: TCS34725 raw clear channel (vendor listings give a 2-10 mm working distance **[src]**, which may explain the earlier failure if it was mounted higher), Pololu QTRX-MD (optimal 10 mm, max recommended 40-50 mm **[src]**), VCNL4040 (I2C, qualitative to about 20 cm **[src]**). Belly clearance must stay at least 2.5 cm for 2 cm stairs, so short-range optics need a low rounded skid mount or a longer-range part. RCJ documentation found only says the tile is highly reflective and suggests reading raw light intensity **[src]**; no team write-up of a robust method was found.
The main PCB carries two identical wired floor-sensor ports (3V3, GND, SDA, SCL, two analog inputs, one LED-enable line) so the module can be swapped later.

## 6. Bumpers

Two curved plates on the front arcs, about 12 to 70 degrees either side of centre (about 9.6 cm long each **[calc]**), hinged at the outer ends, return spring, 4 mm travel. The bumper is the outermost surface: body behind it is recessed by the travel. Decision: the bumpers stick out for now, giving a swept radius of about 10 cm.

- Height band 2.5-5 cm so a 2 cm stair riser or speed bump does not trigger them; the ToF ring sits above at 6.5 cm.
- Sensor: sub-miniature SPDT snap-action lever microswitch (Omron D2F class or generic; part number not verified), normally-open to ground on a pull-up input, 5-10 ms software debounce. A Hall sensor with a magnet is rejected: the BNO055 has a magnetometer and the rules warn of magnetic interference (3.8). Backup: slotted optical interrupter.
- PCB has 4 bumper inputs; populate 2 first (one per plate): left only, right only, both.
- Role: safety net for unseen obstacles, being wedged, and the sensor dead zone; triggers the existing `BACKPEDAL` state. Not a mapping input.
- False triggers: plate ends at 70 degrees reach about 8.9 cm sideways, so a robot more than about 3.7 cm off-centre in the worst-case corridor brushes the wall. Software accepts a bumper hit only when it coincides with forward motion and no wall is reported on that side.
- A very thin post exactly dead-centre could pass between the plates; the front-centre ToF covers it.

## 7. Dropper

Low inside the base under a lift-off lid (the captain loads kits by lifting the lid); handle, start button and victim LED on the lid. Earlier drafts put the plate on top of a taper. That was dropped because impact speed depends on the total fall height, not the exit height: Sprint 3 found kits stayed put from about 3 cm (about 0.77 m/s), while a 15 cm fall hits at about 1.7 m/s **[calc]**, and the extra height raised the centre of mass.

- **Kit:** 10 x 10 x 10 mm weighted roly-poly cube from Sprint 3 (rules minimum is 1 cm per side and 1 cm^3, so it cannot shrink). 8 kits, because there is no reload.
- **Plate:** 14 mm pockets (old design 19 mm; the extra slack was requested), 8 pockets at 30 degree pitch on a 3.3 cm radius circle covering 30 to 240 degrees, plate about 8.9 cm diameter (old at least 15 cm) **[calc]**.
- **Slot logic (replaces an earlier half-pitch-offset requirement, which was wrong: a 14 mm slot half a pitch from a pocket still sits partly under two pockets and leaks, and a full pocket sweeping over the other slot would drop its kit):** two fixed 14 mm slots in the base at 0 and 270 degrees of the plate frame. The blank arc (about 252 to 378 degrees) holds both slots when parked, with about 3 mm margin to the nearest pocket each side. Turn the plate CW to bring the next pocket from the pocket-1 end over slot A, CCW to bring the next pocket from the pocket-8 end over slot B. Always use the pocket nearest the target slot first, so a full pocket never passes a slot; any mix of left and right drops totalling 8 works. A home switch gives the parked reference. The slots are 90 degrees apart and sit symmetric about the robot centre line (A rear-right, B rear-left, blank arc facing the rear); the orientation is to be verified in CAD.
- **Chutes:** two enclosed channels, 15 mm bore, at least 5 mm clearance from motors, wheels and PCB. Exits go through the side wall at x about -5.5 cm, just behind the wheel wells (wheels span x +-4 cm; bumpers, front side ToF and cameras are in the front half), at about 3 cm height and about 9 cm lateral. The kit then lands about 4-5 cm from the wall and the along-wall slack for the 15 cm rule is about +-14 cm instead of about +-9.5 cm **[calc]**. A 30-35 degree slide gives roughly 0.9-1 m/s at the exit at an assumed friction coefficient of 0.3-0.4 **[calc]**, above the roughly 0.8 m/s that matches the Sprint 3 result, so the last section needs a friction patch or switchback; slopes of about 20 degrees or less may let a kit stick. The printed chute test with real kits decides the geometry.
- **Motor:** N20 gearmotor with magnetic encoder (10 x 12 x 41.5 mm, about 882 counts per output rev, 0.5 kg-cm stall at 6 V **[src]**), closed-loop with a home switch. The 28BYJ-48 is dropped as too large. Chosen over a 15 mm stepper (15 x 16.5 mm **[src]**) for torque margin (estimated plate friction a few N-mm **[calc]**), reuse of the drive-motor encoder/PID path, and footprint. Always approach each index from the same direction to cancel backlash. Fallback: 15 mm stepper. The PCB dropper port (DRV8833-class driver, from memory 2.7-10.8 V, about 1.2 A per channel, plus two encoder or home inputs) supports either.
- **Height budget:** base 9 cm plus lid and handle about 2 cm, so about 11-12 cm against the 25 cm limit (the earlier top-mounted draft was about 21 cm) **[calc]**. The plate sits at about 6-8 cm height, above the motors (top at 5.25 cm) and inside the ToF ring. Battery and PCB placement are open.

## 8. Cameras

Two side cameras, probably OpenMV H7 (repo has an Edge Impulse script `ei_object_detection_PhiOmegaPsi.py`, not read); UART to `Serial2` / `Serial3`, 3.3 V logic. Open: dimensions and focus range. The wheel wells occupy the side middle, where the cameras would naturally go (wheel top about 8 cm, victims at 7 cm). Option A: above the wheel wells at 8.5-9 cm, looking straight sideways (sticks out at the roof corner now that the taper is gone). Option B: ahead of the wheel wells at about +-60 degrees, 8-9 cm height, angled 25-30 degrees forward (sees victims earlier, longer focus path). With the chute exits behind the wheel wells at x about -5.5 cm, option A at the axle line puts the kit about 5.5 cm behind where the victim was seen and option B about 10 cm; both are inside the 15 cm radius but A leaves more slack. Recommendation now leans A; decide once dimensions are known. Lens should be about 5-6 cm from the wall or behind a window.

## 9. Electronics

- **Controller:** Teensy 4.1 in sockets (3 I2C, SD slot for run logs, `Serial2/3` unchanged). 3.3 V only: level-shift any 5 V camera UART.
- **Power:** dedicated 3.3 V regulator of at least 1 A (9 ToF at about 19 mA each is about 170 mA **[src]**), reverse-polarity protection, fuse, power switch.
- **Motors:** two channels, two quadrature encoders on interrupt pins; driver chosen from stall current.
- **I/O:** bumper switch inputs (4), start button, power switch, victim LED, status LEDs, SD logging, two floor ports, dropper port, 9 ToF cables.
- **Sensor boards:** off-the-shelf VL53L0X breakouts in printed pockets, 4-pin JST-SH cables to the main board.
- **Fabrication:** ordered 2-layer main PCB; printed chassis (user has a 3D printer); cast silicone tires.

## 10. Firmware impact

| Keep | Adapt | Rewrite or remove |
|---|---|---|
| `MazeTile`, `bitSet`, BFS / navigation, `PID`, `timer`, gyro wrapper, camera UART | `Distance.ino` (9 sensors, two muxes, index map, offset-aware thresholds, pitch gating), `parallel()` (angle-based PID with gate), colour code into a 5-class floor classifier | `movement.ino` and drive init (2 motors, differential kinematics, turns from BNO055), Adafruit Motor Shield dependency; new: bumper handler, dropper routine, floor ports |

`MIN_DIST` and the north/east/south/west mapping in `detectWall()` need re-tuning because the sensor offsets from the tile centre change (front sensors sit about 5.5 cm from the front wall at tile centre).

## 11. Verification before anything is locked

1. 2 cm stair test on the current robot with the front motors free-wheeling (gates wheel size, body radius, spring rate).
2. Silver bench test: three sensors, two angles, real silver and white tiles, normal and low light.
3. Chute test (do first): printed 15 mm bore chute at 30-35 degrees with a friction patch or switchback exit, 20 weighted cubes; measure where they come to rest (proposed pass: within about 3 cm of the first landing point).
4. ToF cross-talk test with the three front sensors.
5. Bench port of `Distance.ino` and the gyro code to a Teensy.
6. Simulator check of turn clearance and layout (sim on `origin/main`; not looked at this session).
7. Dropper motor: N20 vs 15 mm stepper on a printed plate with 8 cubes.

## 12. Open items

- Camera model, dimensions, focus range.
- Robot mass, battery, drive motor and driver (torque budget).
- Stair test result: body radius 9.5 vs 10.5 and omni size.
- Whether the casting molds are 80 mm.
- Slot and pocket geometry in CAD: blank-arc margin about 3 mm, orientation in the robot frame.
- Battery and PCB location now that the dropper is low and the centre of mass is about 1 cm ahead of the axle.
- Camera option A or B.
- Whether the bumper should later be pulled back to the body radius to shrink the swept circle to 9.5.

## 13. Out of scope

Tracked drive, 4WD, mid-run reloading, changes to the mapping or exploration algorithm.

## Sources

[Theseus repo](https://github.com/Arsur24/Theseus), [2026 rules](https://junior.robocup.org/wp-content/uploads/2026/02/RCJRescueMaze2026-final.pdf), Hanafi, Abueejela, Zakaria, "Wall Follower Autonomous Robot Development Applying Fuzzy Incremental Controller", Intelligent Control and Automation 4 (2013) 18-25, [Ctrl+Alt+Defeat](https://community.aisler.net/t/ctrl-alt-defeat-building-an-autonomous-rescue-maze-robot/5733), [Jak&Jonas](https://community.aisler.net/t/jak-jonas-at-the-robocup/4993), [wheel forum thread](https://junior.forum.robocup.org/t/wheel-recommendations-for-robocup-junior-rescue-maze/5384), [VL53L4CD](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html), vendor listings for Pololu QTRX-MD, VCNL4040, TCS34725, 15BY25 steppers and SparkFun N20 motors (retailer pages, not datasheets).
