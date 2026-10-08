# Theseus V3 rev 4: report before the Fusion build (7 Oct 2026)

Status: **nothing has been built in Fusion, saved, committed or printed.** This report answers your last round of questions, says what a dry run of the rebuild plan found, describes the design as it will be built and the numbers the Fusion build should reproduce, and lists what I need from you before I start. The design is [the rev 4 spec](2026-10-07-theseus-v3-mechanical-design.md) (section 0, rows 15 to 18, and sections 3, 4, 5.3, 6.1, 7); the build is [the plan](../plans/2026-10-07-theseus-v3-rev4-fusion-model.md); the emulator is [v3-fusion/dryrun](v3-fusion/dryrun/README.md). Tags as in the spec: **[calc]**, **[check]**, **[src]**, **[placeholder]**.

## 1. The short version

| You asked | Answer |
|---|---|
| Is 2WD a good idea for a navigation robot? | Yes. Two driven wheels and a caster is the standard layout of small indoor navigation robots. Its one real weakness is climbing, and for V3 that is the 2 cm step with 80 mm wheels (a rigid wheel needs a friction coefficient of 1.73 for it). **The stair test with the real tyres is the go or no-go**, not anything in the CAD. |
| Weight balance, wheels further back? | Rechecked with the final layout: centre of mass 0.89 cm ahead of the axle, 6.8 cm high, front load 12.8 %, front lift-off at 1.29 m/s^2. **The axle stays where it is**: moving it back loads the omni and unloads the driven wheels, widens the swept circle, and nothing needs it. |
| Reach the electronics without taking much apart | Floor disc, plate, kits, N20 and hoppers **lift out as one unit** (lid off, two channels pulled, two snap tabs, one connector: about a minute). Then 86 % of the GIGA, 86 % of the shield and 91 % of the battery are open from above. Reflashing needs no lid removal (USB-C in the wall). The battery takes 2 to 3 minutes. |
| Buttons at the front | Control deck on the front bridge, USB-C socket in the front-right wall. Wires to the PCB are 5 to 8 cm instead of 21 to 25 cm. **The victim LED went to the top of the handle bar** (rules 4.2: it must be clearly visible to the referee; on the deck it would have been hidden). |
| Colour sensors further forward | Silver module and front port both at x 7.5, one each side of the omni bay. The silver tile is read 7.5 cm of travel earlier. Terrain clearance stays at 0.5 cm or more on every case. The firmware threshold has to change. |
| Dry run | 13 problems found and fixed before the first Fusion call. Everything now passes on the emulator. |

## 2. Your questions

### 2.1 Is 2WD sound? (general navigation robots, not RoboCup)

| Evidence | What it says |
|---|---|
| TurtleBot3 Burger (two driven wheels plus casters) | climbs 10 mm at most [retailer spec lists: [Elektor](https://elektor.com/products/robotis-turtlebot3-burger-incl-raspberry-pi-4), [Generation Robots](https://www.generationrobots.com/en/402707-turtlebot3-burger-mobile-robot-788.html); ROBOTIS's own page not checked] |
| Roomba (two driven wheels, front caster) | about 16 mm (15 to 20 mm across models) [secondary sources: [Narwal](https://us.narwal.com/blogs/robot-vacuum/robot-vacuum-go-over-thresholds), [SmartRobotReviews](https://smartrobotreviews.com/g/rv/irobot-roomba-i7-7150/faq/what-is-the-maximum-height-the-irobot-roomba-i7-can-climb-1)]; no iRobot spec page found |
| Pioneer 3-DX (2WD + caster) against Pioneer 3-AT (4 wheels, skid-steer) | 2.5 cm against 10 cm step [datasheets: [3-DX](https://hades.mech.northwestern.edu/images/2/2b/Pioneer_p3dx.pdf), [3-AT](https://www.generationrobots.com/media/Pioneer3AT-P3AT-RevA-datasheet.pdf); third-party copies]. 2WD loses on obstacles, wins on odometry |
| Skid-steer (4 or 6 wheels, no steering) | turns only by wheel slip, so the ideal differential-drive model does not hold and odometry needs terrain-dependent corrections [Mandow et al., IROS 2007, [doi 10.1109/IROS.2007.4399139](https://doi.org/10.1109/IROS.2007.4399139); I read secondary summaries, not the paper]. For a mapping robot that counts tiles from encoders this is a real cost of 4WD |
| V3's own step | a 2 cm step on an 80 mm wheel is half the radius: a rigid wheel balanced on the step edge needs a friction coefficient of tan 60 degrees = 1.73 **[calc]**. Silicone tyres are compliant, so the real figure is lower and has to be measured. The omni (60 mm, spring) takes the same step at 3 times its height (a caster rule of thumb from my earlier research, not re-checked) |

What carries over from the comparison: V3 is already well inside the 2WD norm for a robot this size. Most of its gain over V2 comes from the small round body (swept circle 22 cm against 26.5 cm), not from the drive. **Plan B if the stair test fails:** four wheels with the middle pair slightly lower; it costs the odometry quality above, so only if grip cannot be fixed with tyre material or tread.

### 2.2 Weight balance and the axle [calc] `balance`, [check] `axle_shift`

Final masses are **[placeholder]** (printed parts from the model volume at 45 % fill, bought parts from the rev 3 budget, wiring and fasteners as lumps): 1195 g, centre of mass **0.89 cm ahead of the axle and 6.82 cm high**. Moving the controls from the rear to the front moved it 0.26 cm forward, which also lifted the front-lift limit from 0.91 to 1.29 m/s^2: it is no longer a warning.

| Axle behind the body centre | Front load | Lift-off limit | Swept radius | Terrain cases |
|---|---|---|---|---|
| **0 (chosen)** | 12.8 % | 1.29 m/s^2 | 11.00 | all nine required pass; the Dangerous Zone 2 cm bump on a ramp fails, as in rev 3 |
| 1 cm | 20.5 % | 2.36 | 11.98 | same |
| 2 cm | 26.5 % | 3.43 | 12.96 | all pass (chute lip 0.50 cm) |
| 3 cm | 31.3 % | 4.50 | 13.95 | ramps, 30 degree stairs and bump-on-ramp fail |

Why not move it: the lift-off limit already clears the 1.0 m/s^2 the firmware ramp assumes (a 0.25 s soft start covers the rest); every centimetre back takes 7.7 points of load off the driven wheels (9 % less weight for the stair climb) and adds a centimetre of swept radius (11.98 at 1 cm, against a worst-case corridor half-width of 12.6); the only thing 2 cm would gain is the one corner case, at a radius that does not fit the worst-case corridor. **Tuning later:** weigh the real robot; size the battery tray with at least 1 cm of fore-and-aft slack, because moving the 110 g battery 1 cm moves the centre of mass 0.09 cm.

### 2.3 Access to the electronics [calc] `access`, `removal` (reports of the plan, run on the emulator)

| Part (share of its upper surface open from above) | Lid off | Lid and dropper unit off |
|---|---|---|
| GIGA | 55 % | **86 %** |
| Shield | 53 % | **86 %** |
| Battery | 77 % | **91 %** |
| Left drive motor | 0 % | 54 % |
| Right motor, silver module | 0 %, 49 % | 0 %, 49 % (frame off for these) |

- The unit lifts straight up 12 cm past everything (seven removal paths all free: lid, each wheel, kit swap, the unit, the battery, and a full teardown down to the omni).
- **Battery:** Camera L's board hangs 8.6 mm over its outboard end, the ring over its front corner and the power switch over its front, so it cannot go straight up: it slides 4.5 cm back and then 1.5 cm or more inboard, and lifts out (for the placeholder 7.0 x 3.5 x 2.5 cm pack). A real battery needs this re-run.
- **Reset and boot:** the reset button is open from above beside the deck (a pen reaches it); **the boot button is under the ring** and stays a frame-off job. Double-tapping reset brings up the Arduino bootloader without it; boot plus reset is only the recovery route when the bootloader is gone [src: [OpenMV page](https://docs.openmv.io/openmvcam/quickref/arduino-giga-r1-wifi.html), [Arduino support](https://support.arduino.cc/hc/en-us/articles/7991505987612-If-an-LED-on-GIGA-R1-WiFi-is-flashing-red)].
- Still frame-off: the right motor (under the GIGA stack), the silver module and floor port, the omni. The frame is six hex screws and about 15 cables on 200 mm leads, so those take 10 to 25 minutes, as before.

### 2.4 Controls at the front [calc] `wire_runs`

- **Control deck:** the bridge web widens to the +y side (x 5.2 to 9.15, to y 3.6). Power switch and start button are on it; two status LEDs stand on the bridge rib; the lid has a front notch joined to the handle slot. The deck stops at y -1.25 on purpose: my first, symmetric deck would have covered the GIGA's reset button (found by checking the deck against the datasheet picture; a unit test and a point check in the access report now pin it).
- **Victim LED on the handle bar** (z 16.0, the highest point): rules 4.2 ask for "one specific LED or display ... clearly visible to the referee". On the deck it sits 4 cm under the bar and 1.3 cm below the lid top, hidden from straight above and from the sides below about 40 degrees of elevation. On the bar it is visible from everywhere and the lid stays wire-free; the cost is two wires down the post (12 cm run). Alternatives are in the spec (D19).
- **USB-C service socket** in the front-right wall at -24 degrees, flush, 2.0 cm from J12 (6 mm above the bumper band, 8.3 mm from the boss rib).
- Wire runs to J12: 5 to 8 cm for the deck parts, 12 cm for the LED on the bar, against 21 to 25 cm for the rear panel.

### 2.5 Colour sensors further forward [check] `fsens_front`

Silver module (7.5, -2.7) and front port (7.5, +3.2), faces at z 2.8. Smallest clearance between the window and the terrain over each crossing: 1.70 / 2.37 cm for the 2 cm riser up / down, 1.30 for the ramps, 1.70 for the 2 cm bump, 0.82 and 0.50 for the 25 and 30 degree stairs, 0.67 for the Dangerous Zone bump on a ramp (criterion 0.2; they were 2.4 to 2.5 at the axle). At x 8.0 the bump on a ramp falls to 0.17 and at 8.5 it hits, so 7.5 is about the limit. **Firmware:** `color.ino` accepts silver only after half a tile of travel (`overNextTile`), which matched a sensor at the axle; with the module 7.5 cm ahead the threshold becomes TILE_MM/2 - 75 mm, and the reading has to be latched until the robot has gone half a tile so the "more than half the robot on the tile" rule (5.4.4) still holds.

## 3. What the dry run found before any Fusion call

The plan's own code (parameters with 38 unit tests, 19 build stages with about 170 probe and overlap rows, eleven whole-model reports) was run on a geometry emulator, with the output compared against the design. It cannot see Fusion API behaviour, so Fusion stays the authority; but these were all cheaper to find here.

| # | Finding | Fix |
|---|---|---|
| 1 | The wall strip above each wheel arch sat in the middle of the camera's view (9 of 25 rays blocked) | notched 3 cm at the camera window |
| 2 | A frame screw at 101 degrees hung a boss on the tyre's top rim | screws at +-118 degrees, every boss with a rib |
| 3 | Both drive motors sit under another part (battery over the left, GIGA stack over the right) | service order in the spec |
| 4 | The USB-C socket stuck 2 mm out of the wall | flush |
| 5 | Two cuts removed nothing (Fusion would raise); lid bumps and cradle prongs flex | cuts dropped; flex bodies skipped by the removal check |
| 6 | Removal steps were coarse enough to jump a thin wall | steps at most 2.5 mm |
| 7 | The ToF pockets were closed by a 1 mm skin | open at the top |
| 8 | **The handle bar (x 0 to 8) stood over the plate: the dropper unit could not lift out** | bar from x 3.3 (cost: carried nose-up 17 to 33 degrees; D15) |
| 9 | **A symmetric control deck covered the GIGA's reset button** (found by checking the access numbers against the datasheet picture of the board) | deck on the +y side only; unit test and access-report point |
| 10 | Camera L, the ring corner and the power switch trap the battery | removal path 4.5 cm back and inboard |
| 11 | The dropper floor's seat ledges overlapped its rim; the rear tab lay inside the disc | seats are rebates in the existing webs |
| 12 | A tub probe and a unit test asked for floor where the new sensor hole is; one float comparison failed on 1e-16 | rewritten |
| 13 | **The victim LED on the deck would not be "clearly visible"** (found by re-reading the rules, not by the emulator) | on top of the handle bar |

## 4. The design as it will be built (the model)

Round body R 10.5 (wall inner face 10.3), swept radius 11.0 from the bumper tips only, highest point 16.0 cm (limit 25), 51 components (the Wi-Fi antenna was added after this report). Tub with wheel arches (wall strip notched at the cameras), snap cradles for the two drive cartridges, battery tray, GIGA on four posts (connector edge forward), omni **inside** the body (centre x 7.0, one arm on +y), USB-C socket in the front-right wall. Upper frame: ring with nine ToF pockets and two camera windows, front bridge with a rib and the control deck, rear spoke, three half-lap seats, handle post, six M3 screws. **Dropper unit:** floor disc (own part), plate, eight kits, N20 cartridge, two hoppers; two 13 mm square channels plug in from the wall. Snap-on lid with four hooks, a handle slot and a front notch. T-handle bar (5.4 cm) with the victim LED. Floor sensors at x 7.5. A hidden keep-out keeps the 28BYJ-48 option open. Drive axle at the body centre.

## 5. What the Fusion build will do and what it should give

Ten tasks in the plan: parameters and unit tests (no Fusion), plumbing and runner, then tub, drive train, frame, lid and controls, dropper unit, electronics, then eleven whole-model reports and the documentation. Every stage builds its components, runs probes (is there material at this point?) and a local overlap check; a failing script rolls back, so a bad stage is just re-run. About 2 to 4 minutes of tool time per full build and a few more per report **[estimate, not yet run in Fusion]**. The emulator predicts (tolerances in the plan: 0.3 mm, 2 % on volumes, 0.05 cm on the centre of mass):

| Report | Predicted |
|---|---|
| interference | 51 components (50 when this report was written; the Wi-Fi antenna was added), 0 interfering pairs |
| envelope | bumper plates 11.000, everything else at most 10.500, highest point z 16.00 |
| belly line | only the known items below z 3.5 (wheels, omni, nub, motors, floor sensors, chute lip, pivot pin at 3.40) |
| removal | seven paths free |
| access | GIGA 55 to 86 %, shield 53 to 86 %, battery 77 to 91 %; reset open, boot covered (known) |
| ToF cones, cameras | 0 of 17 and 0 of 25 rays blocked |
| spec clearances | silver module to omni wheel 5.0 mm and to GIGA post 2.8 mm; N20 to left motor 11.0; chute to wheel 8.3; omni wheel to GIGA post 2.7; GIGA to tub 1.7; battery to left motor 1.1; camera to wheel 3.6; 28BYJ-48 bay 5.8 / 5.2 / 8.3 |
| mass | about 1195 g, centre of mass +0.89 / +0.15 / 6.82 cm, front load 12.8 %, lift-off 1.29 m/s^2 |

Where Fusion and the emulator disagree by more than 0.3 mm or 1 %, I will say which side was wrong.

## 6. Risks and what is not verified

- **Stair test with the real tyres (rev 3 test 1)** decides 2WD; nothing in CAD can.
- **Masses and the battery are placeholders**: the balance, the front-lift limit and the battery path change with the real battery and printed weights.
- **Not designed yet:** the snap tabs or thumbscrews that hold the dropper unit; the victim LED's wire connector at the bar; a charge port beside the USB-C socket (so the battery need not come out to charge).
- **Handle:** the short bar means a nose-up carry (17 degrees from the rear end of the bar, 33 from the middle). The bridge under the post stays well inside PETG limits (2.0 MPa and 0.19 mm of droop at 15 N, 5.9 MPa and 0.57 mm at 45 N) **[calc]** `handle_calc`.
- **Unverified in Fusion:** every API detail (coincident-face booleans, `pointContainment` on faces, timeouts). The dry run cannot see those.
- **Unchanged open items:** printer bed (assumed 220 mm), encoder board outline, face-hole spacings, OpenMV hole spacing, VL53L0X connector edges and holes, battery model, power switch size, torsion spring source, the GIGA's reset and boot positions (read from the datasheet picture, +-1 mm).

## 7. What I need from you

1. **Go for the Fusion build**, and which execution approach. I recommend **native** (I run all ten tasks myself in this session, then one fresh reviewer checks the whole result) rather than subagent-driven: the tasks share one set of interfaces and one Fusion document that only one driver can use at a time, and a wrong build costs minutes of rebuilding because a failed script rolls back, so a fresh reviewer per task adds cost without catching more than the end review.
2. **Which Fusion document is open?** The plan's first Fusion step replaces every component of the open design (`reset=True`). I will check the active document and tell you before the first write, and I will not save or close it.
3. **D15, the handle bar:** the short bar (default) or the 8 cm bar that you unscrew before lifting the unit out.
4. **D19, the victim LED:** on the bar (default), a light pipe through the lid, or on the deck (hidden from straight above).
5. Anything in section 1 you want changed before I build.

Files changed in this round (all untracked, nothing committed): the [spec](2026-10-07-theseus-v3-mechanical-design.md), the [plan](../plans/2026-10-07-theseus-v3-rev4-fusion-model.md), [figures 7 and 9](v3-figures/fig9_mounting.png), three new checks in [v3-checks](v3-checks/README.md) (`axle_shift`, `fsens_front`, `handle_calc`) and `ring_gaps`, and in [v3-fusion/dryrun](v3-fusion/dryrun/README.md) `balance.py`, `wire_runs.py` and `results/`.

## Sources

[Elektor, TurtleBot3 Burger](https://elektor.com/products/robotis-turtlebot3-burger-incl-raspberry-pi-4); [Generation Robots, TurtleBot3 Burger](https://www.generationrobots.com/en/402707-turtlebot3-burger-mobile-robot-788.html); [Pioneer 3-DX datasheet](https://hades.mech.northwestern.edu/images/2/2b/Pioneer_p3dx.pdf); [Pioneer 3-AT datasheet](https://www.generationrobots.com/media/Pioneer3AT-P3AT-RevA-datasheet.pdf); [Narwal on thresholds](https://us.narwal.com/blogs/robot-vacuum/robot-vacuum-go-over-thresholds); [SmartRobotReviews, Roomba i7](https://smartrobotreviews.com/g/rv/irobot-roomba-i7-7150/faq/what-is-the-maximum-height-the-irobot-roomba-i7-can-climb-1); [Mandow et al., Experimental kinematics for wheeled skid-steer mobile robots (IROS 2007)](https://doi.org/10.1109/IROS.2007.4399139); [OpenMV, GIGA R1 WiFi](https://docs.openmv.io/openmvcam/quickref/arduino-giga-r1-wifi.html); [Arduino support, flashing red LED](https://support.arduino.cc/hc/en-us/articles/7991505987612-If-an-LED-on-GIGA-R1-WiFi-is-flashing-red); the RoboCup Junior Rescue Maze 2026 rules (`rules2026.txt`, rules 4.2 and the victim-identification rule); the scripts and reports named above.
