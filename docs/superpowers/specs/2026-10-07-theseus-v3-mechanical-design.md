# Theseus V3: mechanical structure, mounting and usability (revision 4)

Status: **design confirmed by you (7 Oct: D10 N20, D12 controls on the front deck of the frame, the lift-out dropper unit, the axle at the body centre; 8 Oct: D15 short handle bar and D19 victim LED on the bar, by taking my recommendation); the layout model was rebuilt to match it** ([v3-fusion/](v3-fusion/README.md), `Theseus_V3_rev4.f3d`) and checked in Fusion (probes, interference, envelope, belly line, removal paths, access from above, ToF cones, camera views, omni sweep, spec clearances, mass; raw output in `v3-fusion/results/`). Nothing here is measured or printed. Numbers carry a tag: **[calc]** my arithmetic or an emulator run (`balance` and `wire_runs` are scripts in [v3-fusion/dryrun](v3-fusion/dryrun/README.md), `access` and `removal` reports of the rebuild plan, output in `dryrun/results/dryrun_full.txt`), **[check]** a script in [v3-checks/](v3-checks/README.md) (named; raw output in `v3-checks/results/`), **[src]** a vendor page or datasheet, **[proposal]** a design choice made here, **[measure]** to be measured on the real part, **[placeholder]** a guess.

This document extends [the rev 3 spec](2026-10-06-theseus-v3-robot-design.md), which stays the source for everything not changed here. It answers the review of the first Fusion model: chute and plate, dropper support, lid and electronics access, mounting holes and attachments, the omni position, clip-on motors, and the feasibility and usability of all of it. A second round (rows 16 to 18 of section 0, with the controls in row 7) rechecked the weight balance and the axle position, made the electronics reachable without taking the robot apart, moved the controls to the front and moved the floor sensors forward; a third item (row 19) uses the GIGA's built-in Wi-Fi for cable-free testing. The rebuild plan was dry-run on a geometry emulator before any Fusion call ([v3-fusion/dryrun](v3-fusion/dryrun/README.md)); what it found is in row 15. Where this file disagrees with rev 3 sections 4 (omni), 7 (chutes, dropper motor) or 9 (electronics mounting), this file wins.

Figures: [7 side section](v3-figures/fig7_structure.png), [8 motor cradle](v3-figures/fig8_motor_clip.png), [9 plan of every attachment point](v3-figures/fig9_mounting.png).

## 0. What changes

| # | Item | Rev 3 and the first Fusion model | Rev 4 | Evidence |
|---|---|---|---|---|
| 1 | Front omni | centre x 7.95, front edge 10.95 (outside the body), two arm plates, slot in the front wall | centre **7.0**, front edge **10.0** (wall inner face 10.3), **one arm on the +y side**, no slot | all nine required terrain cases still pass; the GIGA's front-inner mounting hole sits in a two-plate arm swing **[check]** `omni_inside`, `mount_check` |
| 2 | Dropper floor | attached to nothing | its own print, the base of a **lift-out dropper unit** (floor disc, plate, kits, N20 cartridge, both hoppers); it rests on three half-lap seats in the web of the **upper frame**'s front bridge and rear spoke | model review; second round, section 3.3 |
| 3 | Chute | 15 mm round bore, mouth 0.75 cm under the floor | **16 mm slot and hopper void sealed to the floor underside**, **18 mm square channel**, open as a trough under the slot, exit axis z 4.15 (redesigned 8 Oct after a kit simulation, section 6.2) | a 10.3 mm cube passes the 16 mm slot lying at any turn (1.43 mm over its face diagonal) and tipped any way but standing on a corner (16.25 mm), and the 18 mm bore with 0.16 mm to spare over its space diagonal; the dropper must park the plate within 1 mm; lowest point z 2.78 **[check]** `chute_exit`, `mount_check`, `cube_slot`, `chute_dynamics` |
| 4 | Dropper motor and the "pillar" | N20 hanging under the floor | N20 stays (rev 3) as a **cartridge dropped in from above**; it hangs from the floor and carries nothing. The **space for a 28BYJ-48 is kept**: it fits when turned so its body sits 8 mm from the plate axis, toward the front-left | **[check]** `stepper_bay`. Rev 3 section 7 called it "too large"; a body centred on the plate axis is, but its shaft is 8 mm off centre |
| 5 | Lid | plate on the rim, blocked by the handle arch | **snap-on lid**, four hook tongues, lifts straight up | requested |
| 6 | Handle | arch on the chassis | **T-handle**: post on the front bridge, a 5.4 cm bar along x (x 3.3 to 8.7) through a slot in the lid | the arch blocked the lid; the bar stops in front of the dropper unit so that the unit can lift out (D15) |
| 7 | Start button, victim LED, power switch | start button and LED on the lid (rev 3 section 7) | on a **control deck** on the front bridge: switch and start button on its +y side, two status LEDs on the bridge rib, all through a notch and the handle slot in the lid; the **victim LED on top of the handle bar's front end**; USB-C service socket in the **front-right wall** | a lid with wires cannot simply lift off; at the front the wires to the PCB are 5 to 8 cm instead of 21 to 25 cm (12 cm for the LED on the bar) **[calc]** `wire_runs`; the rules want the victim LED clearly visible to the referee, section 7.3. Alternative in D12 and D19 |
| 8 | Drive motors | cylinder in a floor slot | **cartridge** (face plate fixed once with 2 x M2.5) in a **snap cradle** made of two thin prongs and a key | friction alone gives at most 0.16 kg.cm; a 2 cm riser needs 1.63 **[check]** `mount_check` |
| 9 | Wheels | solid cylinders | hub on the D-shaft with a set screw; **wheel arches** in the lower wall | usability, section 5.2 |
| 10 | GIGA | positions only | connector edge to the front, header edge inboard, **4 printed posts** on the datasheet holes | **[check]** `mount_check` |
| 11 | ToF boards and cables | positions only | drop-in pockets from above, **200 mm cables** | **[check]** `mount_check` |
| 12 | Bumpers | rev 3 section 6 | unchanged | section 9 |
| 13 | From the 3D layout model | ToF positions, camera window, chute nose, omni stop | adopted: side and rear ToF moved inward, camera window 2.8 x 2.4 cm, omni stop 2.5 cm | [v3-fusion README](v3-fusion/README.md) findings 1 to 4 |
| 14 | Print rule | none | every snap part flexes **in the layer plane** | section 10 |
| 15 | From a dry run of the rebuild plan (7 Oct) | wall strip above each wheel arch at z 8.3 to 8.7; rear frame screws at +-101 degrees; drive motors "lift out"; ToF pockets closed by a 1 mm skin; USB-C socket 2 mm proud of the wall; handle bar x 0 to 8; control deck 4.8 cm wide | the strip is **notched 3 cm at the camera window**; the rear screws move to **+-118 degrees**, each boss with a rib to the wall; the battery (left) or the GIGA stack (right) comes out before a drive motor; the ToF pockets are open at the top; the socket is flush; the bar starts at x 3.3; the deck is on the +y side only | the strip sat in the middle of the camera's field of view, and a boss at 101 hung on the tyre's top rim; a straight lift of either cartridge hits the part above it; a bar over the plate stops the dropper unit lifting out; a deck over the GIGA's connector edge would cover its reset and boot buttons **[calc]** (3D checks on the emulator), sections 3.3, 5.2, 7 |
| 16 | Weight balance and axle position (second round) | axle at the body centre; no balance recheck after the extra printed mass | **axle stays at the body centre.** With the controls at the front the centre of mass is **0.89 cm ahead of the axle, 6.8 cm high**, front load 12.8 %, front lift-off at **1.29 m/s^2** | a 1 cm shift back adds 7.7 points of front load (9 % less weight on the driven wheels for the stair climb) and 1 cm of swept radius (11.98 against a worst-case corridor half-width of 12.6) for a lift limit that is already above the 1.0 m/s^2 ramp **[calc, placeholder masses]** `balance`, **[check]** `axle_shift`. Section 4, axle position |
| 17 | Access to the electronics (second round) | lid off shows the front 4.5 cm of the stack; everything else needs the frame off | plate, kits, N20, floor and hoppers **lift out as one unit**; with the lid and the unit off 86 % of the GIGA, 86 % of the shield and 91 % of the battery are open from above. USB-C reflash with the lid on | **[calc]** `access` report on the dry run; sections 3.3, 7.4 |
| 18 | Floor sensors (second round) | FP (6.5, 3.0), SM (0, 0) | **FP (7.5, 3.2), SM (7.5, -2.7)**, one each side of the omni bay: the silver tile is seen 7.5 cm of travel (about 250 ms at 0.3 m/s) earlier | terrain clearance of the windows at least 1.3 cm on the required cases, 0.5 cm on the 30 degree stairs **[check]** `fsens_front`; firmware threshold changes, section 5.3 |
| 19 | Wi-Fi and Bluetooth for cable-free testing (third round) | not considered | the **GIGA's built-in module** is used in a **test mode** only (log streaming and test commands); its flat antenna, which ships with the board, is stuck on the inside of the tub's front wall at -6 degrees; flashing stays on the USB-C socket; the radio is never started in a run | the board has no on-board antenna, only the u.FL socket J14; rules 4.1.1 forbid passing information to the robot wirelessly during a run **[src]** datasheet ABX00063, rules; section 8.5 |

## 1. What was asked and where it is answered

| Request (7 Oct) | Answer | Section |
|---|---|---|
| Chute connected to the underside of the plate floor, no gap | hopper sealed to the floor, square channel | 6.2 |
| Chutes support the plate so the "pillar" can go, and leave room for the stepper | the pillar was the N20 hanging under the floor, and the floor itself was unattached. Now the floor is part of the frame and rests on the bridge and spoke; the chutes steady it sideways but do not carry it (they plug into the wall, so they could not carry a floor that lifts off with the frame). The N20 carries nothing. The stepper bay is kept free and a 28BYJ-48 fits | 3.2, 6.1, 6.3 |
| Lid easily removable for kit refill and electronics checks | snap lid, wire-free, T-handle, control deck on the front bridge; what is visible with the lid off | 7 |
| Mounting holes and how each part is placed and attached | component table, GIGA hole check, plan figure | 3.1, 8, Figure 9 |
| Omni not in front of the bumper or outside the body | moved inside, terrain re-checked | 4 |
| Motors clipped on like a bike bottle cage | cartridge and snap cradle, with the key it needs | 5.1 |
| Detail, feasibility and usability of everything | per-section notes, service table, risk table, test plan | 3.3, 10, 11, 12 |
| Second round: is 2WD sound for a navigation robot in general? | yes, with the 2 cm step as the one real risk; the evidence is in the [pre-Fusion report](2026-10-07-theseus-v3-pre-fusion-report.md) | 11, report |
| Second round: check the weight balance again; could the drive wheels move back? | recomputed with the final masses; the axle stays at the body centre | 4 |
| Second round: reach the electronics without taking much apart | the dropper floor, plate, kits, N20 and hoppers lift out as one unit; the battery and the GIGA stack are then 91 and 86 % open from above | 3.3, 6.1, 7.4 |
| Second round: buttons at the front so the wires do not cross the robot | control deck on the front bridge, USB-C socket in the front-right wall | 7.3 |
| Second round: colour sensor pair further forward | silver module and front port at x 7.5, one each side of the omni bay | 5.3 |
| Third round: Wi-Fi so that testing needs no cable | the GIGA already has Wi-Fi and Bluetooth; its antenna is placed on the front wall; the radio runs in a test mode only (rules 4.1.1) | 8.5 |

## 2. Assumptions and constraints

- **Printer:** bed size not measured; I assume a 220 x 220 mm class bed. The shell is 21.0 cm across, so the tub, the frame and the lid are each one print about 210 mm across. If the real bed is smaller they split along y = 0 (section 10). **[measure]**
- **Material:** PETG for everything that flexes and for the frame; PLA for rigid blocks. FDM tolerance about +-0.2 mm: sliding fits 0.2 to 0.3 mm clearance. **[proposal]**
- **Fasteners:** M3 heat-set inserts (5.7 mm long) in printed bosses at least 8 mm across, M3 screws for the frame, M2.5 x 6 for the motor face plates. A 2.5 mm hex key and a small wrench are the only tools. **[proposal]**
- **Masses:** rev 3 uses a placeholder 1.07 kg (`com.py`: printed chassis 150 g, lid plus handle 45 g, dropper plate and floor 30 g, chutes 15 g). A 21 cm PETG tub, ring frame, lid, handle and hoppers will weigh more, perhaps 0.2 to 0.3 kg extra **[placeholder: weigh the printed parts]**; the spring and screw loads below scale with the real mass.
- **Delegated on 7 Oct:** the omni arm, the dropper-motor question, the handle, the lid and the wheel arches were left to this review (D9 to D14, section 13). The print-bed size was left for you to measure.

Hardware facts used:

| Part | Fact | Source |
|---|---|---|
| Pololu 3493, 20Dx44L 195:1 | plain 20 mm cylinder; 4 mm D output shaft, 18 mm long; two threaded M2.5 holes in the gearbox face (3.5 mm maximum screw depth); stall 10 kg.cm at 1.6 A, gearbox limit 5 kg.cm. Occupies y 1.4 to 6.1 including the 3 mm encoder | **[src]** pololu.com/product/3493; rev 3 section 4 |
| Pololu 3499 encoder pair | soldered to the back of the motor, six pads at 2.54 mm; board outline not given | **[src]** pololu.com/product/3499; **[measure]** |
| Arduino GIGA R1 WiFi | 101.6 x 53.34 mm; six 3.2 mm holes in the Mega 2560 pattern; USB-C (J12), USB-A, audio jack, reset (PB1) and boot (PB2) all on one short edge; the underside looks flat in the datasheet pictures (header solder tails are not drawn) | **[src]** datasheet ABX00063: page 18 (holes), page 9 (connector positions, read from the picture) |
| Adafruit VL53L0X board 3317 | 21 x 18 mm, four mounting holes (positions not on the page), two STEMMA QT connectors (edges not stated) | **[src]** adafruit.com/product/3317; rev 3 section 9 |
| OpenMV H7 Plus | 45 x 36 mm board, 29 mm with the lens, M12 lens mount; mounting-hole positions not on the page | **[src]** openmv.io; rev 3 section 8; **[measure]** |
| 28BYJ-48 | body 28 mm across, 19 mm tall; two ears 35 mm apart (4.2 mm holes, R 3.5); shaft 5 mm with 3 mm flats and a 9 x 1.5 mm collar, tip 10 mm from the front face, and 8 mm **off the body centre** at right angles to the ear line; wire block 14.6 mm wide, reaching 17 mm from the body centre on the side opposite the shaft offset | **[src]** Kiatronics datasheet, drawing on page 1 |

## 3. Architecture

### 3.1 Parts and attachments

[Figure 7](v3-figures/fig7_structure.png) is the side section and [Figure 9](v3-figures/fig9_mounting.png) the plan. The robot is a **tub**, an **upper frame** and a **lid**; everything else is a module that plugs into them.

| Part | What it is | Print |
|---|---|---|
| Tub | floor 4 mm (z 3.5 to 3.9) and lower wall 2 mm up to z 8.7; motor cradles, GIGA posts, battery tray, omni pivot ears, floor-sensor pockets, nub boss, bumper hinge sockets and switch pockets, wheel arches (the strip above each one notched at the camera window), chute holes, six insert bosses with ribs, USB-C socket hole in the front-right wall above the bumper band | floor down; supports under the rear chamfer and the arches |
| Upper frame | ring r 9.0 to 10.5, z 8.7 to 11.2 (nine ToF pockets open at the top with their beam windows, two camera cradles and windows, lid seat, hook grooves); front bridge (3 mm web, rib to the front ring) with the control deck on its +y side; rear spoke; three half-lap seats for the dropper floor; handle post | ring bottom (the z 8.7 plane) on the bed |
| Dropper unit | floor disc (R 5.11, 3 mm, slots A and B, N20 pocket), kit plate with the kits, N20 cartridge, two hoppers; **lifts out in one piece** (3.3) | floor and plate flat |
| Lid | 3 mm plate, camera humps, handle slot, front notch over the control deck, finger notches, four hook tongues | top down |
| Hopper and chute (two) | box under each slot (part of the dropper unit), 18 mm square channel to the wall, open as a trough under the slot | channel in two halves, flat (split not redrawn for the trough) |
| Modules | two drive cartridges, N20 cartridge, omni module, two bumper plates, nine ToF boards, two cameras, battery, GIGA and shield | bought or printed |

| Component | Placed | Held by | Mounting holes known? | Tool |
|---|---|---|---|---|
| Drive motors | y 1.4 to 6.1, axle z 4.0 | cartridge: face plate on the gearbox, snap cradle in the floor (5.1) | two M2.5 face holes **[src]**, spacing **[measure]** | none on the robot |
| Wheels | y 7 to 9 | hub on the 4 mm D shaft, one M3 set screw | own design | hex key, from below through the arch |
| Omni | centre (7.0, 3.0) | single arm on a pivot pin between two ears, torsion spring, arc-slot stops | own design | pin |
| Rear nub | x -8.0 | M5 thread in a boss under the chamfer, jam nut | own design | wrench |
| Floor sensors FP and SM | FP (7.5, +3.2), SM (7.5, -2.7), faces z 2.8, one each side of the omni bay (5.3) | snap pockets in the floor, cables up | modules not chosen **[measure]** | none |
| Bumpers | 12 to 58 degrees | hinge pin in a wall socket at the outer end, return spring, microswitch in a pocket at 14 degrees (rev 3 section 6) | switch part not chosen **[measure]** | hex key |
| GIGA R1 | stack at (2.32, -4.03), z 6.15 to 8.05, connector edge forward | four printed posts with M3 inserts | **yes**, six holes, four used **[src]** | hex key |
| Main PCB shield | on the GIGA | plugs into the GIGA headers; its outline starts about 20 mm from the connector edge | designed later | none |
| Battery | (3.9, 4.35), 10 degrees | tray with a notch over the left motor, velcro strap, front stop | no holes | none |
| Dropper floor | R 5.11, 3 mm, z 8.7 to 9.0, centre (-2.0, 0) | three tabs (the top half of the thickness) lie in rebates of the web's top half, on the web's lower half: two beside the bridge rib (x 3.0 to 3.9, y +-0.85 to +-1.25, clear of the handle post), one on the spoke web (x -7.5 to -6.9); two snap tabs or thumbscrews hold it down **[proposal, not drawn]**; lifts out with plate, kits, N20 and hoppers | own design | none (or a hex key for thumbscrews) |
| N20 | (-2.0, 0) | cartridge in the dropper floor's pocket, two snap tabs | face holes **[measure]** | none |
| Kit plate | R 4.91, z 9.0 to 10.2 | friction on the D-shaft plus an M2 set screw; rests on the dropper floor | own design | lift off |
| Hoppers, chutes | slots at (-4.73, +-2.73) | hopper snapped under the dropper floor; channel plugged in from outside through the wall | own design | none |
| Upper frame | z 8.7 and up | six M3 x 30 screws from the ring top (heads 3 mm below the top, under the lid) into inserts in tub bosses (each with a rib to the wall), at +-12, +-60 and +-118 degrees **[check]** `ring_gaps` | own design | hex key |
| ToF boards (9) | in the ring, z 10.0 | drop into pockets from above, two snap tabs, cable out under the ring | four holes **[src]**, positions **[measure]** | none |
| Cameras (2) | +-88 degrees, tilted 20 degrees | slide-in cradle, two snap tabs; M2 screws once the holes are known | not on the page **[measure]** | none |
| Lid | on the ring | four hook tongues (7.1) | own design | none |
| Handle and victim LED | post on the front bridge; the LED (5 mm) on top of the bar's front end (x 8.35, z 15.6 to 16.0) | post printed on the frame; bar fixed with two M3; the LED's two wires run down the post to a small connector at the bar | own design | hex key |
| Control deck | front bridge, x 5.2 to 9.15, y -1.25 to 3.6 | start button and power switch on the +y side, two status LEDs in a row on the bridge rib, parts in printed recesses | part sizes open **[placeholder]** | none |
| USB-C service socket | front-right wall at -24 degrees, above the bumper band, 2.0 cm from J12 | panel-mount, front face flush with the wall (7.3) | part open **[placeholder]** | none |
| Wi-Fi and Bluetooth antenna | inside the tub's front wall at -6 degrees, z 5.5 to 7.0 (x 10.2, y -1.1) | adhesive flat flex strip from the GIGA's box, 100 mm cable to the u.FL socket J14 (8.5) | none: it is a bought part | none |

### 3.2 Load paths

- **Weight:** wheels, motor cradles, tub floor; omni, arm, pivot ears, tub floor. Everything else rides on the tub floor.
- **Lifting by the handle:** bar, post, the bridge (3 mm web with an 8 mm rib, a cantilever of 5 cm from the post to the front ring), the front ring, the six screws into the tub wall, floor. Design load 15 N (12 N weight plus handling), times 3 is 45 N: 7.5 N per screw if all six share it, 22 N each if only the two at +-12 degrees did **[calc, placeholder mass]**. The bridge section gives 2.0 MPa at the web underside and 0.19 mm of droop at 15 N, 5.9 MPa and 0.57 mm at 45 N **[calc]** `handle_calc`. The dropper floor is not on this path (it is a separate part sitting on the bridge and the spoke). Bench test: hang 5 kg from the bar.
- **Dropper:** the unit's floor sits on three half-lap seats, two in the bridge web beside its rib and one in the spoke web, and is held down by two snap tabs or thumbscrews; the chutes plug into the wall and steady it sideways. Plate, kits, floor, motor and hoppers weigh about 125 g **[calc, placeholder masses]** `balance`.
- **Shocks:** a 0.15 kg motor and wheel stopping a 2 cm drop in 3 mm is about 6.7 g, 10 N into the seat **[calc, assumed stopping distance]**. The ground pushes the motor into its seat, so the snap only has to keep it seated on rebounds (5.1).

### 3.3 Assembly order and service

First build: (1) tub: nub, floor modules, bumper sockets; (2) drive cartridges and wheels; (3) omni module; (4) GIGA posts, battery tray, GIGA screwed down, shield, battery; (5) harness to the tub side; (6) on the bench, fit ToF boards, cameras and the control parts to the frame; lower the frame onto the tub, six screws, plug the cables; (7) on the bench, build the dropper unit (hoppers under the floor, N20 cartridge in its pocket, plate on the shaft, kits), lower it onto its three seats, snap it down, plug the N20 connector; (8) chute channels in from outside; (9) lid.

Every removal below was run as a path of small steps on the emulator of the rebuild plan, and each is re-run in Fusion after the build (report `removal`, section 12): lid straight up; each wheel out and down; kit plate and N20 cartridge; the dropper unit straight up; the battery; and the full teardown in the order of the last row.

| Operation | Steps | Tools | Time (estimate) |
|---|---|---|---|
| Kit refill | lid off (lift), drop eight kits into the pockets, lid on | none | under 1 min |
| Reflash firmware | USB-C socket in the front-right wall (7.3), lid on | USB cable | -- |
| Dropper unit out | lid off; pull the two chute channels out through the wall; release the two snap tabs (or thumbscrews); unplug the N20 connector; lift straight up. The handle bar starts at x 3.3, in front of the unit, so nothing is in its way | none | about 1 min |
| Battery swap | lid off, dropper unit out, unstrap, unplug, then slide the battery 4.5 cm back and 2 cm inboard and lift it out: Camera L's board is 8.6 mm over its outboard end, the ring over its front corner and the power switch over its front, so it cannot go straight up. For the placeholder 7.0 x 3.5 x 2.5 cm battery, after the 4.5 cm slide back any inboard move from 1.5 to at least 3.5 cm is free (less than 1.5 and the board catches it); a wider or taller battery needs this path re-checked, and the camera cradle can move outward **[calc]** `removal` | none | 2 to 3 min |
| ToF or camera replace | lid off, lift the board out, unplug from below, swap | none | 1 to 2 min |
| Look at the LEDs, press reset | lid off; the GIGA's connector edge is open from above because the control deck stops at y -1.25 (7.4); the boot button needs the frame off | none (a pen) | -- |
| Tyre swap | robot raised, set screw, wheel out through the arch | hex key | about 3 min |
| Bumper replace | from outside, hinge pin and a plug | hex key | about 3 min |
| Nub height | loosen the jam nut, turn | wrench | about 1 min |
| Omni service | lid off, pull the two chute channels, six screws out, unplug the frame cables (about 15; the dropper unit stays on its seats and travels with the frame), lay the frame beside the tub on its 200 mm cables, pull the pivot pin, lift the omni out | hex key | about 10 min |
| GIGA stack replace | frame off as in the omni service, four screws, shield and GIGA out together | hex key | about 15 min |
| Drive motor service | as above, then the part above the motor comes out first: **left motor, the battery** (it lies 1 mm over the motor and overlaps it by up to 9 mm); **right motor, the GIGA stack** (four screws, the shield cables; it covers the motor). Then the wheel out through the arch, spread the two prongs and lift the cartridge. Nothing else is in the way **[calc]** | hex key | about 15 min left, about 25 min right **[estimate]** |

## 4. Omni module, inside the body

Requested: the omni must not stick out in front of the bumper or the body.

**Geometry.** Centre (7.0, 3.0), radius 3.0, width 2.0. Front edge x 10.0 at rest and 9.62 fully compressed; the wall's inner face is 10.3 (3 mm clear), outer face 10.5. The front wall has no slot. The arm length stays 4.49 cm, so the pivot moves back to (2.55, 3.6); the hard stop is 2.5 cm of axle travel (arm angle -7.7 to +25.0 degrees, an arc of 32.7 degrees) **[calc]**. The stops are the two ends of an arc slot in the pivot ear with a pin on the arm **[proposal]**.

**Single arm.** One 4 mm plate on the +y side (y 1.1 to 1.5, 1 mm clear of the 2.0 cm wheel), a 4 mm axle crossing the wheel with an E-clip on the free end, a 4 mm pivot pin through two ears (as in the first model). The pin at z 3.6 reaches z 3.4, 1 mm below the belly line (3.5), and the Dangerous Zone 30 degree stairs case has only 0.1 mm of margin over its 2 mm threshold, so use a 3 mm pin (0.5 mm below) or sink the pin and ears into the floor; decided in CAD. Reason: the GIGA hole H1 at (5.87, -1.62) lies 3.5 mm inside the swing of a plate at y -1.5; with the plate on +y the boss clears the wheel by 2.7 mm (2.2 mm in the script, which pads the wheel by 0.5 mm) **[check]** `mount_check`. The axle load is a few N at the rim on a lever of about 15 mm, roughly 12 MPa in a 4 mm steel rod for 5 N **[calc, assumed load]**.

**Bridge clearance.** The compressed omni tops out at z 8.5, so the front bridge must stay a 3 mm web (underside z 8.7, 2 mm above the wheel) with its stiffening rib on top **[calc]**.

**Terrain, omni at 7.0** (stiff-spring worst case as in rev 3; the chute lip is the 3D-model value z 2.67, lower than the 2.78 the 18 mm channel gives (2.91 in the 3D model), so this is conservative; cm, degrees) **[check]** `omni_inside`, [results](v3-checks/results/omni_inside.txt). A case passes when every clearance over the whole crossing is at least 0.2 cm.

| Case | Belly | Bumper | Chute lip | Tilt | Result |
|---|---|---|---|---|---|
| 2 cm riser up / down | 1.50 / 1.50 | 2.00 / 2.85 | 1.19 / 0.67 | +14.2 / -17.0 | pass |
| two 1 cm steps up / down | 1.92 / 1.50 | 2.33 / 2.85 | 1.19 / 0.67 | +15.3 / -17.0 | pass |
| 25 degree ramp up / down | 1.61 / 1.62 | 2.05 / 2.06 | 1.14 / 1.13 | +25 / -25 | pass |
| 2 cm / 1 cm bump | 1.50 / 2.50 | 2.00 / 3.00 | 0.67 / 1.67 | -17.0 to +14.2 / +-8 | pass |
| 3 mm seam | 3.20 | 3.70 | 2.41 | +2.4 | pass |
| three 2 cm stairs, 25 degrees, up / down | 1.41 / 2.29 | 1.73 / 2.79 | 1.13 / 2.67 | +29.7 / -32.2 | pass |
| three 2 cm stairs, 30 degrees (Dangerous Zone), up / down | 0.21 / 1.49 | 0.62 / 1.73 | 1.15 / 2.67 | +30.8 / -34.5 | up is marginal: 2.1 mm against the 2 mm threshold (rev 3: 1.0 mm, fail) |
| 1 cm bump on a 25 degree ramp (Dangerous Zone) | 0.78 | 1.08 | 1.14 | +33.1 | pass |
| 2 cm bump on a 25 degree ramp (Dangerous Zone) | 0.01 | 0.10 | 0.67 | +39.2 | **fails**, as in rev 3 (belly 0.06; D8) |

At 7.3 and 6.5 the same code also passes every required case, so 7.0 is a comfortable middle. The smallest clearance in the required cases is the chute lip, 0.67 cm, and the omni does not change it.

**Costs** **[check]** `statics_omni_7` (the same `statics.py` as rev 3, run with the omni 7.0 cm ahead, rev 3 budget masses). The front load rises from 11.9 to 13.6 percent. The 2 cm stair push becomes 0.38 W against 0.34 W with traction 1.30 W against 1.32 W (margin x3.4, was x3.9), and the torque per wheel for a 2 cm riser is 1.60 kg.cm (1.63). Coming down a 25 degree ramp the front load is 56 percent (49), so the spring must carry 5.9 N within the 2.5 cm travel (5.2 N): vertical stiffness at least 1.8 N/cm (1.6) with a preload of about 1.4 N. As a torsion spring on the pivot that is a preload of about 64 N.mm and about 360 N.mm/rad (1.4 N times the 44.5 mm lever; 0.18 N/mm times the lever squared) **[calc]**. All of these scale with the real mass (the stiffness would be about 2.2 N/cm at 1.3 kg). The nose pitches 17.0 degrees coming down a 2 cm riser (14.8 in rev 3), so keep the IMU-pitch gating on the front ToF readings. A custom wire torsion spring has to be sourced; the fallback is a compression spring on a printed seat under a tab of the bridge.

**Acceleration limit.** The front lifts above g x COM_x / COM_h = 1.5 m/s^2 in rev 3 (section 0.4 item 2, D5). The emulator model of this layout (printed parts from the model volume at 45 % fill, bought parts from the rev 3 budget, wiring and fasteners as unmodelled lumps) weighs 1197 g, with the centre of mass **0.90 cm ahead of the axle and 6.81 cm high**: front load 12.8 % (1.51 N on the omni), front lift-off at **1.29 m/s^2**, nose-down tip when braking at 8.8 m/s^2 **[calc, placeholder masses]** `balance --repo` (with the redesigned chute and the Wi-Fi antenna; the first dry run said 1195 g). The Fusion model of the same layout (8 Oct) measures 1198 g, 0.89 cm ahead of the axle, 6.81 cm high, the same 12.8 % front load and the same 1.29 m/s^2 (`v3-fusion/results/v4_mass.txt`). Moving the start button, switch, LEDs and USB-C socket from the rear to the front moved the centre of mass 0.26 cm forward; the first rear-panel draft gave +0.63 cm and 0.91 m/s^2. A 0.3 m/s top speed reached in 0.25 s needs 1.2 m/s^2, so keep a firmware PWM ramp of at least 0.25 s (D5) and weigh the printed parts.

**Axle position (second round: could the drive wheels move back?).** The drive train (wheels, motors, face plates, 302 g) moves with the axle and everything else stays in the body, so each centimetre back moves the centre of mass 0.75 cm forward relative to the axle. **[calc]** `balance` for the balance, **[check]** `axle_shift` for the terrain (same side-view simulator and cases as the omni study, the chute lip kept where the wheel needs it):

| Axle behind the body centre | COM ahead of axle | Front load | Lift-off limit | Swept radius | Terrain cases |
|---|---|---|---|---|---|
| **0 (chosen)** | 0.89 cm | 12.8 % | 1.29 m/s^2 | 11.00 | all nine required cases pass; the Dangerous Zone 2 cm bump on a ramp fails, as in rev 3 |
| 1.0 cm | 1.64 | 20.5 % | 2.36 | 11.98 | same |
| 2.0 cm | 2.38 | 26.5 % | 3.43 | 12.96 | everything passes, the bump on the ramp included (chute lip 0.50 cm) |
| 3.0 cm | 3.13 | 31.3 % | 4.50 | 13.95 | both ramps, the 30 degree stairs and both bump-on-ramp cases fail on the chute lip |

Nothing here asks for it. The lift-off limit is already above the 1.0 m/s^2 the ramp assumes; each centimetre back takes 7.7 points of load off the driven wheels (9 % less weight on the driven wheels for the stair climb, the one required case with little margin) and adds a centimetre to the swept radius (11.98 at 1 cm against a worst-case corridor half-width of 12.6, which leaves 0.6 cm a side instead of 1.6). The only gain is at 2 cm, where the corner case passes, but the radius 12.96 does not fit the worst-case corridor. So the axle stays at the body centre. If the real robot lifts its nose under acceleration or the omni leaves the floor on the stair test, weigh it and move mass before moving the axle: size the battery tray with at least 1 cm of fore-and-aft slack once the real battery is chosen, so that the battery trims the balance (it is the heaviest single part, 110 g, and moving it 1 cm moves the centre of mass 0.09 cm).

**Swept circle.** Rev 3 had the swept radius 11.0 set by the bumper tips and the omni's front edge. Now only the bumper tips reach 11.0, so pulling the bumpers back to the body radius (rev 3 open item) would give a 21 cm circle.

**Usability.** The omni is reached from above with the frame off: pivot pin out, arm and wheel come away as a unit, the spring is replaceable.

## 5. Drive train

### 5.1 Motor cartridge and snap cradle

Requested: motors held by an open-sided snap cradle, not screwed to the robot. [Figure 8](v3-figures/fig8_motor_clip.png) and [Figure 9](v3-figures/fig9_mounting.png).

**A snap alone cannot hold the torque.** The motor body is a smooth 20 mm cylinder **[src]**. A hook that keeps a few N of preload on it gives a friction torque of only 0.04 to 0.16 kg.cm (two hooks, friction 0.35); a 2 cm riser needs 1.63 kg.cm and the gearbox limit is 5 **[check]** `mount_check`. The torque has to go through a key.

**Design.**
- **Cartridge:** the motor plus a 3 mm PETG **face plate** screwed to the gearbox face once (2 x M2.5 x 6: 3 mm of plate plus 3 mm of thread, inside the 3.5 mm limit **[src]**; hole spacing **[measure]**). The plate has a 4.4 mm bore for the D-shaft and **one** ear about 8 x 3 mm on the front (+x) side, 15.5 to 23.5 mm from the axle. One ear, not two: on the right motor a rear ear would run into the GIGA post H4 at (-2.27, -6.43).
- **Cradle (printed with the tub):** the seat is the floor cut to the motor radius (20.3 mm), so the lower flanks rest on the floor edge. An outer web 2.5 mm thick (y 6.45 to 6.7, 3 mm clear of the wheel hub at y 7.0, x -1.6 to 2.75) and a ledge each side of the motor (y 5.65 to 6.05; the front one x 1.06 to 2.75, the rear one only x -1.6 to -1.06) form the slot for the plate, 3.4 to 4.0 mm for the 3.0 mm plate, which also cuts through the floor. The ear sits in the slot beside the front ledge and stops the turning. At the gearbox limit the ear carries 25 N on 24 mm^2, 1.0 MPa **[calc]**.
- **Prongs:** two thin vertical walls, one each side of the motor, cantilevered along the axle from the ledges (free length 32 mm, so y 2.85 to 6.05), with a hook at the free end 5.5 mm above the axle and a 0.75 mm undercut. For 2.4 mm thick, 11 mm tall, 32 mm free length the hook must deflect 2.4 mm to pass the equator: 5.6 N to push in per prong, strain 0.84 %, about 4.6 N to pull out, friction 0.15 kg.cm. Across the options tried (1.6 to 2.9 mm thick, 32 to 40 mm free) push-in is 1.3 to 7.6 N, strain 0.4 to 1.0 %, pull-out 1 to 6 N **[check]** `mount_check` (PETG, E 2 GPa, uniform-thickness beam, wedge estimate for the pull-out; the real hook is thicker, so these are indicative).
- **Why walls along the axle:** a printed arm that stands up from the floor bends across the layers and cracks after a few cycles; a wall that bends sideways bends along them (section 10).
- **Retention is modest by nature:** the hook only keeps the cartridge seated against rebounds. If a drop test shows it popping out, add a small detent on each ear or a finger-tight thumbscrew through each plate ear. Release: spread the prongs with two fingers.

Print three cradles (hook height, undercut and wall thickness varied) and keep the one that needs a firm thumb push to seat and a deliberate spread to release. PLA is too brittle for a snap used many times.

**Encoder.** The encoder board's outline is not on the Pololu page. If it sticks out of the 20 mm silhouette, turn that side up or away from the GIGA post H2 at (-1.62, -1.62). If the face-plate screws are unwanted, the alternative is a notch that keys the encoder board, which needs its outline first. **[measure]**

### 5.2 Wheels, tyres and the arches

80 mm silicone tyres, 20 mm wide, on printed hubs on the 4 mm D-shaft. The shaft is 18 mm long **[src]**: 9 mm run from the gearbox face (y 6.1) across the 3 mm gap and the web to the hub face (y 7.0), and 9 mm engage the hub, with an M3 set screw on the flat. **The wheel cannot leave the robot as modelled:** to clear the shaft it must slide about 1 cm outwards, and its outer corner (r 9.85 at x +-4.0, y 9.0) would then reach r 10.7, past the wall's inner face at 10.3. You will swap tyres often (the stair test depends on friction), so the lower wall gets an **arch** on each side, x +-4.4, z 3.5 to 8.3, and the floor opening runs out to the wall: the wheel slides 1 cm along the shaft and drops out downwards. The ring above stays continuous; the lower wall loses about 9 cm of its circumference on each side and gets its stiffness from the floor and the ring **[proposal]**. A snap-in cover for the arch is possible if the openings prove a snag risk.

Two things follow from the arches (found when the rebuild plan was dry-run, **[calc]**). First, the 4 mm strip of wall above each arch (z 8.3 to 8.7) lies in the middle of the camera's field of view: rays from the lens tip (z 9.3, 1.7 cm inside the wall) at -22 to -30 degrees cross it, and the optical axis at -20 degrees just clips its top edge. It is **notched 3 cm wide at the camera window**, so the window is open from the arch up to z 10.6 over that width. Second, a frame boss must not hang over the tyre: at 101 degrees the boss underside (z 7.70) was 1 mm below the tyre's top rim (7.80) once the wheel slid out, and it would have hung from the thin strip. The rear screws are therefore at **+-118 degrees**, with the boss centre just outside the arch (x -4.6 against the arch edge at -4.4; above z 7.7 the tyre only spans x +-1.5), and every boss has a rib to the wall.

### 5.3 Floor sensors and rear nub

**Floor sensors, further forward (second round).** The silver module SM moves from the axle to (7.5, -2.7) and the front port FP from (6.5, +3.0) to (7.5, +3.2): one each side of the omni bay, faces at z 2.8, in snap pockets in the floor, cables up. They see a tile 7.5 cm of travel earlier (SM) or 1 cm (FP), about 250 ms at 0.3 m/s for the silver module **[calc]**. Terrain: the smallest clearance between the window (2 cm wide, face z 2.8) and the terrain over each crossing is 1.70 / 2.37 cm for the 2 cm riser up / down, 1.30 for the 25 degree ramps, 1.70 for the 2 cm bump, 0.82 for the 25 degree stairs, 0.50 for the 30 degree stairs and 0.67 for the Dangerous Zone bump on a ramp; at the axle they were 2.4 to 2.5, the criterion is 0.2. At x 8.0 the bump on a ramp falls to 0.17 and at 8.5 the sensor would hit it, so 7.5 is about the limit; a face at z 3.3 would add 0.5 cm on the 30 degree stairs and 0.2 on the bump on a ramp **[check]** `fsens_front`. The modules sit 2.8 mm from the GIGA post H1 (1.5 mm needed) and 5.0 mm from the omni wheel in the model (placeholder module size, `clearances` report).

**Firmware.** `color.ino` accepts silver only when the encoders show half a tile of travel (`overNextTile`, TILE_MM/2): that matches a sensor at the axle, which is over the next tile exactly then. A sensor 7.5 cm ahead is over it after TILE_MM/2 - 75 mm, so the V3 firmware takes that threshold; the visited-tile rule (rev 3 5.4.4, more than half the robot on the tile) is unaffected if the reading is latched until the robot has gone half a tile. Black is accepted at any time already **[proposal]**.

The rear nub is a nylon M5 bolt with a PTFE or ball tip in a boss under the chamfer at x -8.0, adjustable 1.5 to 2.5 cm with a jam nut (the nub height test, rev 3 sections 4 and 11) **[proposal]**.

## 6. Dropper

### 6.1 Support and the lift-out unit

The dropper floor (R 5.11, 3 mm) is its own part and the base of the **dropper unit**: floor, kit plate with the kits, N20 cartridge and both hoppers. The unit lifts out in one piece (3.3) and sits on three half-lap seats of the upper frame.

- **Front bridge:** x 3.2 to 9.2, 25 mm wide, a 3 mm web (z 8.7 to 9.0) with a 10 mm wide, 8 mm high rib on top (x 3.25 to 9.05, z 9.0 to 9.8) that carries the handle post. The compressed omni forbids anything under the web (section 4). From x 5.2 to 9.15 the web widens to the +y side as the control deck (7.3).
- **Rear spoke:** x -9.2 to -7.2, 20 mm wide, 3 mm web with a 10 mm wide rib to x -7.9 (z 9.0 to 9.5).
- **Seats:** the disc has three tabs, each the top half of its thickness (z 8.85 to 9.0): two at x 3.0 to 3.9 beside the bridge rib (y +-0.85 to +-1.25, 1.5 mm clear of the handle post) and one at x -7.5 to -6.9 on the spoke. Each lies in a rebate cut into the top half of the web and rests on the web's lower half (z 8.7 to 8.85): the rebates locate the unit in x and y, the lower halves in z. Two snap tabs or thumbscrews on the seats hold it down against rebounds **[proposal, not drawn]**.
- **Lift-out:** the unit leaves straight up past the handle post (the disc is 1.9 mm and the plate 3.9 mm short of it), the bar starts in front of it (7.2), and the two chute channels are pulled out through the wall first. The chutes plug into the wall and steady the unit sideways; they are not the main support. There is no pillar **[proposal]**. The straight lift of 12 cm is free of every other part on the emulator **[calc]** `removal`.

Check: hang 5 kg from the handle and press the plate with a finger (the floor should deflect under 1 mm); lift the unit out and seat it again 100 times and look for wear on the rebates.

### 6.2 Hopper and chute

Rev 3: slot 14.5 mm square at 45 degrees, chute axis starting under the slot at z 7.95. In the first Fusion model the tube mouth was a tilted disc 0.75 cm below the floor and touched it along one edge only. The first rev 4 design (14.5 mm slot and hopper void over a 13 mm square channel, the trough cut flat at z 7.7) passed every static geometry check and then **failed a rigid-body simulation of a kit** (8 Oct). The design below is the redesign that passes it for a plate parked within 1 mm; the failure and what was tried are the last bullets of this section. An independent review of the redesign the same day found that the first simulation left the plate out and shared its poses between friction values; it was redone with the plate's pocket walls and a pose of its own for every kit, and **that moved the answer on parking error: the dropper has to park the plate within +-1 mm** (the two bullets on the simulation).

- **Slot, hopper void and bore:** slot 16.0 mm square (turned 45 degrees), hopper void 16.0 mm square, channel bore 18 x 18 mm inside with 1.6 mm walls (21.2 mm outside). A 10.3 mm cube has a 14.57 mm face diagonal and a 17.84 mm space diagonal. The 16 mm slot and void leave 1.43 mm over the face diagonal, so the cube passes lying at any turn, and tipped any way except within 4.5 degrees of standing on a corner, which needs 16.25 mm (0.9 % of random attitudes; the 14.5 mm slot of the first design fails 33 % of them, since it is narrower than the face diagonal) **[check]** `cube_slot`, `mount_check`. The 18 mm bore is 0.16 mm wider than the space diagonal, so at the nominal size a kit that tumbles after its bounce cannot wedge between two side walls or between floor and ceiling; that margin is thinner than the kit's own tolerance (10.3 +-0.2 mm gives 18.19 mm) or a printed hole's shrinkage, and with bouncy impacts a 10.5 mm kit or a bore printed 0.4 mm small loses 2 to 5 % of the kits in the simulation (D21: 19 mm costs 0.5 mm of the chute-to-wheel gap). The square section is centred 1 mm above the axis (the floor is 8 mm below the axis, the ceiling 10 mm above), so the floor, which sets the lowest point at the wall, stays where it was in a 16 mm channel **[check]** `chute_dynamics`.
- **Hopper:** a box under each slot, 22 mm outside and 16 mm inside (3 mm walls), from z 7.2 up to the floor underside (z 8.7), with a 26 mm flange 1.5 mm thick sealed flat against the floor by two snap tabs. It is a separate part because it hangs below the print plane of the dropper floor, and it lifts out with the unit. Where the channel leaves it, the hopper has a socket: the channel's own section cut out of the downhill wall and floor strip, made 8 mm taller than the channel, so that no piece of the hopper is left hanging above the channel (cut only to the channel's ceiling it left a loose 85 mm3 lintel, and Fusion split each hopper into two bodies; `v4_checks3d` now checks one body per hopper and channel).
- **Open trough:** inside the hopper the channel has no walls and no ceiling. A 25 mm square cut (turned 45 degrees, from z 5.0 up) removes everything of the tube inside it, and a floor slab (outer width, one wall thick, 2.8 cm along the axis from the slot centre) is laid back, so that a kit lands on the inclined floor with no wall end, ledge or ceiling edge to catch it; the cut is bigger than the void so that it goes right through the side walls where the tube begins, and the walls end in a V that leads a kit into the bore. The slab has to reach past the point where the cut's vertical wall meets the sloping floor, 2.55 cm (top face) to 2.65 cm (underside) along the axis, not the 2.04 cm of the cut's corner at axis height: at 2.5 cm it left a notch in the bore floor (found by the review; `test_channel_floor_has_no_hole`, and floor probes in `v4_checks3d` that fail in Fusion with the shorter slab). The tube's top corners just outside the cut would reach into the dropper floor, so they are cut at its underside (z 8.7). The hopper and the channel do not overlap, so the channel pulls out through the wall.
- **Axis:** slot centre (-4.73, +-2.73, 7.95) to the exit (-6.0, +-8.62, **4.15**), slope 32.2 degrees (rev 3: 34.9). A straight tube meets the cylindrical wall at about 22 degrees, so its lower edge runs further down the slope than the 2D end cap showed. Lowest point where the channel leaves the wall **[check]** `chute_exit`:

| Exit axis z | Slope | Round tube, r 0.9 | Square, 16.2 outside (first rev 4) | 18 mm channel, 21.2 outside |
|---|---|---|---|---|
| 3.75 (rev 3) | 34.9 | 2.63 | 2.56 | 2.32 |
| 4.00 | 33.2 | 2.90 | 2.84 | 2.61 |
| 4.15 | 32.2 | 3.06 | 3.00 | **2.78** |

  The 3D model measured 2.67 for the round tube at z 3.75, and the terrain tables use that value. At z 4.15 the 18 mm channel's lowest point is 2.78 by this 2D calculation, which runs the tube out to the body radius, and 2.91 in the Fusion model, whose tube ends flat 0.8 cm past the axis's exit point, so that its lower inner corner stops 2 mm inside the wall's outer face (r 10.30; the belly report, `v3-fusion/results/v4_belly.txt`). Both are above the 2.67 of the terrain tables. Raising the exit axis to z 4.34 would put the 2D value back at 3.00, at a slope of 30.9 degrees (tan 0.60, against 0.63 now): that is as flat as the friction range simulated (0.15 to 0.60), so the exit stays at z 4.15 and the 2.78 is accepted.
- **Kit speed and landing** **[check]** `kit_landing_square` (friction 0.35, assumed): exit speed 0.58 m/s (rev 3: 0.64), flight 60 ms, carries 2.9 cm, lands 2.5 cm from a 28 cm wall (2.7), impact 1.02 m/s (unchanged by the redesign: it depends on the slope and the exit height, not on the bore). The simulation below gives a mean exit speed of 0.63 m/s at that friction (0.52 to 0.69). The kit stays 4.7 to 10.8 cm from the victim for victims 3 cm behind to 3.6 cm ahead of the camera, so the 15 cm rule keeps the same slack. Rev 3's remark stands: the impact speed is above the 0.77 m/s of Sprint 3, so the last section may need a friction patch or switchback. The chute test with real kits (rev 3 test 3) decides.
- **Kit simulation of the final design (8 Oct, redone after the independent review)** **[check]** `chute_dynamics` (MuJoCo 3.15): a 1.2 g kit starts at rest on the floor in its plate pocket over slot B, falls through the slot and the hopper and slides down the channel to the exit. The geometry is the Fusion bodies (0 of 7000 random points differ and 7999 of 8000 points 0.3 mm either side of the faces agree: `chute_geometry_check`). The plate is the four walls of its 14 mm pocket, 5 mm thick, z 9.0 to 10.2, 0.3 mm above the floor, parked exactly over the slot or off along the ring (the pocket turns with the plate). Every kit has its own random turn (up to +-29 degrees, the most the pocket allows) and place in the pocket. **Plate parked exactly: 100 % of 1000 kits get out** (friction 0.15 to 0.60, 100 kits each); **parked up to 1 mm off (1.5 degrees): 100 % of 1000**; with bouncy impacts (contact damping ratio 0.3 instead of 1: 0.45 of the impact speed comes back, against 0.13) and the plate parked exactly: **100 % of 1000**; **parked up to 2 mm off: 82.8 % of 1000**. The run fails unless its controls pass: the engine gives the textbook acceleration g (sin a - mu cos a) on a plane tilted 32.2 degrees (3.02, 2.28 and 1.45 m/s2 at friction 0.25, 0.35 and 0.45, against 3.15, 2.32 and 1.49 by the formula), holds a cube still at friction 0.70 (above tan 32.2 = 0.63), and stops a 17.5 mm cube at the 16 mm slot. The 3D kit-path check in Fusion (`v3-fusion/kit_path.py`) lets the cube fall from its pocket to 2 mm above the floor slab and slide out of both channels, centred and shifted 3 mm to each side and 2.5 mm up and down, without touching a part, and blocks a 17.5 mm cube at the slot and a 19 mm cube in the bore.
- **Parking error and the approach to the slot: the limits found** **[check]** `chute_dynamics`, `plate_tolerance`. *Parking.* A kit lying corner to corner in its pocket is as wide along the ring as the pocket (14 mm) and the slot is 16 mm, so the plate may be parked off by (16 - 14) / 2 = **1.0 mm** (1.5 degrees at the ring) before the kit's corner hangs over the slot's edge and the pocket wall pins it. The simulation agrees and shows how sharp the limit is: of 100 kits each, at friction 0.35 and 0.55, with the plate off by 0.5 mm 100 and 100 get out, by 1.0 mm 96 and 95, by 1.5 mm 59 and 56, by 2.0 mm 26 and 38; in the sweep up to 2 mm the kits that got out were, by how far off the plate was (mm), 0.0 to 0.5: 262 of 262; 0.5 to 1.0: 256 of 258; 1.0 to 1.5: 198 of 253; 1.5 to 2.0: 112 of 227. Friction hardly matters here (the misses start at friction 0.15), and neither did the clearance under the plate (0.1, 0.3 and 0.6 mm gave 86, 86 and 79 of 100 at friction 0.35 and 76, 76 and 90 at 0.55). This is tighter than the +-3 degrees that rev 3 took from `plate_tolerance`; that check (a neighbouring pocket reaching the slot) is not the limit: with the 16 mm slot its first overlap comes at about 4 degrees (margin 1.74 mm). **The dropper must park the plate within +-1 mm, aiming at +-0.5 mm**: approach the park position from the same side every time (the N20's gearbox has backlash) and measure the repeatability on the bench. If it cannot, a 13 mm pocket (rev 3 D6) would raise the allowance to 1.5 mm and a 16.5 mm slot to 1.25 mm **[calc, not simulated; a bigger slot also costs `plate_tolerance` margin]** (D22). *Approach.* A plate that turns the pocket onto the slot (from 20 degrees away, from either side) and stops there at once: at 60 degrees/s only 4 of 30 kits get out at friction 0.55, and 30 of 30 at friction 0.35 and below and at 120 and 200 degrees/s at every friction: the final approach should be at least 120 degrees/s (0.08 m/s at the ring), or the friction under about 0.5 **[assumed friction; a real stop is not instant]**.
- **What the simulation does not know, and the corners of the tolerances:** friction (one Coulomb value for every printed surface, 0.15 to 0.60), the impact softness, a uniform cube (the real kit may be weighted off centre: bench test), the plate's hub, motor and the other pockets, the real plate speed and stop, the tub floor, wall and wheel (a kit that leaves anywhere but the exit counts as lost), and the plate's real clearance. The kit's mass is not decisive: a 6 g kit (the mass budget has 6 g per kit) does as well as the 1.2 g one. The bore has 0.16 mm to spare over the cube's space diagonal, less than the kit's own tolerance: with bouncy impacts a 10.5 mm kit (rev 3's 10.3 +-0.2 mm) gets out in 98.3 % of 180, a bore printed 0.4 mm small in 98.3 % and both together in 95.0 % (nearly inelastic impacts: all of them). The first version of this simulation, with no plate and the same 100 poses for every friction value (so its '1000 kits' were 100), gave 98.1 % at 2 mm off; the same model with a fresh pose for every kit gives 96.7 % (`results/chute_dynamics_no_plate.txt`), and the pocket walls take it to 82.8 %. If the bench test shows bouncy kits, a 1 mm pad of TPU or felt on the channel's floor slab under the slot takes the bounce out (the hopper's own floor strip covers only 6 % of the void) **[proposal, not simulated]**.
- **What failed first and what was tried** (the first design; `v3-checks/results/chute_dynamics_first_design.txt`, `chute_dynamics_first_design_bore147.txt`): the channel runs 32.8 degrees off the sides of the 45-degree slot square, so a kit that falls straight down arrives turned 32.8 degrees to it. A 10.3 mm cube turned by θ is 10.3 (cos θ + sin θ) wide, 14.2 mm at 32.8 degrees, and the 13 mm bore takes it only within 18.2 degrees of its axis. Kits within 10 degrees of the slot square, which is how a kit sits against a pocket wall, did not get out in any of 10 trials at any friction value (they stopped in the hopper mouth on the low side walls of the trough cut flat at z 7.7); kits turned 20 degrees or more got out 10 of 10. It was geometry, not friction. **Widening the bore to 14.7 mm looked like the fix and was not:** a cube fits at any turn, yet the aligned kits rose only to 30 %. Four things had to change together: (1) the trough is cut down to the bore floor, not flat at z 7.7, which left low side walls for kits to rest on; (2) the slot and hopper void must be wider than the cube's face diagonal (14.5 mm is narrower than 14.57: a tipping kit wedges across the diagonal); (3) the bore must be wider than the space diagonal both ways (with bouncy impacts a 16 mm bore still wedged tumbling kits); (4) no wall end or loose fragment may stand where the trough meets the bore. Not taken: a guide in the hopper that turns the falling kit (an extra part, not simulated), and slot positions at 114.9 degrees on the ring so that the channel runs radially from the plate (it still jammed at friction 0.35 to 0.45 in a quick test, and eight pockets at the current 30-degree pitch do not fit between slots 130 degrees apart: the pitch would have to be 27 degrees, and the drop-sequence check `plate_variants` assumes slots 90 degrees apart).
- **Joint:** the channel slides in from outside through a matching hole in the wall (cut from the channel's own profile, so it is longer along the wall) and past the hopper's socket (snap detent). The exit is trimmed flush with the body radius. The first proposal was to print it in two halves lying flat (trough and cover), no supports; that split has not been redrawn for the open trough **[proposal]**.

### 6.3 Motor bay: N20 baseline, 28BYJ-48 fits

**N20 cartridge (rev 3 motor).** The N20 (10 x 12 x 41.5 mm with encoder, rev 3 section 7) drops in from above through a 10.4 x 12.4 mm pocket in the floor. A 2 mm face plate (two M1.6 screws into the gearbox face, hole spacing **[measure]**) sits in a recess in the floor top so the kit plate rides on a flat floor; two snap tabs on the pocket walls hold it; the shaft stands about 10 mm into the plate hub. To remove it, lift the plate off the shaft and pull the cartridge up. It hangs 4.2 cm below the floor and touches nothing: 10.6 mm to the left drive motor in the 3D model **[check]** `v3_checks3d` (the silver module, 6.1 mm away in that model, has moved to the front, 5.3).

**28BYJ-48 (what you asked to keep room for).** Rev 3 dropped it "as too large". A 28 mm body centred on the plate axis is too large, but the datasheet drawing shows the shaft **8 mm off the body centre**, so the body can be turned to sit beside the axis. With the front face on the floor underside (z 8.7) the body occupies z 6.8 to 8.7. I checked all orientations against the GIGA stack, the battery and the two hoppers **[check]** `stepper_bay`:

| Body offset direction | Result |
|---|---|
| 225 to 270 degrees (body to the front-left or left of the axis, ears across it) | **fits**; widest margin at 230 degrees: body centre (-1.49, +0.61), clearances stack 5.0 mm, battery 8.2 mm, hopper A 19.1 mm, hopper B 4.0 mm (the 22 mm hopper is bigger than the first one). At 270 degrees (centre (-2.0, +0.8)) hopper B is 2.8 mm |
| any other direction | collides with the GIGA stack (4 to 12 mm overlap) or leaves under 2 mm |

So the bay is **kept free** at 230 degrees: nothing may enter a 29 mm circle centred (-1.49, +0.61), z 6.8 to 8.7, plus the two ears (1.75 cm either side of the centre across the offset direction, at (-0.15, -0.51) and (-2.83, +1.74), z 8.0 to 8.7) and the wire block out to 1.7 cm from the centre toward (-0.39, +1.92). The stepper variant needs: a 9.4 mm recess 1.6 mm deep on the floor underside for its 9 mm collar, two M3 screws from below through the ear holes into nut traps in the floor top (flush, under the plate), and a plate hub bored for the 5 mm shaft with flats (the N20 shaft is 3 mm). The dropper floor is printed for one motor or the other; the frame and everything else are common.

**Which one.** Both fit, so rev 3's choice stands for now: the N20 (closed loop, reuses the drive-motor encoder and PID path, torque margin) and the bench test (rev 3 test 7) now compares the N20 with the 28BYJ-48 (reuses the V2 `Stepper` code, no encoder, open loop so a jam loses steps silently, pull-in torque about 30 mN.m, plastic gearbox backlash) and the 15 mm stepper. D10.

### 6.4 Plate and kits

Unchanged from rev 3 section 7: plate R 4.91 at (-2.0, 0), z 9.0 to 10.2, eight 14 mm pockets on a 38.6 mm ring, kits 10.3 mm. The plate rests on the floor (PTFE tape or three printed bumps). With the lid off all eight pockets are visible and open for loading.

## 7. Lid, handle, controls

### 7.1 Lid

A 3 mm plate (z 11.2 to 11.5) resting on the ring, and **four hooks** at about +-71 and +-114 degrees. Each hook is a 1.6 mm skirt segment 8 mm deep (z 10.4 to 11.2) hanging from the rim, about 2.2 cm of arc long (12 degrees), in a rebate in the ring's outer face; a 20 x 8 mm strip of it is freed by an L-shaped slit and anchored along one vertical edge, so it flexes radially in the layer plane, and a bump on its inner face snaps into a groove in the ring. There is no continuous skirt: the ToF windows reach z 10.9 and a full skirt down to z 10.4 would cover them. The angles come from the free gaps between the ToF and camera windows: screws need centres in 9 to 16.5, 56.8 to 78 and 98.2 to 123.2 degrees, hooks 63.8 to 75.2 and 101.2 to 116.2, with 3 mm of material to every window **[check]** `ring_gaps` (indicative: the real window widths are set in CAD). The rear screw sits at 118 degrees, inside the angular span of the hook at 114 but 2.9 mm inside it radially (counterbore r 10.05, rebate r 10.34); 118 rather than 101 keeps the boss off the tyre (section 5.2). A hook of this size deflects 0.75 mm with about 1.5 N and 0.45 % strain; with a steep return face four of them hold the lid with roughly 10 N; target 10 to 20 N, set by test print **[calc, estimate]**. The lid lifts straight up; two finger notches at +-12 degrees on the front rim help to start it. It has the camera humps (outer top 12.3, ceiling 12.1), a handle slot 6.1 x 1.8 cm (x 2.95 to 9.05, around the post and the bar) and, joined to it, a front notch over the control deck (x 4.9 to 9.2, y 0.7 to 3.5). Nothing electrical is in it. Mass about 60 to 100 g **[placeholder]**. Optional: a microswitch on the ring pressed by a lid tab, so the firmware does not start with the lid off (one input).

### 7.2 Handle

Post 14 mm across on the bridge rib at x 4.0 (x 3.3 to 4.7), up to z 14.0; bar 16 mm across and **54 mm long** along x (x 3.3 to 8.7) at z 14.0 to 15.6, printed lying flat and fixed to the post with two M3. Total height 15.6 cm, 16.0 cm with the victim LED on the bar, against the 25 cm limit **[calc]**. The lid's slot passes the post and the bar, so the lid lifts over them. This keeps rev 3's rule that the handle is on the chassis, not on the lid (rev 3 section 0.2 item 10).

**Why the bar is short (found by the dry run).** The first draft's bar ran from x 0 to 8, over the kit plate and the floor disc, whose front edges are at x 2.9 and 3.1. The removal check on the emulator showed that the dropper unit then cannot lift out: its plate meets the bar after 5 cm of lift. A bar that starts at x 3.3, in front of the post's rear face, lets the unit and everything else leave straight up. The price is the carry. The centre of mass is 0.89 cm ahead of the axle and 6.8 cm high, so hanging from the rear end of the bar (x 3.3, grip at z 14.8) the robot points 17 degrees nose-up, 33 degrees from the middle of the bar and 44 degrees from its front end; the long bar allowed a level carry at x 0.9 **[calc]** `handle_calc`. Alternative (D15): keep the 8 cm bar and unscrew it (two M3) before lifting the unit out.

### 7.3 Controls and the USB-C socket

Rev 3 put the start button and the victim LED on the lid. A lid carrying wires cannot lift off freely, and the first rev 4 draft moved them to a panel at the rear of the ring, 21 to 25 cm of wire from the main PCB, whose connectors are at the front. They now sit on a **control deck** on the front bridge:

- **Deck:** the bridge web widens to the +y side from x 5.2 to 9.15 (to y 3.6). Its -y edge stays at the bridge's (y -1.25), because the GIGA's connector edge starts at y -1.36 and a symmetric deck would have covered the reset button at its inner corner (PB1, about (7.1, -1.5), read from the datasheet picture); found by checking the dry run's access numbers against that picture. With this deck the button is 3 mm outside the deck's edge: a pen reaches it from above.
- **Parts (sizes are placeholders):** power switch (the BOM's circuit breaker or a 12 V rocker, 1.3 x 1.9 cm) at (6.0, 2.15); start button (1.2 cm across) at (7.55, 2.15), both clear of the bar's shadow; two status LEDs in a row on top of the bridge rib at x 7.5 and 6.6, showing through the handle slot.
- **Victim LED (rules 4.2: 'clearly visible to the referee', 500 ms on and off for 5 s):** a 5 mm LED on top of the front end of the handle bar (x 8.35, z 16.0, the highest point of the robot), wires down the post. The first draft had it on the deck; a LED there sits 4 cm under the bar and 1.3 cm below the lid top, so it is hidden from straight above and from the sides below about 40 degrees of elevation, which is not 'clearly visible'. Alternatives (D19): a light pipe through the lid, or the LED on the deck.
- **Lid:** a front notch (x 4.9 to 9.2, y 0.7 to 3.5) joined to the handle slot opens the deck; nothing electrical is in the lid.
- **Wires:** 5 to 8 cm from the deck parts to J12 and the shield's connector strip, 12 cm for the LED on the bar, against 21 to 25 cm from the rear panel **[calc]** `wire_runs`.
- **USB-C service socket:** a panel-mount part in the **front-right wall at -24 degrees**, above the bumper band (z 7.1 to 7.9, 6 mm above the band; front face flush with the wall at r 10.45, so the body stays inside the 10.5 cylinder; 13 mm deep, 8.3 mm from the boss and rib at -12 degrees), 2.0 cm from J12. Reflashing needs no lid removal **[calc]** `wire_runs`, plan test.

If the breaker is wider than 19 mm the deck grows toward the front ring and the notch follows. Alternative if you want the controls on the lid as in rev 3: a plunger in the lid pressing a switch on the ring, and a light pipe over the LED. D12.

### 7.4 What can be checked with the lid off

Share of each part's upper surface that has nothing above it (a grid of rays straight down onto the part and straight up again, on the emulator of the rebuild plan; the GIGA and its shield count as one stack) **[calc]** `access`, and what a hand can do:

| Item | Lid off | Lid and dropper unit off | Reachable |
|---|---|---|---|
| Kits and plate | all | (in the unit) | lift off |
| GIGA | 55 % | **86 %** | connector edge open from above: J12, USB-A, audio jack, LEDs, and the reset button (PB1) with a pen. The boot button (PB2, outer corner at about (7.1, -6.4), r 9.6) is **under the ring**, and Camera R's board hangs over the outboard strip (about 12 % of the board): both need the frame off. Double-tapping reset brings up the Arduino bootloader for uploads without PB2; PB2 plus reset is only the recovery route when the bootloader itself is gone **[src]** OpenMV and Arduino support pages, Sources |
| Shield | 53 % | **86 %** | yes |
| Battery and its connector | 77 % | **91 %** | leaves by the path in 3.3 |
| ToF boards, cameras | all | all | lift out |
| Silver module | 49 % | 49 % | to its pocket from above with a tool; replace with the frame off |
| Left drive motor | 0 % | 54 % | after the battery is out |
| Right drive motor, floor port, omni | no | no (right motor 0 %) | frame off, the GIGA stack off first for the right motor |

**PCB rule for the main board:** connectors, LEDs and jumpers go on the front edge or on the outboard strip (y below -5.1), which the floor disc does not cover; and the shield starts about 20 mm from the GIGA's connector edge so that J12, the USB-A, the audio jack, PB1 and PB2 stay uncovered (check against the datasheet picture).

## 8. Electronics mounting

### 8.1 GIGA and shield

Four orientations against the parts around the six holes **[check]** `mount_check` ([results](v3-checks/results/mount_check.txt)); "usable" means a hole with no conflict when the omni arm is on +y only:

| Orientation | Usable holes | Where they are | Verdict |
|---|---|---|---|
| connector edge to the **front**, header edge inboard | H1 H2 H3 H4 (H1 only with the single arm; all four at least 1.5 mm clear of the right motor's cradle) | the four corners of the board, x -2.3 to 6.0 | **chosen**; connectors reachable from above at the front |
| connector front, header outboard | H1 H3 H4 | three corners | fewer holes |
| connector rear, header inboard | H2 H4 H5 H6 | all in x 3.9 to 6.9, the front 3 cm of a 10 cm board | rear half unsupported; connectors under the dropper floor |
| connector rear, header outboard | H2 H3 H4 H5 H6 (H3 is 0.2 mm from the right motor) | x -1.4 to 6.9 | connectors face the right-hand hopper and chute at the rear |

With the front orientation the holes land at:

| Hole (u, v from the connector and header edges, mm) | Robot frame (x, y cm) | Status |
|---|---|---|
| H1 (15.24, 2.54) | (5.87, -1.62) | usable with the single arm (3.5 mm inside a two-plate swing) |
| H2 (90.17, 2.54) | (-1.62, -1.62) | clear; keep the right motor's encoder pads away from it |
| H3 (13.9, 50.7) | (6.01, -6.43) | clear |
| H4 (96.7, 50.7) | (-2.27, -6.43) | clear, 3.2 mm from the right cradle's web (the plate has no rear ear for this reason) |
| H5 (66.1, 17.8) | (0.79, -3.14) | over the right motor, unused |
| H6 (66.1, 45.6) | (0.79, -5.92) | over the right motor, unused |

Posts: 7 mm bosses with M3 inserts from the floor (z 3.9) to the GIGA underside (z 6.15), 2.25 cm tall; check header solder tails against the floor modules when the model is rebuilt. Screw the GIGA down first, then plug the shield into the headers. The stack's outer front corner is 3.1 mm from the plain wall **[check]** `packing`, and 1.7 mm from the thickened wall behind the right bumper plate in the first 3D model ([v3-fusion README](v3-fusion/README.md)).

**USB-C and buttons.** J12 sits about 13.8 mm from the header edge on the connector edge (read from the datasheet picture), so at about (7.40, -2.74). In front of it there is 2.5 cm to the tub wall but only 1.2 cm to the ring above z 8.7, so a straight USB-C plug does not fit with its cable: use a right-angle plug at J12 with the cable to the front-right wall socket, 2.0 cm away (7.3). The reset button at the inner corner of that edge is reachable from above beside the control deck; the boot button at the outer corner is under the ring (7.4).

### 8.2 Battery

Tray at z 4.9 to 5.1 with a front stop, two strap slots for a 10 mm velcro strap, and the connector lead at the +x end where a hand reaches it. The placeholder 7.0 x 3.5 x 2.5 cm battery overlaps the left motor's footprint by up to 0.9 cm and sits 1.0 mm above it, so the tray has a notch over the motor and a 1 mm foam pad on the motor top. Battery model not chosen. **[placeholder]**

### 8.3 ToF boards and cables

Positions as in the 3D model: F (9.5, 0), FL/FR (8.97, +-4.18), SFL/SFR (7.18, +-6.0), SRL/SRR (-7.18, +-6.0), RL/RR (-8.4, +-4.5). Each board (21 mm tall, 18 mm wide, four holes, two STEMMA QT connectors **[src]**) drops into a pocket from above and is held by two snap tabs (vertical strips anchored along one edge, so they flex in the layer plane); the cable leaves through a 6 x 3 mm notch under the ring and drops to the PCB. Which edges carry the connectors is not on the page; if they are on the 18 mm edges the pockets widen by about 6 mm each side. **[measure]**

| Sensor | Plan distance to the stack | Routed run (x1.3 + 3 cm) |
|---|---|---|
| F | 2.5 | 6.3 |
| FL | 5.8 | 10.5 |
| FR | 1.6 | 5.0 |
| SFL | 7.4 | 12.6 |
| SFR | 0.0 | 3.0 |
| SRL | 8.6 | 14.2 |
| SRR | 4.4 | 8.8 |
| RL | 8.1 | 13.6 |
| RR | 5.6 | 10.3 |

100 mm cables reach only 4 of 9; **200 mm cables for all**, which also lets the frame lie beside the tub with everything plugged in. Every cable from the frame (ToF, cameras, the N20 motor, the control deck) is at least 20 cm **[check]** `mount_check` (the routing factor is a guess).

**Effect of moving the modules inward** (first 3D model, finding 1, not yet applied to rev 3 or `v3_params.py`): at the rev 3 positions the four side modules stuck out of the shell by up to 6 mm. With these positions all corners are inside r 10.23 and the readings in a 28 cm path become side 80 mm (was 67), toed-out 55.5 (51.5), rear 56 (47), front centre 45; the worst-case side reading is 46 mm instead of 33, clear of the VL53L0X minimum range. Firmware: `TOF_SIDE_OUT_MM` 72.8 to 60.0 and `TARGET_SIDE_GAP_MM` 67.2 to 80.0.

### 8.4 Cameras

OpenMV H7 Plus, 45 x 36 mm board with a lens block 2.3 deep x 2.0 x 2.0 cm, tilted 20 degrees down, in a slide-in cradle in the ring held by two snap tabs; M2 screws are possible once the hole spacing is known **[measure]**. The wall window is 2.8 x 2.4 cm (3D rays, v3-fusion README finding 2); in the tub it continues down through the notch in the wall strip above the wheel arch (section 5.2). The UART and power cables run up the ring and down into the tub.

### 8.5 Wi-Fi and Bluetooth

The GIGA R1 WiFi carries a Murata 1DX module: Wi-Fi 802.11 b/g/n (65 Mbps) as access point, station or both at once, and Bluetooth Low Energy **[src]** datasheet ABX00063 (sections 2 and 8). There is no on-board antenna. The module uses the u.FL socket J14, which sits at the front-inner corner of the board beside the reset button (about (7.2, -1.9), read from the datasheet picture, +-1 mm), and a flat flex antenna comes in the box (the datasheet lists "Micro UFL antenna (Included)"; its ISED section names a Molex 206994-0100 FPC antenna; a shop listing gives about 15.4 x 6.4 mm on a 100 mm cable **[src, secondary]** [Molex](https://www.molex.com/en-us/products/part-detail/2069940100)).

- **Antenna position [proposal]:** stuck on the inside of the tub's front wall at -6 degrees (x 10.2, y -1.1), strip vertical at z 5.5 to 7.0: plastic all round, at least 1.5 mm from the boss rib at -12 degrees, at least 5 mm from the omni wheel even fully compressed, and about 3 cm from J14, so the routed run (about 7 cm) fits the 100 mm cable **[calc]** unit test `test_wifi_antenna_sits_on_the_front_wall...`. It stays with the tub, so taking the frame off does not unplug it. Not RF-tested: before the final print, check the range with the robot assembled (laptop and phone at 2 m and 5 m).
- **Use [proposal]:** a **test mode** chosen at power-up (for example the start button held while the power switch is turned on, with an LED showing that the radio is on), in which the firmware starts the radio, streams its log to a laptop (TCP, or Bluetooth serial for a phone) and accepts test commands. In the normal mode the radio is never started. **Rules 4.1.1:** "passing information ... wirelessly ... to the robot is not allowed" during a run, so the competition build receives nothing and the referee can be shown that. Do the final validation runs in the normal mode: the Wi-Fi stack changes timing.
- **Flashing:** over the USB-C socket in the wall, lid on. Wireless upload is possible with the `Arduino_Portenta_OTA` library but needs a one-time partitioning of the board's flash and has rough edges in the forums; the usual `ArduinoOTA` is reported not to support the GIGA. Treat it as an experiment **[src]** [Arduino support](https://support.arduino.cc/hc/en-us/articles/12370721200540-Configure-GIGA-R1-WiFi-Portenta-H7-and-Portenta-Machine-Control-for-Over-The-Air-OTA-uploads), [forum](https://forum.arduino.cc/t/arduinoota-on-giga-r1/1348426).
- **Venue:** 2.4 GHz is crowded at a competition; test mode is for the pit and the practice field.

## 9. Bumpers

Unchanged from rev 3 section 6: two plates 12 to 58 degrees either side of the centre line, hinged at the outer ends, return spring, 4 mm travel, a microswitch at the inner end of each (band z 4.0 to 6.5, swept radius 11.0). The hinge pin sits in a wall socket; the microswitch in a pocket in the wall at 14 degrees, below the frame boss at 12 degrees. A flexure plate fixed at one end could replace the hinge and spring, but a contact near the fixed end would need several newton, so I do not propose it unless the hinge fails. The omni no longer reaches the swept circle (section 4).

## 10. Printing and fits

| Part | Orientation | Supports | Notes |
|---|---|---|---|
| Tub | floor down | under the rear chamfer and the arches | motor prongs are vertical walls that bend sideways, along the layers |
| Upper frame | ring bottom (z 8.7 plane) down | camera cradle slots, hook grooves | floor disc, spokes and bridge all lie in that plane |
| Lid | top down | none | hook tongues are vertical strips that bend radially, along the layers |
| Chute channel | two halves flat | none | glue or snap; hopper separate |
| Cartridge plates, handle bar | flat | none | |

- **Layer rule:** FDM parts are weakest between layers. Every snap feature is a wall or strip that bends in the print plane; nothing that stands up from the bed bends toward you. Test prints confirm each.
- **Fits:** insert holes 4.0 mm across and 6 mm deep for M3 x 5.7 inserts. Sliding fits 0.2 to 0.3 mm. Plate slot 3.4 to 4.0 mm for the 3.0 mm plate.
- **Size:** a 21 cm part on a 220 mm bed leaves 5 mm each side, and a large PETG ring may warp (brim, enclosure). If the bed is smaller than about 215 mm, the tub and the frame split along y = 0 with a 3 mm tongue and groove and four M3 across the joint; the ring stays continuous across the frame screws. **[measure]**

## 11. Feasibility and risk summary

| Design | Verdict | Main risk | What settles it |
|---|---|---|---|
| Omni inside the body | feasible **[check]** | tilt 17 degrees down a riser; spring 14 % stiffer; Dangerous Zone corner case still fails; a single 4 mm PETG arm twists about 1.5 degrees at 5 N **[calc, rough]**; the 4 mm pivot pin reaches 1 mm below the belly line | stair test; spring source; a deeper or metal arm if the wheel leans; a 3 mm pin or a sunk pin |
| Upper frame as one print | feasible if the bed is at least about 215 mm | warping of a 21 cm ring; bed size unknown | measure the bed; split fallback |
| Frame bolted down, handle carries the robot | feasible | six PETG insert pull-outs | hang 5 kg from the bar |
| Lift-out dropper unit on three half-lap seats | feasible **[calc]** | the snap tabs or thumbscrews are not designed; wear of the rebates; the N20 connector; the 5 cm cantilever of the bridge under the handle post (2.0 MPa at 15 N) | 100 lift-outs, press test, 5 kg hang |
| Hopper and square channel | feasible in simulation **[check]** if the plate parks within 1 mm (the first design jammed it, 8 Oct; redesigned) | `chute_dynamics` with the plate's pocket walls: 100 % of 1000 kits out with the plate parked within 1 mm and with bouncy impacts, 82.8 % up to 2 mm off (the slot leaves (16 - 14) / 2 = 1.0 mm over a kit corner to corner in its pocket), 4 of 30 when the plate turns onto the slot at 60 degrees/s at friction 0.55; the bore has 0.16 mm over the kit's space diagonal (10.5 mm kits: 98.3 % with bouncy impacts); friction, kit mass and impact softness are assumed; joint leaks | the chute test with 20 cubes at 32 degrees (rev 3 test 3), the plate's parking repeatability from both sides, a printed bore against 10.5 mm kits; then repeat `chute_dynamics` with the measured friction and bounce |
| N20 cartridge | feasible | face-hole spacing unknown | measure the motor |
| 28BYJ-48 variant | feasible **[check]** | 5.0 mm to the GIGA stack and 4.0 mm to hopper B; open-loop jam | bench test 7 |
| Motor snap cradle | feasible with a key | hooks hold only 1 to 6 N; strain near 1 %; the left motor is under the battery and the right one under the GIGA stack, so a motor swap takes 15 to 25 min **[estimate]** | test prints, detent or thumbscrew fallback |
| Wheel arches | feasible | wall stiffness (the strip above each arch is also notched 3 cm at the camera), a snag edge | print and handle it; snap-in cover |
| Snap-on lid | feasible | hook wear, retention near 10 N | 100-cycle test |
| T-handle, 5.4 cm bar | feasible | nose-up carry: 17 degrees from the rear end of the bar, 33 from its middle | D15: the long bar, removable |
| GIGA on four posts | feasible **[check]** | USB-C access; the boot button is under the ring | right-angle plug and the front-right socket; double-tap reset |
| Battery tray with notch | feasible | battery model and size open | choose the battery |
| ToF pockets, cameras | feasible | connector edges and camera holes unknown | measure the boards |
| Control deck and the USB-C socket in the front-right wall | feasible **[calc]** | breaker size open; the socket is 6 mm above the bumper band and 8.3 mm from the boss rib | choose the parts |
| Wi-Fi and Bluetooth test mode | feasible; the module is on the GIGA | antenna range inside the body untested; the radio must be off in a run (rules 4.1.1); wireless flashing unreliable | range test; test-mode switch; flash over USB-C |
| Battery swap path | feasible for the placeholder battery **[calc]** | Camera L's board, the ring corner and the power switch are over the battery, so it leaves 4.5 cm back and 1.5 cm or more inboard; another battery needs the path re-checked | choose the battery, re-run `removal` |
| Floor sensors at x 7.5 | feasible **[check]** | 0.5 cm clearance on the 30 degree stairs; silver is read 75 mm earlier, so the firmware threshold changes | `fsens_front`, firmware |
| 2WD with 80 mm wheels on a 2 cm step | sound on paper, the one real risk | the friction a rigid wheel needs for a half-radius step is 1.73; the tyre material decides | stair test with the real tyres (rev 3 test 1); plan B in the pre-Fusion report |
| Bumpers | unchanged from rev 3 | none new | none |

## 12. Verification

**Done:** omni terrain cases `omni_inside` (section 4) and its statics `statics_omni_7`; GIGA holes, USB-C space, ToF cable runs, motor prong and plate numbers, cube slack `mount_check`; chute exit lowest point `chute_exit` and kit landing `kit_landing_square`; the kit chute in a rigid-body simulation `chute_dynamics` and its geometry against the Fusion bodies `chute_geometry_check`, which cube attitudes fit a square slot `cube_slot` (section 6.2); 28BYJ-48 bay `stepper_bay`; ring room for screws and hooks `ring_gaps`; Figures 7 to 9 `structure_figs`; floor-sensor clearance at x 0 to 8.5 `fsens_front`; the terrain cases for an axle 0 to 3 cm back `axle_shift`; the handle numbers `handle_calc`.

**Dry run (7 Oct):** the rebuild plan's own code (parameters with 38 unit tests, 19 build stages with about 170 probes and an overlap check each, and the whole-model reports) was run on a geometry emulator before any Fusion call ([v3-fusion/dryrun](v3-fusion/dryrun/README.md)): all 51 components without a single overlap, the seven removal paths free, no ToF cone or camera ray blocked, the access report and the 14 spec clearances (13 before the Wi-Fi antenna was added) within their limits, mass and balance as in section 4. It cannot see Fusion API behaviour, so every number is re-measured in Fusion after the build and compared with the emulator's prediction (plan, Task 9). After the chute redesign (8 Oct) the repo files were dry-run the same way (`dryrun.py --repo --clearances`, whose output is now `dryrun/results/dryrun_full.txt`): 44 unit tests, 19 stages with 200 probe and overlap rows, 51 components without an overlap, seven removal paths free, no ToF cone or camera ray blocked, 14 clearances within their limits, mass 1197 g; the plan's own code blocks keep the first chute design.

**In the rebuilt 3D model (run on 8 Oct, results below):** interference between all parts; nearest-distance list; assembly and removal paths (lid lifting over the handle, the dropper unit lifting out, the battery path, frame lowering onto the tub, motor cartridge drop-in, wheel out through the arch, chute channel in from outside, GIGA, battery and N20 out); access from above; swept radius and height; ToF cones and camera views with the new ring and the real window sizes; omni sweep with the single arm and the bridge; centre of mass from printed-part masses; the stepper variant.

**Done in the rebuilt model (8 Oct, all of it run again after the chute redesign; raw output in [v3-fusion/results/](v3-fusion/results/)):** every stage probe and all eleven reports passed, and they agree with the emulator's prediction. 51 components without an overlap (Fusion's own body-by-body analysis agrees: ten overlaps, all of them joints of flexing prongs and hook bumps with the tub and lid, or the bumper plates touching their switches, `v3-fusion/results/v4_interference_native.txt`; sixteen parts touch nothing, the ToF boards, cameras, floor sensors, USB-C socket, nub and antenna, whose holders are details left to CAD, `results/v4_loose_parts.txt`), seven removal paths free (the lid hooks and cradle prongs treated as flexing), no ToF cone or camera ray blocked, the omni sweep free at 13 positions, 14 spec clearances above their limits, the stepper bay free. A 3D kit-path check added afterwards (`v3-fusion/kit_path.py`, `results/v4_kit_path.txt`) lets a 10.3 mm cube fall from its pocket through slot and hopper and slide out of both channels without touching a part. It first passed the 14.5 mm design, which a physics simulation then showed to jam kits that arrive square to the slot (section 6.2); the chute was redesigned (16 mm slot and hopper void, 18 mm channel, open trough) and rebuilt in Fusion, and the simulation, with the plate's pocket walls in it, takes 100 % of 1000 kits out with the plate parked within 1 mm and with bouncy impacts and 82.8 % with it up to 2 mm off: the dropper has to park within 1 mm (`v3-checks/results/chute_dynamics.txt`; friction, kit mass and impact softness are assumptions, so the chute bench test still decides). What was not a plain pass: the belly report lists twelve known parts below z 3.5, among them the 4 mm omni pivot pin 1.0 mm under the line (section 4 leaves a 3 mm or sunk pin to CAD); the GIGA's BOOT0 button is under the ring (section 7.4, known); the battery leaves only along the 4.5 cm back and 2 cm inboard path; two clearances are larger in 3D than quoted (N20 to the left motor 10.86 mm against 10.6, stepper bay to the GIGA stack 5.76 against 5.0 in plan view) and two are smaller than the numbers quoted before the redesign, both above their limits (chute to wheel 5.62 mm, the 18 mm channel being wider: rev 3 quoted 6.7 and the first rev 4 chute had 8.24, numbers this file does not repeat; stepper bay to hopper B 3.97 mm, 5.2 with the first hopper, corrected in section 6.3); the mass report gives 1198 g, centre of mass 0.89 cm ahead of the axle and 6.81 cm high, front-lift limit 1.29 m/s^2 (emulator 1197 g and 0.90 cm) **[placeholder masses]**; the highest point is the victim LED on the bar, z 16.0 against the limit 25.

**Independent review of the chute redesign (8 Oct, a fresh reviewer on the working tree; the first reviewer run on 2e1d6e6 had failed on a usage limit):** it found the plate missing from the simulation and 100 poses shared by all friction values (both fixed: the 2 mm result went from 98.1 to 82.8 %), a notch in the bore floor where the slab ended 0.15 cm short of the trough cut (slab 2.5 to 2.8 cm, with a test and floor probes), '16 mm passes the cube in any attitude' (a corner stand needs 16.25 mm: wording, `cube_slot` and the test renamed), a turning-room check in `kit_path.py` that could no longer fail (now a width check), a bore margin thinner than the kit tolerance (tolerance corners, D21), the rebound figure (0.45, not the textbook 0.37), the place of a pad (the slab, not the hopper's floor strip), the kit mass (6 g in the mass budget: tried, no effect) and stale numbers (`plate_tolerance` for the 16 mm slot).

**Bench:** three motor cradles; the chute channel with 20 cubes at 32 degrees; the plate parked 0.5, 1 and 2 mm off with kits at several turns, and the plate's parking repeatability from both sides; a printed bore against 10.5 mm kits; the kit's slide-off angle on printed PETG along the layer direction (friction) and its rebound; a weighted kit with an off-centre centre of mass; the seam of the two-half channel and the step where the hopper meets the floor slab; lid hooks; hopper seal; battery tray fit; wheel hub on the D-shaft; 5 kg hang from the handle; frame screw pull-out in PETG; N20 against 28BYJ-48 (rev 3 test 7).

## 13. Decisions and open items

New decisions, delegated to this review on 7 Oct (alternatives that also work in brackets):

| # | Decision | Default |
|---|---|---|
| D9 | Omni centre x | 7.0 (7.3 and 6.5 also pass) |
| D10 | Dropper motor | **N20, confirmed by you on 7 Oct** (rev 3 motor); the 28BYJ-48 fits and its bay is kept free in case the bench test (rev 3 test 7) disagrees |
| D11 | Omni arm | single, on +y |
| D12 | Lid, handle, controls | snap lid with four tongue hooks, T-handle on the bridge, controls moved off the lid onto a control deck on the front bridge and the USB-C socket into the front-right wall (**asked for by you on 7 Oct, second round; layout approved**; alternative not taken: plunger and light pipe through the lid) |
| D13 | Wheel arches | yes |
| D14 | Chute channel | 18 mm square bore (21.2 mm outside) centred 1 mm above the axis, open trough under a 16 mm slot, exit axis z 4.15 (redesigned 8 Oct, section 6.2; it was a 13 mm square) |
| D15 | Handle bar | 5.4 cm bar from x 3.3 so that the dropper unit lifts out past it (carried 17 to 33 degrees nose-up) (**confirmed by you on 8 Oct**); alternative: the 8 cm bar, unscrewed (two M3) before the unit comes out |
| D16 | Axle position | at the body centre (1 cm back adds 7.7 points of front load and 1 cm of swept radius; nothing needs it) |
| D17 | Dropper unit | floor, plate, kits, N20 and hoppers lift out as one unit on three half-lap seats; snap tabs or thumbscrews to be designed |
| D18 | Floor sensors | silver module and front port both at x 7.5 beside the omni bay; the firmware `overNextTile` threshold becomes TILE_MM/2 - 75 mm |
| D19 | Victim LED | on top of the handle bar's front end (rules: clearly visible to the referee) (**confirmed by you on 8 Oct**); alternatives: a light pipe through the lid from a LED on the deck, or the LED on the deck (hidden from straight above) |
| D20 | Wi-Fi and Bluetooth | the GIGA's built-in module in a test mode only (log streaming and test commands); antenna stuck on the front wall; flashing stays on USB-C; the radio is never started in a run |
| D21 | Channel bore | 18 mm (0.16 mm over the cube's space diagonal) as built; **19 mm if the kits can be 10.5 mm or the printer shrinks holes**: costs 0.5 mm of the 5.6 mm chute-to-wheel gap (limit 5.0) and needs the model and `chute_dynamics` run again (**open, your call**) |
| D22 | Dropper parking | plate parked within +-1 mm (aim +-0.5 mm), approached from one side, final approach at 120 degrees/s or more (**requirement from the simulation, to be confirmed on the bench**); alternatives if the plate cannot: 13 mm pockets (rev 3 D6, allowance 1.5 mm) or a 16.5 mm slot (1.25 mm) |

Chute (section 6.2): the first design jammed kits in the simulation (8 Oct) and was redesigned; the redesign passes it for a plate parked within 1 mm (82.8 % at up to 2 mm: D22). What remains is the bench test with real kits: friction, bounce, the plate's parking, the bore against 10.5 mm kits (D21).

Measurements needed from the real parts **[measure]**: Pololu encoder board outline and the 20D face hole spacing; N20 face hole spacing; OpenMV mounting-hole spacing; which edges of the VL53L0X board carry the connectors and where its holes are; printer bed size; battery model (the removal path of 3.3 depends on it); power switch size; the GIGA's reset and boot button and J14 antenna socket positions (read from the datasheet picture, +-1 mm); the antenna's range inside the body. Also open: where to source the torsion spring; the real ring window widths (set in CAD); the snap tabs or thumbscrews that hold the dropper unit; whether to add a charge port beside the USB-C socket so the battery need not come out to charge (not designed). Rev 3's open items (stair test, silver bench test, chute test with the new geometry, D1 to D8) are unchanged.

## 14. Out of scope

Tracked drive, 4WD, mid-run reloading, changes to the mapping algorithm, the PCB layout itself (only its rules, section 7.4), wiring harness drawings, a custom omni wheel design.

## Sources

[Arduino GIGA R1 WiFi datasheet ABX00063](https://docs.arduino.cc/resources/datasheets/ABX00063-datasheet.pdf) (pages 9 and 18); [Pololu 3493](https://www.pololu.com/product/3493) and [3499](https://www.pololu.com/product/3499); [Adafruit 3317](https://www.adafruit.com/product/3317) and [858](https://www.adafruit.com/product/858); [Kiatronics 28BYJ-48 datasheet](https://pdf.direnc.net/upload/28byj-48-reduktorlu-step-motor-datasheet.pdf) (dimension drawing, page 1); [OpenMV H7 Plus](https://openmv.io/products/openmv-cam-h7-plus); [Molex 206994-0100 antenna](https://www.molex.com/en-us/products/part-detail/2069940100); [OpenMV page for the GIGA R1 WiFi](https://docs.openmv.io/openmvcam/quickref/arduino-giga-r1-wifi.html) and the [Arduino support article on the flashing red LED](https://support.arduino.cc/hc/en-us/articles/7991505987612-If-an-LED-on-GIGA-R1-WiFi-is-flashing-red) (double-tap reset, BOOT0); the rev 3 spec; the scripts in [v3-checks/](v3-checks/README.md) (`omni_inside.py`, `statics.py`, `kit_final.py`, `chute_exit.py`, `mount_check.py`, `stepper_bay.py`, `ring_gaps.py`, `structure_figs.py`) and the 3D checks in [v3-fusion/](v3-fusion/README.md).
