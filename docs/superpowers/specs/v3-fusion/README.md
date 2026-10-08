# Theseus V3 in Autodesk Fusion

A 3D layout model of the V3 robot, built by script so it can be regenerated and checked. **Revision 4** (the structure, mounting and usability redesign in [2026-10-07-theseus-v3-mechanical-design.md](../2026-10-07-theseus-v3-mechanical-design.md)) is the current model. The revision 3 layout model it replaced is kept as `Theseus_V3_baseline.f3d` and `v3_model.py` (its README is [README_rev3.md](README_rev3.md)). All rev 4 numbers come from [v3_params4.py](../v3-checks/v3_params4.py), which is unit-tested by `test_v3_params4.py`. Units cm, frame x forward, y left, z up, origin at the midpoint of the drive axle on the floor line (Fusion is set to Z up here).

**Status:** layout model. Every part sits at its specified position and every interface the spec defines is modelled. Nothing has been measured or printed; masses and the PCB are placeholders as in the spec. It does not replace the CAD detail design (see "Not modelled").

## Files

| File | What it is |
|---|---|
| `fusion_lib.py` | Helpers: sketch shapes, extrude with offset start, cuts, `axis_prism` along a sloped axis, colours |
| `v4_model.py` | The rev 4 model: one function per stage, registered in build order (`STAGES`), `build()` |
| `v4_checks3d.py` | Stage probes, local interference, whole-model reports (interference, envelope, belly line, removal paths, access from above, mass and centre of mass, spec clearances, stepper bay) |
| `v4_run.py` | Entry points for the Fusion script runner: `stage`, `report`, `selftest`, `show` |
| `kit_path.py` | An extra functional check: a 10.3 mm cube falls from its plate pocket through slot and hopper and slides down each channel to the outside (static geometry, temporary B-reps). Not part of the plan's code, not in the emulator; `kit_path.report()` from a Fusion script, about 15 s |
| `native_interference.py` | Two extra checks with Fusion's own analysis (`design.analyzeInterference`), about 5 s each: `report()` (every body against every other) and `report_loose()` (which parts touch nothing). Not part of the plan's code and not in the emulator; call them from a Fusion script |
| `v3_checks3d.py` | Read-only 3D checks shared with rev 3: ToF cones, camera fields of view, omni sweep |
| `v3_model.py`, `v3-fusion.py`, `v3-fusion.manifest`, `Theseus_V3_baseline.f3d` | The rev 3 layout model, its entry point and its export; rev 4 reuses its wheel, motor, nub, bumper, camera, GIGA stack, battery and floor-sensor builders |
| `README_rev3.md` | The README of the rev 3 model, with its four findings against the rev 3 spec (kept because these files are not under version control) |
| `Theseus_V3_rev4.f3d` | Export of the finished rev 4 model (3.8 MB, 8 Oct; safety copy; the Fusion document itself is unsaved) |
| `results/v4_*.txt` | Raw output of each whole-model report on the finished model |
| `dryrun/` | The same build and reports on a geometry emulator, no Fusion needed (about 4 minutes): predicted the Fusion results and found a dozen problems in the plan and the design before the first Fusion call. Its README says what it can and cannot see |
| `views/rev4_*.png` | Screenshots of rev 4: top (lid and handle hidden), iso, side, underside. `views/fusion_*.png` are the rev 3 views |

## Rebuild

With the Fusion MCP tools: one call of `fusion_mcp_execute` (`featureType: "script"`) per step, with this script (change the last line):

```python
import sys
sys.path.insert(0, r"<repo>\docs\superpowers\specs\v3-fusion")
import v4_run


def run(context):
    print(v4_run.stage('all', reset=True))
```

`reset=True` replaces every component in the open design (it also switches a Part design to hybrid). A whole build is about two to four minutes: build in groups (`['tub']`, `['drive', 'cartridges', 'omni', 'nub', 'bumpers']`, ...) if a call times out. `v4_run.stage(names, build=False)` only runs the probes. `v4_run.report('interference')` (and `envelope`, `belly`, `removal`, `access`, `omni`, `tof`, `camera`, `clearances`, `stepper`, `mass`) runs one whole-model report. `v4_run.selftest()` checks the probe API on a throw-away part.

Running it through the MCP, three things to know:

- **A tool call is cut off after about 60 s.** The Fusion server then answers "unavailable" until the script has finished, and the model is intact afterwards. The `removal` report is the slow one: run it as `report('removal', only=[0, 1, 2, 3])`, `only=[4]`, `only=[5]`, and the teardown as `only=[6], window=(group, 1, 10**6)` for each of its 12 groups (`results/v4_removal.txt` was compiled that way and says so in its header). `window` vouches only for the steps it names.
- **Screenshots: set the camera by script.** The preset view directions of the screenshot tool, and `adsk.core.ViewOrientations`, give a rolled view in this document. Set `cam.target`, `cam.eye` and `cam.upVector` on `vp.camera`, call `vp.fit()`, then take the screenshot with `direction: "current"` (side view: eye (0, -300, 8), up (0, 0, 1); underside: eye (0, 0, -300), up (0, -1, 0); target (0, 0, 8)).
- **Do not save or close the document from a script.** The export `Theseus_V3_rev4.f3d` is a copy; whether the Fusion document goes into a project is the user's decision.

## What is modelled

Tub: floor 4 mm, wall to z 8.7, rear chamfer, two wheel openings that run out through arches (the strip of wall above each arch is notched at the camera window), two motor cradles (seat, shaft notch, plate slot, web, ledges, and two prongs each with the hook shape, as separate bodies because they flex), omni bay and arm slot, floor-sensor holes, square chute holes, bumper recesses with switch pockets, six insert bosses with ribs to the wall (at +-12, +-60 and +-118 degrees), battery tray on three legs with the notch over the left motor, USB-C hole in the front-right wall above the bumper band. Two drive wheels and Pololu motors, two printed face plates with one front ear, the 60 mm omni at x 7.0 on one arm on the +y side, rear nub, two bumper plates with microswitches. Upper frame in one piece: ring with nine ToF pockets and beam windows and two camera windows, front bridge (3 mm web with a rib, widened to a control deck at x 5.2 to 9.15) and rear spoke with half-lap seats for the dropper floor, handle post, six screw holes, four hook rebates and grooves. Nine VL53L0X boards, two OpenMV boards, the lid (plate, camera humps, handle slot, front notch over the control deck, finger notches, four skirt segments with hook bumps), the handle bar with the victim LED on top of its front end, the four control parts on the deck and the rib (power switch, start button, two status LEDs) and the USB-C socket in the wall. The lift-out dropper unit: the dropper floor (its own component, with slots A and B and the N20 pocket), kit plate with eight 14 mm pockets, eight kits, N20 cartridge in its floor pocket with its face plate, two hoppers with sockets; and two 13 mm square channels. Four GIGA posts, GIGA R1 plus shield stack with the USB-C J12 body, battery, floor sensors FP and SM (7.5 cm ahead of the axle), the Wi-Fi antenna stuck on the inside of the front wall. A hidden keep-out for the 28BYJ-48 bay.

## Not modelled

Heat-set inserts and screws, snap-tab and L-slit details (the lid hook tongues; the hook bumps are separate bodies), flexing of the prongs and hooks (their forces are calculations, see the spec; the removal report ignores both), the omni pivot ears, lugs and springs, bumper hinge sockets, hinges and return springs, the nub boss, camera and ToF cradles with their snap tabs and the cable notches, hopper snap tabs, battery strap slots, cables and connectors, fillets and rounded channel corners, the plate hub, the encoder board outline, the ToF connector edges, the main PCB (a placeholder block). Part sizes for the power switch, start button, LEDs and USB-C socket are placeholders; the snap tabs or thumbscrews that hold the dropper unit on its seats are not drawn.

## Checks

| Report | Pass criterion |
|---|---|
| per-stage probes | material (or none) at the listed points of each component; no overlap of the new components with any other |
| `interference` | no pair of components overlaps (hidden stepper bay excluded) |
| `native_interference.report()` | Fusion's own analysis of every body against every other: only the expected overlaps (prong and hook-bump roots, bumper plates touching their switches), each under its limit |
| `native_interference.report_loose()` | the parts that touch no other part are exactly the known ones (pocket and hole parts with clearance, and parts whose holder is not modelled), with the distance to their nearest neighbour; a support that gets lost shows up as an unknown part |
| `envelope` | nothing beyond r 10.5 except the bumper plates (11.0) and their switches behind them; highest point 16.0, the victim LED on the bar (limit 25); omni front edge inside the wall |
| `belly` | only the known parts hang below z 3.5 |
| `removal` | lid, wheels, kit plate and N20, channels, the dropper unit, battery, frame group and GIGA leave along their paths without touching anything |
| `access` | with the lid and the dropper unit off, the GIGA, shield and battery are at least 70, 70 and 80 % open from above |
| `omni`, `tof`, `camera` | omni sweep without interference; no ToF or camera ray blocked by a robot part |
| `clearances` | the gaps the spec quotes are at least their smallest acceptable value |
| `kit_path.report()` | the kit cube reaches the trough floor and slides out of both channels without touching a part; the two controls (cubes too big for the slot and the bore) are blocked |
| `stepper` | the 28BYJ-48 bay is free once the N20 cartridge is out |
| `mass` | centre of mass and front-lift limit (warns below 1.0 m/s^2) |

## Findings of the rev 4 build

The whole build and the eleven reports were run in Fusion on 8 Oct (`results/`); every probe and every report passed, and each number matched the emulator's prediction within the tolerances the plan set (distances 0.3 mm, volumes 2 %, centre of mass 0.05 cm, access 5 points; the largest differences: N20 to the left motor 10.86 against 10.9 to 11.0 mm, mass 1197 against 1195 to 1196 g). Nothing needed an entry in the `ALLOW` table of `v4_checks3d.py` (it is empty: no pair of the 51 components overlaps above 0.001 cm3). What is not a plain pass:

- **A second interference check, Fusion's own analysis, finds the same and a little more** (`native_interference.py`, `results/v4_interference_native.txt`; added after the plan's reports, because the plan's check compares components and ignores overlaps below 0.001 cm3). It compares every body with every other body and lists 10 overlapping pairs, all expected: the four cradle prongs (0.126 cm3 each: the root is sunk 0.5 mm into the floor and the outer end sits inside the cradle ledge) and the four lid hook bumps (0.0098 cm3 each: the outer face is sunk 0.1 to 0.2 mm into the skirt) are separate bodies only because they flex and are one printed part with the tub and the lid; the two bumper plates touch their microswitches by 0.00035 cm3 each, as in the rev 3 model. No other pair of parts overlaps. I checked the first two kinds by probing points inside both bodies, not only by their volumes.
- **Sixteen parts touch nothing in the model** (`report_loose()`, `results/v4_loose_parts.txt`; each of the other 35 touches at least one other part): the nine ToF boards (0.50 mm from their ring pockets), the two floor sensors FP and SM and the USB-C socket (0.50 mm from the tub), the two camera boards (0.48 mm under their lid humps; the cradle that holds them is not modelled), the rear nub (2.11 mm short of the tub; its boss is not modelled) and the Wi-Fi antenna (0.05 to 0.1 mm off the wall, on purpose, it is stuck on). These are modelling choices at layout level, not placement errors, but they are what the CAD detail design has to hold: snap tabs, cradles, a nub boss and fixings.
- **Kit path** (`kit_path.py`, `results/v4_kit_path.txt`, added after the plan's reports because delivering kits is the robot's main job): a 10.3 mm cube falls from its plate pocket through the slot and hopper to the trough floor (z 7.7) and slides down each 13 mm channel to 2 cm past the exit, centred and 1.2 mm off in four directions, without touching any part; the two controls (a 14.7 mm cube in the slot, a 14.0 mm cube in the channel) are blocked, so the check can see a blockage. What geometry cannot settle is the turn in the trough: each channel runs 32.8 degrees off the slot square, so a falling kit has to turn at least 14.6 degrees (it enters the bore up to 18.2 degrees off the axis) and can turn at most 39.5 degrees (what the 14.5 mm square allows), a margin of 6.7 degrees at the top end. **A physics simulation of the turn then showed that it does not happen for a kit that arrives square to the slot** (MuJoCo on this geometry, `v3-checks/chute_dynamics.py`, `v3-checks/results/chute_dynamics.txt`; its geometry agrees with the Fusion bodies at 7000 random points, `chute_geometry_check.py`, `chute_samples.py`): kits within 10 degrees of the slot square did not get out in 10 of 10 trials at friction 0.25, 0.35 and 0.45, kits turned 20 degrees or more toward the channel got out 10 of 10, and a 14.7 mm bore alone lifts the square kits only to 30 %. This is an open design problem (spec 6.2, 11 and 13), found by a quick check, not a tuned model; the slide at 32 degrees and the real friction stay the bench test (spec section 12). My first version of the drop test kept the cube turned like the slot all the way to the hopper floor and reported 0.0025 cm3 against the channel walls below z 7.7, where the kit has to be in the bore anyway; it now stops at the trough floor and states the turning room.
- **Belly line (z 3.5): twelve known parts hang below it**, all by design: both wheels and the omni wheel (z 0.00), the rear nub (2.00), the omni arm (2.45) and its pins (2.80), both drive motors (3.00), both chutes' lower lips (3.04; the spec's `chute_exit` calculation says 3.00 and its terrain table uses the lower 2.67, so both stay on the safe side), the floor sensors FP and SM (2.80). The 4 mm omni pivot pin reaches z 3.40, **1.0 mm below the line**: the spec (section 4) leaves "a 3 mm pin or a sunk pin" to the CAD detail design, and the model still has the 4 mm pin.
- **The GIGA's BOOT0 button PB2 is covered by the ring** (known limit, spec 7.4: double-tap reset enters the bootloader). The reset button PB1 and the USB-C connector J12 are open from above with the lid off; the `access` report lists all three so that a change which hides PB1 or uncovers PB2 shows up.
- **Access from above:** with the lid off the GIGA is 55 %, the main PCB 53 % and the battery 77 % open; with the lid and the dropper unit lifted out 86, 86 and 91 % (criteria 70, 70, 80 %). The silver module SM (49 %) and the two drive motors (left 0 %, 54 % with the unit out; right 0 %) are under the frame: the report prints them as "frame off" and does not judge them; the removal teardown covers them.
- **Removal paths:** all seven pass, with the lid hook bumps and the cradle prongs treated as flexing parts (`FLEX` in `v4_checks3d.py`; their bending is a calculation in the spec). The battery leaves only by sliding 4.5 cm back and 2 cm inboard before lifting, because Camera L and the ring corner are over it and the bridge with its power switch is in front of it. The report was run in pieces because of the 60 s tool limit (see Rebuild).
- **Spec clearances: all 14 are above their smallest acceptable value, and no spec number needed correcting.** Three are larger in 3D than the number the spec quotes (all on the safe side, spec numbers left as they are): chute to wheel 8.24 mm (spec 6.7; the rev 3 model had 8.15), N20 to the left drive motor 10.86 mm (first model 10.6), stepper bay to the GIGA stack 5.76 mm (spec 5.0, a plan-view figure; 3D is larger where the heights differ). The tightest are the omni wheel to the GIGA post H1 (2.67 mm, spec 2.7), the silver module SM to H1 (2.78 mm, placeholder module size), the battery to the left motor (1.10 mm, spec 1.0) and the GIGA stack to the tub (1.72 mm, spec 1.7).
- **Mass 1197 g [placeholder masses]** (printed parts from the model volume at 45 % fill and 1.27 g/cm3, bought parts and the unmodelled wiring and fasteners from the rev 3 budget in `com.py`, whose total was 1070 g; the emulator had 1195 to 1196 g). Centre of mass 0.90 cm ahead of the axle, 0.15 cm to the left, 6.82 cm high; front load 12.8 %; front lift-off limit **1.29 m/s^2** (rev 3: 1.5), above the 1.0 the firmware's PWM ramp assumes. Weigh the printed parts: the limit moves with them.
- **Envelope:** the bumper plates reach 11.000 (their switches 10.752), everything else 10.5 at most; the USB-C socket's face is at 10.470 and the Wi-Fi antenna at 10.295, inside the wall's 10.3 inner face. The highest point is the victim LED on the handle bar at z 16.0 (bar top 15.6, limit 25).
- **Emulator against Fusion:** the dry run's prediction table (plan, Task 9) held for all eleven reports. What only Fusion showed was about the tool, not the model: the 60 s tool limit and the rolled preset views (both in Rebuild).
