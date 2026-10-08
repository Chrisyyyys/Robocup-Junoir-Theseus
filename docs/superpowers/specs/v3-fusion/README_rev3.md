# Theseus V3 in Autodesk Fusion (revision 3 model, kept for reference)

> This is the README of the first Fusion model (7 Oct 2026, `v3_model.py`, `Theseus_V3_baseline.f3d`). It was replaced by the rev 4 model on 8 Oct; the current README is [README.md](README.md). Its four findings against the rev 3 spec are all in the rev 4 spec and `v3_params4.py`.

A 3D layout model of the V3 baseline in [the design spec](../2026-10-06-theseus-v3-robot-design.md), built by script so it can be regenerated and checked.
All numbers come from [v3_params.py](../v3-checks/v3_params.py), the same file the 2D checks and figures use. Units cm, frame x forward, y left, z up, origin at the
midpoint of the drive axle on the floor line (Fusion is set to Z up here). 39 components, about 360 timeline features.

**Status:** layout model. Every part sits at its specified position and the shell has all the openings. Structure that the spec does not define is not modelled (below).
Nothing here has been measured or printed; masses and the PCB are placeholders as in the spec.

## Files

| File | What it is |
|---|---|
| `fusion_lib.py` | Helpers: sketch shapes, extrude with offset start, cuts, tilted frames, colours |
| `v3_model.py` | The model: one function per subsystem (`build_chassis`, `build_drive`, ...), `build()`, `verify()` |
| `v3_checks3d.py` | Read-only 3D checks: ToF cones, camera fields of view, omni sweep, nearest distances |
| `v3-fusion.py`, `v3-fusion.manifest` | Entry point for Fusion's Scripts and Add-Ins dialog (not run through that dialog yet; the same calls were run through a script runner) |
| `Theseus_V3_baseline.f3d` | Export of the finished model (safety copy; the Fusion document itself is unsaved) |
| `views/` | Screenshots: top (lid hidden), side, iso, underside |

## Rebuild

In Fusion: Utilities > Add-Ins > Scripts and Add-Ins > Add existing script > this folder > Run. It needs an empty design; it switches the document to a hybrid design
(a part design can hold only one component) and replaces whatever components are in the root. `build('all', reset=True)` is the same call; `build(['chassis'])`
builds one stage. `TOF_MODE = 'spec'` in `v3_model.py` puts the nine ToF modules exactly where the spec has them (see finding 1).

## What is modelled

Chassis shell (2 mm wall, 4 mm floor, belly z 3.5, rear chamfer, openings for wheels, motors, omni bay, floor sensors, chutes, nine ToF windows, two camera windows,
bumper recesses with switch pockets, handle), lid with camera humps, button and LED, ToF ring with nine pockets, nine VL53L0X modules, two drive wheels (80 mm) and
motors with encoders and shafts, front omni (60 mm) with arm plates and pins, rear nub, two bumper plates and microswitches, two OpenMV boards with tilted lens
blocks, dropper floor with two slots, plate with eight 14 mm pockets, eight kits, N20 motor, two chutes (15 mm bore, trimmed flush with the body), GIGA R1 plus
shield stack (placeholder), battery (placeholder), floor sensors FP and SM.

## Chosen here because the spec does not say [not in spec]

Floor thickness 4 mm. Wall top z 11.2 with a 0.3 lid plate (roof 11.5). Camera hump skin 0.2 over the 12.1 clearance (outer top 12.3). ToF ring z 8.7 to 11.15,
bore r 9.0. Handle: an arch of two posts at (-4.5, +-9.04) and a bar at z 14.6 (total height 15.0, limit 25), joined to the chassis; the lid has two notches.
**The handle and lid do not work together yet**: the lid cannot be lifted past the bar. Button at (-7, 0), LED at (-7, -3) on the lid. Wheel and motor-slot shapes,
arm plate shape, omni hub, header strips and component envelope on the stack. The spec only fixes the positions and sizes it quotes.

## Not modelled

Decks, brackets, standoffs, fasteners, wiring, springs (omni arm, bumper return), omni rollers, bumper hinge pins, dropper home switch and plate hub, the tilted
sensor of the silver module, the funnel between a dropper slot and the chute bore (the slot is 14.5 mm square; the bore mouth is 15 mm across but only 8.6 mm
along the chute axis, and the kit is 10.3 mm).

## 3D checks on the finished model (all run from `v3_checks3d.py`, `verify()` in `v3_model.py`)

| Check | Result |
|---|---|
| Interference between all 39 components | none above 0.0004 cm3 (bumper plate against its microswitch, touching) |
| Farthest radius of every part from the axle | bumper plates 11.000, omni 10.991, everything else at most 10.5; the swept circle R 11.0 holds |
| Height | 12.3 cm without the handle (lid hump), 15.0 with it; limit 25 |
| ToF cones (25 degrees, 17 rays each) | no ray blocked on any of the nine sensors |
| Camera fields of view (65.9 x 51.8 degrees, 25 rays each) | no ray blocked on either camera, with the 2.8 x 2.4 cm window |
| Omni sweep, 13 positions over 2.5 cm | no interference with any other part |
| Lens block to wheel top | 3.60 mm (spec: 3.6 mm) |
| Camera board to lid hump | 0.48 mm (spec: board top 12.05 under lid 12.1) |
| Chute to wheel | 8.15 mm (spec: 6.7 mm) |
| Battery to motor | 1.10 mm (packing: 0.86 mm) |
| Main PCB shield to wall | 3.2 mm (the GIGA board is 1.7 mm from the thickened wall behind the bumper recess) |

## Findings: the 3D model disagrees with the spec in four places

1. **Side ToF modules stick out of the shell.** Each module is a 1.8 x 0.45 cm board; at the spec's positions (centre on the R 10.3 circle) the four side modules
   (aimed sideways at 45 and 135 degrees) have a corner at r 11.11, 0.61 cm outside the body and 0.11 outside the swept circle; the two rear modules reach 10.92 and
   the two toed-out ones 10.56. The 2D packing treated the modules as points. Model positions (cm): SFL/SFR (7.18, +-6.0), SRL/SRR (-7.18, +-6.0), RL/RR (-8.4, +-4.5),
   FL/FR (8.97, +-4.18); F unchanged. All corners then lie at r 10.23 or less. Side baseline 14.36 cm (was 14.56), rear 9.0 cm (unchanged). Readings in a 28 cm path:
   side 80 mm (was 67), toed-out 55.5 (51.5), rear 56 (47), front centre 45; the worst-case side reading becomes 46 mm instead of 33, which also clears the VL53L0X
   minimum range. Firmware: `TOF_SIDE_OUT_MM` 72.8 -> 60.0 and `TARGET_SIDE_GAP_MM` 67.2 -> 80.0. **The spec, `v3_params.py` and the figures still have the old positions.**
2. **The camera window must be about 2.8 x 2.4 cm, not about 2 x 2.** The lens is 1.7 cm behind the outer surface; a 2 cm window passes 58 degrees of the 65.9 degree
   horizontal field. Ray tests: 2.3 x 2.2 and 2.6 x 2.4 still clip the corners, 2.8 x 2.4 does not (the lowest ray leaves at z 7.55).
3. **The chute exit's lowest point is z 2.67, not 3.0.** The tube meets the cylindrical wall at about 22 degrees, so its lower lip runs 3.3 mm further down the slope than
   the 2D end cap showed. The 2D terrain table (nose clearance 1.00 cm at worst) would drop to about 0.7 cm; it still passes but was not re-run. Fix: raise the exit axis
   by 3.3 mm (slope 34.9 -> 32.8 degrees).
4. **A 3 cm omni travel clips the GIGA board.** At 2.8 and 3.0 cm the arm plate touches the board corner by about 0.2 mm. The model uses a 2.5 cm stop (the value the terrain
   checks used; the Dangerous Zone stairs need 2.1). The arm also swings on an arc that bulges 2.7 mm ahead of the straight line from rest to compressed, which the
   chassis bay allows for.

Smaller points: the lid at 12.1 in the spec is the clearance over the camera boards, so the hump's outer top is 12.3; the bumper recess wall I added behind the plates brings
the wall within 1.7 mm of the GIGA corner.
