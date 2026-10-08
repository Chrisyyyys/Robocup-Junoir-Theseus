# V3 geometry and statics checks

These scripts produce every **[check]** number in [2026-10-06-theseus-v3-robot-design.md](../2026-10-06-theseus-v3-robot-design.md) and [2026-10-07-theseus-v3-mechanical-design.md](../2026-10-07-theseus-v3-mechanical-design.md) and draw the figures in [../v3-figures/](../v3-figures/). They are a model, not the robot: nothing here was measured on hardware, and all masses are placeholders.

```
pip install shapely numpy matplotlib
python run_all.py            # every check, raw output in results/*.txt (about 10 minutes)
python run_all.py terrain    # one check (names below)
python figures.py ../v3-figures
```

Run from this folder (the scripts read and write `pack_v3.json` here). Units are cm and degrees; x forward, y left, z up, origin at the axle midpoint on the floor line.

## What each check answers

| Check (`run_all.py` name) | Script | Spec section | Result it supports |
|---|---|---|---|
| `terrain` | `nubtable.py`, `sidesim2.py`, `sidesim.py` | 4 | Side-view crossing of a 2 cm riser, two 1 cm steps, 25 degree ramp, bumps, seam, three-step stairs and the Dangerous Zone bump on a ramp; nub-height table; the first-draft and belly-only variants |
| `omni_travel` | `more_checks.py travel` | 4 | Omni travel 2.5 versus 3.0 cm changes nothing for the required cases |
| `silver_module`, `front_floor_port` | `more_checks.py axle`, `fsens.py` | 5 | Clearance of the two floor-sensor positions on every terrain case |
| `ramp_foot` | `tofcone.py` | 5 | What a front ToF reads at the foot of a 25 degree ramp at heights 6.5, 8.1, 10.0 (first lines also trace the old layout) |
| `plate_variants`, `plate_tolerance` | `variants.py`, `platesim.py`, `platesim2.py`, `platetol.py` | 7 | All 128 drop sequences (0 violations), wall and parked margin for each pocket size and ring radius, positioning tolerance |
| `kit_landing` | `kit_final.py` | 7 | Chute slope, exit speed, landing point, kit-to-victim distance |
| `bumper_and_chute_exit` | `geom_checks.py` | 6, 7 | Bumper end angle against the other parts and the corridor wall; chute exit x against the wheel |
| `packing` | `final_pack2.py`, `com.py`, `layout.py`, `place2.py`, `chamfer_pack.py` | 9 | Controller stack and battery placement against all fixed parts, the cameras and the rear chamfer; centre of mass |
| `tof_cones` | `tof_check2.py` | 5 | All nine 25 degree cones traced against the robot's own parts |
| `controller_fit`, `controller_fit_with_battery` (slow, run only when named) | `fit_stack_only.py`, `fit_R.py` | 0.2, 0.5 (D1), 9 | Which controller stack heights fit at R 9.5, 10.0, 10.5 with the wheel track, motors, chute exit and camera re-fitted for each radius; what small main board plus battery fits at 9.5 |
| `camera_beside_wheel`, `camera_recess`, `camera_view` | `beside.py` + `place.py`, `cam_recess.py`, `camview.py` | 8 | No placement beside the wheel; the recessed lens fits; what the lens sees of the wall |
| `statics` | `statics.py` | 4 | Friction needed to climb a step, omni push, motor torque, ramp statics, spring rate (edit `x_com`, `h_com` to try other centres of mass) |
| `figures` | `figures.py`, `v3_params.py` | 14 | The six figures; every dimension in them comes from `v3_params.py` and the simulator |

### Mechanical design (2026-10-07)

These back the numbers in [2026-10-07-theseus-v3-mechanical-design.md](../2026-10-07-theseus-v3-mechanical-design.md) (the structure, mounting and usability review). Results are in `results/`.

| Check (`run_all.py` name) | Script | Spec section | Result it supports |
|---|---|---|---|
| `omni_inside` (slow, run only when named, about 3 minutes) | `omni_inside.py` (uses `sidesim2.py`, `nubtable.py`) | 4 | All nine required terrain cases with the omni centre at 7.95 (rev 3), 7.3, 7.0 and 6.5 cm and the chute lip at z 2.67; the Dangerous Zone extras at 7.0 and 7.95 |
| `statics_omni_7` | `statics.py 7.0` | 4 | Front load, omni push, torque and the spring requirement with the omni 7.0 cm ahead (`statics.py` takes the omni distance as an argument, default 7.95) |
| `chute_exit` | `chute_exit.py` | 6.2 | Lowest point of a round and of a 13 mm square chute where it leaves the R 10.5 wall, and the exit axis height (z 4.15) that keeps it at z 3.0 |
| `kit_landing_square` | `kit_final.py 4.15` | 6.2 | Exit speed, landing point and kit-to-victim distance with the exit axis at z 4.15 (`kit_final.py` takes the exit axis height as an argument, default 3.75) |
| `chute_dynamics` (not in `run_all.py`: needs MuJoCo, `pip install mujoco`; about 10 s quick, a minute full) | `chute_dynamics.py [quick\|full] [bore in cm]` (uses `chute_geometry.py`) | 6.2 | Rigid-body simulation of a 10.3 mm kit dropped from the plate pocket through slot B, the hopper and the 13 mm channel to the exit, on the geometry of the Fusion model, for friction 0.15 to 0.60 and random kit turn and position in the pocket. Controls: textbook acceleration on a 32.2 degree plane, hold at mu 0.7, an oversize cube must jam. **Result 8 Oct: FAIL, kits that arrive square to the slot jam in the hopper mouth** (`results/chute_dynamics.txt`; with a 14.7 mm bore `chute_dynamics_bore147.txt`) |
| `chute_geometry_check` (needs Fusion, three steps, see its docstring) | `chute_geometry_check.py` + `../v3-fusion/chute_samples.py` | 6.2 | The convex-piece model of the hopper, channel and slot used by `chute_dynamics` against the Fusion bodies: 0 of 7000 random points differ, 9 of 8000 points at the faces differ (the exit trim, a plane here and a cylinder in Fusion): `results/chute_geometry.txt` |
| `stepper_bay` | `stepper_bay.py` | 6.3 | Orientations of a 28BYJ-48 under the dropper floor that clear the GIGA stack, battery and hoppers (shaft 8 mm off the body centre) |
| `mount_check` | `mount_check.py` | 5.1, 8 | Where the six GIGA mounting holes land in each of four orientations (usable with a single omni arm), USB-C space, ToF cable runs, motor snap-prong and face-plate numbers, cube-in-channel slack |
| `ring_gaps` | `ring_gaps.py` | 3.1, 7.1 | Room on the ring between the ToF and camera windows for the six frame screws and the four lid hooks |
| `structure_figs` | `structure_figs.py` | 3, 5, 8 | Figures 7 (side section), 8 (motor cradle) and 9 (plan of every attachment point) |
| `fsens_front` | `fsens_front.py` (uses `fsens.py`, `omni_inside.py`, `sidesim2.py`, `nubtable.py`) | 5.3 | Smallest clearance between a floor-sensor window and the terrain over each crossing, with the sensors 0 to 8.5 cm ahead of the axle (the silver module and front port now sit at 7.5) |
| `handle_calc` | `handle_calc.py` | 3.2, 7.2 | The front bridge as a cantilever under the handle post (stress and deflection at 15 and 45 N) and the nose-up angle when carried by different points of the bar |
| `axle_shift` (about 3 minutes) | `axle_shift.py` (uses `sidesim2.py`, `nubtable.py`) | 4 | The terrain cases with the drive axle 0, 1, 2 and 3 cm behind the body centre: why the axle stays at the centre |

## How the model works

- **Plan and height:** parts are shapely polygons with a z interval (round parts are cut into slabs); collisions are tested per 0.25 cm height band with a 1.5 mm margin. The packing is a seeded random search over two boxes (battery, GIGA R1 + main PCB stack), so it repeats exactly.
- **Side view:** for every drive-wheel position along the terrain, the omni may be compressed by any amount from 0 to its travel; the body pitch follows from the omni touching the terrain; a pose is feasible if the rear support does not penetrate. The reported clearances use the smallest feasible compression (a stiff spring, the worst case). Parts: belly line with the rear chamfer, bumper bottom edge, chute nose, rear nub, omni on its arm.
- **ToF cones:** 25 degree full angle, traced as 5 x 5 rays to 30 cm against every part except the ToF boards.
- **Plate:** exposure of the kit footprint over each slot, for every state (kits used from each end, side, one or two kits, parked and during each sweep).

## Limits to keep in mind

- The side view is 2D at y = 0 with projected parts: no roll, no wheel slip, no dynamics, rigid wheels. Terrain polygons end at x = 45 to 90 cm, so pitch minima for the "up" cases and maxima for the "down" cases near that end are artefacts; only the clearances and the maximum tilt in the direction of the crossing are used in the spec.
- The packing has no PCB layout, cable or connector clearances; battery size and all masses are placeholders.
- The camera, chute and kit numbers assume a friction coefficient of 0.35 and a lens model from the OpenMV listing, not measurements.

## Fixed during the audit

- `sidesim2.mirror_profile` started descending profiles at x = 0 instead of x = -60, so the terrain behind the robot was a 20 degree slope; the "descending steps" rows were wrong until it was fixed (two 1 cm steps down: belly 1.50, bumper 3.24, chute nose 1.00 cm).
- The first packing put a small board under the rear chamfer (the packing model had a flat belly); `chamfer_pack.chamfer_gap` is now a constraint in `final_pack2.py` and `cam_recess.py`.
- The 14 mm pocket on a 33 mm ring in the approved draft gives 0.0 mm walls; see `plate_variants`.
