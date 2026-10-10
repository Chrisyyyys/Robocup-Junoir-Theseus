# Dry run of the rev 4 Fusion rebuild, without Fusion

`dryrun.py` builds the rev 4 model and runs its checks on a small geometry emulator instead of Autodesk Fusion. It exists to test
[the rebuild plan](../../../plans/2026-10-07-theseus-v3-rev4-fusion-model.md) before any Fusion call is made, and to re-test it quickly after a change (about 4 minutes against
several minutes per Fusion step, and no tool timeouts). It does **not** replace the Fusion run: the plan's Task 9 results come from Fusion.

```bash
cd docs/superpowers/specs/v3-fusion/dryrun
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py              # plan code: unit tests, 19 stages with probes and overlaps, ten reports
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py --clearances # also the nearest-distance report (about 10 more minutes; with the camera cages of 9 Oct more than an hour: not run, Fusion measures the clearances)
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py --repo       # use v3-fusion/v4_*.py and v3-checks/v3_params4.py as they are in the repo (10 Oct, with the omni spring's legs: 91 unit tests, 20 stages, about 5 minutes)
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py removal tof  # only these reports
PYTHONPATH="C:/Users/christopher.shu/pl4" python balance.py [--repo]     # masses by group, centre of mass, front load and lift-off limit for an axle 0 to 3 cm back (--repo: the repo files, as for dryrun.py)
PYTHONPATH="C:/Users/christopher.shu/pl4" python wire_runs.py           # wire lengths from the controls to J12: rear panel against the front deck
PYTHONPATH="C:/Users/christopher.shu/pl4" python selftest.py            # only the emulator's own tests
```

By default the code under test is **pulled out of the plan document** (`plan_code.py` writes the plan's modules to a temporary folder and applies the plan's two textual patches
to copies), so a mistake in the plan's code shows up before it is typed into the repo. Nothing in the repo is changed. **Since the chute redesign of 8 Oct the plan's code blocks keep the first chute design and no longer equal the repo files: run `--repo` (and `balance.py --repo`) to test the current model.**

## What it runs

| Part | How |
|---|---|
| Builders (`v4_model.py`, the reused rev 3 builders) | unchanged; `fusion_lib.prism`, `combine`, `move_body`, `new_part` are replaced by `fake_fusion.py`, which keeps every body as a list of extruded shapes and boolean tools |
| Probes (`has_material`) | unchanged; `pointContainment` is exact (strictly inside or not) |
| Local and whole-model interference, removal paths, omni sweep | the plan's own functions on a fake `TemporaryBRepManager`; the volume of an intersection is a Monte-Carlo estimate (40 000 random points per pair inside the overlap of the two bounding boxes, so an overlap far below 0.002 cm3 can be missed; the Fusion run catches those) |
| ToF cones, camera rays | `v3_checks3d._cast` replaced by a march along the ray (0.05 mm step) |
| Envelope, clearances, mass | mesh vertices, `measureMinimumDistance` and `physicalProperties` replaced by sampling (distances good to about 0.1 mm, mass to about 1 %) |
| Cuts and joins | the emulator also notes a cut that removes nothing and a join that touches nothing (Fusion raises on both) |

Not emulated: sketches, real boolean topology, anything about Fusion's timeline or the document. A pass here means the plan's code and its numbers agree with each other; Fusion can still
disagree about an API detail. `selftest.py` checks the emulator on synthetic cases (a known overlap, a cut cavity, a placed component, a body along a sloped axis, a rotation).

## What it found in the plan (7 Oct, before any Fusion call)

First round:

1. The wall strip above each wheel arch sat in the middle of the camera's field of view (9 of 25 rays blocked): notched at the camera window.
2. The rear frame screws at 101 degrees put a boss on the tyre's top rim, so the wheel could not slide out and drop: moved to 118 degrees.
3. Both drive cartridges are under another part: the battery above the left one, the GIGA stack above the right one. They lift out freely once those are out (service table in the spec).
4. The USB-C service socket stuck 2 mm out of the wall (envelope r 10.72): front face now flush.
5. Two arm-slot cuts removed nothing and would have made Fusion raise: dropped. The lid hook bumps and cradle prongs pass the removal check only if they are treated as flexing parts: they are separate bodies now.
6. The removal paths were stepped too coarsely (a part could jump over a thin wall): steps are at most 2.5 mm now.

Second round (after the access, controls and sensor changes were folded into the plan):

7. The ToF pockets were closed by a 1 mm skin (found by the visibility test): they are open at the top, as the spec says the boards drop in from above.
8. The handle bar (x 0 to 8) stood over the kit plate and the floor disc, so the dropper unit could not lift out (plate against bar after 5 cm): the bar starts at x 3.3.
9. A control deck 4.8 cm wide covered the GIGA's reset button (found by checking the access numbers against the datasheet picture of the board): the deck is on the +y side only (its -y edge is the bridge's). The boot button is under the ring and stays a frame-off job (the `access` report lists it as a known limit).
10. Camera L's board is over the battery's outboard end and the ring over its front corner, so the battery cannot lift straight out: the removal scenario uses a 4.5 cm slide back and an inboard move of 1.5 cm or more.
11. The dropper floor's seat ledges overlapped the disc rim (0.1 cm3) and the rear tab lay inside the disc: the seats are now rebates in the existing bridge and spoke webs, and the rear tab reaches 4 mm past the rim.
12. A tub probe and a unit test asked for floor where the silver module's hole now is, and one comparison failed on a rounding error (0.85 against 0.8500000000000001): both rewritten.
13. Not found by the emulator but while reviewing its output against the rules: the victim LED on the control deck sat 4 cm under the handle bar and below the lid top, so it was hidden from straight above, and rules 4.2 want it "clearly visible to the referee". It is now on top of the bar's front end (z 16.0, the highest point); a unit test (`test_victim_led_is_on_top_of_the_bar`) and the envelope report's highest-point line pin that.

Third round (8 Oct, the chute redesign after the kit simulation failed the first design; run with `--repo`):

14. The 21.2 mm channel's top corners reached 0.003 cm3 into the dropper floor near the slot centre (found by the stage's overlap test): the channel is cut at the floor's underside (z 8.7).
15. A tub probe for the wall beside the bigger chute hole asked for material that is inside the hole: the probe point moved.
16. What the emulator got wrong or cannot see: its test for a cut that removes nothing missed that same 0.003 cm3 sliver on the right chute and found it on the left one (random points over the whole tool): it now takes a second, denser look inside the target's bounding box before it warns (`fake_fusion._focused_overlap`). And it cannot see a body that a cut splits in two: the bigger socket cut left a loose 85 mm3 lintel on each hopper that only Fusion showed (92 bodies instead of 90). The one-body rows of the chute stage check (`ONE_BODY`) cannot fail in the emulator and matter in Fusion; `test_hopper_is_one_piece` sees it on the convex model. `balance.py` had been failing since the Wi-Fi antenna was added (it did not know the component): fixed.

Fourth round (9 Oct, the ToF, camera, frame and IMU revision; run with `--repo`):

17. Probe points that lie exactly on a face count as empty, in the emulator and in Fusion alike: a lid probe at x -1.3 sat on the camera pocket's wall and a frame probe for the ToF posts fell into the pilot hole. Both probes were moved (to x -1.5, and 0.18 cm off the pilot); the stage probes of the new ToF, camera, lid and IMU stages are 22, 29, 17 and 6.
18. What only Fusion could see: the lid hump's corner at r 10.512 (the emulator's envelope report samples points; Fusion's mesh has the vertices; the humps were narrowed, `hump_y`) and a set of overlaps of 0.2 mm between camera parts that the first camera model made on purpose (lens holder, barrel, SD socket and screw shafts), which Fusion's own body-by-body analysis counts as overlaps: the parts now touch with exactly coincident faces.
19. `balance.py` did not know the camera cages and the IMU (the same failure as with the Wi-Fi antenna on 8 Oct, an assertion that every component belongs to a group): they are in the groups now.
20. The emulator's nearest-distance report samples every body of a component against every other: with the camera cages (several bodies each) it took more than an hour and was abandoned; the 9 Oct clearances in `../results/v4_clearances.txt` are Fusion's alone.

Fifth round (9 Oct, the omni mount with the real wheel; run with `--repo`, three runs):

21. The M4 axle screw's head, first drawn standing 4 mm out of the wheel's inboard face, swept through the floor beside the opening during the 13-position omni sweep (`Omni pins x Tub` 0.078 cm3 at 0.62 cm of travel) and in the teardown. Widening the opening to take it would have cut into the GIGA post H1's footprint (the post's edge is 1.4 mm from the opening): the head and its washer now sit inside the wheel's bore, with the inboard bearing seated 5 mm in.
22. The 7 mm head of the first M4 pivot screw (axis z 4.2) reached z 3.85, 0.5 mm into the floor's top (0.005 cm3 in the stage overlap test; the same sliver scraped the floor along the whole pull in the removal path): a relief would have had to be a trench as long as the pull, so the pivot became an M3 screw (5.5 mm head, 0.25 mm clear of the floor).
23. The first new antenna place at -17.5 degrees (z 5.5 to 7.04) overlapped the tub by 0.036 cm3: it lay inside the right bumper's recess wall (z 3.9 to 6.6, angles -62 to -8 degrees, inner face r 10.15), which the earlier hand check had not looked at. It moved to -22 degrees, z 5.0 to 6.54, on that wall, 8.65 mm from the bumper switch and 6.10 mm under the USB-C socket (Fusion `clearances`).
24. The 25.6 mm wheel could not leave straight up in the teardown: it caught the frame's insert boss at +12 degrees (`Omni wheel x Tub` 0.087 cm3 at 5.25 cm of lift; the 20 mm wheel of 8 Oct cleared it by 0.1 cm). The removal path is now: pivot and stop screws out inboard, the axle screw unscrewed 3.5 mm out of the arm (pulling it out inboard hit the silver module, 0.27 cm3), the wheel set with the screw still in it down through the opening, the arm with the spring on its tube up.
25. Also from this round: the 8 Oct stadium-shaped bay cuts, extruded over the whole height, nicked the front wall and the +12 degree boss (found by reading the numbers, not by a check): they became two plan cuts through the floor, and the second stadium cut then removed nothing in the new layout, which the emulator's warning for cuts that remove nothing caught (Fusion raises on those). `balance.py` knew none of the six new omni components: they are in its omni group now.

## Results of the last full run

`results/dryrun_full.txt` is the complete output of `python dryrun.py --repo` on the repo files after the 10 Oct omni spring (the forward leg on its seat pin, the arbor, the straight rear leg on an M3 x 6 bolt, on top of the 9 Oct omni mount: the real wheel, its arm, spring and screws, the pillar and ear, FP at y 3.5, the antenna at -22 degrees; and D21, D22, the real ToF and camera boards, the ring at z 11.7, the cages and the IMU), **without** `--clearances`: 91 unit tests, 20 stages (the omni stage has 41 probe rows since 10 Oct, 32 before; the omni sweep has 11 moving bodies), no cut that removes nothing and no join that touches nothing, 57 components without an overlap, seven removal paths free (the teardown with the omni in four steps), access from above (GIGA 53 % to 84 %, shield 53 % to 86 %, battery 66 % to 80 % with the lid off and with the lid and dropper unit off), no ToF or camera ray blocked, the chute lips at z 2.91, mass 1286 g with the centre of mass 1.09 cm ahead of the axle and 6.79 cm high (front load 15.9 %, lift-off limit 1.58 m/s^2; the mass and distances are sampled, so a repeat run differs by about 1 g and 0.1 mm). `results/balance.txt` (run with `--repo`) and `results/wire_runs.txt` are the outputs of the two helper scripts. **The Fusion run of 9 Oct on the rebuilt model reproduced these numbers** (1287 g, 1.09 cm and 6.79 cm, the same access shares, no blocked ray, 57 components without an overlap, the seven removal paths free; clearances, 29 pairs, in `../results/v4_clearances.txt`, summary in `../README.md`). The numbers of the earlier 9 Oct run (before the omni mount): 1228 g, 0.86 cm and 6.96 cm, 54 components. The earlier full runs, which included the clearance report (8 Oct: 44 unit tests, 19 stages, 51 components, mass 1197 g; the first dry run of the plan's code with the first chute: 38 tests, 1196 g), are superseded and in the git history.
