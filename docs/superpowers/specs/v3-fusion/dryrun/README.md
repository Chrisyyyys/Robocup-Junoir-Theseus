# Dry run of the rev 4 Fusion rebuild, without Fusion

`dryrun.py` builds the rev 4 model and runs its checks on a small geometry emulator instead of Autodesk Fusion. It exists to test
[the rebuild plan](../../../plans/2026-10-07-theseus-v3-rev4-fusion-model.md) before any Fusion call is made, and to re-test it quickly after a change (about 4 minutes against
several minutes per Fusion step, and no tool timeouts). It does **not** replace the Fusion run: the plan's Task 9 results come from Fusion.

```bash
cd docs/superpowers/specs/v3-fusion/dryrun
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py              # plan code: unit tests, 19 stages with probes and overlaps, ten reports
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py --clearances # also the nearest-distance report (about 10 more minutes)
PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py --repo       # use v3-fusion/v4_*.py and v3-checks/v3_params4.py as they are in the repo
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

## Results of the last full run

`results/dryrun_full.txt` is the complete output of `python dryrun.py --repo --clearances` on the repo files after the chute redesign (8 Oct): 44 unit tests, 19 stages (200 probe and overlap rows, 35 of them for the chutes), no cut that removes nothing and no join that touches nothing, 51 components without an overlap, seven removal paths free, access from above (GIGA 55 % to 86 %, shield 53 % to 86 %, battery 77 % to 91 % with the lid off and with the lid and dropper unit off), no ToF or camera ray blocked, 14 spec clearances (chute to wheel 5.65 and 5.64 mm, stepper bay to hopper B 3.98 mm), the chute lips at z 2.91, mass 1197 g with the centre of mass 0.90 cm ahead of the axle and 6.81 cm high (lift-off limit 1.29 m/s^2; the mass and distances are sampled, so a repeat run differs by about 1 g and 0.1 mm). `results/balance.txt` (run with `--repo`) and `results/wire_runs.txt` are the outputs of the two helper scripts. **The Fusion runs of 8 Oct, the first build and the rerun after the chute redesign, reproduced every number within the tolerances of the plan, Task 9** (Fusion on the redesigned model with the 2.8 cm floor slab: 1198 g, 0.89 cm and 6.81 cm, chute to wheel 5.62 mm, stepper bay to hopper B 3.97 mm, chute lips 2.91, N20 to the left motor 10.86 mm against the emulator's 11.00; results in `../results/`, summary in `../README.md`). The first dry run, of the plan's code with the first chute (38 tests, 1196 g), is superseded.
