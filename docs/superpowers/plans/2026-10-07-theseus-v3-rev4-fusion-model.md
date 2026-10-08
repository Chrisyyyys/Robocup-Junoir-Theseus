# Theseus V3 Rev 4 Fusion Layout Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the 3D layout model of Theseus V3 in Autodesk Fusion so that it matches the revision 4 mechanical design, and prove it with automated checks.

**Architecture:** One pure-Python parameter module (`v3_params4.py`, unit-tested) feeds a stage-by-stage Fusion builder (`v4_model.py`) that reuses the rev 3 builders that did not change. Every stage has "probe" tests (is there material at this point of this component?) and a local interference test. Whole-model reports (interference, envelope, belly line, removal paths, access from above, ToF cones, camera views, omni sweep, mass and centre of mass, spec clearances) run at the end. Fusion is driven through the MCP `fusion_mcp_execute` tool with a short runner script that reloads the repo modules on every call. The same code was dry-run on a geometry emulator before the plan was written down (`docs/superpowers/specs/v3-fusion/dryrun/`, see "Dry run" below): that run found five design problems, already fixed in the plan and the spec, and predicts what Fusion should report.

**Tech Stack:** Python 3 (stdlib `unittest`, shapely, numpy and matplotlib through the existing check scripts), the Autodesk Fusion Python API (`adsk.core`, `adsk.fusion`) via the Fusion MCP tools.

**Spec:** `docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md` (revision 4; the plan argues from it, read it first). Rev 3 for everything not changed: `docs/superpowers/specs/2026-10-06-theseus-v3-robot-design.md`.

## Global Constraints

- Units cm and degrees. Frame: x forward, y left, z up, origin at the midpoint of the drive axle on the floor line (axle z 4.0). Fusion is set to Z up for this user.
- Body radius 10.5 (wall inner face 10.3); swept radius 11.0 set by the bumper tips only; height limit 25 with the handle top at 15.6 and the victim LED on it at 16.0; belly line z 3.5.
- Omni: centre (7.0, 3.0), radius 3.0, width 2.0, front edge 10.0, arm length 4.49, pivot (2.55, 3.6), travel 2.5, one arm on the +y side (y 1.1 to 1.5).
- Chute: 13 x 13 mm inside, 1.6 mm walls, exit axis z 4.15, slope 32.2 degrees, lowest point where it leaves the wall z 3.00.
- Printed in PETG, FDM tolerance +-0.2 mm, M3 heat-set inserts, frame screws M3 x 30. Every snap feature flexes in the layer plane.
- The motor face plate has ONE ear, on the front (+x) side. The GIGA is fixed on four posts H1 to H4. The 28BYJ-48 bay stays free.
- Frame screws at +-12, +-60 and +-118 degrees (not 101: that boss hung on the tyre rim), each boss with a rib to the wall; the wall strip above each wheel arch is notched 3 cm at the camera window; the USB-C service socket is flush with the wall, in the front-right wall at -24 degrees. The handle bar starts at x 3.3 and the control deck is on the +y side only, because the dropper unit (floor, plate, kits, N20, hoppers) must lift out straight up and the GIGA's reset button must stay open from above. The dropper floor is its own component on three half-lap seats of the frame. The floor sensors are at x 7.5 (FP +3.2, SM -2.7). The drive axle stays at the body centre. These came out of the dry run or the second review round and are in the spec (section 0, rows 15 to 18). The Wi-Fi/Bluetooth antenna that ships with the GIGA is stuck on the inside of the tub's front wall at -6 degrees (spec 8.5); the radio is only for test mode.
- Do not run `git commit`, `git push` or `git add` unless the user asks for it in that turn (user rule); every new file stays untracked.
- Fusion: never save or close the user's document without asking; `reset=True` replaces every component in the open design; an `.f3d` export into the repo folder is local and allowed.
- Run Python checks from `docs/superpowers/specs/v3-checks` with `PYTHONPATH="C:/Users/christopher.shu/pl4"` (shapely, numpy, matplotlib live there).
- A Fusion tool call is cut off after about 60 s (found in the run of 8 Oct; the server then answers "unavailable" until the script has finished, and the model is intact): keep builds (one to three stages) and reports in separate calls, and run the removal report in pieces (`only=`, `window=`, Task 9). A script that raises is rolled back completely, so a failed stage can simply be re-run.
- Write scripts and long text with the Write tool, not Bash heredocs (a heredoc with triple single quotes breaks the Bash tool).

## Review Focus

The spec is a design, not a behaviour list. These are the failure modes it implies that no stage probe exercises, most likely first, and the task that owns each test:

1. A part that cannot be taken out by the documented service path (lid over the handle, wheel through the arch, the dropper unit straight up past the handle post, the battery 4.5 cm back and inboard, plate before the N20, frame lift with the chutes out), or electronics that a frame part hides (the control deck over the GIGA's reset button). Test: removal-path report and access report, Task 9; the bar and deck unit tests, Task 1.
2. Something hanging below the belly line other than the known items (wheels, omni, nub, motors, floor sensors, chute lip, pivot pin at z 3.4). Test: belly report with an allowlist, Task 9.
3. Parts that collide, or block a sensor's view, because a later stage moved or added something (the wall strip over a wheel arch sat in the middle of the camera's field of view). Test: local interference after every stage (Tasks 3 to 8), the whole-model interference report, the ToF cone and camera ray reports (Task 9).
4. The model drifting from the spec numbers (omni inside the wall, chute lowest point, screw and hook angles between the windows, stepper bay clearances, ear stress, GIGA hole clearances). Test: `test_v3_params4.py`, Task 1, and the spec clearance report, Task 9.
5. The added printed mass pushing the centre of mass up and the front-lift limit below the 1.0 m/s^2 the firmware ramp is assumed to cover. Test: mass and centre-of-mass report with a warning threshold, Task 9.

Not testable here, so tracked as open items in the spec: printer bed size, real part dimensions (encoder board, face-hole spacings, ToF connector edges), snap-fit retention (prong flexing is not modelled), cable routing and length, FDM shrinkage.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `docs/superpowers/specs/v3-checks/v3_params4.py` | Create | Rev 4 parameters, pure Python, no Fusion imports. Single source for the model and the tests |
| `docs/superpowers/specs/v3-checks/test_v3_params4.py` | Create | `unittest` checks of the parameters against the spec numbers and the existing 2D check scripts |
| `docs/superpowers/specs/v3-fusion/fusion_lib.py` | Modify | Add `axis_prism` (any 2D shape along a straight axis); `axis_tool` calls it |
| `docs/superpowers/specs/v3-fusion/v3_checks3d.py` | Modify | `omni_sweep` takes the omni geometry as optional arguments (default unchanged) |
| `docs/superpowers/specs/v3-fusion/v4_model.py` | Create | Rev 4 builders, one function per stage, registered in build order |
| `docs/superpowers/specs/v3-fusion/v4_checks3d.py` | Create | Stage probes, local interference, whole-model reports |
| `docs/superpowers/specs/v3-fusion/v4_run.py` | Create | Entry points called from the Fusion script runner (`stage`, `report`, `selftest`) |
| `docs/superpowers/specs/v3-fusion/dryrun/` (`dryrun.py`, `fake_fusion.py`, `emu_checks.py`, `plan_code.py`, `selftest.py`, `balance.py`, `wire_runs.py`, `results/`, `README.md`) | Exists already (written while validating this plan; not part of a task) | Dry run of the plan's code and checks on a geometry emulator, no Fusion needed; the balance and wire-run helpers behind two spec numbers; the last full output |
| `docs/superpowers/specs/v3-fusion/README.md` | Modify | Rev 4 status, how to rebuild, what is and is not modelled, results |
| `docs/superpowers/specs/v3-fusion/Theseus_V3_rev4.f3d`, `views/rev4_*.png` | Create | Export and screenshots of the finished model |
| `docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md` | Modify | Status line and a short results table |

`v3_model.py`, `v3_params.py` and the rev 3 figures stay untouched, so the rev 3 baseline (`Theseus_V3_baseline.f3d`) can still be regenerated.

### How to run a Fusion step

Every Fusion step is one call of `mcp__Autodesk_Fusion__fusion_mcp_execute` with `featureType: "script"` and `object.script` set to the runner below (change the last line). `RUNNER` in later tasks means this text with the `run` body replaced.

```python
import sys
sys.path.insert(0, r"C:\Users\christopher.shu\OneDrive - St. Andrew's College\Desktop\Robocup-Junoir-Theseus\docs\superpowers\specs\v3-fusion")
import v4_run


def run(context):
    print(v4_run.stage(['tub'], reset=True))
```

The report ends with `RESULT: PASS` or `RESULT: FAIL`. A `FAIL` line names the probe. A failing probe means either the builder or the probe is wrong: work out the geometry by hand before changing either, and say which one was wrong in the task notes.

**Dry run.** `docs/superpowers/specs/v3-fusion/dryrun/` runs this plan's builders, probes, interference, removal paths, ray checks and reports on a geometry emulator (README there). After writing or changing code, run from that folder `PYTHONPATH="C:/Users/christopher.shu/pl4" python dryrun.py --repo` (about 4 minutes, `--clearances` adds about 10): every stage must pass and every report must give the result listed under "Predicted by the dry run" in Task 9. A failure there is a bug to fix before the Fusion call. It cannot see Fusion API differences (booleans on coincident faces, what `pointContainment` says on a face, timeouts), so the Fusion steps stay, and where Fusion and the dry run disagree by more than 0.3 mm or 1 % the disagreement is itself a finding: say which side was wrong. `python dryrun.py` without `--repo` runs the code as it is written in this document.

---

### Task 1: Rev 4 parameters and their unit tests

**Files:**
- Create: `docs/superpowers/specs/v3-checks/test_v3_params4.py`
- Create: `docs/superpowers/specs/v3-checks/v3_params4.py`

**Interfaces:**
- Consumes: `v3_params` (rev 3: `R_BODY`, `R_INT`, `R_SWEPT`, `Z_BELLY`, `Z_ROOF`, `Z_LID`, `AXLE_Z`, `WHEEL`, `MOTOR`, `NUB`, `CHAMFER`, `BUMPER`, `FLOOR_FRONT`, `SILVER`, `BUMPER_SW_DEG`, `TOF_Z`, `TOF_W`, `TOF_H`, `TOF_T`, `CAM`, `PLATE`, `slot_xy(which)`, `pocket_xy(i)`, `load_pack()`, `PART_SIZES`), `chute_exit.tube_min_z`, `stepper_bay.clearances`, `ring_gaps.all_windows`, `mount_check.robot_xy`.
- Produces (used by every later task): `Z_TUB_TOP`, `Z_FLOOR_TOP`, `FLOOR_T`, `CAM_X`, `OMNI`, `OMNI_BAY_Y`, `OMNI_ARM_SLOT_Y`, `omni_at(travel)`, `TOF`, `CHUTE`, `chute_ends(side)`, `FRAME`, `HANDLE`, `SCREW`, `HOOK`, `LID`, `CONTROLS`, `DROPPER_FLOOR`, `USB`, `usb_pose()`, `ARCH`, `CRADLE`, `FACE_PLATE`, `GIGA_HOLES_MM`, `GIGA_USED`, `GIGA_POST`, `J12`, `J14`, `ANTENNA`, `giga_hole_xy(i)`, `giga_j12()`, `giga_j14()`, `antenna_pose()`, `TRAY`, `battery_pose()`, `STEPPER`, `stepper_geometry()`, `KIT`, `polar(r, deg)`, `screw_angles()`, `hook_angles()`, `PETG_G_CM3`, `PRINT_FILL`, `PRINTED`, `BOUGHT_G`, `UNMODELLED_G`.

- [ ] **Step 1: Write the failing tests**

Create `docs/superpowers/specs/v3-checks/test_v3_params4.py`:

```python
"""Unit tests for v3_params4.py: the rev 4 numbers against the spec and against the 2D check scripts. Run from this folder:
PYTHONPATH="C:/Users/christopher.shu/pl4" python -m unittest test_v3_params4 -v"""
import itertools
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

import chute_exit
import mount_check
import ring_gaps
import stepper_bay
import v3_params4 as P


class OmniInsideTheBody(unittest.TestCase):
    def test_pivot_and_arm_length(self):
        px, pz = P.OMNI['pivot']
        self.assertAlmostEqual(px, 2.55, places=2)
        self.assertAlmostEqual(pz, 3.6, places=6)
        ox, oz = P.OMNI['rest']
        self.assertAlmostEqual(math.hypot(ox - px, oz - pz), 4.49, places=2)

    def test_front_edge_is_inside_the_wall(self):
        self.assertAlmostEqual(P.OMNI['rest'][0] + P.OMNI['r'], 10.0, places=6)
        self.assertLessEqual(P.OMNI['rest'][0] + P.OMNI['r'], P.R_INT - 0.25)
        xc, zc = P.omni_at(P.OMNI['travel'])
        self.assertAlmostEqual(xc + P.OMNI['r'], 9.62, places=2)
        self.assertAlmostEqual(zc + P.OMNI['r'], 8.5, places=6)

    def test_arm_angles_over_the_travel(self):
        px, pz = P.OMNI['pivot']
        ox, oz = P.OMNI['rest']
        xc, zc = P.omni_at(P.OMNI['travel'])
        self.assertAlmostEqual(math.degrees(math.atan2(oz - pz, ox - px)), -7.7, places=1)
        self.assertAlmostEqual(math.degrees(math.atan2(zc - pz, xc - px)), 25.0, places=1)

    def test_bridge_underside_clears_the_compressed_omni(self):
        xc, zc = P.omni_at(P.OMNI['travel'])
        self.assertGreaterEqual(P.FRAME['z0'] - (zc + P.OMNI['r']), 0.19)

    def test_single_arm_is_on_the_left_and_clear_of_the_wheel(self):
        y0, y1 = P.OMNI['arm_y']
        self.assertGreater(y0, P.OMNI['w'] / 2)
        self.assertGreaterEqual(y0 - P.OMNI['w'] / 2, 0.09)
        self.assertGreaterEqual(P.OMNI_ARM_SLOT_Y[0], 0.8)
        self.assertLessEqual(P.OMNI_ARM_SLOT_Y[1] - y1, 0.3)


class Chute(unittest.TestCase):
    def test_square_channel_lowest_point(self):
        p0, e = P.chute_ends(1)
        low = chute_exit.tube_min_z(p0, e, 'square', P.CHUTE['out_w'] / 2)
        self.assertAlmostEqual(low, 3.0, delta=0.02)

    def test_slope(self):
        p0, e = P.chute_ends(1)
        slope = math.degrees(math.atan2(p0[2] - e[2], math.hypot(e[0] - p0[0], e[1] - p0[1])))
        self.assertAlmostEqual(slope, 32.2, delta=0.1)

    def test_exit_is_on_the_body_radius_and_the_sides_mirror(self):
        p0, e = P.chute_ends(1)
        q0, f = P.chute_ends(-1)
        self.assertAlmostEqual(math.hypot(e[0], e[1]), P.R_BODY, places=6)
        self.assertAlmostEqual(f[1], -e[1], places=9)
        self.assertAlmostEqual(q0[1], -p0[1], places=9)

    def test_cube_slack_in_the_channel(self):
        self.assertAlmostEqual(P.CHUTE['in_w'] - 1.03, 0.27, places=3)
        self.assertAlmostEqual(P.CHUTE['out_w'], 1.62, places=6)


class RingScrewsAndHooks(unittest.TestCase):
    def test_tof_positions_match_the_ring_gap_script(self):
        mine = {n: (x, y, a) for n, x, y, a in P.TOF}
        for n, v in ring_gaps.TOF.items():
            for got, want in zip(mine[n], v):
                self.assertAlmostEqual(got, want, places=2)

    def test_screws_and_hooks_keep_3_mm_from_every_window(self):
        wins = ring_gaps.all_windows()
        for a in P.screw_angles():
            x, y = P.polar(P.SCREW['r'], a)
            d = min(w.distance(Point(x, y).buffer(P.SCREW['hole_r'] * 2)) for w in wins.values())
            self.assertGreaterEqual(d, 0.28, 'screw at %s degrees' % a)
        for a in P.hook_angles():
            x, y = P.polar(P.HOOK['r_out'], a)
            half = P.HOOK['r_out'] * math.sin(math.radians(P.HOOK['half_deg']))
            d = min(w.distance(Point(x, y).buffer(half)) for w in wins.values())
            self.assertGreaterEqual(d, 0.28, 'hook at %s degrees' % a)

    def test_screw_bosses_stay_out_of_the_wheel_path(self):
        """The wheel slides 1.2 cm out along its shaft and drops through the arch, so no boss over its y range (7.0 to 10.2) may reach down to the tyre's top rim.
        The 3D removal check found a boss at 101 degrees that did (tyre top 7.80 against a boss underside at 7.70)."""
        S, W = P.SCREW, P.WHEEL
        for a in P.screw_angles():
            x, y = P.polar(S['r'], a)
            if abs(y) + S['boss_r'] < W['y0'] or abs(y) - S['boss_r'] > W['y1'] + 1.2:
                continue                                                         # not above the tyre's sliding range
            u = abs(x) - S['boss_r']                                             # the boss edge nearest the wheel's centre line
            top = P.AXLE_Z + math.sqrt(max(W['r'] ** 2 - max(u, 0.0) ** 2, 0.0)) if u < W['r'] else 0.0
            self.assertLessEqual(top, S['boss_z0'] - 0.1, 'screw at %s degrees hangs over the tyre' % a)

    def test_counterbore_clears_the_hook_rebate(self):
        """The rear screw (118 degrees) lies in the angular span of the hook at 114, so only the radius keeps them apart."""
        self.assertGreaterEqual(P.HOOK['r_in'] - (P.SCREW['r'] + P.SCREW['cbore_r']), 0.25)
        spans = [(a - P.HOOK['half_deg'], a + P.HOOK['half_deg']) for a in P.hook_angles()]
        self.assertTrue(any(lo <= 118.0 <= hi for lo, hi in spans))

    def test_hook_groove_is_inside_the_skirt_segment(self):
        self.assertLess(P.HOOK['groove_half_deg'], P.HOOK['half_deg'])
        self.assertLess(P.HOOK['skirt_z0'], P.HOOK['bump_z'][0])
        self.assertGreater(P.FRAME['z1'], P.HOOK['bump_z'][1])

    def test_front_controls_fit_the_deck_and_the_lid_openings(self):
        """Switch and button sit on the +y side of the deck, beside the bar (half width 0.8), inside the ring, with a finger's width of lid notch around them; the two status LEDs
        sit on top of the bridge rib and show through the handle slot (the bar is 4 cm above them)."""
        D, N, Ld, H = P.FRAME['deck'], P.LID['front_notch'], P.LID, P.HANDLE
        for nm, x, y, shape, z0, z1, colour in P.CONTROLS:
            hx, hy = (shape[1], shape[1]) if shape[0] == 'round' else (shape[1] / 2, shape[2] / 2)
            self.assertGreaterEqual(x - hx - D['x0'], 0.1, nm)
            self.assertGreaterEqual(D['x1'] - (x + hx), 0.1, nm)
            self.assertGreaterEqual(y - hy - D['y0'], 0.1, nm)
            self.assertGreaterEqual(D['y1'] - (y + hy), 0.1, nm)
            self.assertLessEqual(math.hypot(abs(x) + hx, abs(y) + hy), P.FRAME['r_in'] - 0.1, nm + ' must stay inside the ring')
            self.assertLessEqual(z1, Ld['z0'] - 0.5, nm)
            if 'LED' in nm:
                B = P.FRAME['bridge']
                self.assertGreaterEqual(x - hx, Ld['slot_x'][0], nm)
                self.assertLessEqual(x + hx, Ld['slot_x'][1], nm)
                self.assertLessEqual(abs(y) + hy, Ld['slot_half'] + 0.01, nm + ' must show through the handle slot')
                self.assertAlmostEqual(z0, B['rib_z1'], places=6, msg=nm + ' stands on the bridge rib')
                self.assertLessEqual(abs(y) + hy, B['rib_half'], nm)
                self.assertTrue(B['x_rib0'] <= x - hx and x + hx <= B['x_rib1'], nm)
            else:
                self.assertGreaterEqual(x - hx - N[0], 0.3, nm)
                self.assertGreaterEqual(N[1] - (x + hx), 0.3, nm)
                self.assertGreaterEqual(N[3] - (y + hy), 0.3, nm)
                self.assertGreaterEqual(y - hy, H['bar_half'] + 0.05, nm + ' must not hide under the handle bar')
        def shape_of(x, y, s):
            return Point(x, y).buffer(s[1]) if s[0] == 'round' else box(x - s[1] / 2, y - s[2] / 2, x + s[1] / 2, y + s[2] / 2)

        for (na, xa, ya, sa, *_), (nb, xb, yb, sb, *_) in itertools.combinations(P.CONTROLS, 2):
            self.assertGreaterEqual(shape_of(xa, ya, sa).distance(shape_of(xb, yb, sb)), 0.1, na + ' to ' + nb)

    def test_victim_led_is_on_top_of_the_bar(self):
        """Rules 4.2: the victim LED must be clearly visible to the referee. A LED on the deck or the rib sits under the bar and below the lid (visible only from steep angles), so it
        stands on the front end of the bar, the highest point of the robot."""
        H = P.HANDLE
        led = H['led']
        self.assertGreaterEqual(led['x'] - led['r'], H['bar_x'][0])
        self.assertLessEqual(led['x'] + led['r'], H['bar_x'][1])
        self.assertLessEqual(abs(led['y']) + led['r'], H['bar_half'])
        self.assertAlmostEqual(led['z'][0], H['bar_z'][1], places=6)
        self.assertLessEqual(led['z'][1], 25.0)
        self.assertGreater(led['z'][1], max(P.FRAME['z1'], P.LID['z1'], H['bar_z'][1]))      # above the lid, the ring and the bar

    def test_deck_leaves_the_front_edge_of_the_giga_open(self):
        """Reset and boot sit at the corners of the GIGA's connector edge and J12 on it: no frame material may stand over the board (found when the first deck was +-2.4 wide)."""
        D = P.FRAME['deck']
        cx, cy, length, width = P.giga_stack()
        board = box(cx - length / 2, cy - width / 2, cx + length / 2, cy + width / 2)
        self.assertGreaterEqual(box(D['x0'], D['y0'], D['x1'], D['y1']).distance(board), 0.05)
        B = P.FRAME['bridge']
        self.assertGreaterEqual(box(B['x0'], -B['half'], B['x1'], B['half']).distance(board), 0.05)
        self.assertGreaterEqual(D['y0'], -B['half'])                                  # the deck adds nothing on the -y side of the bridge

    def test_bar_does_not_stand_over_the_dropper_unit(self):
        """The unit leaves straight up with the lid off (3D removal check: a bar from x 0 to 8 stood over the plate and the disc): the bar starts in front of both, at the post."""
        H, D = P.HANDLE, P.DROPPER_FLOOR
        self.assertGreaterEqual(H['bar_x'][0] - (P.PLATE['cx'] + D['r']), 0.15)
        self.assertGreaterEqual(H['bar_x'][0], H['x'] - H['post_w'] / 2 - 0.01)       # and still sits on the post
        self.assertLessEqual(H['bar_x'][1], P.FRAME['r_in'])                           # inside the ring in plan

    def test_usb_socket_is_flush_and_clear_of_the_bumper_band_the_boss_and_the_stack(self):
        U, S = P.USB, P.SCREW
        cx, cy, ang = P.usb_pose()
        a = math.radians(ang)
        ux, uy = math.cos(a), math.sin(a)
        vx, vy = -uy, ux
        corners = [(cx + sx * U['depth'] / 2 * ux + sy * (U['half'] - 0.05) * vx, cy + sx * U['depth'] / 2 * uy + sy * (U['half'] - 0.05) * vy) for sx in (1, -1) for sy in (1, -1)]
        self.assertLessEqual(max(math.hypot(x, y) for x, y in corners), P.R_BODY - 0.02)             # nothing outside the body cylinder
        self.assertGreaterEqual(U['z'][0] - P.BUMPER['z1'], 0.5)                                     # above the bumper band
        jx, jy = P.giga_j12()
        self.assertLessEqual(math.hypot(cx - jx, cy - jy), 3.0)                                      # a short cable to J12
        bx, by = P.polar(S['r'], -12.0)                                                              # the 12 degree boss with its rib to the wall
        rx, ry = P.polar((S['r'] + S['rib_to']) / 2, -12.0)
        half_l = (S['rib_to'] - S['r']) / 2
        boss = Point(bx, by).buffer(S['boss_r']).union(affinity.rotate(box(rx - half_l, ry - S['rib_w'] / 2, rx + half_l, ry + S['rib_w'] / 2), -12.0, origin=(rx, ry)))
        self.assertGreaterEqual(Polygon(corners).distance(boss), 0.5)

    def test_floor_sensors_sit_beside_the_omni_bay_clear_of_the_posts_and_the_wall(self):
        fp, sm = P.FLOOR_FRONT, P.SILVER
        mods = {'FP': box(fp['x'] - fp['w'] / 2, fp['y'] - fp['w'] / 2, fp['x'] + fp['w'] / 2, fp['y'] + fp['w'] / 2),
                'SM': box(sm['x'] - sm['l'] / 2, sm['y'] - sm['w'] / 2, sm['x'] + sm['l'] / 2, sm['y'] + sm['w'] / 2)}
        y0, y1 = P.OMNI_BAY_Y
        bay = box(P.OMNI['rest'][0] - P.OMNI['r'] - 0.15, y0, P.OMNI['rest'][0] + P.OMNI['r'] + 0.15, y1)
        inside_wall = Point(0, 0).buffer(P.R_INT - 0.5)
        for nm, m in mods.items():
            self.assertGreaterEqual(m.distance(bay), 0.3, nm + ' to the omni bay')
            self.assertTrue(inside_wall.contains(m), nm + ' must stay 5 mm inside the wall')
            for i in P.GIGA_USED:
                self.assertGreaterEqual(m.distance(Point(*P.giga_hole_xy(i)).buffer(P.GIGA_POST['r'])), 0.2, '%s to post H%d' % (nm, i + 1))
        self.assertGreaterEqual(sm['x'], 7.0)                                                        # the silver pair is now 7.5 cm ahead of the axle (rev 3: at the axle)
        self.assertGreaterEqual(fp['x'], 7.0)


class StepperBay(unittest.TestCase):
    def test_bay_is_the_checked_orientation_and_clear(self):
        g = P.stepper_geometry()
        (cx, cy), rows = stepper_bay.clearances(P.STEPPER['theta'])
        self.assertAlmostEqual(g['centre'][0], cx, places=2)
        self.assertAlmostEqual(g['centre'][1], cy, places=2)
        self.assertGreaterEqual(min(v for _, v in rows), 4.0)       # mm in plan against the stack, battery, hoppers, wall


class GigaMounting(unittest.TestCase):
    def test_hole_positions_match_the_mount_check(self):
        for i in P.GIGA_USED:
            u, v = P.GIGA_HOLES_MM[i]
            x, y = mount_check.robot_xy(u, v, True, True)
            hx, hy = P.giga_hole_xy(i)
            self.assertAlmostEqual(hx, x, places=3)
            self.assertAlmostEqual(hy, y, places=3)

    def test_posts_clear_the_omni_wheel_and_the_right_cradle(self):
        wheel = box(P.OMNI['rest'][0] - P.OMNI['r'], -P.OMNI['w'] / 2, P.OMNI['rest'][0] + P.OMNI['r'], P.OMNI['w'] / 2)
        C = P.CRADLE
        right = unary_union([
            box(C['web_x'][0], -C['web_y'][1], C['web_x'][1], -C['web_y'][0]),
            box(C['ledge_rear_x'][0], -C['ledge_y'][1], C['ledge_front_x'][1], -C['ledge_y'][0]),
            box(-C['prong_x'][1], -C['prong_y'][1], C['prong_x'][1], -C['prong_y'][0]),
            box(C['slot_x'][0], -C['slot_y'][1], C['slot_x'][1], -C['slot_y'][0]),
            box(-1.0, -6.1, 1.0, -1.4)])                                   # the right motor body
        for i in P.GIGA_USED:
            post = Point(*P.giga_hole_xy(i)).buffer(P.GIGA_POST['r'])
            self.assertGreaterEqual(post.distance(wheel), 0.25, 'post H%d to the omni wheel' % (i + 1))
            self.assertGreaterEqual(post.distance(right), 0.25, 'post H%d to the right cradle and motor' % (i + 1))

    def test_connector_edge_and_j12(self):
        x, y = P.giga_j12()
        self.assertAlmostEqual(x, 7.40, places=2)
        self.assertAlmostEqual(y, -2.74, places=2)

    def test_wifi_antenna_sits_on_the_front_wall_clear_of_the_omni_the_boss_rib_and_the_cable_length(self):
        """The GIGA has no on-board antenna, only the u.FL socket J14 beside the reset button. The antenna that ships with it (a flat flex strip on a 100 mm cable) is stuck on the
        inside of the front wall just right of centre: in plastic, away from the motors and the battery (spec 8.5)."""
        A, S = P.ANTENNA, P.SCREW
        cx, cy, ang = P.antenna_pose()
        a = math.radians(ang)
        ux, uy = math.cos(a), math.sin(a)
        vx, vy = -uy, ux
        corners = [(cx + sx * A['t'] / 2 * ux + sy * A['w'] / 2 * vx, cy + sx * A['t'] / 2 * uy + sy * A['w'] / 2 * vy) for sx in (1, -1) for sy in (1, -1)]
        self.assertLess(max(math.hypot(x, y) for x, y in corners), P.R_INT)                          # inside the wall's inner face, not in it
        self.assertGreaterEqual(A['z'][0] - P.Z_FLOOR_TOP, 1.0)                                      # well above the floor and the floor modules
        self.assertGreaterEqual(S['boss_z0'] - A['z'][1], 0.5)                                       # below the frame boss at z 7.7
        bx, by = P.polar(S['r'], -12.0)
        rx, ry = P.polar((S['r'] + S['rib_to']) / 2, -12.0)
        half_l = (S['rib_to'] - S['r']) / 2
        boss = Point(bx, by).buffer(S['boss_r']).union(affinity.rotate(box(rx - half_l, ry - S['rib_w'] / 2, rx + half_l, ry + S['rib_w'] / 2), -12.0, origin=(rx, ry)))
        self.assertGreaterEqual(Polygon(corners).distance(boss), 0.15)                               # 1.5 mm from the boss rib at -12 degrees
        xs = [x for x, y in corners]
        for wx, wz in (P.OMNI['rest'], P.omni_at(P.OMNI['travel'])):                                 # the omni wheel at rest and fully compressed, seen in the x-z plane
            dx = max(min(xs) - wx, 0.0, wx - max(xs))
            dz = max(A['z'][0] - wz, 0.0, wz - A['z'][1])
            self.assertGreaterEqual(math.hypot(dx, dz) - P.OMNI['r'], 0.5, 'antenna to the omni wheel')
        jx, jy = P.giga_j14()
        run = math.sqrt((cx - jx) ** 2 + (cy - jy) ** 2 + ((A['z'][0] + A['z'][1]) / 2 - P.J14['z']) ** 2)
        self.assertLessEqual(run * 1.3 + 3.0, A['cable'])                                            # routed run (x1.3 + 3 cm, as for the ToF cables) fits the 100 mm cable


class Cradle(unittest.TestCase):
    def test_prong_free_length_is_32_mm(self):
        y0, y1 = P.CRADLE['prong_y']
        self.assertAlmostEqual((y1 - y0) * 10, 32.0, delta=0.5)

    def test_seated_prong_follows_the_motor_without_touching_it(self):
        C = P.CRADLE
        z0, z1 = C['arc_z']
        for k in range(8):
            z = z0 + (z1 - z0) * k / 7
            x_prong = math.sqrt(C['arc_r'] ** 2 - (z - P.AXLE_Z) ** 2)
            x_motor = math.sqrt(P.MOTOR['r'] ** 2 - (z - P.AXLE_Z) ** 2)
            self.assertGreaterEqual(x_prong - x_motor, 0.015)

    def test_plate_sits_in_its_slot_with_clearance(self):
        C, F = P.CRADLE, P.FACE_PLATE
        self.assertGreaterEqual(F['y'][0] - C['ledge_y'][1], 0.04)
        self.assertGreaterEqual(C['web_y'][0] - F['y'][1], 0.04)
        self.assertGreaterEqual(F['y'][0] - C['slot_y'][0], 0.04)
        self.assertGreaterEqual(C['slot_y'][1] - F['y'][1], 0.04)
        self.assertGreaterEqual(P.WHEEL['y0'] - C['web_y'][1], 0.25)         # web to the wheel hub at y 7.0

    def test_single_ear_stress_at_the_gearbox_limit(self):
        xs = [x for x, z in P.FACE_PLATE['pts']]
        ear_arm_mm = (max(xs) + 1.55) / 2 * 10                                # centre of the ear beyond the 15.5 mm body width
        force = 490.0 / ear_arm_mm
        self.assertAlmostEqual(ear_arm_mm, 19.5, delta=0.1)
        self.assertLessEqual(force / 24.0, 1.2)                               # MPa on 8 x 3 mm

    def test_plate_has_one_ear_on_the_front_side(self):
        xs = [x for x, z in P.FACE_PLATE['pts']]
        self.assertGreater(max(xs), 2.0)
        self.assertGreaterEqual(min(xs), -1.6)


class FrameLidHandle(unittest.TestCase):
    def test_handle_passes_through_the_lid_slot(self):
        H, Ld = P.HANDLE, P.LID
        self.assertGreaterEqual(H['bar_x'][0] - Ld['slot_x'][0], 0.3)
        self.assertGreaterEqual(Ld['slot_x'][1] - H['bar_x'][1], 0.3)
        self.assertGreaterEqual(Ld['slot_half'] - H['bar_half'], 0.05)
        self.assertGreaterEqual(Ld['slot_half'] - H['post_w'] / 2, 0.1)

    def test_height_and_post_position(self):
        self.assertAlmostEqual(P.HANDLE['bar_z'][1], 15.6, places=6)
        self.assertLessEqual(P.HANDLE['bar_z'][1], 25.0)
        plate_front = P.PLATE['cx'] + P.PLATE['R']
        self.assertGreaterEqual(P.HANDLE['x'] - P.HANDLE['post_w'] / 2 - plate_front, 0.3)
        self.assertAlmostEqual(P.HANDLE['post_z'][1], P.HANDLE['bar_z'][0], places=6)

    def test_frame_geometry_follows_the_spec(self):
        F, D = P.FRAME, P.DROPPER_FLOOR
        self.assertAlmostEqual(F['r_out'], P.R_BODY, places=6)
        self.assertAlmostEqual(F['z0'], P.Z_TUB_TOP, places=6)
        self.assertAlmostEqual(D['r'], 5.113, places=2)
        self.assertGreaterEqual(F['bridge']['x_rib0'], P.PLATE['cx'] + P.PLATE['R'] + 0.15)
        self.assertGreaterEqual(F['bridge']['x0'] - (P.PLATE['cx'] + D['r']), 0.05)         # the lift-out floor disc does not overlap the frame webs
        self.assertGreaterEqual((P.PLATE['cx'] - D['r']) - F['spoke']['x1'], 0.05)
        self.assertGreaterEqual((P.PLATE['cx'] - D['r']) - F['spoke']['x_rib1'], 0.05)


class DropperUnit(unittest.TestCase):
    def test_unit_lifts_out_clear_of_the_handle_post_and_the_ribs(self):
        """Floor disc, plate, kits, N20 and hoppers leave straight up with the lid off and the two chute channels out: nothing of the frame may stand over them."""
        D, H, B, S = P.DROPPER_FLOOR, P.HANDLE, P.FRAME['bridge'], P.FRAME['spoke']
        post = box(H['x'] - H['post_w'] / 2, -H['post_w'] / 2, H['x'] + H['post_w'] / 2, H['post_w'] / 2)
        brib = box(B['x_rib0'], -B['rib_half'], B['x_rib1'], B['rib_half'])
        srib = box(S['x0'], -S['rib_half'], S['x_rib1'], S['rib_half'])
        disc = Point(P.PLATE['cx'], P.PLATE['cy']).buffer(D['r'])
        plate = Point(P.PLATE['cx'], P.PLATE['cy']).buffer(P.PLATE['R'])
        for name, obstacle in (('handle post', post), ('bridge rib', brib), ('spoke rib', srib)):
            self.assertGreaterEqual(disc.distance(obstacle), 0.1, 'disc to ' + name)
            self.assertGreaterEqual(plate.distance(obstacle), 0.1, 'plate to ' + name)
            for x0, x1, y0, y1 in D['tabs']:
                tab = box(x0, y0, x1, y1)
                self.assertTrue(tab.intersects(disc), 'a tab must grow out of the disc')
                self.assertGreaterEqual(tab.distance(obstacle), 0.05, 'tab to ' + name)

    def test_lid_sits_on_the_ring(self):
        self.assertAlmostEqual(P.LID['z0'], P.FRAME['z1'], places=6)
        self.assertAlmostEqual(P.LID['z1'] - P.LID['z0'], 0.3, places=6)

    def test_six_screws_four_hooks(self):
        self.assertEqual(len(P.screw_angles()), 6)
        self.assertEqual(len(P.hook_angles()), 4)

    def test_wall_strip_above_the_arch_blocks_the_camera_unless_notched(self):
        """Found by the 3D camera rays: the 4 mm strip of wall above the wheel arch lies inside the camera's field of view, so the tub notches it at the window."""
        cam = P.CAM
        ty, tz = cam['tip_r'] * math.sin(math.radians(cam['psi'])), cam['zl']
        band = (math.degrees(math.atan2(P.ARCH['z1'] - tz, P.R_BODY - ty)), math.degrees(math.atan2(P.Z_TUB_TOP - tz, P.R_INT - ty)))     # rays from the lens tip through the strip
        fov = (-cam['tilt'] - cam['vfov'] / 2, -cam['tilt'] + cam['vfov'] / 2)
        self.assertLess(band[0], band[1])
        self.assertTrue(fov[0] < band[1] and band[0] < fov[1], 'strip outside the field of view: the notch would not be needed')
        self.assertAlmostEqual(P.CAM_X, 0.307, places=3)
        self.assertGreaterEqual(P.ARCH['half_x'] - P.CAM_X - (2.8 + 0.2) / 2, 1.0)           # the notch (camera window 2.8 + 0.2) stays well inside the arch


class Masses(unittest.TestCase):
    def test_every_printed_part_has_a_density_and_bought_parts_a_mass(self):
        self.assertAlmostEqual(P.PETG_G_CM3, 1.27, places=2)
        self.assertTrue(0.2 <= P.PRINT_FILL <= 1.0)
        self.assertIn('Tub', P.PRINTED)
        self.assertIn('Battery', P.BOUGHT_G)
        self.assertEqual(set(P.PRINTED) & set(P.BOUGHT_G), set())


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run the tests to verify they fail**

Run (from `docs/superpowers/specs/v3-checks`):

```bash
PYTHONPATH="C:/Users/christopher.shu/pl4" python -m unittest test_v3_params4 -v
```

Expected: FAIL at import with `ModuleNotFoundError: No module named 'v3_params4'`.

- [ ] **Step 3: Write the parameter module**

Create `docs/superpowers/specs/v3-checks/v3_params4.py`:

```python
"""Revision 4 parameters (docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md): the single source for the rev 4 Fusion model and its tests.
cm and degrees; x forward, y left, z up; origin at the midpoint of the drive axle on the floor line (axle z 4.0).
Everything that did not change is re-exported from v3_params (rev 3). No Fusion imports here, so the tests can run anywhere."""
import math

from v3_params import *  # noqa: F401,F403  R_BODY R_INT R_SWEPT Z_BELLY Z_ROOF Z_LID AXLE_Z WHEEL MOTOR NUB CHAMFER BUMPER FLOOR_FRONT SILVER BUMPER_SW_DEG TOF_Z TOF_W TOF_H TOF_T CAM PLATE
from v3_params import PART_SIZES, PLATE, R_BODY, load_pack, slot_xy

Z_TUB_TOP = 8.7             # top of the tub wall = underside of the upper frame
Z_FLOOR_TOP = 3.9           # tub floor z 3.5 to 3.9
FLOOR_T = 0.4
KIT = 1.03                  # kit 10.3 mm
CAM_X = CAM['tip_r'] * math.cos(math.radians(CAM['psi']))      # x of both camera lens tips (0.307): centre of the camera windows and humps


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


# ---- front omni, inside the body (spec section 4)
OMNI_ARM_LEN = math.hypot(7.95 - 3.5, 3.0 - 3.6)           # 4.49 cm, the rev 3 arm length, kept
OMNI = dict(r=3.0, w=2.0, rest=(7.0, 3.0), travel=2.5, arm_y=(1.1, 1.5), arm_half=0.55, pin_r=0.2, axle_r=0.2, pin_y=(0.9, 1.7))
OMNI['pivot'] = (OMNI['rest'][0] - math.sqrt(OMNI_ARM_LEN ** 2 - 0.6 ** 2), 3.6)          # (2.55, 3.6)
OMNI_BAY_Y = (-1.15, 1.75)                                  # floor opening for the wheel and the arm (the arm is on +y)
OMNI_ARM_SLOT_Y = (0.85, 1.75)                              # floor slot for the arm and its pin along the whole swing


def omni_at(travel):
    """Axle position when the sprung arm has been pushed up by `travel` cm (the axle moves on an arc about the pivot)."""
    ox, oz = OMNI['rest']
    px, pz = OMNI['pivot']
    zc = oz + travel
    return px + math.sqrt((ox - px) ** 2 + (oz - pz) ** 2 - (zc - pz) ** 2), zc


# ---- ToF modules, moved inward (3D finding 1: all corners inside r 10.23)
_C25, _S25 = math.cos(math.radians(25)), math.sin(math.radians(25))
TOF = [('F', 9.5, 0.0, 0), ('FL', 9.9 * _C25, 9.9 * _S25, 25), ('FR', 9.9 * _C25, -9.9 * _S25, -25),
       ('SFL', 7.18, 6.0, 90), ('SFR', 7.18, -6.0, -90), ('SRL', -7.18, 6.0, 90), ('SRR', -7.18, -6.0, -90),
       ('RL', -8.4, 4.5, 180), ('RR', -8.4, -4.5, 180)]                                   # name, x, y, aim (deg, 0 = forward, 90 = left)

# ---- hopper and 13 mm square channel (spec section 6.2)
CHUTE = dict(in_w=1.3, wall=0.16, exit_x=-6.0, exit_axis_z=4.15, start_z=7.95, hopper_out=1.95, hopper_in=1.45, flange=2.35, flange_t=0.15, hopper_z0=7.2, hole_clear=0.04)
CHUTE['out_w'] = CHUTE['in_w'] + 2 * CHUTE['wall']          # 1.62
CHUTE['exit_y'] = math.sqrt(R_BODY ** 2 - CHUTE['exit_x'] ** 2)


def chute_ends(side):
    """Axis from the slot centre (z 7.95) to the exit on the body radius (z 4.15). side +1 = left (slot B), -1 = right (slot A)."""
    sx, sy = slot_xy('B' if side > 0 else 'A')
    return (sx, sy, CHUTE['start_z']), (CHUTE['exit_x'], side * CHUTE['exit_y'], CHUTE['exit_axis_z'])


# ---- upper frame, dropper unit, lid, handle, front controls (spec sections 3.1, 6.1, 7)
FRAME = dict(r_in=9.0, r_out=10.5, z0=8.7, z1=11.2, floor_t=0.3,
             bridge=dict(x0=3.2, x1=9.2, half=1.25, x_rib0=3.25, x_rib1=9.05, rib_half=0.5, rib_z1=9.8),
             spoke=dict(x0=-9.2, x1=-7.2, half=1.0, x_rib1=-7.9, rib_half=0.5, rib_z1=9.5),
             deck=dict(x0=5.2, x1=9.15, y0=-1.25, y1=3.6))          # control deck: the bridge web widened to the +y side in front of the handle post; the -y edge stays at the bridge's, so the GIGA's connector edge (y -1.36 and beyond) stays open from above
# The dropper unit (floor disc, plate, kits, N20, hoppers) lifts out in one piece. The disc is 3 mm thick, R 5.11, and rests on three half-lap seats of the frame webs:
# a tab (top half of the disc thickness, z 8.85 to 9.0) lies in a rebate of the web's top half on the web's lower half (z 8.7 to 8.85). Tabs: x0, x1, y0, y1; two beside the bridge rib (clear of the handle post, half width 0.7), one on the spoke.
DROPPER_FLOOR = dict(r=PLATE['R'] + 0.2, t=0.3, n20_pocket=(1.04, 1.24), n20_recess=(1.64, 1.84, 0.2),
                     tabs=((3.0, 3.9, 0.85, 1.25), (3.0, 3.9, -1.25, -0.85), (-7.5, -6.9, -0.5, 0.5)), seat_margin=0.05)
HANDLE = dict(x=4.0, post_w=1.4, post_z=(9.7, 14.0), bar_x=(3.3, 8.7), bar_half=0.8, bar_z=(14.0, 15.6),
              led=dict(x=8.35, y=0.0, r=0.25, z=(15.6, 16.0)))      # victim LED on top of the bar's front end: the rules want it clearly visible to the referee, so it goes on the highest point of the robot      # the bar starts in front of the dropper unit's disc (x 3.11): found by the 3D removal check, a bar from x 0 stood over the plate
SCREW_DEG = (12.0, 60.0, 118.0)                             # +- each; free ranges from ring_gaps.py. 118 (not 101): at 101 the boss hung on the wheel's top rim and in the camera notch's lee (3D removal check)
SCREW = dict(r=9.75, hole_r=0.165, cbore_r=0.3, cbore_depth=0.3, boss_r=0.6, boss_z0=7.7, insert_r=0.2, insert_depth=0.6, rib_to=10.4, rib_w=1.0)       # the boss has a rib to the wall (r 10.3 to 10.5)
HOOK_DEG = (71.0, 114.0)
HOOK = dict(r_in=10.34, r_out=10.5, skirt_z0=10.4, half_deg=5.5, groove_half_deg=4.5, groove_r_in=10.24, bump_z=(10.55, 10.85))
LID = dict(z0=11.2, z1=11.5, slot_x=(2.95, 9.05), slot_half=0.9, front_notch=(4.9, 9.2, 0.7, 3.5), finger_deg=12.0, hump_skin=0.2)       # front_notch: x0, x1, y0, y1, over the control deck; it joins the handle slot
CONTROLS = (('Start button', 7.55, 2.15, ('round', 0.6), 9.0, 9.7, '#639922'), ('Power switch', 6.0, 2.15, ('rect', 1.3, 1.9), 9.0, 10.3, '#3B6D11'),
            ('Status LED 1', 7.5, 0.0, ('round', 0.2), 9.8, 10.2, '#888780'), ('Status LED 2', 6.6, 0.0, ('round', 0.2), 9.8, 10.2, '#888780'))              # name, x, y, shape (round: radius; rect: x size, y size), z0, z1, colour: the button and switch on the +y side of the deck, the two status LEDs in a row on top of the bridge rib (z 9.8), all reached through the lid's slot and front notch; the victim LED is on the handle bar (HANDLE['led'])
USB = dict(angle=-24.0, r_face=10.45, depth=1.3, half=0.7, z=(7.1, 7.9))   # USB-C service socket in the front-right wall above the bumper band: front face flush (r 10.45 on the axis, 10.47 at the edges), 13 mm deep; at -24 degrees it is 8 mm from the 12 degree boss rib


def usb_pose():
    """Centre of the socket body in plan (x, y) and its direction in degrees."""
    r = USB['r_face'] - USB['depth'] / 2
    a = math.radians(USB['angle'])
    return r * math.cos(a), r * math.sin(a), USB['angle']


# ---- floor sensors, moved forward for earlier detection (spec 5.3): one each side of the omni bay, 7.5 cm ahead of the axle (rev 3: FP at 6.5, SM at the axle)
FLOOR_FRONT = dict(FLOOR_FRONT, x=7.5, y=3.2)
SILVER = dict(SILVER, x=7.5, y=-2.7)


def screw_angles():
    return SCREW_DEG + tuple(-a for a in SCREW_DEG)


def hook_angles():
    return HOOK_DEG + tuple(-a for a in HOOK_DEG)


# ---- wheel arches, motor cradle and face plate (spec sections 5.1, 5.2)
ARCH = dict(half_x=4.4, z0=3.5, z1=8.3, y0=6.9, y1=10.8)
CRADLE = dict(
    seat_r=1.015, seat_y=(1.3, 6.45),
    prong_x=(1.06, 1.30), prong_y=(2.85, 6.05), prong_z=(3.85, 4.80), tip=(0.80, 4.80), arc_r=1.02, arc_z=(4.65, 4.0),
    ledge_y=(5.65, 6.05), ledge_front_x=(1.06, 2.75), ledge_rear_x=(-1.6, -1.06), ledge_z=(3.85, 4.9),
    web_y=(6.45, 6.7), web_x=(-1.6, 2.75), web_z=(3.85, 5.2), notch_half=0.3,
    slot_x=(-1.6, 2.45), slot_y=(6.05, 6.45), slot_z=(3.45, 3.95))              # slot cut through the floor for the face plate
FACE_PLATE = dict(y=(6.1, 6.4), pts=[(-1.55, 3.55), (2.35, 3.55), (2.35, 4.6), (1.55, 4.6), (1.55, 5.1), (-1.55, 5.1)], bore_r=0.22)   # (x, z): one ear on the +x side

# ---- GIGA posts (spec section 8.1): connector edge forward, header edge inboard
GIGA_HOLES_MM = [(15.24, 2.54), (90.17, 2.54), (13.9, 50.7), (96.7, 50.7), (66.1, 17.8), (66.1, 45.6)]      # datasheet p.18: (u from the connector edge, v from the header edge)
GIGA_USED = (0, 1, 2, 3)
GIGA_POST = dict(r=0.35, insert_r=0.2, insert_depth=0.6, z=(3.9, 6.15))
J12 = dict(from_header=1.38, out=0.3, half=0.45, z=(6.31, 6.66))


def giga_stack():
    g = load_pack()[0]['GIGA R1 + main PCB stack']
    length, width = PART_SIZES['GIGA R1 + main PCB stack']
    return g['x'], g['y'], length, width


def giga_hole_xy(i):
    cx, cy, length, width = giga_stack()
    u, v = GIGA_HOLES_MM[i]
    return cx + length / 2 - u / 10.0, cy + width / 2 - v / 10.0


def giga_j12():
    cx, cy, length, width = giga_stack()
    return cx + length / 2, cy + width / 2 - J12['from_header']


# ---- Wi-Fi/Bluetooth antenna (spec 8.5). The GIGA has no on-board antenna: only the u.FL socket J14 at the front-inner corner, beside the reset button, with a flat flex antenna in the box
# (Molex 206994-0100, about 15.4 x 6.4 mm on a 100 mm cable). It is stuck on the inside of the tub's front wall just right of centre.
J14 = dict(from_header=0.5, from_edge=0.22, z=6.4)             # read from the datasheet picture (+-1 mm); the socket top is about 1 mm over the board
ANTENNA = dict(angle=-6.0, r_face=10.29, w=0.64, t=0.05, z=(5.5, 7.04), cable=10.0)       # r_face 10.29: 0.1 mm off the wall's inner face (r 10.3); w tangential, t radial, z range = the 15.4 mm strip


def giga_j14():
    cx, cy, length, width = giga_stack()
    return cx + length / 2 - J14['from_edge'], cy + width / 2 - J14['from_header']


def antenna_pose():
    """Centre of the antenna strip in plan (x, y) and its direction in degrees (its thin side faces the robot's centre)."""
    r = ANTENNA['r_face'] - ANTENNA['t'] / 2
    a = math.radians(ANTENNA['angle'])
    return r * math.cos(a), r * math.sin(a), ANTENNA['angle']


# ---- battery tray (spec section 8.2) and the 28BYJ-48 bay (spec section 6.3)
TRAY = dict(z=(4.9, 5.1), margin=0.1, notch=(-1.2, 1.2, 1.3, 6.2), stop=(3.65, 0.2, 0.6), legs=((3.0, 1.2), (0.8, -1.4), (-0.6, 0.2)), leg_r=0.3)   # stop: local x, thickness, height; legs: local x, y (clear of the floor-port hole and the cradle)


def battery_pose():
    b = load_pack()[0]['battery (placeholder 7.0x3.5x2.5)']
    return b['x'], b['y'], b['angle']


STEPPER = dict(theta=230.0, offset=0.8, r=1.4, z=(6.8, 8.7), ear_span=1.75, ear_from=1.2, ear_r=0.35, ear_z=(8.0, 8.7), block_w=1.46, block_from=1.2, block_to=1.7)


def stepper_geometry():
    """Body centre, ear centres and wire-block centre of the 28BYJ-48 bay: the shaft sits on the plate axis, the body 0.8 cm away from it."""
    t = math.radians(STEPPER['theta'])
    ux, uy = math.cos(t), math.sin(t)
    vx, vy = -uy, ux
    cx, cy = PLATE['cx'] - STEPPER['offset'] * ux, PLATE['cy'] - STEPPER['offset'] * uy
    ears = [(cx + s * STEPPER['ear_span'] * vx, cy + s * STEPPER['ear_span'] * vy) for s in (1, -1)]
    mid = (STEPPER['block_from'] + STEPPER['block_to']) / 2
    return dict(centre=(cx, cy), ears=ears, block=(cx - mid * ux, cy - mid * uy), dir=(ux, uy))


# ---- masses for the centre-of-mass report (rev 3 budget in com.py for the bought parts; printed parts from the model volume) [placeholder]
PETG_G_CM3 = 1.27
PRINT_FILL = 0.45
PRINTED = ('Tub', 'Upper frame', 'Dropper floor', 'Lid', 'Handle bar', 'Hopper A right', 'Hopper B left', 'Chute A right', 'Chute B left', 'Dropper plate',
           'Face plate L', 'Face plate R', 'N20 face plate', 'Omni arm', 'GIGA posts', 'Bumper L', 'Bumper R')
BOUGHT_G = {'Wheel L': 55.0, 'Wheel R': 55.0, 'Motor L': 95.0, 'Motor R': 95.0, 'Omni wheel': 30.0, 'Omni pins': 5.0, 'N20 motor': 12.0,
            'Arduino GIGA R1': 55.0, 'Main PCB': 25.0, 'Battery': 110.0, 'Bumper switch L': 1.0, 'Bumper switch R': 1.0, 'Kits': 48.0,
            'Camera L': 27.5, 'Camera R': 27.5, 'Rear nub': 5.0, 'Power switch': 8.0, 'Start button': 3.0, 'Victim LED': 1.0,
            'Status LED 1': 1.0, 'Status LED 2': 1.0, 'USB-C service socket': 4.0, 'Floor port FP': 4.0, 'Silver module SM': 4.0, 'Wi-Fi antenna': 1.0}
for _nm, _x, _y, _a in TOF:
    BOUGHT_G['ToF ' + _nm] = 3.3
UNMODELLED_G = (('mux, regulator, wiring', 55.0, 0.0, 6.0), ('BNO055, TCS34725, LCD, hardware', 70.0, 0.0, 7.0), ('cables and fasteners', 40.0, 0.0, 8.0))     # name, g, x, z


if __name__ == '__main__':
    print('pivot', [round(v, 3) for v in OMNI['pivot']], 'compressed axle', [round(v, 3) for v in omni_at(OMNI['travel'])])
    print('chute A', [[round(v, 3) for v in p] for p in chute_ends(-1)])
    print('GIGA holes', [[round(v, 3) for v in giga_hole_xy(i)] for i in GIGA_USED], 'J12', [round(v, 3) for v in giga_j12()])
    print('stepper', {k: v for k, v in stepper_geometry().items()})
```

- [ ] **Step 4: Run the tests to verify they pass**

Run:

```bash
PYTHONPATH="C:/Users/christopher.shu/pl4" python -m unittest test_v3_params4 -v
PYTHONPATH="C:/Users/christopher.shu/pl4" python v3_params4.py
```

Expected: every test `ok`, `Ran 38 tests ... OK`, and the second command prints the pivot `[2.55, 3.6]`, the compressed axle `[6.618, 5.5]`, the four GIGA holes `(5.872, -1.617) (-1.621, -1.617) (6.006, -6.433) (-2.274, -6.433)`, J12 `[7.396, -2.743]` and the stepper centre `(-1.486, 0.613)`. If `test_screws_and_hooks_keep_3_mm_from_every_window` fails, the margin is below 2.8 mm: move that angle inside the free range printed by `python ring_gaps.py` and update the spec, do not loosen the test.

- [ ] **Step 5: Checkpoint (no commit)**

Nothing is committed (user rule). Confirm the two files exist and that `git status --short docs/superpowers/specs/v3-checks` lists them as untracked.

---

### Task 2: Fusion plumbing, the runner and the self-test

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/fusion_lib.py:201-206` (`axis_tool`)
- Modify: `docs/superpowers/specs/v3-fusion/v3_checks3d.py:112-159` (`omni_sweep`)
- Create: `docs/superpowers/specs/v3-fusion/v4_run.py`
- Create: `docs/superpowers/specs/v3-fusion/v4_checks3d.py`
- Create: `docs/superpowers/specs/v3-fusion/v4_model.py`

**Interfaces:**
- Consumes: `fusion_lib` (`prism`, `ring_prism`, `move_body`, `combine`, `new_part`, `reset_design`, `frame_from_zy`, `world_bbox`, `Palette`), `v3_model` (`Ctx`, `tof_rect`, `t_exit`, builders), `v3_params4`.
- Produces: `fusion_lib.axis_prism(comp, name, shape, p_start, p_end)`; `v3_checks3d.omni_sweep(ctx, steps=13, omni=None, omni_at=None, travel=None)`; `v4_model.STAGES`, `v4_model.stage(name)` decorator, `v4_model.ymirror(s, a, b)`, `v4_model.make_ctx()`, `v4_model.build(stages=None, reset=False)`; `v4_checks3d.PROBES`, `probes(stage)` decorator, `has_material(ctx, name, pt)`, `check_stage(ctx, stage)`, `local_interference(ctx, new_names, exclude=(), floor=1e-3)`, `STAGE_COMPONENTS`, `ALLOW`, `selftest(ctx)`; `v4_run.stage(names, reset=False, build=True, check=True)`, `v4_run.report(kind, **kw)`, `v4_run.selftest()`, `v4_run.show(hide=(), ghost=False)`.

- [ ] **Step 1: Confirm the Fusion document before anything is reset**

Call `mcp__Autodesk_Fusion__fusion_mcp_read` with `queryType: "document"`, `operation: "open"`. Expected: the open documents include an active, modified document that holds the rev 3 layout model (an unsaved "Untitled" or a name the user gave it). Also run `ls -la docs/superpowers/specs/v3-fusion/Theseus_V3_baseline.f3d` (expected: about 2.9 MB). If the active document is anything else, or the baseline export is missing, STOP and ask the user before continuing: Task 3 uses `reset=True`.

- [ ] **Step 2: Add `axis_prism` to `fusion_lib.py`**

Replace the existing `axis_tool` (lines 201 to 206) with:

```python
def axis_prism(comp, name, shape, p_start, p_end):
    """Extrude a 2D `shape` (drawn on a local xy plane, centred on the axis) along the straight line p_start -> p_end; returns the new body.
    The local y axis points as close to world z as possible, so a rectangle has two faces that stay vertical-ish along a sloped axis."""
    d = vsub(p_end, p_start)
    body = prism(comp, name, 'xy', [shape], 0.0, vlen(d))
    move_body(comp, body, frame_from_zy(p_start, d, (0.0, 0.0, 1.0) if abs(unit(d)[2]) < 0.99 else (1.0, 0.0, 0.0)))
    return body


def axis_tool(comp, name, p_start, p_end, r):
    """Solid cylinder of radius r from p_start to p_end (a straight tool body for cutting sloped holes)."""
    return axis_prism(comp, name, circle((0.0, 0.0), r), p_start, p_end)
```

- [ ] **Step 3: Let `omni_sweep` take the omni geometry as arguments**

In `v3_checks3d.py` replace the header and the first lines of `omni_sweep` (from `def omni_sweep(ctx, steps=13):` down to `a_rest = math.atan2(oz - pz, ox - px)`) with:

```python
def omni_sweep(ctx, steps=13, omni=None, omni_at=None, travel=None):
    """Push the sprung omni, its arm plates and pins through the full mechanical travel and intersect with every other body (temporary B-reps, nothing is changed).
    The defaults are the rev 3 omni (prm.OMNI, M.omni_at, M.OMNI_TRAVEL_MECH); rev 4 passes its own."""
    omni = omni or prm.OMNI
    omni_at = omni_at or M.omni_at
    travel = M.OMNI_TRAVEL_MECH if travel is None else travel
    tbm = adsk.fusion.TemporaryBRepManager.get()
    movers, others = [], []
    for i in range(ctx.root.occurrences.count):
        o = ctx.root.occurrences.item(i)
        for j in range(o.bRepBodies.count):
            (movers if o.component.name in ('Omni wheel', 'Omni arm', 'Omni pins') else others).append((o.component.name, o.bRepBodies.item(j)))
    px, pz = omni['pivot']
    ox, oz = omni['rest']
    a_rest = math.atan2(oz - pz, ox - px)
```

and, further down in the same function, replace `trav = M.OMNI_TRAVEL_MECH * k / (steps - 1)` / `xc, zc = M.omni_at(trav)` with `trav = travel * k / (steps - 1)` / `xc, zc = omni_at(trav)`, and in the final `print` replace `M.OMNI_TRAVEL_MECH` with `travel`.

Check: run `grep -n "OMNI_TRAVEL_MECH\|M.omni_at" docs/superpowers/specs/v3-fusion/v3_checks3d.py`. Expected: three lines remain: the docstring sentence `The defaults are the rev 3 omni (...)`, `omni_at = omni_at or M.omni_at` and `travel = M.OMNI_TRAVEL_MECH if travel is None else travel`.

- [ ] **Step 4: Create `v4_run.py`**

```python
"""Entry points for the Fusion MCP script runner (see README.md). Every call reloads the repo modules, so edits take effect without restarting Fusion.
Inside a Fusion script:
    import sys; sys.path.insert(0, r"<repo>\\docs\\superpowers\\specs\\v3-fusion"); import v4_run
    def run(context): print(v4_run.stage(['tub'], reset=True))"""
import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CHECKS = os.path.join(os.path.dirname(HERE), 'v3-checks')
for _p in (HERE, CHECKS):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load():
    mods = {}
    for n in ('fusion_lib', 'v3_params', 'v3_params4', 'v3_model', 'v3_checks3d', 'v4_model', 'v4_checks3d'):
        mods[n] = importlib.reload(importlib.import_module(n))
    return mods['v4_model'], mods['v4_checks3d']


def stage(names, reset=False, build=True, check=True):
    """Build the named stages ('all' for every stage), then run each stage's probes and the local interference of its components. Returns a report string."""
    M, K = _load()
    ctx = M.build(names, reset=reset) if build else M.make_ctx()
    wanted = [n for n, _ in M.STAGES] if names in (None, 'all') else (list(names) if isinstance(names, (list, tuple)) else [names])
    out, ok = [], True
    if check:
        for nm in wanted:
            good, lines = K.check_stage(ctx, nm)
            ok = ok and good
            out += lines
    out.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(out)


def report(kind, **kw):
    """Run one whole-model report from v4_checks3d: interference, envelope, belly, removal, access, mass, clearances, tof, camera, omni, stepper."""
    M, K = _load()
    return getattr(K, 'report_' + kind)(M.make_ctx(), **kw)


def selftest():
    M, K = _load()
    return K.selftest(M.make_ctx())


def show(hide=(), ghost=False):
    """Show every component except the named ones (and except the hidden 28BYJ-48 bay unless ghost=True); used before screenshots."""
    M, K = _load()
    hidden = tuple(hide) + (() if ghost else ('Stepper bay 28BYJ-48',))
    return sys.modules['v3_model'].show_only(M.make_ctx(), hide=hidden)
```

- [ ] **Step 5: Create `v4_checks3d.py` (core only)**

```python
"""Checks on the rev 4 Fusion model (run inside Fusion through v4_run.py): per-stage probe tests and local interference now, whole-model reports in Task 9.
A probe asks "is there material at this point of this component"; the probes pin the features each build stage must produce."""
import math

import adsk.core
import adsk.fusion

import fusion_lib as L
import v3_checks3d as K3
import v3_params4 as P

P3, V3 = L.P3, L.V3
PROBES = {}                       # stage name -> function returning rows (component, (x, y, z), expect_solid, label)
STAGE_COMPONENTS = {}             # stage name -> components the stage creates (for the local interference test)
ALLOW = {}                        # frozenset({component a, component b}) -> largest accepted overlap in cm3; every entry needs a comment saying why


def probes(stage):
    def deco(fn):
        PROBES[stage] = fn
        return fn
    return deco


def _occ(ctx, name):
    for i in range(ctx.root.occurrences.count):
        o = ctx.root.occurrences.item(i)
        if o.component.name == name:
            return o
    raise KeyError(name)


def _bodies(ctx, name):
    o = _occ(ctx, name)
    return [o.bRepBodies.item(j) for j in range(o.bRepBodies.count)]


def has_material(ctx, name, pt):
    """True if the point lies strictly inside a body of the component (a point on a face counts as empty)."""
    inside = adsk.fusion.PointContainment.PointInsidePointContainment
    return any(b.pointContainment(P3(*pt)) == inside for b in _bodies(ctx, name))


def local_interference(ctx, new_names, exclude=(), floor=1e-3):
    """Overlap volumes (cm3) of the bodies of `new_names` with every other body and with each other, using temporary B-reps with a bounding-box prefilter."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    new, others = [], []
    for i in range(ctx.root.occurrences.count):
        o = ctx.root.occurrences.item(i)
        nm = o.component.name
        if nm in exclude:
            continue
        for j in range(o.bRepBodies.count):
            (new if nm in new_names else others).append((nm, o.bRepBodies.item(j)))
    rows = {}

    def overlap(a, b):
        bb1, bb2 = a.boundingBox, b.boundingBox
        if (bb1.maxPoint.x < bb2.minPoint.x or bb1.minPoint.x > bb2.maxPoint.x or bb1.maxPoint.y < bb2.minPoint.y or bb1.minPoint.y > bb2.maxPoint.y
                or bb1.maxPoint.z < bb2.minPoint.z or bb1.minPoint.z > bb2.maxPoint.z):
            return 0.0
        t1, t2 = tbm.copy(a), tbm.copy(b)
        try:
            return t1.volume if tbm.booleanOperation(t1, t2, adsk.fusion.BooleanTypes.IntersectionBooleanType) else 0.0
        except Exception:
            return 0.0

    for k, (nn, nb) in enumerate(new):
        for on, ob in others + new[k + 1:]:
            if on == nn:
                continue
            v = overlap(nb, ob)
            if v > floor:
                key = tuple(sorted((nn, on)))
                rows[key] = max(rows.get(key, 0.0), v)
    return sorted(((v, a, b) for (a, b), v in rows.items()), reverse=True)


def check_stage(ctx, stage):
    """Probe tests and local interference for one stage. Returns (ok, lines)."""
    lines = ['stage %s' % stage]
    ok = True
    fn = PROBES.get(stage)
    if fn is None:
        lines.append('  no probes defined')
    else:
        for comp, pt, want, label in fn():
            try:
                got = has_material(ctx, comp, pt)
            except KeyError:
                ok = False
                lines.append('  FAIL %-26s component missing (%s)' % (comp, label))
                continue
            good = (got == want)
            ok = ok and good
            lines.append('  %s %-26s %-46s expected %-5s got %s' % ('ok  ' if good else 'FAIL', comp, label, 'solid' if want else 'empty', 'solid' if got else 'empty'))
    names = STAGE_COMPONENTS.get(stage, [])
    if names:
        rows = local_interference(ctx, names, exclude=('Stepper bay 28BYJ-48',))
        bad = [(v, a, b) for v, a, b in rows if v > ALLOW.get(frozenset((a, b)), 0.0) + 1e-3]
        for v, a, b in rows:
            lines.append('  %s overlap %9.4f cm3  %s  x  %s' % ('FAIL' if (v, a, b) in bad else 'ok  ', v, a, b))
        if not rows:
            lines.append('  ok   no overlap with any other component')
        ok = ok and not bad
    return ok, lines


def selftest(ctx):
    """Throw-away component: a box and a sloped square tube. Checks the probe API (point containment in world coordinates) and axis_prism."""
    occ, comp = L.new_part(ctx.root, 'Selftest')
    L.prism(comp, 'Box', 'xy', [L.rect(0.5, 0.5, 1.0, 1.0)], 0.0, 1.0)
    L.prism(comp, 'Slab xz', 'xz', [L.rect(0.5, 0.5, 1.0, 1.0)], 2.0, 3.0)              # sketch (x, z), extruded along +y from 2 to 3
    L.prism(comp, 'Slab yz', 'yz', [L.rect(0.5, 0.5, 1.0, 1.0)], 4.0, 5.0)              # sketch (y, z), extruded along +x from 4 to 5
    tube = L.axis_prism(comp, 'Tube', L.rect(0, 0, 1.0, 1.0), (4.0, 0.0, 2.0), (7.0, 0.0, 5.0))
    inner = L.axis_prism(comp, 'Bore', L.rect(0, 0, 0.6, 0.6), (3.5, 0.0, 1.5), (7.5, 0.0, 5.5))
    L.combine(comp, tube, [inner])
    rows = [('box inside', (0.5, 0.5, 0.5), True), ('box outside', (1.5, 0.5, 0.5), False), ('xz slab on +y', (0.5, 2.5, 0.5), True), ('xz slab not on -y', (0.5, -2.5, 0.5), False),
            ('yz slab on +x', (4.5, 0.5, 0.5), True), ('yz slab not on -x', (-4.5, 0.5, 0.5), False),
            ('tube wall', (5.5, 0.4, 3.5), True), ('tube bore', (5.5, 0.0, 3.5), False), ('beyond the tube end', (7.6, 0.0, 5.6), False)]
    ok = True
    out = []
    for label, pt, want in rows:
        got = has_material(ctx, 'Selftest', pt)
        ok = ok and got == want
        out.append('  %s %-20s expected %-5s got %s' % ('ok  ' if got == want else 'FAIL', label, want, got))
    occ.deleteMe()
    out.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(out)
```

- [ ] **Step 6: Create `v4_model.py` (skeleton)**

```python
"""Theseus V3 revision 4 as a Fusion assembly (docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md). Run inside Fusion through v4_run.py.
Units cm; x forward, y left, z up; origin at the drive-axle midpoint on the floor line. Numbers come from ../v3-checks/v3_params4.py.
Unchanged rev 3 builders (wheels and motors, nub, bumpers, cameras, GIGA stack, battery, floor sensors) are reused from v3_model."""
import math
import os
import sys

import adsk.core
import adsk.fusion
import fusion_lib as L
from fusion_lib import circle, poly, rect, stadium, sector, NEW, JOIN, CUT

HERE = os.path.dirname(os.path.abspath(__file__))
CHECKS = os.path.join(os.path.dirname(HERE), 'v3-checks')
if CHECKS not in sys.path:
    sys.path.insert(0, CHECKS)
import v3_params4 as P
import v3_model as M3

STAGES = []                        # (name, builder) in build order; each task appends its stages


def stage(name):
    def deco(fn):
        STAGES.append((name, fn))
        return fn
    return deco


def ymirror(s, a, b):
    """The interval a..b along y for the left side (s = +1), mirrored for the right side (s = -1)."""
    return (a, b) if s > 0 else (-b, -a)


def rot(cx, cy, deg, lx, ly):
    """World position of the local point (lx, ly) of a frame centred on (cx, cy) and turned by deg."""
    a = math.radians(deg)
    return cx + lx * math.cos(a) - ly * math.sin(a), cy + lx * math.sin(a) + ly * math.cos(a)


def make_ctx():
    ctx = M3.Ctx('fixed')
    for nm, x, y, aim in P.TOF:                       # the rev 3 'fixed' ToF positions are the rev 4 ones
        got = ctx.tof[nm]
        assert abs(got[0] - x) < 1e-6 and abs(got[1] - y) < 1e-6 and got[2] == aim, 'ToF %s differs between v3_model and v3_params4' % nm
    return ctx


def build(stages=None, reset=False):
    ctx = make_ctx()
    if reset:
        L.reset_design(ctx.design)
    names = [s[0] for s in STAGES]
    wanted = names if stages in (None, 'all') else (list(stages) if isinstance(stages, (list, tuple)) else [stages])
    for nm, fn in STAGES:
        if nm in wanted:
            fn(ctx)
            print('stage', nm, 'done; timeline', ctx.design.timeline.count)
    return ctx
```

- [ ] **Step 7: Run the self-test in Fusion**

Call `mcp__Autodesk_Fusion__fusion_mcp_execute` (`featureType: "script"`) with:

```python
import sys
sys.path.insert(0, r"C:\Users\christopher.shu\OneDrive - St. Andrew's College\Desktop\Robocup-Junoir-Theseus\docs\superpowers\specs\v3-fusion")
import v4_run


def run(context):
    print(v4_run.selftest())
```

Expected:

```
  ok   box inside           expected True  got True
  ok   box outside          expected False got False
  ok   xz slab on +y        expected True  got True
  ok   xz slab not on -y    expected False got False
  ok   yz slab on +x        expected True  got True
  ok   yz slab not on -x    expected False got False
  ok   tube wall            expected True  got True
  ok   tube bore            expected False got False
  ok   beyond the tube end  expected False got False
RESULT: PASS
```

If the import fails (path with an apostrophe, a missing module) fix the path string first. If `pointContainment` raises `AttributeError`, search the API with `fusion_mcp_read` (`queryType: "apiDocumentation"`, `searchPattern: "pointContainment"`) and adapt `has_material`. If an `xz` or `yz` slab row fails, the extrusion direction of that base plane differs from the `fusion_lib` docstring (xz along +y, yz along +x): every probe and builder of the plan assumes it, so stop and report it. If the tube wall or bore rows fail, the `frame_from_zy` orientation of `axis_prism` differs from what the probes assume: print `body.boundingBox` of the tube and correct the probe points, not the helper (the rev 3 chute holes relied on the same helper).

- [ ] **Step 8: Checkpoint (no commit)**

`git status --short docs/superpowers/specs/v3-fusion` should list `v4_model.py`, `v4_checks3d.py`, `v4_run.py` as untracked and nothing else new. Rerun the unit tests from Task 1 once: `PYTHONPATH="C:/Users/christopher.shu/pl4" python -m unittest test_v3_params4` (expected: OK).

---
### Task 3: The tub

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append the tub probes and the stage entry)
- Modify: `docs/superpowers/specs/v3-fusion/v4_model.py` (append `_cradle`, `build_tub`)

**Interfaces:**
- Consumes: `P.R_BODY`, `P.R_INT`, `P.Z_BELLY`, `P.Z_FLOOR_TOP`, `P.Z_TUB_TOP`, `P.CAM_X`, `M3.CAM_WINDOW_W`, `P.CHAMFER`, `P.ARCH`, `P.CRADLE`, `P.SCREW`, `P.OMNI`, `P.OMNI_BAY_Y`, `P.OMNI_ARM_SLOT_Y`, `P.omni_at`, `P.FLOOR_FRONT`, `P.SILVER`, `P.CHUTE`, `P.chute_ends`, `P.BUMPER`, `P.BUMPER_SW_DEG`, `P.TRAY`, `P.battery_pose`, `P.USB`, `P.polar`, `P.screw_angles`; `fusion_lib.prism/combine/axis_prism/new_part`; `v4_model.ymirror/rot`.
- Produces: component `Tub` (body `Tub shell`: floor 4 mm at z 3.5 to 3.9, wall to z 8.7, rear chamfer, two wheel openings that run out through arches (the strip of wall above each arch is notched at the camera window), two motor seats with shaft notches and plate slots, omni bay and arm slot, floor-sensor holes, square chute holes, bumper recesses with switch pockets, six insert bosses, two motor cradles (web, ledges; the four prongs are separate bodies `Cradle prong ...` because they flex), battery tray with notch and stop, USB-C hole in the front-right wall above the bumper band). Later tasks measure against it and cut nothing from it.

- [ ] **Step 1: Write the failing probes**

Append to `v4_checks3d.py`:

```python
STAGE_COMPONENTS['tub'] = ['Tub']


@probes('tub')
def _tub_probes():
    S = P.SCREW
    p0, e = P.chute_ends(1)
    d = L.unit(L.vsub(e, p0))
    in_hole = L.vsub(e, L.vmul(d, 0.15))                                  # on the chute axis just inside the wall
    ang = math.degrees(math.atan2(e[1], e[0]))
    tx, ty = -math.sin(math.radians(ang)), math.cos(math.radians(ang))    # wall tangent at the exit
    wx, wy = P.polar(10.4, ang)
    in_wall = (wx + 1.2 * tx, wy + 1.2 * ty, e[2])                        # 12 mm along the wall from the hole centre
    bx, by = P.polar(S['r'], 12.0)
    ux, uy = math.cos(math.radians(12.0)), math.sin(math.radians(12.0))      # radial direction at the 12 degree boss, vx/vy tangential
    vx, vy = -uy, ux
    rib = (bx + 0.47 * ux + 0.45 * vx, by + 0.47 * uy + 0.45 * vy)           # on the rib, outside the boss circle and inside the wall's inner face
    beside = (bx + 0.47 * ux + 0.65 * vx, by + 0.47 * uy + 0.65 * vy)        # same radius, beside the rib (half width 0.5)
    usb_in = P.polar(10.4, P.USB['angle'])                                    # the USB-C socket hole is in the front-right wall, above the bumper band
    usb_side = P.polar(10.4, P.USB['angle'] - 6.0)                            # 11 mm along the wall from the hole centre: solid wall
    bat = P.battery_pose()
    return [
        ('Tub', (0.0, 8.0, 3.7), False, 'floor opening under the left wheel'),
        ('Tub', (0.0, 10.4, 5.0), False, 'left wheel arch through the wall'),
        ('Tub', (3.0, math.sqrt(10.4 ** 2 - 3.0 ** 2), 8.5), True, 'wall strip above the arch (z 8.3 to 8.7), beside the camera notch'),
        ('Tub', (P.CAM_X, 10.4, 8.5), False, 'camera notch in that strip (the camera sees through it)'),
        ('Tub', (0.0, -10.4, 5.0), False, 'right wheel arch through the wall'),
        ('Tub', (-3.0, -3.0, 3.7), True, 'plain floor'),
        ('Tub', (0.0, 3.0, 3.7), False, 'left motor seat'),
        ('Tub', (1.3, 3.0, 3.7), True, 'floor beside the seat'),
        ('Tub', (1.2, 4.5, 4.3), True, 'front prong, left cradle'),
        ('Tub', (0.5, 4.5, 4.3), False, 'motor space between the prongs'),
        ('Tub', (-1.2, 4.5, 4.3), True, 'rear prong, left cradle'),
        ('Tub', (2.0, 5.85, 4.3), True, 'front ledge, left cradle'),
        ('Tub', (2.0, 5.85, 5.5), False, 'above the front ledge'),
        ('Tub', (-1.4, 5.85, 4.3), True, 'rear ledge, left cradle'),
        ('Tub', (-2.3, -5.85, 4.3), False, 'no rear ledge beside GIGA post H4'),
        ('Tub', (2.0, 6.55, 4.5), True, 'web, front block'),
        ('Tub', (0.0, 6.55, 4.5), False, 'shaft notch in the web'),
        ('Tub', (-2.3, -6.55, 4.5), False, 'no web beside GIGA post H4'),
        ('Tub', (1.8, 6.25, 3.7), False, 'plate slot through the floor'),
        ('Tub', (7.0, 0.0, 3.7), False, 'omni bay in the floor'),
        ('Tub', (6.0, -2.0, 3.7), True, 'floor between the omni bay and the silver-module hole'),
        ('Tub', (3.0, 1.3, 3.7), False, 'arm slot near the pivot'),
        ('Tub', (3.0, 0.0, 3.7), True, 'floor beside the arm slot'),
        ('Tub', (P.FLOOR_FRONT['x'], P.FLOOR_FRONT['y'], 3.7), False, 'front floor port hole, 7.5 cm ahead of the axle'),
        ('Tub', (P.SILVER['x'], P.SILVER['y'], 3.7), False, 'silver module hole, 7.5 cm ahead of the axle beside the omni bay'),
        ('Tub', (0.0, 0.0, 3.7), True, 'plain floor at the axle line (the silver module moved forward)'),
        ('Tub', in_hole, False, 'chute hole through the wall'),
        ('Tub', in_wall, True, 'wall beside the chute hole'),
        ('Tub', (bx, by, 7.9), True, 'insert boss at 12 degrees'),
        ('Tub', (bx, by, 8.5), False, 'insert hole in the boss'),
        ('Tub', (rib[0], rib[1], 8.2), True, 'rib from the 12 degree boss to the wall'),
        ('Tub', (beside[0], beside[1], 8.2), False, 'beside the rib'),
        ('Tub', (bat[0], bat[1], 5.0), True, 'battery tray'),
        ('Tub', (0.9, 5.0, 5.0), False, 'notch in the tray over the left motor'),
        ('Tub', (1.6, 5.0, 5.0), True, 'tray beside the notch'),
        ('Tub', (usb_in[0], usb_in[1], 7.5), False, 'USB-C socket hole in the front-right wall'),
        ('Tub', (usb_side[0], usb_side[1], 7.5), True, 'wall beside the socket hole'),
        ('Tub', (-10.4, 0.0, 7.5), True, 'rear wall is plain now'),
    ]
```

- [ ] **Step 2: Run the probes to verify they fail**

Call the Fusion runner with `print(v4_run.stage(['tub'], build=False))`.
Expected: every row `FAIL Tub component missing`, then `RESULT: FAIL` (the document still holds the rev 3 model, which has no component named `Tub`).

- [ ] **Step 3: Implement the tub**

Append to `v4_model.py`:

```python
def _cradle(comp, shell, s):
    """Motor cradle on one side (s = +1 left, -1 right): web with the shaft notch, ledges (front one carries the plate ear), two prongs. Spec 5.1, Figure 8."""
    C = P.CRADLE
    tag = 'L' if s > 0 else 'R'

    def join(nm, shapes, a0, a1):
        return L.prism(comp, '%s %s' % (nm, tag), 'xz', shapes, a0, a1, op=JOIN, targets=[shell])

    n = C['notch_half']
    zc = (C['web_z'][0] + C['web_z'][1]) / 2
    hz = C['web_z'][1] - C['web_z'][0]
    w0, w1 = ymirror(s, *C['web_y'])
    join('Cradle web rear', [rect((C['web_x'][0] - n) / 2, zc, -n - C['web_x'][0], hz)], w0, w1)
    join('Cradle web front', [rect((n + C['web_x'][1]) / 2, zc, C['web_x'][1] - n, hz)], w0, w1)
    l0, l1 = ymirror(s, *C['ledge_y'])
    zl = (C['ledge_z'][0] + C['ledge_z'][1]) / 2
    hl = C['ledge_z'][1] - C['ledge_z'][0]
    for nm, (xa, xb) in (('front', C['ledge_front_x']), ('rear', C['ledge_rear_x'])):
        join('Cradle ledge ' + nm, [rect((xa + xb) / 2, zl, xb - xa, hl)], l0, l1)
    p0, p1 = ymirror(s, *C['prong_y'])
    z0, z1 = C['arc_z']
    arc = [(math.sqrt(C['arc_r'] ** 2 - (z0 + (z1 - z0) * k / 7 - P.AXLE_Z) ** 2), z0 + (z1 - z0) * k / 7) for k in range(8)]
    pts = [(C['prong_x'][1], C['prong_z'][0]), (C['prong_x'][1], C['prong_z'][1]), C['tip']] + arc + [(C['prong_x'][0], C['prong_z'][0])]
    for side, nm in ((1, 'front'), (-1, 'rear')):          # separate bodies: they bend out of the way when a cartridge is pushed in or lifted out, so the removal report skips them
        L.prism(comp, 'Cradle prong %s %s' % (nm, tag), 'xz', [poly([(side * x, z) for x, z in pts])], p0, p1)


@stage('tub')
def build_tub(ctx):
    occ, comp = L.new_part(ctx.root, 'Tub')
    zf = P.Z_FLOOR_TOP
    shell = L.prism(comp, 'Tub shell', 'xy', [circle((0, 0), P.R_BODY)], P.Z_BELLY, P.Z_TUB_TOP)

    def cut(nm, plane, shapes, a0, a1):
        return L.prism(comp, nm, plane, shapes, a0, a1, op=CUT, targets=[shell])

    def join(nm, plane, shapes, a0, a1):
        return L.prism(comp, nm, plane, shapes, a0, a1, op=JOIN, targets=[shell])

    cut('Cavity', 'xy', [circle((0, 0), P.R_INT)], zf, P.Z_TUB_TOP + 0.5)
    (cx0, cz0), (cx1, cz1) = P.CHAMFER
    slope = (cz1 - cz0) / (cx1 - cx0)
    xe = -11.2
    cut('Rear chamfer', 'xz', [poly([(cx0, cz0), (xe, cz0 + slope * (xe - cx0)), (xe, 2.9), (cx0, 2.9)])], -11.0, 11.0)
    A, C, S = P.ARCH, P.CRADLE, P.SCREW
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = ymirror(s, A['y0'], A['y1'])
        cut('Wheel opening and arch ' + tag, 'xy', [rect(0, (y0 + y1) / 2, 2 * A['half_x'], y1 - y0)], A['z0'] - 0.1, A['z1'])
        # the 4 mm strip of wall above the arch (z 8.3 to 8.7) sits in the middle of the camera's view (3D check, camera rays at -20 to -30 degrees): notch it at the camera window
        cut('Camera notch ' + tag, 'xy', [rect(P.CAM_X, (y0 + y1) / 2, M3.CAM_WINDOW_W + 0.2, y1 - y0)], A['z1'] - 0.1, P.Z_TUB_TOP + 0.1)
        y0, y1 = ymirror(s, *C['seat_y'])
        cut('Motor seat ' + tag, 'xz', [circle((0, P.AXLE_Z), C['seat_r'])], y0, y1)
        y0, y1 = ymirror(s, C['web_y'][0], A['y0'])
        cut('Shaft notch ' + tag, 'xz', [rect(0, 3.95, 2 * C['notch_half'], 1.0)], y0, y1)
        y0, y1 = ymirror(s, *C['slot_y'])
        cut('Plate slot ' + tag, 'xy', [rect((C['slot_x'][0] + C['slot_x'][1]) / 2, (y0 + y1) / 2, C['slot_x'][1] - C['slot_x'][0], y1 - y0)], C['slot_z'][0], C['slot_z'][1])
    # omni bay (wheel path, swept rest -> half travel -> full travel) and arm slot. The arm slot is cut at rest only: the floor is 4 mm thick, so the arm at half and
    # full travel only crosses floor that the rest-position slot has already removed (a cut that removes nothing makes Fusion raise, so the extra cuts are left out)
    ox, oz = P.OMNI['rest']
    px, pz = P.OMNI['pivot']
    xc, zc = P.omni_at(P.OMNI['travel'])
    xm, zm = P.omni_at(P.OMNI['travel'] / 2)
    cut('Omni bay 1', 'xz', [stadium((ox, oz), (xm, zm), P.OMNI['r'] + 0.15)], *P.OMNI_BAY_Y)
    cut('Omni bay 2', 'xz', [stadium((xm, zm), (xc, zc), P.OMNI['r'] + 0.15)], *P.OMNI_BAY_Y)
    cut('Arm slot', 'xz', [stadium((px, pz), (ox, oz), 0.8)], *P.OMNI_ARM_SLOT_Y)
    fp, sm = P.FLOOR_FRONT, P.SILVER
    cut('Floor port FP hole', 'xy', [rect(fp['x'], fp['y'], fp['w'] + 0.1, fp['w'] + 0.1)], P.Z_BELLY - 0.1, zf + 0.1)
    cut('Silver module SM hole', 'xy', [rect(sm['x'], sm['y'], sm['l'] + 0.1, sm['w'] + 0.1)], P.Z_BELLY - 0.1, zf + 0.1)
    # square chute holes through the wall (the channel crosses the cylinder obliquely, so the hole is cut with the channel's own profile)
    w_hole = P.CHUTE['out_w'] + 2 * P.CHUTE['hole_clear']
    for s, tag in ((1, 'B left'), (-1, 'A right')):
        p0, e = P.chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        tool = L.axis_prism(comp, 'Chute hole tool ' + tag, rect(0, 0, w_hole, w_hole), p0, L.vadd(e, L.vmul(d, 0.8)))
        L.combine(comp, shell, [tool])
    # bumper recess behind each plate (the body is recessed by the 4 mm travel), wall kept, switch pocket at the inner end
    bp = P.BUMPER
    r_body = bp['r_out'] - bp['t'] - 0.4
    for s, tag in ((1, 'L'), (-1, 'R')):
        a0, a1 = (bp['a0'] - 4, bp['a1'] + 4) if s > 0 else (-bp['a1'] - 4, -bp['a0'] + 4)
        cut('Bumper recess ' + tag, 'xy', [sector(r_body, P.R_BODY + 0.3, a0, a1)], bp['z0'], bp['z1'])
        join('Bumper recess wall ' + tag, 'xy', [sector(r_body - 0.2, P.R_INT + 0.02, a0, a1)], bp['z0'] - 0.1, bp['z1'] + 0.1)
        a = math.radians(s * P.BUMPER_SW_DEG)
        cut('Bumper switch pocket ' + tag, 'xy', [rect(10.225 * math.cos(a), 10.225 * math.sin(a), 0.35, 0.55, s * P.BUMPER_SW_DEG)], 4.65, 5.85)
    # six insert bosses that carry the upper frame
    for a in P.screw_angles():
        x, y = P.polar(S['r'], a)
        rx, ry = P.polar((S['r'] + S['rib_to']) / 2, a)
        join('Insert boss %+.0f' % a, 'xy', [circle((x, y), S['boss_r']), rect(rx, ry, S['rib_to'] - S['r'], S['rib_w'], a)], S['boss_z0'], P.Z_TUB_TOP)
    for s in (1, -1):
        _cradle(comp, shell, s)
    # battery tray on three legs, front stop, notch over the left drive motor
    T = P.TRAY
    bx, by, ba = P.battery_pose()
    for lx, ly in T['legs']:
        x, y = rot(bx, by, ba, lx, ly)
        join('Tray leg', 'xy', [circle((x, y), T['leg_r'])], zf - 0.05, T['z'][0] + 0.05)
    join('Battery tray', 'xy', [rect(bx, by, 7.0 + 2 * T['margin'], 3.5 + 2 * T['margin'], ba)], T['z'][0], T['z'][1])
    stop_lx, stop_t, stop_h = T['stop']
    x, y = rot(bx, by, ba, stop_lx, 0.0)
    join('Battery stop', 'xy', [rect(x, y, stop_t, 3.5 + 2 * T['margin'], ba)], T['z'][1] - 0.05, T['z'][1] + stop_h)
    n0, n1, m0, m1 = T['notch']
    cut('Battery tray notch', 'xy', [rect((n0 + n1) / 2, (m0 + m1) / 2, n1 - n0, m1 - m0)], T['z'][0] - 0.1, T['z'][1] + 0.1)
    for a in P.screw_angles():
        x, y = P.polar(S['r'], a)
        cut('Insert hole %+.0f' % a, 'xy', [circle((x, y), S['insert_r'])], P.Z_TUB_TOP - S['insert_depth'], P.Z_TUB_TOP + 0.1)
    U = P.USB
    ua = math.radians(U['angle'])
    cut('USB-C socket hole', 'xy', [rect((U['r_face'] - 0.1) * math.cos(ua), (U['r_face'] - 0.1) * math.sin(ua), 0.9, 2 * U['half'], U['angle'])], U['z'][0], U['z'][1])
    ctx.pal.paint(shell, '#C8C6BE', 0.45)
    return occ
```

- [ ] **Step 4: Build the tub and run the probes**

Call the Fusion runner with `print(v4_run.stage(['tub'], reset=True))` (this replaces the rev 3 model in the open design; Step 1 of Task 2 confirmed the baseline export exists).
Expected: `stage tub done; timeline N` (N is the number of features built so far, about 100), then 36 probe rows `ok`, then `ok   no overlap with any other component`, and `RESULT: PASS`.
If a row fails: print the nearest geometry with a throw-away script (`_bodies(ctx, 'Tub')[0].boundingBox`, or probe a few points along the line through the failing one) and decide whether the builder or the probe is wrong before editing either. Typical causes: a cut that starts exactly on a face (use `-0.1` / `+0.1` margins), a mirrored `ymirror` interval, a probe on a face (counts as empty).

- [ ] **Step 5: Look at it**

Call `mcp__Autodesk_Fusion__fusion_mcp_read` with `queryType: "screenshot"`, `direction: "iso-top-left"`. Check by eye that the tub shows two wheel arches, two cradles with prongs, the omni bay at the front, the battery tray and six bosses. Do not keep the image; the final screenshots are taken in Task 10.

- [ ] **Step 6: Checkpoint (no commit)**

Re-run the unit tests (`python -m unittest test_v3_params4`). Expected: `OK`.

---

### Task 4: Drive train, cartridges, omni module, nub and bumpers

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append probes, stage components, `report_omni`)
- Modify: `docs/superpowers/specs/v3-fusion/v4_model.py` (append the stages)

**Interfaces:**
- Consumes: `Tub` (Task 3), `M3.build_drive`, `M3.build_rear_nub`, `M3.build_bumpers` (unchanged rev 3 builders; they read only the parameters that did not change: `WHEEL`, `MOTOR`, `NUB`, `CHAMFER`, `BUMPER`, `CAM`), `P.FACE_PLATE`, `P.OMNI`.
- Produces: components `Wheel L/R`, `Motor L/R` (motor body, encoder, shaft), `Face plate L/R`, `Omni wheel`, `Omni arm`, `Omni pins`, `Rear nub`, `Bumper L/R`, `Bumper switch L/R`; `v4_checks3d.report_omni(ctx)`.

- [ ] **Step 1: Write the failing probes**

Append to `v4_checks3d.py`:

```python
STAGE_COMPONENTS.update({'drive': ['Wheel L', 'Motor L', 'Wheel R', 'Motor R'], 'cartridges': ['Face plate L', 'Face plate R'], 'omni': ['Omni wheel', 'Omni arm', 'Omni pins'],
                         'nub': ['Rear nub'], 'bumpers': ['Bumper L', 'Bumper switch L', 'Bumper R', 'Bumper switch R']})


@probes('drive')
def _drive_probes():
    return [
        ('Wheel L', (0.0, 8.0, 7.0), True, 'left wheel, upper part'),
        ('Wheel L', (0.0, 8.0, 4.0), False, 'shaft bore at the axle'),
        ('Wheel R', (0.0, -8.0, 7.0), True, 'right wheel, upper part'),
        ('Motor L', (0.0, 3.0, 4.5), True, 'left gearmotor'),
        ('Motor L', (0.0, 3.0, 5.5), False, 'above the left motor (top z 5.0)'),
        ('Motor L', (0.0, 7.0, 4.0), True, 'left motor shaft'),
        ('Motor R', (0.0, -3.0, 4.5), True, 'right gearmotor'),
    ]


@probes('cartridges')
def _cartridge_probes():
    return [
        ('Face plate L', (0.8, 6.25, 4.4), True, 'plate body'),
        ('Face plate L', (0.0, 6.25, 4.0), False, 'shaft bore'),
        ('Face plate L', (2.0, 6.25, 4.0), True, 'one ear, front side'),
        ('Face plate L', (-2.0, 6.25, 4.0), False, 'no ear on the rear side'),
        ('Face plate R', (2.0, -6.25, 4.0), True, 'one ear, right plate'),
    ]


@probes('omni')
def _omni_probes():
    ox, oz = P.OMNI['rest']
    px, pz = P.OMNI['pivot']
    xm = 4.8
    zm = pz + (oz - pz) * (xm - px) / (ox - px)          # centre line of the arm at x 4.8
    return [
        ('Omni wheel', (ox, 0.0, oz + 2.5), True, 'wheel above the hub'),
        ('Omni wheel', (ox, 0.0, oz), False, 'axle bore'),
        ('Omni wheel', (ox + 2.9, 0.0, oz), True, 'front edge of the wheel (x 9.9)'),
        ('Omni wheel', (ox + 3.1, 0.0, oz), False, 'beyond the front edge (x 10.1)'),
        ('Omni arm', (xm, 1.3, zm), True, 'single arm plate on +y'),
        ('Omni arm', (xm, -1.3, zm), False, 'no second arm plate on -y'),
        ('Omni pins', (px, 1.3, pz), True, 'pivot pin'),
    ]


@probes('nub')
def _nub_probes():
    return [('Rear nub', (P.NUB['x'], 0.0, 3.0), True, 'nub below the chamfer')]


@probes('bumpers')
def _bumper_probes():
    bx, by = P.polar(10.9, 35.0)
    sx, sy = P.polar(10.425, P.BUMPER_SW_DEG)
    return [('Bumper L', (bx, by, 5.0), True, 'left bumper plate at 35 degrees'), ('Bumper R', (bx, -by, 5.0), True, 'right bumper plate'),
            ('Bumper switch L', (sx, sy, 5.2), True, 'left microswitch at the inner end')]


def report_omni(ctx):
    """Omni, arm and pins swept through the full travel against every other body. Expect 'no interference'."""
    K3.omni_sweep(ctx, omni=P.OMNI, omni_at=P.omni_at, travel=P.OMNI['travel'])
    return 'omni sweep done'
```

- [ ] **Step 2: Run the probes to verify they fail**

Call the Fusion runner with `print(v4_run.stage(['drive', 'cartridges', 'omni', 'nub', 'bumpers'], build=False))`.
Expected: `FAIL ... component missing` for every row, `RESULT: FAIL`.

- [ ] **Step 3: Implement the stages**

Append to `v4_model.py`:

```python
STAGES.append(('drive', M3.build_drive))                 # wheels 80 mm on y 7 to 9, Pololu 20D motors on y 1.4 to 6.1: unchanged from rev 3


@stage('cartridges')
def build_cartridges(ctx):
    """Printed face plate on each gearbox: one ear on the front side, bore for the D-shaft (spec 5.1)."""
    F = P.FACE_PLATE
    for s, tag in ((1, 'L'), (-1, 'R')):
        occ, comp = L.new_part(ctx.root, 'Face plate ' + tag)
        y0, y1 = ymirror(s, *F['y'])
        pl = L.prism(comp, 'Face plate ' + tag, 'xz', [poly(F['pts'])], y0, y1)
        L.prism(comp, 'Shaft bore', 'xz', [circle((0, P.AXLE_Z), F['bore_r'])], y0 - 0.1, y1 + 0.1, op=CUT, targets=[pl])
        ctx.pal.paint(pl, '#B4B2A9')


@stage('omni')
def build_omni(ctx):
    """60 mm omni at x 7.0 on ONE arm on the +y side; the pivot pin and the axle are as long as the floor slot allows (spec 4)."""
    O = P.OMNI
    ox, oz = O['rest']
    px, pz = O['pivot']
    hw = O['w'] / 2
    occ, comp = L.new_part(ctx.root, 'Omni wheel')
    wh = L.prism(comp, 'Omni wheel', 'xz', [circle((ox, oz), O['r'])], -hw, hw)
    L.prism(comp, 'Omni hub', 'xz', [circle((ox, oz), 1.0)], -hw - 0.1, hw + 0.1, op=JOIN, targets=[wh])
    L.prism(comp, 'Axle bore', 'xz', [circle((ox, oz), O['axle_r'])], -hw - 0.2, hw + 0.2, op=CUT, targets=[wh])
    ctx.pal.paint(wh, '#1D9E75')
    occ, comp = L.new_part(ctx.root, 'Omni arm')
    y0, y1 = O['arm_y']
    arm = L.prism(comp, 'Arm plate', 'xz', [stadium((px, pz), (ox, oz), O['arm_half'])], y0, y1)
    L.prism(comp, 'Pivot hole', 'xz', [circle((px, pz), O['pin_r'])], y0 - 0.1, y1 + 0.1, op=CUT, targets=[arm])
    L.prism(comp, 'Axle hole', 'xz', [circle((ox, oz), O['axle_r'])], y0 - 0.1, y1 + 0.1, op=CUT, targets=[arm])
    ctx.pal.paint(arm, '#0F6E56')
    occ, comp = L.new_part(ctx.root, 'Omni pins')
    pv = L.prism(comp, 'Pivot pin', 'xz', [circle((px, pz), O['pin_r'])], *O['pin_y'])
    ax = L.prism(comp, 'Axle pin', 'xz', [circle((ox, oz), O['axle_r'])], -hw - 0.05, y1 + 0.1)
    ctx.pal.paint([pv, ax], '#5F5E5A')


STAGES.append(('nub', M3.build_rear_nub))
STAGES.append(('bumpers', M3.build_bumpers))
```

- [ ] **Step 4: Build and run the probes**

Call the Fusion runner with `print(v4_run.stage(['drive', 'cartridges', 'omni', 'nub', 'bumpers']))` (no reset: the tub from Task 3 stays).
Expected: five `stage ... done` lines, all probe rows `ok`, every stage's overlap lines `ok   no overlap with any other component`, `RESULT: PASS`.
If `Wheel L` overlaps `Tub`: the arch cut (`ARCH`) is narrower than the wheel (x +-4.0) or the floor opening starts inside the wheel (y 7.0); check `P.ARCH` against `P.WHEEL`. If `Motor L` overlaps `Tub`: the seat radius or its y range is wrong. If `Omni pins` overlaps `Tub`: the pin is longer than the floor slot (`P.OMNI['pin_y']` must stay inside `P.OMNI_ARM_SLOT_Y`).

- [ ] **Step 5: Sweep the omni through its travel**

Call the Fusion runner with `print(v4_run.report('omni'))`.
Expected (from `omni_sweep`): `omni sweep: 3 moving bodies against N others`, then `no interference at any of the 13 positions between 0 and 2.5 cm`, then `omni sweep done`. Any `Omni ... x Tub` line means the bay or the arm slot is too tight at that travel: widen the stadium margin (`+ 0.15` and `0.8` in `build_tub`), re-run Task 3 Step 4 (it rebuilds with `reset=True`), then Task 4 Step 4, and note the change.

- [ ] **Step 6: Checkpoint (no commit)**

Unit tests still `OK`; `git status --short docs/superpowers/specs/v3-fusion` shows only untracked new files.

---

### Task 5: Upper frame, ToF modules and cameras

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append probes and stage components)
- Modify: `docs/superpowers/specs/v3-fusion/v4_model.py` (append the stages)

**Interfaces:**
- Consumes: `P.FRAME`, `P.HANDLE`, `P.SCREW`, `P.HOOK`, `P.PLATE`, `P.slot_xy`, `P.TOF`, `P.TOF_Z/H/W/T`, `P.CAM`, `P.CAM_X`, `M3.tof_rect`, `M3.t_exit`, `M3.CAM_WINDOW_W`, `M3.build_cameras`, `ctx.tof`.
- Produces: component `Upper frame` (ring r 9.0 to 10.5, z 8.7 to 11.2, with nine ToF pockets and beam windows, two camera windows, six screw holes with counterbores, four hook rebates and grooves; front bridge (rib under the handle post only) with the control deck on its +y side, rear spoke, three half-lap seats for the lift-out dropper floor, handle post; the dropper floor itself is built in Task 7); components `ToF F` to `ToF RR`; `Camera L/R`.

- [ ] **Step 1: Write the failing probes**

Append to `v4_checks3d.py`:

```python
STAGE_COMPONENTS.update({'frame': ['Upper frame'], 'tof': ['ToF %s' % n for n, x, y, a in P.TOF], 'cameras': ['Camera L', 'Camera R']})


@probes('frame')
def _frame_probes():
    S = P.SCREW
    free = P.polar(S['r'], 66.0)                      # between the screw at 60 and the hook at 71 degrees: no window, no screw
    hole = P.polar(S['r'], 12.0)
    reb = P.polar(10.45, 71.0)
    groove = P.polar(10.3, 71.0)
    ring_ok = P.polar(10.45, 62.0)
    return [
        ('Upper frame', (free[0], free[1], 10.0), True, 'ring wall between screw and hook'),
        ('Upper frame', (6.0, 1.0, 8.85), True, 'bridge web'),
        ('Upper frame', (4.5, 0.0, 9.5), True, 'bridge rib under the handle post'),
        ('Upper frame', (4.5, 1.0, 9.5), False, 'beside the bridge rib'),
        ('Upper frame', (8.4, 0.0, 9.5), True, 'bridge rib to the front ring: the two status LEDs stand on it'),
        ('Upper frame', (-8.0, 0.0, 8.85), True, 'rear spoke web'),
        ('Upper frame', (-8.5, 0.0, 9.3), True, 'rear spoke rib'),
        ('Upper frame', (-2.0, 3.0, 8.85), False, 'no floor disc in the frame: it is the lift-out dropper unit'),
        ('Upper frame', (7.0, 2.0, 8.85), True, 'control deck beside the bridge web'),
        ('Upper frame', (7.0, 3.9, 8.85), False, 'beyond the control deck'),
        ('Upper frame', (7.0, -1.6, 8.85), False, 'no deck over the GIGA front edge: reset, boot and J12 stay open from above'),
        ('Upper frame', (3.5, 1.0, 8.95), False, 'front seat: top half of the web removed'),
        ('Upper frame', (3.5, 1.0, 8.75), True, 'front seat: lower half of the web under the tab'),
        ('Upper frame', (3.5, -1.0, 8.75), True, 'front seat on the other side'),
        ('Upper frame', (-7.4, 0.0, 8.75), True, 'rear seat: lower half of the spoke web under the tab'),
        ('Upper frame', (-7.4, 0.0, 8.95), False, 'rear seat: top half of the spoke web removed'),
        ('Upper frame', (4.0, 0.0, 12.0), True, 'handle post'),
        ('Upper frame', (4.0, 1.0, 12.0), False, 'beside the handle post'),
        ('Upper frame', (hole[0], hole[1], 10.0), False, 'screw hole at 12 degrees'),
        ('Upper frame', (hole[0], hole[1], 11.1), False, 'counterbore at 12 degrees'),
        ('Upper frame', (reb[0], reb[1], 10.8), False, 'hook rebate at 71 degrees'),
        ('Upper frame', (groove[0], groove[1], 10.7), False, 'hook groove at 71 degrees'),
        ('Upper frame', (ring_ok[0], ring_ok[1], 10.8), True, 'ring beside the rebate'),
        ('Upper frame', (9.5, 0.0, 10.0), False, 'ToF pocket F'),
        ('Upper frame', (9.5, 0.0, 11.15), False, 'ToF pocket F is open to the top (the board drops in from above, the lid covers it)'),
        ('Upper frame', (10.2, 0.0, 10.0), False, 'ToF beam window F'),
        ('Upper frame', (0.307, 9.6, 9.3), False, 'camera window, left'),
    ]


@probes('tof')
def _tof_probes():
    return [('ToF F', (9.5, 0.0, 10.0), True, 'front module'), ('ToF SFL', (7.18, 6.0, 10.0), True, 'side-front left module'),
            ('ToF SFL', (7.18, 6.0, 11.2), False, 'above the module (top z 11.05)')]


@probes('cameras')
def _camera_probes():
    t = math.radians(P.CAM['tilt'])
    tip = (P.CAM['tip_r'] * math.cos(math.radians(P.CAM['psi'])), P.CAM['tip_r'] * math.sin(math.radians(P.CAM['psi'])), P.CAM['zl'])
    inside = (tip[0], tip[1] - 1.0 * math.cos(t), tip[2] + 1.0 * math.sin(t))        # 1 cm behind the lens tip along the lens block
    return [('Camera L', inside, True, 'lens block, 1 cm behind the tip')]
```

- [ ] **Step 2: Run the probes to verify they fail**

Call the Fusion runner with `print(v4_run.stage(['frame', 'tof', 'cameras'], build=False))`.
Expected: `FAIL ... component missing` rows, `RESULT: FAIL`.

- [ ] **Step 3: Implement the stages**

Append to `v4_model.py`:

```python
@stage('frame')
def build_frame(ctx):
    """Upper frame in one piece: ring, front bridge with the control deck, rear spoke, seats for the lift-out dropper floor, handle post (spec 3.1, 6.1, 7.2, 7.3)."""
    F = P.FRAME
    occ, comp = L.new_part(ctx.root, 'Upper frame')
    z0, z1, ft = F['z0'], F['z1'], F['floor_t']
    ring = L.ring_prism(comp, 'Frame ring', 'xy', circle((0, 0), F['r_out']), circle((0, 0), F['r_in']), z0, z1)

    def join(nm, shapes, a0, a1):
        return L.prism(comp, nm, 'xy', shapes, a0, a1, op=JOIN, targets=[ring])

    def cut(nm, plane, shapes, a0, a1):
        return L.prism(comp, nm, plane, shapes, a0, a1, op=CUT, targets=[ring])

    B, Sp, Dk, D, H = F['bridge'], F['spoke'], F['deck'], P.DROPPER_FLOOR, P.HANDLE
    join('Bridge web', [rect((B['x0'] + B['x1']) / 2, 0, B['x1'] - B['x0'], 2 * B['half'])], z0, z0 + ft)
    join('Bridge rib', [rect((B['x_rib0'] + B['x_rib1']) / 2, 0, B['x_rib1'] - B['x_rib0'], 2 * B['rib_half'])], z0 + ft - 0.05, B['rib_z1'])
    join('Spoke web', [rect((Sp['x0'] + Sp['x1']) / 2, 0, Sp['x1'] - Sp['x0'], 2 * Sp['half'])], z0, z0 + ft)
    join('Spoke rib', [rect((Sp['x0'] + Sp['x_rib1']) / 2, 0, Sp['x_rib1'] - Sp['x0'], 2 * Sp['rib_half'])], z0 + ft - 0.05, Sp['rib_z1'])
    join('Control deck', [rect((Dk['x0'] + Dk['x1']) / 2, (Dk['y0'] + Dk['y1']) / 2, Dk['x1'] - Dk['x0'], Dk['y1'] - Dk['y0'])], z0, z0 + ft)
    join('Handle post', [rect(H['x'], 0, H['post_w'], H['post_w'])], H['post_z'][0], H['post_z'][1])
    m = D['seat_margin']                       # half-lap seats of the lift-out dropper floor: each tab (top half of the disc thickness) lies in a rebate (upper half of the web removed) on the web's lower half
    for i, (x0, x1, y0, y1) in enumerate(D['tabs']):
        yc, wy = (y0 + y1) / 2, y1 - y0 + 2 * m
        xa, xb = (B['x0'] - 0.05, x1 + m) if x0 > 0 else (x0 - m, Sp['x1'] + 0.05)          # front tabs sit on the bridge web beside its rib, the rear tab on the spoke web behind the disc rim
        cut('Seat rebate %d' % i, 'xy', [rect((xa + xb) / 2, yc, xb - xa, wy)], z0 + ft / 2, z0 + ft + 0.1)
    S = P.SCREW
    for a in P.screw_angles():
        x, y = P.polar(S['r'], a)
        cut('Screw hole %+.0f' % a, 'xy', [circle((x, y), S['hole_r'])], z0 - 0.1, z1 + 0.1)
        cut('Screw counterbore %+.0f' % a, 'xy', [circle((x, y), S['cbore_r'])], z1 - S['cbore_depth'], z1 + 0.1)
    Hk = P.HOOK
    for a in P.hook_angles():
        cut('Hook rebate %+.0f' % a, 'xy', [sector(Hk['r_in'], Hk['r_out'] + 0.1, a - Hk['half_deg'] - 0.5, a + Hk['half_deg'] + 0.5)], Hk['skirt_z0'], z1 + 0.1)
        cut('Hook groove %+.0f' % a, 'xy', [sector(Hk['groove_r_in'], Hk['r_in'] + 0.01, a - Hk['groove_half_deg'], a + Hk['groove_half_deg'])], Hk['bump_z'][0], Hk['bump_z'][1])
    t_half = P.TOF_T / 2 + 0.06
    for nm, (x, y, aim) in ctx.tof.items():
        cut('Pocket ' + nm, 'xy', [M3.tof_rect(x, y, aim, -t_half, t_half, P.TOF_W + 0.12)], P.TOF_Z - P.TOF_H / 2 - 0.05, z1 + 0.1)          # open at the top: the board drops in from above
        cut('Beam ' + nm, 'xy', [M3.tof_rect(x, y, aim, 0.0, M3.t_exit(x, y, aim), 1.8)], 9.1, 10.9)
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = ymirror(s, 8.4, 10.8)
        cut('Camera window ' + tag, 'xz', [rect(P.CAM_X, (z0 - 0.1 + 10.6) / 2, M3.CAM_WINDOW_W, 10.6 - (z0 - 0.1))], y0, y1)
    ctx.pal.paint(ring, '#B4B2A9', 0.55)


@stage('tof')
def build_tof_modules(ctx):
    """Nine VL53L0X boards, 1.8 x 2.1 x 0.45 cm, at z 10.0, in the pockets of the ring."""
    for nm, (x, y, aim) in ctx.tof.items():
        occ, comp = L.new_part(ctx.root, 'ToF ' + nm)
        mod = L.prism(comp, 'VL53L0X module ' + nm, 'xy', [rect(x, y, P.TOF_T, P.TOF_W, aim)], P.TOF_Z - P.TOF_H / 2, P.TOF_Z + P.TOF_H / 2)
        a = math.radians(aim)
        off = P.TOF_T / 2 + 0.05
        chip = L.prism(comp, 'VL53L0X chip ' + nm, 'xy', [rect(x + off * math.cos(a), y + off * math.sin(a), 0.1, 0.44, aim)], P.TOF_Z - 0.12, P.TOF_Z + 0.12)
        ctx.pal.paint(mod, '#378ADD')
        ctx.pal.paint(chip, '#042C53')


STAGES.append(('cameras', M3.build_cameras))             # two OpenMV H7 Plus boards tilted 20 degrees behind the windows: unchanged from rev 3
```

- [ ] **Step 4: Build and run the probes**

Call the Fusion runner with `print(v4_run.stage(['frame', 'tof', 'cameras']))`.
Expected: `stage frame done`, `stage tof done`, `stage cameras done`, all rows `ok`, no overlaps, `RESULT: PASS`.
The frame touches the tub only along the plane z 8.7: that is not an overlap. If `Upper frame x Tub` shows a volume, a boss or the bridge web was extruded below z 8.7 (check `z0 - 0.1` margins on cuts only, never on joins). If a `ToF` module overlaps `Upper frame`, the pocket is narrower than `P.TOF_T`: the pocket half-width is `TOF_T / 2 + 0.06`.

- [ ] **Step 5: Re-run the omni sweep with the bridge in place**

Call the Fusion runner with `print(v4_run.report('omni'))`. Expected: `no interference at any of the 13 positions between 0 and 2.5 cm` (the compressed omni top is z 8.5, the bridge underside 8.7). This is the check behind the spec's "bridge is a 3 mm web".

- [ ] **Step 6: Checkpoint (no commit)**

Unit tests `OK`; nothing committed.

---
### Task 6: Lid, handle and front controls

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append probes and stage components)
- Modify: `docs/superpowers/specs/v3-fusion/v4_model.py` (append the stages)

**Interfaces:**
- Consumes: `P.LID`, `P.HOOK`, `P.HANDLE`, `P.CONTROLS`, `P.USB`, `P.usb_pose`, `P.CAM`, `P.CAM_X`, `P.Z_ROOF`, `P.Z_LID`, `P.R_BODY`, `P.hook_angles`, `P.polar`, `P.FRAME`.
- Produces: components `Lid` (3 mm plate on the ring, camera humps, handle slot and the front notch over the control deck, two finger notches, four skirt segments; the four hook bumps are separate bodies `Hook bump ...` because they flex), `Handle bar` (x 3.3 to 8.7, z 14.0 to 15.6: it starts in front of the dropper unit so the unit can lift out) with the component `Victim LED` on top of its front end (z 16.0, the highest point), `Power switch`, `Start button` (on the +y side of the control deck), `Status LED 1`, `Status LED 2` (on the bridge rib), `USB-C service socket` (in the front-right wall, above the bumper band).

- [ ] **Step 1: Write the failing probes**

Append to `v4_checks3d.py`:

```python
STAGE_COMPONENTS.update({'lid': ['Lid'], 'handle': ['Handle bar', 'Victim LED'], 'controls': [c[0] for c in P.CONTROLS] + ['USB-C service socket']})


@probes('lid')
def _lid_probes():
    sk = P.polar(10.42, 71.0)
    bump = P.polar(10.30, 71.0)
    between = P.polar(10.42, 90.0)
    finger = P.polar(10.3, 12.0)
    return [
        ('Lid', (-5.0, 0.0, 11.35), True, 'lid plate'),
        ('Lid', (4.0, 0.0, 11.35), False, 'handle slot'),
        ('Lid', (-9.5, 0.0, 11.35), True, 'lid plate at the rear (the panel moved to the front, no rear notch)'),
        ('Lid', (7.0, 2.0, 11.35), False, 'front notch over the control deck'),
        ('Lid', (7.0, 3.8, 11.35), True, 'plate beside the front notch'),
        ('Lid', (7.0, -1.6, 11.35), True, 'plate on the -y side of the slot: the notch is on the +y side only'),
        ('Lid', (sk[0], sk[1], 10.8), True, 'hook skirt at 71 degrees'),
        ('Lid', (bump[0], bump[1], 10.7), True, 'hook bump at 71 degrees'),
        ('Lid', (between[0], between[1], 10.8), False, 'no skirt between the hooks'),
        ('Lid', (finger[0], finger[1], 11.35), False, 'finger notch at 12 degrees'),
        ('Lid', (0.307, 7.3, 12.2), True, 'camera hump skin'),
        ('Lid', (0.307, 7.3, 11.6), False, 'space under the camera hump'),
    ]


@probes('handle')
def _handle_probes():
    led = P.HANDLE['led']
    return [('Handle bar', (4.0, 0.0, 14.8), True, 'handle bar'), ('Handle bar', (2.0, 0.0, 14.8), False, 'behind the rear end of the bar (x 3.3): the dropper unit lifts out past it'),
            ('Handle bar', (4.0, 1.0, 14.8), False, 'beside the bar (half width 0.8)'),
            ('Victim LED', (led['x'], led['y'], 15.8), True, 'victim LED on the front end of the bar'), ('Victim LED', (led['x'] + 0.5, led['y'], 15.8), False, 'beside the LED')]


@probes('controls')
def _controls_probes():
    ux, uy, _ = P.usb_pose()
    return [('Start button', (7.55, 2.15, 9.3), True, 'start button on the control deck'), ('Power switch', (6.0, 2.15, 9.6), True, 'power switch'),
            ('Status LED 1', (7.5, 0.0, 10.0), True, 'status LED 1 on the rib'), ('Status LED 2', (6.6, 0.0, 10.0), True, 'status LED 2 on the rib'),
            ('USB-C service socket', (ux, uy, 7.5), True, 'USB-C service socket in the front-right wall')]
```

- [ ] **Step 2: Run the probes to verify they fail**

Call the Fusion runner with `print(v4_run.stage(['lid', 'handle', 'controls'], build=False))`. Expected: `FAIL ... component missing`, `RESULT: FAIL`.

- [ ] **Step 3: Implement the stages**

Append to `v4_model.py`:

```python
@stage('lid')
def build_lid(ctx):
    """Snap-on lid: plate on the ring, camera humps, handle slot, front notch over the control deck, finger notches, four skirt segments with hook bumps (spec 7.1)."""
    Ld, Hk = P.LID, P.HOOK
    occ, comp = L.new_part(ctx.root, 'Lid')
    lid = L.prism(comp, 'Lid plate', 'xy', [circle((0, 0), P.R_BODY)], Ld['z0'], Ld['z1'])
    for s in (1, -1):
        y0, y1 = ymirror(s, 5.3, 9.3)
        L.prism(comp, 'Camera hump', 'xy', [rect(P.CAM_X, (y0 + y1) / 2, 5.0, y1 - y0)], P.Z_ROOF - 0.05, P.Z_LID + Ld['hump_skin'], op=JOIN, targets=[lid])
    for s in (1, -1):
        y0, y1 = ymirror(s, 5.5, 9.1)
        L.prism(comp, 'Camera pocket', 'xy', [rect(P.CAM_X, (y0 + y1) / 2, 4.6, y1 - y0)], Ld['z0'] - 0.1, P.Z_LID, op=CUT, targets=[lid])
    sx0, sx1 = Ld['slot_x']
    L.prism(comp, 'Handle slot', 'xy', [rect((sx0 + sx1) / 2, 0, sx1 - sx0, 2 * Ld['slot_half'])], Ld['z0'] - 0.1, Ld['z1'] + 0.1, op=CUT, targets=[lid])
    fx0, fx1, fy0, fy1 = Ld['front_notch']
    L.prism(comp, 'Front notch', 'xy', [rect((fx0 + fx1) / 2, (fy0 + fy1) / 2, fx1 - fx0, fy1 - fy0)], Ld['z0'] - 0.1, Ld['z1'] + 0.1, op=CUT, targets=[lid])
    for a in (Ld['finger_deg'], -Ld['finger_deg']):
        x, y = P.polar(P.R_BODY - 0.2, a)
        L.prism(comp, 'Finger notch', 'xy', [rect(x, y, 0.8, 0.6, a)], Ld['z0'] - 0.1, Ld['z1'] + 0.1, op=CUT, targets=[lid])
    bumps = []
    for a in P.hook_angles():
        L.prism(comp, 'Hook skirt %+.0f' % a, 'xy', [sector(Hk['r_in'], Hk['r_out'], a - Hk['half_deg'], a + Hk['half_deg'])], Hk['skirt_z0'], Ld['z0'] + 0.05, op=JOIN, targets=[lid])
        bump = L.prism(comp, 'Hook bump %+.0f' % a, 'xy', [sector(Hk['groove_r_in'], Hk['r_in'] + 0.02, a - Hk['groove_half_deg'], a + Hk['groove_half_deg'])], Hk['bump_z'][0], Hk['bump_z'][1])
        bumps.append(bump)                                     # a separate body: the hook flexes it out of the groove when the lid is lifted, so the removal report skips it
    ctx.pal.paint([lid] + bumps, '#F1EFE8', 0.55)


@stage('handle')
def build_handle(ctx):
    """Bar of the T-handle: 5.4 cm along x (3.3 to 8.7) at z 14.0 to 15.6, bolted to the post that is part of the upper frame, and the victim LED on top of its front end (spec 7.2, 7.3)."""
    H = P.HANDLE
    occ, comp = L.new_part(ctx.root, 'Handle bar')
    bar = L.prism(comp, 'Handle bar', 'xy', [rect((H['bar_x'][0] + H['bar_x'][1]) / 2, 0, H['bar_x'][1] - H['bar_x'][0], 2 * H['bar_half'])], H['bar_z'][0], H['bar_z'][1])
    ctx.pal.paint(bar, '#444441')
    led = H['led']
    occ, comp = L.new_part(ctx.root, 'Victim LED')
    b = L.prism(comp, 'Victim LED', 'xy', [circle((led['x'], led['y']), led['r'])], led['z'][0], led['z'][1])
    ctx.pal.paint(b, '#E24B4A')


@stage('controls')
def build_controls(ctx):
    """Start button, power switch and two status LEDs on the control deck in front of the handle post; USB-C service socket in the front-right wall (spec 7.3). Part sizes are placeholders."""
    for nm, x, y, shape, z0, z1, colour in P.CONTROLS:
        occ, comp = L.new_part(ctx.root, nm)
        geom = circle((x, y), shape[1]) if shape[0] == 'round' else rect(x, y, shape[1], shape[2])
        b = L.prism(comp, nm, 'xy', [geom], z0, z1)
        ctx.pal.paint(b, colour)
    U = P.USB
    cx, cy, ang = P.usb_pose()
    occ, comp = L.new_part(ctx.root, 'USB-C service socket')
    b = L.prism(comp, 'USB-C service socket', 'xy', [rect(cx, cy, U['depth'], 2 * U['half'] - 0.1, ang)], U['z'][0] + 0.05, U['z'][1] - 0.05)
    ctx.pal.paint(b, '#7A4B9C')
```

- [ ] **Step 4: Build and run the probes**

Call the Fusion runner with `print(v4_run.stage(['lid', 'handle', 'controls']))`.
Expected: three `stage ... done` lines, all rows `ok`, no overlaps, `RESULT: PASS`.
Likely failures: `Lid x Camera L/R` (the hump pocket is smaller than the lens block: the numbers come unchanged from rev 3, so check that `Camera pocket` still runs from `Ld['z0'] - 0.1`); `Lid x Upper frame` (the skirt reaching into the rebate wall: rebate half-angle is `half_deg + 0.5`, skirt `half_deg`); `Handle bar x Lid` (the slot is narrower than the bar: `slot_half` 0.9 against `bar_half` 0.8).

- [ ] **Step 5: Checkpoint (no commit)**

Unit tests `OK`; nothing committed.

---

### Task 7: Dropper plate, N20 cartridge, hoppers and chutes

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append probes and stage components)
- Modify: `docs/superpowers/specs/v3-fusion/v4_model.py` (append the stages)

**Interfaces:**
- Consumes: `P.PLATE`, `P.pocket_xy`, `P.slot_xy`, `P.KIT`, `P.FRAME`, `P.DROPPER_FLOOR`, `P.CHUTE`, `P.chute_ends`, `P.R_BODY`; `L.axis_prism`, `L.combine`, `L.ring_prism`, `L.unit/vsub/vadd/vmul`.
- Produces: components `Dropper floor` (R 5.11, z 8.7 to 9.0, slots A and B, N20 pocket and face-plate recess, three seat tabs), `Dropper plate` (R 4.91, z 9.0 to 10.2, eight 14 mm pockets, shaft bore), `N20 motor` (body z 4.65 to 8.8, shaft to z 9.8), `N20 face plate` (in the floor recess, z 8.8 to 9.0), `Kits` (eight 10.3 mm cubes), `Hopper A right`, `Hopper B left` (19.5 mm box with 23.5 mm flange and a socket for the channel), `Chute A right`, `Chute B left` (13 mm square channel with an open trough under the slot, trimmed at the body radius).

- [ ] **Step 1: Write the failing probes**

Append to `v4_checks3d.py`:

```python
STAGE_COMPONENTS.update({'dropper': ['Dropper floor', 'Dropper plate', 'N20 motor', 'N20 face plate', 'Kits'], 'chutes': ['Hopper A right', 'Chute A right', 'Hopper B left', 'Chute B left']})


@probes('dropper')
def _dropper_probes():
    cx, cy = P.PLATE['cx'], P.PLATE['cy']
    p1 = P.pocket_xy(1)
    sx, sy = P.slot_xy('A')
    return [
        ('Dropper floor', (-2.0, 3.0, 8.85), True, 'floor disc'),
        ('Dropper floor', (sx, sy, 8.85), False, 'slot A through the floor'),
        ('Dropper floor', (cx, cy, 8.85), False, 'N20 pocket'),
        ('Dropper floor', (cx, 0.8, 8.95), False, 'N20 face plate recess'),
        ('Dropper floor', (cx, 0.8, 8.75), True, 'floor under the recess'),
        ('Dropper floor', (3.5, 1.0, 8.95), True, 'front tab, top half of the floor thickness'),
        ('Dropper floor', (3.5, 1.0, 8.75), False, 'nothing under the tab: the bridge web is the ledge'),
        ('Dropper floor', (-7.3, 0.0, 8.95), True, 'rear tab, beyond the disc rim'),
        ('Dropper floor', (-7.3, 0.0, 8.75), False, 'nothing under the rear tab: the spoke web is the ledge'),
        ('Dropper plate', (cx, cy + 0.5, 9.6), True, 'plate near the centre'),
        ('Dropper plate', (cx, cy, 9.6), False, 'shaft bore'),
        ('Dropper plate', (p1[0], p1[1], 9.6), False, 'pocket 1 (14 mm)'),
        ('Dropper plate', (cx - 3.86, cy, 9.6), True, 'blank arc facing the rear'),
        ('N20 motor', (cx, cy, 6.0), True, 'N20 gearmotor'),
        ('N20 motor', (cx, cy + 0.5, 9.5), False, 'above the floor only the shaft remains'),
        ('N20 motor', (cx, cy, 9.5), True, 'N20 shaft in the plate hub'),
        ('N20 face plate', (cx - 0.6, cy, 8.9), True, 'face plate in the floor recess'),
        ('N20 face plate', (cx, cy, 8.9), False, 'shaft bore in the face plate'),
        ('Kits', (p1[0], p1[1], 9.5), True, 'kit in pocket 1'),
    ]


@probes('chutes')
def _chute_probes():
    rows = []
    for s, side in ((1, 'B left'), (-1, 'A right')):
        p0, e = P.chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        lat = L.unit((-d[1], d[0], 0.0))
        mid = L.vadd(p0, L.vmul(d, 3.5))
        rows += [
            ('Hopper ' + side, (p0[0], p0[1], 8.3), False, 'hopper void under the slot'),
            ('Hopper ' + side, L.vadd((p0[0], p0[1], 8.0), L.vmul(lat, 0.95)), True, 'hopper wall beside the channel socket'),
            ('Chute ' + side, mid, False, 'channel bore'),
            ('Chute ' + side, L.vadd(mid, L.vmul(lat, 0.73)), True, 'channel wall'),
            ('Chute ' + side, L.vadd(e, L.vmul(d, 0.3)), False, 'channel trimmed at the body radius'),
        ]
    return rows
```

- [ ] **Step 2: Run the probes to verify they fail**

Call the Fusion runner with `print(v4_run.stage(['dropper', 'chutes'], build=False))`. Expected: `FAIL ... component missing`, `RESULT: FAIL`.

- [ ] **Step 3: Implement the stages**

Append to `v4_model.py`:

```python
@stage('dropper')
def build_dropper(ctx):
    """The lift-out dropper unit: floor disc with slots A and B, the N20 pocket and the seat tabs; kit plate; N20 cartridge through the floor pocket; eight kits (spec 6.1, 6.3, 6.4)."""
    pl, F, D = P.PLATE, P.FRAME, P.DROPPER_FLOOR
    cx, cy = pl['cx'], pl['cy']
    occ, comp = L.new_part(ctx.root, 'Dropper floor')
    disc = L.prism(comp, 'Dropper floor', 'xy', [circle((cx, cy), D['r'])], F['z0'], F['z0'] + D['t'])
    for i, (x0, x1, y0, y1) in enumerate(D['tabs']):
        L.prism(comp, 'Dropper floor tab %d' % i, 'xy', [rect((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0)], F['z0'] + D['t'] / 2, F['z0'] + D['t'], op=JOIN, targets=[disc])
    for nm in 'AB':
        sx, sy = P.slot_xy(nm)
        L.prism(comp, 'Slot ' + nm, 'xy', [rect(sx, sy, pl['slot'], pl['slot'], 45)], F['z0'] - 0.1, F['z0'] + D['t'] + 0.1, op=CUT, targets=[disc])
    n1, n2 = D['n20_pocket']
    r1, r2, rd = D['n20_recess']
    L.prism(comp, 'N20 pocket', 'xy', [rect(cx, cy, n1, n2)], F['z0'] - 0.1, F['z0'] + D['t'] + 0.1, op=CUT, targets=[disc])
    L.prism(comp, 'N20 face plate recess', 'xy', [rect(cx, cy, r1, r2)], F['z0'] + D['t'] - rd, F['z0'] + D['t'] + 0.1, op=CUT, targets=[disc])
    ctx.pal.paint(disc, '#B4B2A9')
    occ, comp = L.new_part(ctx.root, 'Dropper plate')
    plate = L.prism(comp, 'Dropper plate', 'xy', [circle((cx, cy), pl['R'])], pl['z0'], pl['z0'] + pl['t'])
    for i in range(1, pl['n'] + 1):
        x, y, a = P.pocket_xy(i)
        L.prism(comp, 'Pocket %d' % i, 'xy', [rect(x, y, pl['pocket'], pl['pocket'], a)], pl['z0'] - 0.1, pl['z0'] + pl['t'] + 0.1, op=CUT, targets=[plate])
    L.prism(comp, 'Shaft bore', 'xy', [circle((cx, cy), 0.15)], pl['z0'] - 0.1, pl['z0'] + pl['t'] + 0.1, op=CUT, targets=[plate])
    ctx.pal.paint(plate, '#BA7517')
    z_top = F['z0'] + D['t'] - D['n20_recess'][2]                       # 8.8: motor top, under the face plate in the floor recess
    occ, comp = L.new_part(ctx.root, 'N20 motor')
    n1, n2 = pl['n20_w']
    n20 = L.prism(comp, 'N20 gearmotor with encoder', 'xy', [rect(cx, cy, n1, n2)], z_top - pl['n20_len'], z_top)
    shaft = L.prism(comp, 'N20 shaft', 'xy', [circle((cx, cy), 0.15)], z_top, pl['z0'] + pl['t'] - 0.4)
    ctx.pal.paint(n20, '#888780')
    ctx.pal.paint(shaft, '#C8C6BE')
    occ, comp = L.new_part(ctx.root, 'N20 face plate')
    fp = L.prism(comp, 'N20 face plate', 'xy', [rect(cx, cy, r1 - 0.04, r2 - 0.04)], z_top, F['z0'] + D['t'])
    L.prism(comp, 'Shaft bore', 'xy', [circle((cx, cy), 0.17)], z_top - 0.1, F['z0'] + D['t'] + 0.1, op=CUT, targets=[fp])
    ctx.pal.paint(fp, '#B4B2A9')
    occ, comp = L.new_part(ctx.root, 'Kits')
    kits = []
    for i in range(1, pl['n'] + 1):
        x, y, a = P.pocket_xy(i)
        kits.append(L.prism(comp, 'Kit %d' % i, 'xy', [rect(x, y, P.KIT, P.KIT, a)], pl['z0'], pl['z0'] + P.KIT))
    ctx.pal.paint(kits, '#444441')


@stage('chutes')
def build_chutes(ctx):
    """Per side: a hopper sealed to the underside of the frame floor under the slot (with a socket cut for the channel), and the 13 mm square channel
    from the slot axis to the body radius with an open trough where the kit falls in (spec 6.2). They do not overlap, so the channel can slide out."""
    Ch, F = P.CHUTE, P.FRAME
    for s, side in ((1, 'B left'), (-1, 'A right')):
        p0, e = P.chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        sx, sy = p0[0], p0[1]
        far = L.vadd(e, L.vmul(d, 0.8))
        occ, comp = L.new_part(ctx.root, 'Hopper ' + side)
        hop = L.prism(comp, 'Hopper', 'xy', [rect(sx, sy, Ch['hopper_out'], Ch['hopper_out'], 45)], Ch['hopper_z0'], F['z0'])
        L.prism(comp, 'Hopper flange', 'xy', [rect(sx, sy, Ch['flange'], Ch['flange'], 45)], F['z0'] - Ch['flange_t'], F['z0'], op=JOIN, targets=[hop])
        L.prism(comp, 'Hopper void', 'xy', [rect(sx, sy, Ch['hopper_in'], Ch['hopper_in'], 45)], Ch['hopper_z0'] + 0.16, F['z0'] + 0.1, op=CUT, targets=[hop])
        sock = L.axis_prism(comp, 'Socket tool', rect(0, 0, Ch['out_w'], Ch['out_w']), L.vsub(p0, L.vmul(d, 0.5)), L.vadd(p0, L.vmul(d, 3.0)))
        L.combine(comp, hop, [sock])
        ctx.pal.paint(hop, '#993C1D', 0.6)
        occ, comp = L.new_part(ctx.root, 'Chute ' + side)
        tube = L.axis_prism(comp, 'Channel', rect(0, 0, Ch['out_w'], Ch['out_w']), p0, far)
        bore = L.axis_prism(comp, 'Channel void', rect(0, 0, Ch['in_w'], Ch['in_w']), L.vsub(p0, L.vmul(d, 0.2)), L.vadd(far, L.vmul(d, 0.2)))
        L.combine(comp, tube, [bore])
        L.prism(comp, 'Trough cut', 'xy', [rect(sx, sy, Ch['hopper_in'], Ch['hopper_in'], 45)], 7.7, F['z0'] + 0.3, op=CUT, targets=[tube])
        trim = L.ring_prism(comp, 'Trim', 'xy', circle((0, 0), 14.0), circle((0, 0), P.R_BODY), 2.0, 9.0)
        L.combine(comp, tube, [trim])
        ctx.pal.paint(tube, '#993C1D', 0.6)
```

- [ ] **Step 4: Build and run the probes**

Call the Fusion runner with `print(v4_run.stage(['dropper', 'chutes']))`.
Expected: `stage dropper done`, `stage chutes done`, all rows `ok`, `RESULT: PASS`.
Likely failures: `N20 motor x Upper frame` (pocket 1.04 x 1.24 against the body 1.0 x 1.2: check `n20_w`); `Dropper plate x N20 face plate` (plate bottom z 9.0 must equal the face plate top 9.0); `Chute ... x Tub` (the wall hole is `out_w + 0.08`; it must be cut with the same axis as the channel: both come from `P.chute_ends`); `Hopper x Chute` (the socket tool must use `out_w`, not `in_w`).

- [ ] **Step 5: Checkpoint (no commit)**

Unit tests `OK`; nothing committed.

---

### Task 8: GIGA posts, electronics, connector, Wi-Fi antenna and the stepper bay

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append probes and stage components)
- Modify: `docs/superpowers/specs/v3-fusion/v4_model.py` (append the stages)

**Interfaces:**
- Consumes: `P.GIGA_USED`, `P.GIGA_POST`, `P.giga_hole_xy`, `P.giga_stack`, `P.giga_j12`, `P.J12`, `P.ANTENNA`, `P.antenna_pose`, `P.battery_pose`, `P.FLOOR_FRONT`, `P.SILVER`, `P.STEPPER`, `P.stepper_geometry`, `M3.build_electronics` (GIGA board, headers, shield, envelope, battery, floor sensors FP and SM; the rev 3 builder, called with the rev 4 sensor positions), `L.find_occ`.
- Produces: components `GIGA posts` (four M3-insert posts z 3.9 to 6.15), `Arduino GIGA R1` (with an added body `USB-C J12`), `Main PCB`, `Battery`, `Floor port FP`, `Silver module SM`, `Wi-Fi antenna` (a strip on the inside of the front wall), and the hidden `Stepper bay 28BYJ-48`.

- [ ] **Step 1: Write the failing probes**

Append to `v4_checks3d.py`:

```python
STAGE_COMPONENTS.update({'posts': ['GIGA posts'], 'electronics': ['Arduino GIGA R1', 'Main PCB', 'Battery', 'Floor port FP', 'Silver module SM'], 'antenna': ['Wi-Fi antenna']})


@probes('posts')
def _post_probes():
    x1, y1 = P.giga_hole_xy(0)
    x4, y4 = P.giga_hole_xy(3)
    return [('GIGA posts', (x1, y1, 5.0), True, 'post H1'), ('GIGA posts', (x1, y1, 6.0), False, 'M3 insert hole in the top of post H1'),
            ('GIGA posts', (x4, y4, 5.0), True, 'post H4')]


@probes('electronics')
def _electronics_probes():
    g = P.giga_stack()
    b = P.battery_pose()
    return [('Arduino GIGA R1', (g[0], g[1], 6.2), True, 'GIGA board'), ('Battery', (b[0], b[1], 6.0), True, 'battery'),
            ('Floor port FP', (P.FLOOR_FRONT['x'], P.FLOOR_FRONT['y'], 3.5), True, 'front floor sensor, 7.5 cm ahead of the axle'),
            ('Silver module SM', (P.SILVER['x'], P.SILVER['y'], 3.5), True, 'silver module, 7.5 cm ahead of the axle'),
            ('Silver module SM', (0.0, 0.0, 3.5), False, 'nothing at the axle line any more')]


@probes('connector')
def _connector_probes():
    x, y = P.giga_j12()
    return [('Arduino GIGA R1', (x + 0.1, y, 6.5), True, 'USB-C J12 on the connector edge')]


@probes('antenna')
def _antenna_probes():
    x, y, ang = P.antenna_pose()
    return [('Wi-Fi antenna', (x, y, 6.3), True, 'antenna strip on the inside of the front wall'), ('Wi-Fi antenna', (x, y, 7.3), False, 'above the strip')]


@probes('stepper')
def _stepper_probes():
    g = P.stepper_geometry()
    return [('Stepper bay 28BYJ-48', (g['centre'][0], g['centre'][1], 7.5), True, 'bay body'),
            ('Stepper bay 28BYJ-48', (g['ears'][0][0], g['ears'][0][1], 8.4), True, 'ear 1'),
            ('Stepper bay 28BYJ-48', (g['block'][0], g['block'][1], 7.5), True, 'wire block')]
```

- [ ] **Step 2: Run the probes to verify they fail**

Call the Fusion runner with `print(v4_run.stage(['posts', 'electronics', 'connector', 'antenna', 'stepper'], build=False))`. Expected: `FAIL ... component missing`, `RESULT: FAIL`.

- [ ] **Step 3: Implement the stages**

Append to `v4_model.py`:

```python
@stage('posts')
def build_posts(ctx):
    """Four printed posts under the GIGA, one at each of the datasheet corner holes H1 to H4, with M3 insert holes in the top (spec 8.1)."""
    G = P.GIGA_POST
    occ, comp = L.new_part(ctx.root, 'GIGA posts')
    posts = []
    for i in P.GIGA_USED:
        x, y = P.giga_hole_xy(i)
        b = L.prism(comp, 'GIGA post H%d' % (i + 1), 'xy', [circle((x, y), G['r'])], G['z'][0], G['z'][1])
        L.prism(comp, 'Insert hole H%d' % (i + 1), 'xy', [circle((x, y), G['insert_r'])], G['z'][1] - G['insert_depth'], G['z'][1] + 0.1, op=CUT, targets=[b])
        posts.append(b)
    ctx.pal.paint(posts, '#5F5E5A')


@stage('electronics')
def build_electronics(ctx):
    """GIGA R1 plus shield stack, battery and the two floor sensors: the rev 3 builder, with the rev 4 floor-sensor positions (both 7.5 cm ahead of the axle, one each side of the omni bay)."""
    saved = M3.prm.FLOOR_FRONT, M3.prm.SILVER
    M3.prm.FLOOR_FRONT, M3.prm.SILVER = P.FLOOR_FRONT, P.SILVER          # the builder reads these two names from the rev 3 parameter module; restored below
    try:
        M3.build_electronics(ctx)
    finally:
        M3.prm.FLOOR_FRONT, M3.prm.SILVER = saved


@stage('connector')
def build_connector(ctx):
    """USB-C J12 on the connector edge of the GIGA, a placeholder body added to the GIGA component (spec 8.1)."""
    occ = L.find_occ(ctx.root, 'Arduino GIGA R1')
    x, y = P.giga_j12()
    J = P.J12
    L.prism(occ.component, 'USB-C J12', 'xy', [rect(x + J['out'] / 2 - 0.05, y, J['out'] + 0.1, 2 * J['half'])], J['z'][0], J['z'][1])


@stage('antenna')
def build_antenna(ctx):
    """The Wi-Fi/Bluetooth flex antenna that ships with the GIGA, stuck on the inside of the tub's front wall (spec 8.5): a 15.4 x 6.4 mm strip, its 100 mm cable to J14 is not modelled."""
    A = P.ANTENNA
    cx, cy, ang = P.antenna_pose()
    occ, comp = L.new_part(ctx.root, 'Wi-Fi antenna')
    strip = L.prism(comp, 'Wi-Fi antenna', 'xy', [rect(cx, cy, A['t'], A['w'], ang)], A['z'][0], A['z'][1])
    ctx.pal.paint(strip, '#F0997B')


@stage('stepper')
def build_stepper_bay(ctx):
    """Hidden keep-out of the 28BYJ-48 alternative: body, two ears, wire block (spec 6.3). Excluded from every report except `report_stepper`."""
    St = P.STEPPER
    g = P.stepper_geometry()
    ux, uy = g['dir']
    ang = math.degrees(math.atan2(uy, ux))
    vx, vy = -uy, ux
    cx, cy = g['centre']
    occ, comp = L.new_part(ctx.root, 'Stepper bay 28BYJ-48')
    body = L.prism(comp, 'Body', 'xy', [circle((cx, cy), St['r'])], St['z'][0], St['z'][1])
    for s in (1, -1):
        p1 = (cx + s * St['ear_from'] * vx, cy + s * St['ear_from'] * vy)
        p2 = (cx + s * St['ear_span'] * vx, cy + s * St['ear_span'] * vy)
        L.prism(comp, 'Ear', 'xy', [stadium(p1, p2, St['ear_r'])], St['ear_z'][0], St['ear_z'][1], op=JOIN, targets=[body])
    bx, by = g['block']
    L.prism(comp, 'Wire block', 'xy', [rect(bx, by, St['block_to'] - St['block_from'], St['block_w'], ang)], St['z'][0], St['z'][1], op=JOIN, targets=[body])
    ctx.pal.paint(body, '#7A4B9C', 0.35)
    occ.isLightBulbOn = False
```

- [ ] **Step 4: Build and run the probes**

Call the Fusion runner with `print(v4_run.stage(['posts', 'electronics', 'connector', 'antenna', 'stepper']))`.
Expected: five `stage ... done` lines, all rows `ok`, `RESULT: PASS`. The stepper stage has no overlap lines on purpose (it is hidden and excluded).
Likely failures: `GIGA posts x Tub` (the post bottom must be exactly 3.9, the floor top: `GIGA_POST['z']`); `Battery x Tub` (battery bottom 5.1 against the tray top 5.1 is coincident, not an overlap: if it overlaps, the tray was extruded to z 5.2); `Floor port FP x Tub` (the hole is `w + 0.1` wide).

- [ ] **Step 5: Whole model, one look**

Call the Fusion runner with `print(v4_run.stage('all', build=False))` (all probes again, no building). Expected: every probe `ok`, `RESULT: PASS`. This also proves that the stage order did not matter for any probe.

- [ ] **Step 6: Checkpoint (no commit)**

Unit tests `OK`; nothing committed.

---
### Task 9: Whole-model reports

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/v4_checks3d.py` (append the reports)
- Create: `docs/superpowers/specs/v3-fusion/results/v4_interference.txt`, `v4_envelope.txt`, `v4_belly.txt`, `v4_removal.txt`, `v4_access.txt`, `v4_omni.txt`, `v4_tof.txt`, `v4_camera.txt`, `v4_clearances.txt`, `v4_stepper.txt`, `v4_mass.txt` (raw output of each report, written by hand from the tool result)

**Interfaces:**
- Consumes: every component built in Tasks 3 to 8 (names as listed in `STAGE_COMPONENTS`), `P.*`, `K3.tof_cones`, `K3.camera_fov`, `K3.omni_sweep`, `local_interference` (Task 2).
- Produces: `report_interference`, `report_envelope`, `report_belly`, `report_removal`, `report_access`, `report_tof`, `report_camera`, `report_clearances`, `report_stepper`, `report_mass` (each takes `ctx`, returns text ending in `RESULT: PASS`, `FAIL` or, for the mass report only, `WARN`); `BELLY_KNOWN`, `SPEC_PAIRS`, `FLEX`, `UNIT` (the lift-out dropper unit), `ELECTRONICS`, `STACK`, `ACCESS_MIN`, `removal_sequence(ctx, plan, max_step=0.25, floor=1e-3, flex=FLEX, window=None)`, `removal_scenarios()`; `report_removal(ctx, only=None, window=None)`.

These reports are the acceptance tests of the whole model. They are written once, run once each, and any failure is a finding to resolve, not a test to loosen.

**Predicted by the dry run** (emulator, same code; compare each Fusion result with these before believing either; tolerances: distances 0.3 mm, volumes 2 %, centre of mass 0.05 cm):

| Report | Predicted |
|---|---|
| `interference` | 51 components, 0 interfering pairs, `PASS` |
| `envelope` | bumper plates 11.000, their switches 10.752, Lid, Tub, Upper frame and both channels 10.500, USB-C socket 10.470, ToF boards at most 10.227; highest point z 16.00 (the victim LED); `PASS` |
| `belly` | known lines only: wheels 0.00, omni wheel 0.00, rear nub 2.00, motors 3.00, channels 3.04, floor port FP 2.80, silver module SM 2.80, omni arm 2.45, omni pins 2.80; pivot pin z 3.40; `PASS` |
| `removal` | seven `ok` lines (lid, both wheels, kit swap, dropper unit out, battery swap, teardown), `PASS` |
| `access` | share of the upper surface seen from above, lid off against lid and dropper unit off: GIGA 55 % against 86 %, shield 53 % against 86 %, battery 77 % against 91 %; silver module 49 %, left motor 0 % against 54 %, right motor 0 %; `PASS` (tolerance 5 points) |
| `omni` | `no interference at any of the 13 positions between 0 and 2.5 cm` |
| `tof`, `camera` | `blocked  0 / 17` for all nine sensors; `0 / 25` for both cameras |
| `clearances` (mm) | silver module to omni wheel 5.0, to GIGA post H1 2.8, N20 to left motor 11.0, channel to wheel 8.3 (both), omni wheel to GIGA post 2.7, GIGA stack to tub 1.7, battery to left motor 1.1, plate to lid 10.0, camera to wheel 3.6, stepper bay to shield 5.8, to hopper B 5.2, to battery 8.3; `PASS` |
| `stepper` | `ok   no overlap`, `PASS` |
| `mass` | total about 1195 g, centre of mass x +0.89, y +0.15, z 6.82 cm, front load 12.8 %, front-lift limit 1.29 m/s^2: `PASS` (above the 1.0 the firmware ramp assumes) |

- [ ] **Step 1: Write the reports**

Append to `v4_checks3d.py`:

```python
# ---------------------------------------------------------------------------------------------------- whole-model reports
GHOST = 'Stepper bay 28BYJ-48'


def _all_occurrences(ctx):
    return [ctx.root.occurrences.item(i) for i in range(ctx.root.occurrences.count)]


def report_interference(ctx):
    """Every pair of components (the hidden stepper bay is left out), with the same temporary-B-rep intersections as the per-stage check. Expect no pair above the allowlist."""
    names = [o.component.name for o in _all_occurrences(ctx) if o.component.name != GHOST]
    rows = local_interference(ctx, names, exclude=(GHOST,))
    bad = [r for r in rows if r[0] > ALLOW.get(frozenset((r[1], r[2])), 0.0) + 1e-3]
    lines = ['components %d, interfering pairs %d, above the allowlist %d' % (len(names), len(rows), len(bad))]
    lines += ['  %s %9.4f cm3  %s  x  %s' % ('FAIL' if r in bad else 'ok  ', r[0], r[1], r[2]) for r in rows]
    lines.append('RESULT: ' + ('PASS' if not bad else 'FAIL'))
    return '\n'.join(lines)


def _max_radius(body):
    calc = body.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
    nc = calc.calculate().nodeCoordinatesAsDouble
    return max(math.hypot(nc[k], nc[k + 1]) for k in range(0, len(nc), 3))


def report_envelope(ctx):
    """Farthest radius of every component (body 10.5; only the bumper plates and their switches reach past it, plates to 11.0), highest point (the victim LED on the bar, 16.0, limit 25), omni front edge inside the wall."""
    rows, zmax = [], -1e9
    for o in _all_occurrences(ctx):
        nm = o.component.name
        if nm == GHOST:
            continue
        lo, hi = L.world_bbox(o)
        zmax = max(zmax, hi[2])
        rows.append((max(_max_radius(o.bRepBodies.item(j)) for j in range(o.bRepBodies.count)), nm))
    rows.sort(reverse=True)

    def limit(nm):                  # the bumper plates set the swept radius; their switches sit in the recess behind the plate (rev 3 layout, r 10.75)
        return P.R_SWEPT if nm in ('Bumper L', 'Bumper R', 'Bumper switch L', 'Bumper switch R') else P.R_BODY

    bad = [(r, nm) for r, nm in rows if r > limit(nm) + 0.005]
    lines = ['farthest radius from the axle, largest first (body %.1f, bumper plates %.1f):' % (P.R_BODY, P.R_SWEPT)]
    lines += ['  %s %-24s r_max %7.3f  (limit %.1f)' % ('FAIL' if (r, nm) in bad else 'ok  ', nm, r, limit(nm)) for r, nm in rows[:12]]
    lines.append('highest point z %.2f (victim LED on the bar, bar top %.1f, limit 25)' % (zmax, P.HANDLE['bar_z'][1]))
    front = P.OMNI['rest'][0] + P.OMNI['r']
    lines.append('omni front edge at rest x %.2f, wall inner face %.1f' % (front, P.R_INT))
    ok = not bad and abs(zmax - P.HANDLE['led']['z'][1]) < 0.05 and zmax <= 25.0 and front <= P.R_INT - 0.25
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)


BELLY_KNOWN = {'Wheel L', 'Wheel R', 'Omni wheel', 'Omni arm', 'Omni pins', 'Rear nub', 'Motor L', 'Motor R', 'Floor port FP', 'Silver module SM', 'Chute A right', 'Chute B left'}


def report_belly(ctx, z_belly=3.5):
    """Everything with a point below the belly line z 3.5. Only the known items are allowed (rev 3: wheels, omni, nub, motors, floor sensors, chute lip)."""
    lines, bad = ['components with a point below the belly line z %.1f:' % z_belly], []
    for o in _all_occurrences(ctx):
        nm = o.component.name
        if nm == GHOST:
            continue
        lo, hi = L.world_bbox(o)
        if lo[2] < z_belly - 1e-4:
            known = nm in BELLY_KNOWN
            if not known:
                bad.append(nm)
            lines.append('  %s %-24s lowest z %5.2f  (%+.1f mm against the belly line)' % ('known' if known else 'FAIL ', nm, lo[2], (lo[2] - z_belly) * 10))
    z_pin = P.OMNI['pivot'][1] - P.OMNI['pin_r']
    lines.append('pivot pin (4 mm) lowest point z %.2f: %+.1f mm against the belly line (spec section 4: a 3 mm pin or a sunk pin, decided in CAD)' % (z_pin, (z_pin - z_belly) * 10))
    lines.append('RESULT: ' + ('PASS' if not bad else 'FAIL'))
    return '\n'.join(lines)


FLEX = ('Hook bump', 'Cradle prong')       # the lid hooks and the cradle prongs bend out of the way by design (spec 5.1, 7.1): the removal paths ignore them


def removal_sequence(ctx, plan, max_step=0.25, floor=1e-3, flex=FLEX, window=None):
    """plan: list of (group of component names, legs), legs = list of (dx, dy, dz) moves made one after the other. Each group slides along its legs in steps of at most `max_step` cm
    (a step must stay below the thinnest wall, or a part could jump through it); temporary copies are intersected with every body not yet removed; bodies named in `flex` are ignored
    on both sides; afterwards the group counts as removed. Returns [(group, worst)], worst = {(mover, other): (cm3, step along the group's whole path)}.
    window = (group index, first step, last step): only those steps of that group are checked and every other group is just marked removed. A Fusion tool call times out after
    about 60 s, so the long paths are checked in pieces."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    removed, results = set(), []
    for gi, (group, legs) in enumerate(plan):
        movers, others = [], []
        for o in _all_occurrences(ctx):
            nm = o.component.name
            if nm in removed or nm == GHOST:
                continue
            for j in range(o.bRepBodies.count):
                b = o.bRepBodies.item(j)
                if not b.name.startswith(tuple(flex)):
                    (movers if nm in group else others).append((nm, b))
        if not movers:
            raise KeyError('no component of %s in the design' % (group,))
        if window is not None and window[0] != gi:
            legs = []
        boxes = [(on, ob, ob.boundingBox, []) for on, ob in others]            # bounding box once per body; its temporary copy (the tool of every intersection, which a boolean leaves unchanged) when first needed
        worst, base, n = {}, (0.0, 0.0, 0.0), 0
        for leg in legs:
            steps = max(2, int(math.ceil(L.vlen(leg) / max_step)))
            for k in range(1, steps + 1):
                n += 1
                if window is not None and not (window[1] <= n <= window[2]):
                    continue
                f = k / float(steps)
                m = adsk.core.Matrix3D.create()
                m.translation = V3(base[0] + leg[0] * f, base[1] + leg[1] * f, base[2] + leg[2] * f)
                for mn, mb in movers:
                    t1 = tbm.copy(mb)
                    tbm.transform(t1, m)
                    bb1 = t1.boundingBox
                    for on, ob, bb2, tool in boxes:
                        if (bb1.maxPoint.x < bb2.minPoint.x or bb1.minPoint.x > bb2.maxPoint.x or bb1.maxPoint.y < bb2.minPoint.y or bb1.minPoint.y > bb2.maxPoint.y
                                or bb1.maxPoint.z < bb2.minPoint.z or bb1.minPoint.z > bb2.maxPoint.z):
                            continue
                        if not tool:
                            tool.append(tbm.copy(ob))
                        t2 = tbm.copy(t1)
                        try:
                            v = t2.volume if tbm.booleanOperation(t2, tool[0], adsk.fusion.BooleanTypes.IntersectionBooleanType) else 0.0
                        except Exception:
                            v = 0.0
                        if v > floor and v > worst.get((mn, on), (0.0, 0))[0]:
                            worst[(mn, on)] = (v, n)
            base = (base[0] + leg[0], base[1] + leg[1], base[2] + leg[2])
        results.append((group, worst))
        removed |= set(group)
    return results


def _along_chute(side, length):
    p0, e = P.chute_ends(side)
    d = L.unit(L.vsub(e, p0))
    return (d[0] * length, d[1] * length, d[2] * length)


UNIT = ('Dropper floor', 'Dropper plate', 'Kits', 'N20 motor', 'N20 face plate', 'Hopper A right', 'Hopper B left')    # the lift-out dropper unit (spec 3.3, 6.1)


def removal_scenarios():
    tof = ['ToF %s' % n for n, x, y, a in P.TOF]
    frame = ['Upper frame', 'Handle bar', 'Victim LED', 'Camera L', 'Camera R'] + tof + [c[0] for c in P.CONTROLS]          # what the six screws hold; the control parts sit on its bridge
    lid = (['Lid'], [(0.0, 0.0, 6.0)])
    chutes = [(['Chute A right'], [_along_chute(-1, 5.0)]), (['Chute B left'], [_along_chute(1, 5.0)])]
    unit_up = (list(UNIT), [(0.0, 0.0, 12.0)])
    return [
        ('lid lifts straight up over the handle (the hooks flex)', [lid]),
        ('left wheel slides 1.2 cm out along its shaft, then drops through the arch', [(['Wheel L'], [(0.0, 1.2, 0.0), (0.0, 0.0, -9.0)])]),
        ('right wheel, same path mirrored', [(['Wheel R'], [(0.0, -1.2, 0.0), (0.0, 0.0, -9.0)])]),
        ('kit swap: lid, then kit plate with the kits, then the N20 cartridge up through the floor pocket',
         [lid, (['Dropper plate', 'Kits'], [(0.0, 0.0, 3.0)]), (['N20 motor', 'N20 face plate'], [(0.0, 0.0, 6.0)])]),
        ('dropper unit out: lid, both channels along their axes, then floor disc, plate, kits, N20 cartridge and both hoppers straight up', [lid] + chutes + [unit_up]),
        ('battery swap: the dropper unit out, then the battery 4.5 cm back and 2 cm inboard (Camera L and the ring corner are over it, the bridge and its switch in front) and straight up',
         [lid] + chutes + [unit_up, (['Battery'], [(-4.5, -2.0, 0.0), (0.0, 0.0, 8.0)])]),
        ('teardown: lid, channels, dropper unit, frame group up, battery up, left wheel out, left cartridge up (prongs flex), GIGA stack up, right wheel out, '
         'right cartridge up (prongs flex), omni module up',
         [lid] + chutes + [unit_up, (frame, [(0.0, 0.0, 12.0)]),
          (['Battery'], [(0.0, 0.0, 8.0)]),
          (['Wheel L'], [(0.0, 1.2, 0.0), (0.0, 0.0, -9.0)]), (['Motor L', 'Face plate L'], [(0.0, 0.0, 10.0)]),
          (['Arduino GIGA R1', 'Main PCB'], [(0.0, 0.0, 10.0)]),
          (['Wheel R'], [(0.0, -1.2, 0.0), (0.0, 0.0, -9.0)]), (['Motor R', 'Face plate R'], [(0.0, 0.0, 10.0)]),
          (['Omni wheel', 'Omni arm', 'Omni pins'], [(0.0, 0.0, 12.0)])]),
    ]


def report_removal(ctx, only=None, window=None):
    """The service paths of spec 3.3: each must be free of interference from start to end (hooks and prongs excepted, they flex). `only`: scenario numbers (0 to 6) when a call has to
    be split to stay inside the tool timeout (about 60 s); `window` = (group index, first step, last step) for one scenario of `only` (see removal_sequence): the line is then marked as
    a window and vouches only for those steps."""
    lines, ok = [], True
    for i, (title, plan) in enumerate(removal_scenarios()):
        if only is not None and i not in only:
            continue
        results = removal_sequence(ctx, plan, window=window)
        bad = [(g, w) for g, w in results if w]
        ok = ok and not bad
        lines.append('%s [%d] %s%s' % ('FAIL' if bad else 'ok  ', i, title, '' if window is None else '   (window: group %d, steps %d to %d only)' % tuple(window)))
        for g, w in bad:
            for (mn, on), (v, k) in sorted(w.items(), key=lambda kv: -kv[1][0]):
                lines.append('       %s x %s: %.4f cm3 at step %d' % (mn, on, v, k))
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)


ELECTRONICS = ('Arduino GIGA R1', 'Main PCB', 'Battery', 'Silver module SM', 'Motor L', 'Motor R')
STACK = ('Arduino GIGA R1', 'Main PCB')                                  # the shield plugs into the GIGA: the two count as one stack
ACCESS_MIN = {'Arduino GIGA R1': 0.70, 'Main PCB': 0.70, 'Battery': 0.80}      # seen from above with the lid and the dropper unit off (spec 7.4; the dry run gives 86, 86 and 91 %); the rest needs the frame off


def _visible_share(ctx, name, off, n=7):
    """Share of an n x n grid over the plan-view box of a part that lies on its upper surface with nothing above it once the components in `off` are taken away: one ray down finds the
    part's top at that point, one ray up from just above it must leave the model without a hit."""
    group = STACK if name in STACK else (name,)
    occ = [o for o in _all_occurrences(ctx) if o.component.name == name][0]
    lo, hi = L.world_bbox(occ)
    skip = tuple(off) + (GHOST,)
    seen = total = 0
    for i in range(n):
        for j in range(n):
            x = lo[0] + (hi[0] - lo[0]) * (i + 0.5) / n
            y = lo[1] + (hi[1] - lo[1]) * (j + 0.5) / n
            hits = [d for d, nm in K3._cast(ctx.root, (x, y, hi[2] + 1.0), (0.0, 0.0, -1.0), own=skip) if nm in group]
            if not hits:
                continue                                                  # the point is outside the part's outline
            total += 1
            z_top = hi[2] + 1.0 - hits[0]
            if not K3._cast(ctx.root, (x, y, z_top + 0.02), (0.0, 0.0, 1.0), own=skip + group):
                seen += 1
    return seen / float(total) if total else 0.0


ACCESS_POINTS = (('GIGA reset button PB1 (inner corner of the connector edge)', (7.12, -1.53, 6.45), True),
                 ('GIGA USB-C J12', (7.40, -2.74, 6.75), True),
                 ('GIGA boot button PB2 (outer corner, under the ring)', (7.12, -6.44, 6.45), False))      # (x, y, z above the board), expected open from above; positions read from the datasheet picture, +-1 mm


def report_access(ctx):
    """What can be seen, and so reached, from straight above (spec 3.3, 7.4): with the lid off, and with the lid and the dropper unit off. Each row is the share of that part's upper
    surface with nothing above it. The parts in ACCESS_MIN need the share shown; the others are reached with the frame off. Then three points on the GIGA's connector edge, straight up
    through everything except the lid and the GIGA stack itself. The boot button is expected to be covered by the ring (a known limit, spec 7.4): it is listed so that a change that hides the
    reset button, or that uncovers the boot button, shows up."""
    lines = ['share of the upper surface seen from straight above (grid of rays; the GIGA and its shield count as one stack):',
             '  %-20s %9s %24s' % ('part', 'lid off', 'lid + dropper unit off')]
    ok = True
    for nm in ELECTRONICS:
        a = _visible_share(ctx, nm, ('Lid',))
        b = _visible_share(ctx, nm, ('Lid',) + UNIT)
        need = ACCESS_MIN.get(nm)
        good = need is None or b >= need
        ok = ok and good
        lines.append('  %s %-20s %8.0f%% %23.0f%%   %s' % ('ok  ' if good else 'FAIL', nm, 100 * a, 100 * b, 'at least %.0f%%' % (100 * need) if need else 'frame off'))
    lines.append('points on the GIGA seen from straight above with the lid off and the dropper unit in:')
    for nm, p, expect in ACCESS_POINTS:
        rows = K3._cast(ctx.root, p, (0.0, 0.0, 1.0), own=('Lid', GHOST) + STACK)
        is_open = not rows
        good = is_open == expect
        ok = ok and good
        lines.append('  %s %-58s %s%s' % ('ok  ' if good else 'FAIL', nm, 'open' if is_open else 'covered by ' + rows[0][1], '' if expect else '   (known, spec 7.4)'))
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)


def report_omni(ctx):
    """Omni, arm and pins swept through the full travel against every other body. Expect 'no interference'."""
    K3.omni_sweep(ctx, omni=P.OMNI, omni_at=P.omni_at, travel=P.OMNI['travel'])
    return 'omni sweep done'


def report_tof(ctx):
    """Nine 25 degree cones, 17 rays each, against every part but the ToF boards. Rev 3 model: no ray blocked on any sensor."""
    K3.tof_cones(ctx)
    return 'ToF cones done'


def report_camera(ctx):
    """25 rays through each camera's 65.9 x 51.8 degree field, from the lens face. Rev 3 model: no ray blocked."""
    K3.camera_fov(ctx)
    return 'camera views done'


SPEC_PAIRS = [   # (component, component, number quoted by the spec or None, smallest acceptable mm, label)
    ('Silver module SM', 'Omni wheel', None, 3.0, 'silver module (x 7.5, beside the omni bay) to the omni wheel [placeholder module size]'),
    ('Silver module SM', 'GIGA posts', None, 1.5, 'silver module to the GIGA post H1 [placeholder module size]'),
    ('Wi-Fi antenna', 'Omni wheel', None, 5.0, 'Wi-Fi antenna on the front wall to the omni wheel at rest'),
    ('N20 motor', 'Motor L', 10.6, 5.0, 'N20 to the left drive motor (first model 10.6 mm)'),
    ('Chute B left', 'Wheel L', 6.7, 5.0, 'chute to the left wheel (rev 3 6.7 mm)'),
    ('Chute A right', 'Wheel R', 6.7, 5.0, 'chute to the right wheel'),
    ('Omni wheel', 'GIGA posts', 2.7, 2.0, 'omni wheel to GIGA post H1 (spec 2.7 mm)'),
    ('Arduino GIGA R1', 'Tub', 1.7, 1.0, 'GIGA stack to the tub (3.1 mm plain wall, 1.7 mm behind the right bumper)'),
    ('Battery', 'Motor L', 1.0, 0.5, 'battery above the left drive motor (spec 1.0 mm)'),
    ('Dropper plate', 'Lid', None, 5.0, 'kit plate to the lid'),
    ('Camera L', 'Wheel L', 3.6, 2.5, 'camera lens block to the wheel top (rev 3 3.6 mm)'),
    (GHOST, 'Main PCB', 5.0, 3.0, 'stepper bay to the GIGA stack, nearest at the shield (spec 5.0 mm in plan view; 3D is larger where the heights differ)'),
    (GHOST, 'Hopper B left', 5.2, 3.0, 'stepper bay to hopper B (spec 5.2 mm)'),
    (GHOST, 'Battery', 8.2, 5.0, 'stepper bay to the battery (spec 8.2 mm)'),
]


def report_clearances(ctx):
    """Nearest 3D distance of the pairs the spec quotes, against the number in the spec and a smallest acceptable value."""
    mm = ctx.app.measureManager
    lines, ok = [], True
    for a, b, quoted, minimum, label in SPEC_PAIRS:
        best = min(mm.measureMinimumDistance(ba, bb).value for ba in _bodies(ctx, a) for bb in _bodies(ctx, b)) * 10.0
        good = best >= minimum
        ok = ok and good
        lines.append('  %s %6.2f mm  (spec %s, at least %.1f)  %-24s | %-18s  %s' % ('ok  ' if good else 'FAIL', best, '%.1f' % quoted if quoted else '--', minimum, a, b, label))
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)


def report_stepper(ctx):
    """With the N20 cartridge lifted out, the 28BYJ-48 bay must be free of every other component (spec 6.3)."""
    rows = local_interference(ctx, [GHOST], exclude=('N20 motor', 'N20 face plate'))
    lines = ['stepper bay against everything except the N20 cartridge:'] + ['  FAIL overlap %.4f cm3  %s x %s' % r for r in rows]
    if not rows:
        lines.append('  ok   no overlap')
    lines.append('RESULT: ' + ('PASS' if not rows else 'FAIL'))
    return '\n'.join(lines)


def report_mass(ctx):
    """Mass and centre of mass: printed parts from the model volume, bought parts from the rev 3 budget [placeholder]. Front load and the front-lift acceleration limit."""
    rows, tot, mx, my, mz, unassigned, com = [], 0.0, 0.0, 0.0, 0.0, [], {}
    for o in _all_occurrences(ctx):
        nm = o.component.name
        if nm == GHOST:
            continue
        vol, cx, cy, cz = 0.0, 0.0, 0.0, 0.0
        for j in range(o.bRepBodies.count):
            b = o.bRepBodies.item(j)
            c = b.physicalProperties.centerOfMass
            v = b.volume
            vol += v
            cx, cy, cz = cx + c.x * v, cy + c.y * v, cz + c.z * v
        if vol <= 0:
            continue
        cx, cy, cz = cx / vol, cy / vol, cz / vol
        if nm in P.PRINTED:
            g, note = vol * P.PETG_G_CM3 * P.PRINT_FILL, 'printed'
        elif nm in P.BOUGHT_G:
            g, note = P.BOUGHT_G[nm], 'bought'
        else:
            unassigned.append(nm)
            continue
        rows.append((g, nm, vol, cx, cz, note))
        com[nm] = (cx, cy, cz)
        tot, mx, my, mz = tot + g, mx + g * cx, my + g * cy, mz + g * cz
    for nm, g, x, z in P.UNMODELLED_G:
        rows.append((g, nm, 0.0, x, z, 'not modelled'))
        tot, mx, mz = tot + g, mx + g * x, mz + g * z
    X, Y, Z = mx / tot, my / tot, mz / tot
    lines = ['mass budget [placeholder]: printed = volume x %.2f g/cm3 x fill %.2f, bought = rev 3 budget (com.py)' % (P.PETG_G_CM3, P.PRINT_FILL)]
    lines += ['  %6.1f g  %-26s %-12s vol %8.2f cm3  x %6.2f  z %5.2f' % (g, nm, note, vol, x, z) for g, nm, vol, x, z, note in sorted(rows, reverse=True)[:14]]
    lift = 9.81 * X / Z if Z > 0 else 0.0
    lines.append('total %.0f g (rev 3 budget 1070 g), centre of mass x %+.2f  y %+.2f  z %.2f cm (rev 3: +0.90 and 6.3)' % (tot, X, Y, Z))
    lines.append('front load %.1f %% at the omni 7.0 cm ahead (rev 3: 11.9 %% at 7.95)' % (100.0 * X / P.OMNI['rest'][0]))
    lines.append('front lift limit g x COM_x / COM_h = %.2f m/s^2 (rev 3: 1.5; the firmware PWM ramp assumes at least 1.0)' % lift)
    problems = []
    if unassigned:
        problems.append('no mass assigned to: %s' % ', '.join(unassigned))
    cam = com.get('Camera L')
    if cam and abs(cam[1]) < 5.0:            # the camera component is placed with a transform: its centre of mass must come back in world coordinates (y about 8.8)
        problems.append('Camera L centre of mass %s is not in world coordinates: use occurrence.physicalProperties for placed components' % (tuple(round(v, 2) for v in cam),))
    lines += ['FAIL ' + p for p in problems]
    lines.append('RESULT: ' + ('FAIL' if problems else ('PASS' if lift >= 1.0 else 'WARN')))
    return '\n'.join(lines)
```

- [ ] **Step 2: Interference of the whole model**

Call the Fusion runner with `print(v4_run.report('interference'))` (about 1 to 3 minutes; if the call times out, read the document state and retry once).
Expected: `components 51, interfering pairs 0, above the allowlist 0` and `RESULT: PASS`. (51 = tub, 2 wheels, 2 motors, 2 face plates, 3 omni, nub, 2 bumpers, 2 switches, frame, 9 ToF, 2 cameras, lid, handle bar, 5 control parts, USB socket, dropper floor, plate, N20, N20 face plate, kits, 2 hoppers, 2 chutes, posts, GIGA, PCB, battery, FP, SM, Wi-Fi antenna; the hidden stepper bay is not counted.)
If a pair overlaps: decide whether the geometry or the parameter is wrong; fix the builder or `v3_params4.py` (and the spec if a spec number changes), rebuild the affected stages with `v4_run.stage([...], reset=False)` after deleting nothing (a stage rebuild duplicates its components, so rebuild with `reset=True` and 'all'). Only add `ALLOW[frozenset((a, b))] = volume` for an overlap that is deliberate, with a comment that says why.

- [ ] **Step 3: Envelope, belly line**

Call the Fusion runner with `print(v4_run.report('envelope'))`, then `print(v4_run.report('belly'))`.
Expected envelope: the bumper plates at `r_max 11.000`, their two switches at about 10.75 (behind the plate, rev 3 layout), nothing else above 10.5 (the USB-C socket front face is flush at 10.47), `highest point z 16.00`, `omni front edge at rest x 10.00`, `RESULT: PASS`.
Expected belly: `known` lines for exactly the items in `BELLY_KNOWN` (wheels 0.00, omni wheel 0.00, rear nub 2.00, motors 3.00, floor sensors 2.80, chute lip about 3.00, omni arm about 2.45, omni pins about 2.80), the pivot pin line `z 3.40: -1.0 mm against the belly line`, and `RESULT: PASS`. A `FAIL` line names a component that hangs below the belly: the likely culprit is a boss, leg or post extruded from `z 3.85` down (they must start at the floor top 3.85 to 3.9, never lower).

- [ ] **Step 4: Removal paths**

Call the Fusion runner with `print(v4_run.report('removal', only=[0, 1, 2, 3]))`, then `only=[4, 5]`, then `only=[6]` (the teardown scenario repeats the dropper unit and frame group lifts and takes the longest). A call is cut off after about 60 s, so in the run of 8 Oct the pieces were: scenarios 0 to 3 in one call, 4 and 5 one call each, and the teardown group by group, `only=[6], window=(g, 1, 10**6)` for each group g from 0 to 11 (each group's whole path against every body not yet removed; the result file says so in its header).
Expected: seven `ok` lines (numbered 0 to 6) and `RESULT: PASS` for each call. The lid hooks and the cradle prongs are ignored on purpose (`FLEX`: they bend out of the way, their forces are in the spec). A `FAIL` line lists the pair and the step within its leg: a wheel path failing means the arch is too small or something hangs into it (the first model had a screw boss over the tyre's top rim); the frame group failing on `Camera L x Wheel L` means the lens block cannot lift past the wheel; the lid failing against `Handle bar` means the slot is too small; the dropper unit failing against the frame means a seat or the handle post is in its way (a lift-out unit has to leave through the open middle of the ring); a cartridge failing on `Battery` or `Main PCB` means the part above it was not taken out first (the left motor is under the battery edge, the right motor under the GIGA stack, which the spec's service table must say).

- [ ] **Step 5: Access from above**

Call the Fusion runner with `print(v4_run.report('access'))` (a few hundred rays; a minute or two).
Expected: a table of the share of each electronics part that has nothing above it, with the lid off and with the lid plus the dropper unit off; the GIGA, shield and battery rows `ok` against `ACCESS_MIN`, and `RESULT: PASS`. The silver module and the right drive motor stay at 0 % on purpose (they need the frame off; spec 3.3). A `FAIL` row means a part of the frame or the control deck covers more of the stack than the dry run predicted: look at which component the up-ray hits before changing a threshold.

- [ ] **Step 6: Omni sweep, ToF cones, camera views**

Call the Fusion runner three times: `print(v4_run.report('omni'))`, `print(v4_run.report('tof'))`, `print(v4_run.report('camera'))`.
Expected: omni `no interference at any of the 13 positions between 0 and 2.5 cm`; ToF cones: for each of the nine sensors `blocked  0 / 17  first blocker None` (rev 3 model result; if a rev 4 part blocks a ray, the line names it and the distance, and that is a design finding for the spec); camera: `camera L: 0 / 25 view rays hit a robot part within 3.0 cm of the lens`, same for R.

- [ ] **Step 7: Spec clearances and the stepper bay**

Call the Fusion runner with `print(v4_run.report('clearances'))`, then `print(v4_run.report('stepper'))`.
Expected: every row `ok` against its smallest acceptable value and `RESULT: PASS`; stepper report `ok   no overlap`, `RESULT: PASS`. A row that is lower than the spec number by more than 1 mm is a spec error to correct, with the measured value as the new number.

- [ ] **Step 8: Mass and centre of mass**

Call the Fusion runner with `print(v4_run.report('mass'))`.
Expected: `RESULT: PASS` or `RESULT: WARN`. Compare with the spec section 4 numbers (centre of mass +0.89 cm ahead of the axle and 6.82 cm high, front load 12.8 %, lift limit 1.29 m/s^2; the Fusion masses of the printed parts come from the real solid volumes, so expect them to differ from the emulator by a few percent). A `WARN` means the lift limit is below 1.0 m/s^2: not a failure of the model, but it goes into the spec (section 4 acceleration paragraph) and into the open items for the firmware ramp (rev 3 D5). `FAIL` means a component has no mass assigned (add it to `P.PRINTED`, `P.BOUGHT_G` or `P.UNMODELLED_G` and run again) or that the camera's centre of mass came back in its own coordinates (switch the report to `occurrence.physicalProperties` for every component).

- [ ] **Step 9: Save the raw outputs**

For each report above write the exact text the tool returned into `docs/superpowers/specs/v3-fusion/results/v4_<kind>.txt` with the Write tool (`interference`, `envelope`, `belly`, `removal`, `access`, `omni`, `tof`, `camera`, `clearances`, `stepper`, `mass`). Do not edit the text.

- [ ] **Step 10: Checkpoint (no commit)**

Unit tests `OK`; the eleven result files exist; nothing committed.

---

### Task 10: Documentation, views, export and notes

**Files:**
- Modify: `docs/superpowers/specs/v3-fusion/README.md` (rewrite for rev 4)
- Create: `docs/superpowers/specs/v3-fusion/Theseus_V3_rev4.f3d`, `docs/superpowers/specs/v3-fusion/views/rev4_top_lid_off.png`, `rev4_iso.png`, `rev4_side.png`, `rev4_underside.png`
- Modify: `docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md` (status line, section 12)
- Modify: memory notes in `C:\Users\christopher.shu\.claude\projects\C--Users-christopher-shu-OneDrive---St--Andrew-s-College-Desktop-Robocup-Junoir-Theseus\memory\`

**Interfaces:**
- Consumes: the finished model and the result files of Task 9.
- Produces: the documentation a later session needs to rebuild, check and extend the model.

- [ ] **Step 1: Export the model**

Call `mcp__Autodesk_Fusion__fusion_mcp_execute` with `featureType: "script"` and:

```python
import adsk.core
import adsk.fusion


def run(context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    path = r"C:\Users\christopher.shu\OneDrive - St. Andrew's College\Desktop\Robocup-Junoir-Theseus\docs\superpowers\specs\v3-fusion\Theseus_V3_rev4.f3d"
    opts = design.exportManager.createFusionArchiveExportOptions(path)
    print('exported', design.exportManager.execute(opts))
```

Expected: `exported True`. Check with `ls -la docs/superpowers/specs/v3-fusion/Theseus_V3_rev4.f3d` (a few MB). The Fusion document itself stays unsaved: saving or closing it is the user's decision.

- [ ] **Step 2: Screenshots**

Call the Fusion runner with `print(v4_run.show(['Lid', 'Handle bar']))`, then `mcp__Autodesk_Fusion__fusion_mcp_read` with `queryType: "screenshot"`, `direction: "top"`; copy the file named in the tool result with `cp "<path from the result>" docs/superpowers/specs/v3-fusion/views/rev4_top_lid_off.png`. Then `print(v4_run.show([]))` and take `direction: "iso-top-left"` (save as `rev4_iso.png`), `"left"` (save as `rev4_side.png`) and `"bottom"` (save as `rev4_underside.png`). Open each image with the Read tool and check it shows the model (wheel arches, cradles, omni inside the body, frame ring with the ToF boards, lid with the handle slot, T-handle). Leave the design showing every component except the stepper bay.

- [ ] **Step 3: Rewrite the README**

Replace the whole of `docs/superpowers/specs/v3-fusion/README.md` with the text below, then fill the last section from the Task 9 result files as the instruction there says.

````markdown
# Theseus V3 in Autodesk Fusion

A 3D layout model of the V3 robot, built by script so it can be regenerated and checked. **Revision 4** (the structure, mounting and usability redesign in [2026-10-07-theseus-v3-mechanical-design.md](../2026-10-07-theseus-v3-mechanical-design.md)) is the current model. The revision 3 layout model it replaced is kept as `Theseus_V3_baseline.f3d` and `v3_model.py`. All rev 4 numbers come from [v3_params4.py](../v3-checks/v3_params4.py), which is unit-tested by `test_v3_params4.py`. Units cm, frame x forward, y left, z up, origin at the midpoint of the drive axle on the floor line (Fusion is set to Z up here).

**Status:** layout model. Every part sits at its specified position and every interface the spec defines is modelled. Nothing has been measured or printed; masses and the PCB are placeholders as in the spec. It does not replace the CAD detail design (see "Not modelled").

## Files

| File | What it is |
|---|---|
| `fusion_lib.py` | Helpers: sketch shapes, extrude with offset start, cuts, `axis_prism` along a sloped axis, colours |
| `v4_model.py` | The rev 4 model: one function per stage, registered in build order (`STAGES`), `build()` |
| `v4_checks3d.py` | Stage probes, local interference, whole-model reports (interference, envelope, belly line, removal paths, access from above, mass and centre of mass, spec clearances, stepper bay) |
| `v4_run.py` | Entry points for the Fusion script runner: `stage`, `report`, `selftest`, `show` |
| `v3_checks3d.py` | Read-only 3D checks shared with rev 3: ToF cones, camera fields of view, omni sweep |
| `v3_model.py`, `v3-fusion.py`, `v3-fusion.manifest`, `Theseus_V3_baseline.f3d` | The rev 3 layout model, its entry point and its export; rev 4 reuses its wheel, motor, nub, bumper, camera, GIGA stack, battery and floor-sensor builders |
| `Theseus_V3_rev4.f3d` | Export of the finished rev 4 model (safety copy; the Fusion document itself is unsaved) |
| `results/v4_*.txt` | Raw output of each whole-model report on the finished model |
| `dryrun/` | The same build and reports on a geometry emulator, no Fusion needed (about 4 minutes): predicted the Fusion results and found a dozen problems in the plan and the design before the first Fusion call. Its README says what it can and cannot see |
| `views/rev4_*.png` | Screenshots: top (lid and handle hidden), iso, side, underside |

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

## What is modelled

Tub: floor 4 mm, wall to z 8.7, rear chamfer, two wheel openings that run out through arches (the strip of wall above each arch is notched at the camera window), two motor cradles (seat, shaft notch, plate slot, web, ledges, and two prongs each with the hook shape, as separate bodies because they flex), omni bay and arm slot, floor-sensor holes, square chute holes, bumper recesses with switch pockets, six insert bosses with ribs to the wall (at +-12, +-60 and +-118 degrees), battery tray on three legs with the notch over the left motor, USB-C hole in the front-right wall above the bumper band. Two drive wheels and Pololu motors, two printed face plates with one front ear, the 60 mm omni at x 7.0 on one arm on the +y side, rear nub, two bumper plates with microswitches. Upper frame in one piece: ring with nine ToF pockets and beam windows and two camera windows, front bridge (3 mm web with a rib, widened to a control deck at x 5.2 to 9.15) and rear spoke with half-lap seats for the dropper floor, handle post, six screw holes, four hook rebates and grooves. Nine VL53L0X boards, two OpenMV boards, the lid (plate, camera humps, handle slot, front notch over the control deck, finger notches, four skirt segments with hook bumps), the handle bar with the victim LED on top of its front end, the four control parts on the deck and the rib (power switch, start button, two status LEDs) and the USB-C socket in the wall. The lift-out dropper unit: the dropper floor (its own component, with slots A and B and the N20 pocket), kit plate with eight 14 mm pockets, eight kits, N20 cartridge in its floor pocket with its face plate, two hoppers with sockets; and two 13 mm square channels. Four GIGA posts, GIGA R1 plus shield stack with the USB-C J12 body, battery, floor sensors FP and SM (7.5 cm ahead of the axle), the Wi-Fi antenna stuck on the inside of the front wall. A hidden keep-out for the 28BYJ-48 bay.

## Not modelled

Heat-set inserts and screws, snap-tab and L-slit details (the lid hook tongues; the hook bumps are separate bodies), flexing of the prongs and hooks (their forces are calculations, see the spec; the removal report ignores both), the omni pivot ears, lugs and springs, bumper hinge sockets, hinges and return springs, the nub boss, camera and ToF cradles with their snap tabs and the cable notches, hopper snap tabs, battery strap slots, cables and connectors, fillets and rounded channel corners, the plate hub, the encoder board outline, the ToF connector edges, the main PCB (a placeholder block). Part sizes for the power switch, start button, LEDs and USB-C socket are placeholders; the snap tabs or thumbscrews that hold the dropper unit on its seats are not drawn.

## Checks

| Report | Pass criterion |
|---|---|
| per-stage probes | material (or none) at the listed points of each component; no overlap of the new components with any other |
| `interference` | no pair of components overlaps (hidden stepper bay excluded) |
| `envelope` | nothing beyond r 10.5 except the bumper plates (11.0) and their switches behind them; highest point 16.0, the victim LED on the bar (limit 25); omni front edge inside the wall |
| `belly` | only the known parts hang below z 3.5 |
| `removal` | lid, wheels, kit plate and N20, channels, the dropper unit, battery, frame group and GIGA leave along their paths without touching anything |
| `access` | with the lid and the dropper unit off, the GIGA, shield and battery are at least 70, 70 and 80 % open from above |
| `omni`, `tof`, `camera` | omni sweep without interference; no ToF or camera ray blocked by a robot part |
| `clearances` | the gaps the spec quotes are at least their smallest acceptable value |
| `stepper` | the 28BYJ-48 bay is free once the N20 cartridge is out |
| `mass` | centre of mass and front-lift limit (warns below 1.0 m/s^2) |

## Findings of the rev 4 build

Write one bullet per line of the Task 9 result files that is not a plain pass: each `known` line of `results/v4_belly.txt`, each `WARN` or `FAIL` line, each `ALLOW` entry in `v4_checks3d.py` with its reason, and every spec number that the clearances report corrected. If there are none, write "None."
````

- [ ] **Step 4: Update the spec**

In `docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md`: replace the first sentence of the status paragraph (`Status: **proposal for review.** Nothing here is built, measured or printed, and the Fusion model still shows rev 3 (see [v3-fusion/](v3-fusion/README.md)).`) with:

```
Status: **design confirmed by you on 7 Oct (D10 N20, D12 controls on the front deck of the frame, the lift-out dropper unit, the axle at the body centre); the layout model was rebuilt to match it** ([v3-fusion/](v3-fusion/README.md), `Theseus_V3_rev4.f3d`) and checked in Fusion (probes, interference, envelope, belly line, removal paths, ToF cones, camera views, omni sweep, spec clearances, mass). Nothing here is measured or printed.
```

and, in section 12 under **After approval, in the rebuilt 3D model:** add one paragraph after the list: `Done in the rebuilt model (results in v3-fusion/results/): <one sentence per report that did not simply pass, copied from the README findings, or "all reports passed">.`

- [ ] **Step 5: Update the memory notes**

Write these into the memory folder (Write tool, then fix `MEMORY.md`): in `project_theseus_v3_fusion_model.md` replace the status paragraph with a short statement that the model was rebuilt to rev 4 on the date of the run, where `v4_*` files live, how to run (`v4_run.stage`, `v4_run.report`), and the findings from the README; in `project_theseus_v3_structure_review.md` set the state to "spec confirmed, model rebuilt, awaiting the user's review of the model"; keep both under the existing front matter. Add one line to `reference_fusion_mcp_scripting.md` for any new Fusion API quirk met during the run (for example `pointContainment` semantics, `Matrix3D.translation`, interference with coincident faces).

- [ ] **Step 6: Final check (no commit)**

Run:

```bash
cd docs/superpowers/specs/v3-checks && PYTHONPATH="C:/Users/christopher.shu/pl4" python -m unittest test_v3_params4 && PYTHONPATH="C:/Users/christopher.shu/pl4" python run_all.py structure_figs mount_check stepper_bay ring_gaps
git status --short docs
```

Expected: the unit tests `OK`; the four checks `ok`; `git status` lists only untracked files under `docs/superpowers` (nothing staged, nothing committed). Report to the user: what was built, the report results, the findings list, and the files to look at ([README](../specs/v3-fusion/README.md), `views/rev4_iso.png`). Ask whether they want the Fusion document saved into their project (not done) and whether to commit (not done).
