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
        down = P.CHUTE['out_w'] / 2 - P.CHUTE['lift']        # the outer floor is this far below the axis; the section is out_w wide
        low = chute_exit.tube_min_z(p0, e, 'square', down, half_lat=P.CHUTE['out_w'] / 2)
        self.assertAlmostEqual(low, 2.78, delta=0.02)        # the 18 mm channel hangs 2 mm lower at the wall than the 13 mm one did (3.00)
        self.assertGreaterEqual(low, 2.70)                    # the terrain tables of the spec assume a chute lip at z 2.67

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

    def test_cube_passes_the_slot_the_hopper_and_the_bore_at_any_turn(self):
        """The simulation of 8 Oct (chute_dynamics.py): a kit that arrives square to the slot jams in a 13 mm bore, and a kit that tips on its way into a 14.5 mm slot wedges across its
        diagonal (14.57 mm). A 16 mm slot, hopper void and bore take the cube lying at any turn (face diagonal 14.57 mm, with 0.12 mm to spare) and tipped any way except within 4.5 degrees
        of standing on a corner (0.9 % of attitudes, 16.25 mm: cube_slot.py); the review of 8 Oct caught 'in any attitude' as too strong."""
        diag = 1.03 * math.sqrt(2)
        for name, width in (('slot', P.PLATE['slot']), ('hopper void', P.CHUTE['hopper_in']), ('bore', P.CHUTE['in_w'])):
            self.assertGreaterEqual(width, diag + 0.12, name)
        self.assertGreaterEqual(P.CHUTE['in_w'], P.CHUTE['hopper_in'])                  # no wall end catches a kit between the hopper and the channel
        self.assertGreaterEqual(P.CHUTE['hopper_in'], P.PLATE['slot'])
        self.assertAlmostEqual(P.CHUTE['out_w'], P.CHUTE['in_w'] + 2 * P.CHUTE['wall'], places=6)
        self.assertGreaterEqual(P.CHUTE['hopper_out'] - P.CHUTE['hopper_in'], 0.4)      # hopper walls at least 2 mm
        self.assertGreaterEqual(P.CHUTE['flange'] - P.CHUTE['hopper_out'], 0.3)

    def test_bore_is_wider_and_taller_than_a_tumbling_cube(self):
        """With bouncy impacts the chute simulation wedged tumbling kits between the two side walls and between the floor and the ceiling of a 16 mm bore: a cube in a general attitude is
        up to its space diagonal, 17.84 mm, wide. An 18 mm bore cannot wedge it. The square section is centred `lift` above the axis, so the floor is 8 mm below the axis and the
        ceiling 10 mm above it: the floor, which sets the lowest point at the wall, stays where the 16 mm design had it. The margin is thin: 0.16 mm at the nominal 10.3 mm kit. A 10.5 mm kit (the
        rev 3 tolerance: space diagonal 18.19 mm) or a bore printed 0.4 mm small does wedge some bouncing kits (the tolerance corners of chute_dynamics.py: 94 to 97 % get out): 19 mm would
        cost 0.5 mm of the chute-to-wheel gap (spec 6.2, D21)."""
        self.assertGreaterEqual(P.CHUTE['in_w'], 1.03 * math.sqrt(3) + 0.005)
        self.assertAlmostEqual(P.CHUTE['in_w'] / 2 - P.CHUTE['lift'], 0.8, places=6)

    def test_parking_allowance_is_what_the_slot_leaves_over_the_pocket(self):
        """A kit lying corner to corner in its pocket is as wide along the ring as the pocket (14 mm), so the plate may be parked off by (slot - pocket) / 2 = 1.0 mm before the kit's corner
        overhangs the slot's edge; the simulation with the pocket walls in (chute_dynamics.py) takes every kit out up to 1 mm off and loses about a fifth at 2 mm. This number is the
        dropper's parking requirement (spec 6.2); change the slot or the pocket and the spec's 1 mm has to be re-derived."""
        self.assertAlmostEqual((P.PLATE['slot'] - P.PLATE['pocket']) / 2, 0.1, places=6)

    def test_drop_sequences_with_the_bigger_slot(self):
        import platesim
        import platesim2
        pl = platesim.Plate(r=38.6, pocket=14.0, slot=P.PLATE['slot'] * 10.0)
        res, viol = platesim2.check(pl)
        self.assertEqual(res['violations'], 0, viol[:3])
        self.assertGreaterEqual(res['park_margin_mm'], 1.5)

    def test_slot_stays_inside_the_dropper_floor(self):
        sx, sy = P.slot_xy('B')
        r_slot = math.hypot(sx - P.PLATE['cx'], sy - P.PLATE['cy'])
        far = math.hypot(r_slot + P.PLATE['slot'] / 2, P.PLATE['slot'] / 2)           # the slot's corner farthest from the plate axis
        self.assertLessEqual(far, P.DROPPER_FLOOR['r'] - 0.3)

    def test_channel_floor_has_no_hole(self):
        """The trough cut is a vertical prism through a sloping floor, so it meets the floor further down the axis than its plan corner: 2.55 cm at the floor's top face and 2.65 cm at its
        underside, not the 2.04 cm of the corner at axis height. A slab that ended at 2.5 cm left a notch in the bore floor (found by the reviewer, 8 Oct). Scan the floor 0.1 mm under its
        top face, across the bore width, from the slot centre on: every point must be material."""
        import numpy as np
        import chute_geometry as cg
        solids = cg.build(1)['channel']
        p0, d, ex, ey, ez, e = cg.channel_frame(1)
        Ch = P.CHUTE
        wi = Ch['in_w'] / 2
        y_floor = Ch['lift'] - wi - 0.01                                          # in the channel's frame: 0.1 mm under the face a kit slides on
        s, a = np.meshgrid(np.arange(0.02, 4.0, 0.01), np.arange(-wi + 0.01, wi, 0.02), indexing='ij')
        pts = (p0 + s[..., None] * d + a[..., None] * ex + y_floor * ey).reshape(-1, 3)
        inside = np.zeros(len(pts), bool)
        for v, p in solids:
            N, b = np.array([h.n for h in p]), np.array([h.b for h in p])
            inside |= (pts @ N.T <= b + 1e-9).all(axis=1)
        missing = pts[~inside]
        self.assertEqual(len(missing), 0, 'floor points that are not material: %d, first at %s' % (len(missing), np.round(missing[0], 3) if len(missing) else None))

    def test_hopper_is_one_piece(self):
        """Cutting the channel's own section (21.2 mm wide, topped at its ceiling plane) out of the hopper left the hopper's downhill corner, above the channel, hanging free: Fusion split
        each hopper into two bodies, the second an 85 mm3 lintel (8 Oct). The convex model of chute_geometry.py, gridded at 0.5 mm, must be one connected piece; it is two when the
        socket tool stops at the channel's ceiling (CHUTE['socket_up'] = 0)."""
        import numpy as np
        import chute_geometry as cg
        solids = cg.build(1)['hopper']
        verts = np.vstack([v for v, p in solids])
        step = 0.05
        lo, hi = verts.min(axis=0) - 0.1, verts.max(axis=0) + 0.1
        axes = [np.arange(l, h, step) + step / 2 for l, h in zip(lo, hi)]
        grid = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1)
        pts = grid.reshape(-1, 3)
        inside = np.zeros(len(pts), bool)
        for v, p in solids:
            N, b = np.array([h.n for h in p]), np.array([h.b for h in p])
            inside |= (pts @ N.T <= b + 1e-9).all(axis=1)
        inside = inside.reshape(grid.shape[:3])
        seen = np.zeros_like(inside)
        sizes = []
        for start in zip(*np.nonzero(inside)):
            if seen[start]:
                continue
            seen[start] = True
            todo, n = [start], 0
            while todo:
                i, j, k = todo.pop()
                n += 1
                for di, dj, dk in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                    q = (i + di, j + dj, k + dk)
                    if 0 <= q[0] < inside.shape[0] and 0 <= q[1] < inside.shape[1] and 0 <= q[2] < inside.shape[2] and inside[q] and not seen[q]:
                        seen[q] = True
                        todo.append(q)
            sizes.append(n * step ** 3)
        sizes = [s for s in sizes if s > 0.005]                                          # one to three cells are the gridding of a sliver piece (0.0004 cm3), not a part
        self.assertEqual(len(sizes), 1, 'hopper pieces (cm3): %s' % ', '.join('%.4f' % s for s in sorted(sizes)))
        self.assertGreater(sizes[0], 1.5)                                                # the wall, the flange and the floor strip: about 1.9 cm3


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
        self.assertGreaterEqual(min(v for _, v in rows), 3.5)       # mm in plan against the stack, battery, hoppers, wall (4.0 mm to hopper B since the chute redesign of 8 Oct made the hoppers 2.5 mm bigger; it was 5.2)


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
