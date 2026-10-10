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
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

import chute_exit
import mount_check
import ring_gaps
import stepper_bay
import tof_mount as T
import v3_params4 as P


class OmniInsideTheBody(unittest.TestCase):
    def test_the_wheel_is_the_nexus_14145(self):
        """Nexus datasheet 14145 (read 9 Oct): 60 mm, 25.6 mm wide, 12 mm axle hole, 73 g. The 8 Oct layout had assumed a 20 mm wide wheel."""
        W = P.OMNI_WHEEL
        self.assertAlmostEqual(2 * W['r'], 6.0, places=6)
        self.assertAlmostEqual(W['w'], 2.56, places=6)
        self.assertAlmostEqual(2 * W['bore_r'], 1.2, places=6)
        self.assertAlmostEqual(W['mass_g'], 73.0, places=6)
        self.assertEqual((P.OMNI['r'], P.OMNI['w']), (W['r'], W['w']))

    def test_pivot_arm_and_rest_position(self):
        px, pz = P.OMNI['pivot']
        ox, oz = P.OMNI['rest']
        self.assertAlmostEqual(math.hypot(ox - px, oz - pz), 4.6, places=6)
        self.assertAlmostEqual(pz, 4.2, places=6)
        self.assertAlmostEqual(px, 2.409, places=2)
        self.assertEqual((ox, oz), (6.85, 3.0))

    def test_front_rim_stays_inside_the_wall_over_the_whole_travel(self):
        """The rim's outer corner (the wheel's outer face, y 1.58) against the wall's inner face r 10.3: at least 1.5 mm at every point of the arc. The 8 Oct layout (a 20 mm wheel on the centre line) had 2.1 mm."""
        outer_y = max(abs(y) for y in P.OMNI['y_wheel'])
        for k in range(101):
            xc, zc = P.omni_at(P.OMNI['travel'] * k / 100)
            self.assertGreaterEqual(P.R_INT - math.hypot(xc + P.OMNI['r'], outer_y), 0.15, 'travel %.2f' % (P.OMNI['travel'] * k / 100))

    def test_front_edge_and_top_at_the_ends_of_the_travel(self):
        self.assertAlmostEqual(P.OMNI['rest'][0] + P.OMNI['r'], 9.85, places=6)
        xc, zc = P.omni_at(P.OMNI['travel'])
        self.assertAlmostEqual(xc + P.OMNI['r'], 9.82, places=2)
        self.assertAlmostEqual(zc + P.OMNI['r'], 8.5, places=6)

    def test_arm_angles_over_the_travel(self):
        px, pz = P.OMNI['pivot']
        ox, oz = P.OMNI['rest']
        xc, zc = P.omni_at(P.OMNI['travel'])
        self.assertAlmostEqual(math.degrees(math.atan2(oz - pz, ox - px)), -15.1, places=1)
        self.assertAlmostEqual(math.degrees(math.atan2(zc - pz, xc - px)), 16.4, places=1)

    def test_bridge_underside_clears_the_compressed_omni(self):
        xc, zc = P.omni_at(P.OMNI['travel'])
        self.assertGreaterEqual(P.FRAME['z0'] - (zc + P.OMNI['r']), 0.19)

    def test_single_arm_is_on_the_left_clear_of_the_wheel_and_the_gigas_post(self):
        y0, y1 = P.OMNI['arm_y']
        w0, w1 = P.OMNI['y_wheel']
        self.assertAlmostEqual(y0 - w1, 0.1, places=6)                                       # 1 mm between the wheel's outer face and the arm
        self.assertAlmostEqual(y1 - y0, 0.3, places=6)                                       # a 3 mm aluminium bar
        post = P.giga_hole_xy(0)[1] + P.GIGA_POST['r']                                       # the +y edge of the H1 post
        self.assertGreaterEqual(w0 - post, 0.25)                                             # the wheel's inner face is 2.5 mm or more from it
        self.assertAlmostEqual(P.OMNI['yc'], (w0 + w1) / 2, places=6)
        self.assertGreaterEqual(P.OMNI_ARM_SLOT_Y[0], w1 - 0.2)
        self.assertLessEqual(P.OMNI_ARM_SLOT_Y[1] - y1, 0.15)

    def test_floor_port_keeps_a_floor_wall_to_the_arm_slot(self):
        fp = P.FLOOR_FRONT
        self.assertGreaterEqual(fp['y'] - (fp['w'] + 0.1) / 2 - P.OMNI_ARM_SLOT_Y[1], 0.3)


class OmniMount(unittest.TestCase):
    def test_pin_hole_spring_and_arm_stay_above_the_belly_line(self):
        """The pin of the 8 Oct layout hung 1.0 mm under the belly line (z 3.5), where the Dangerous Zone 30 degree stairs case has 0.1 mm of margin. The pivot is raised to z 4.2:
        3.5 mm of printed material under the pin hole, the spring coil and the arm all above the belly line."""
        pz = P.OMNI['pivot'][1]
        M = P.OMNI_MOUNT
        self.assertGreaterEqual((pz - M['hole_r']) - P.Z_BELLY, 0.35)
        self.assertGreaterEqual(pz - P.OMNI_SPRING['od'] / 2, P.Z_BELLY + 0.1)
        self.assertGreaterEqual(pz - P.OMNI['arm_half'], P.Z_BELLY)
        self.assertGreaterEqual(pz - P.OMNI_PIVOT['tube_r'][1], P.Z_BELLY + 0.3)

    def test_pillar_ear_and_pocket_stay_clear_of_the_motor_the_wheel_and_the_tray(self):
        M = P.OMNI_MOUNT
        px, pz = P.OMNI['pivot']
        self.assertGreaterEqual(P.CRADLE['seat_y'][0] - M['pillar']['y'][1], 0.3)            # the pillar and its tail are inboard of the left motor seat (it starts at y 1.3): 5 mm clear
        self.assertGreaterEqual(M['ear']['x'][0] - P.CRADLE['prong_x'][1], 0.5)              # the outer ear (y up to 2.98) reaches the cradle prongs' y range (from 2.85): 6 mm clear in x
        self.assertGreaterEqual(P.TRAY['z'][0] - M['ear']['z'][1], 0.1)                      # under the battery tray (underside 4.9), whose near edge is at y 2.2 to 2.4 over the ear
        self.assertGreaterEqual(M['pillar']['y'][0], 0.2)
        xc, zc = P.omni_at(P.OMNI['travel'])
        r = P.OMNI['r']
        for z in (M['pillar']['z'][0] + 0.05, 4.2, 4.6, M['pillar']['z'][1]):                # the wheel's rear surface, at rest and fully compressed, against the pillar's front face
            for cx, cz in (P.OMNI['rest'], (xc, zc)):
                if abs(z - cz) < r:
                    self.assertGreaterEqual((cx - math.sqrt(r * r - (z - cz) ** 2)) - M['pillar']['x'][1], 0.3, 'wheel to pillar at z %.2f' % z)

    def test_the_stop_pin_runs_in_an_arc_slot_of_the_arm_with_a_wall_all_round(self):
        """The stop is a fixed M3 screw from the pillar to the ear, 8 mm ahead of the pivot; the arm carries the slot (an arc of radius 8 mm about the pivot, 3.3 mm wide), whose two ends are
        the rest stop and the compression stop. A pin on the arm instead, cantilevered 11 mm from the arm into the pillar, would carry the 72 N stop load at 600 MPa in an M3 screw."""
        M = P.OMNI_MOUNT
        S = M['stop']
        px, pz = P.OMNI['pivot']
        slot = P.omni_stop_slot()
        self.assertAlmostEqual(slot['r'], S['r'], places=9)
        self.assertGreaterEqual(slot['hw'] - S['pin_r'], 0.015)                              # 0.15 mm of play to each side of the screw
        for k in range(101):                                                                 # the pin, seen from the arm, over the whole travel
            xc, zc = P.omni_at(P.OMNI['travel'] * k / 100)
            phi = math.atan2(zc - pz, xc - px)
            theta = math.radians(P.omni_stop_psi()) - phi
            self.assertGreaterEqual(theta, math.radians(slot['a_lo']) - (slot['hw'] - S['pin_r']) / S['r'] - 1e-9, 'travel %d' % k)
            self.assertLessEqual(theta, math.radians(slot['a_hi']) + (slot['hw'] - S['pin_r']) / S['r'] + 1e-9, 'travel %d' % k)
        reach = max(abs(S['r'] * math.sin(math.radians(a))) for a in (slot['a_lo'], slot['a_hi'])) + slot['hw']
        self.assertGreaterEqual(P.OMNI['arm_half'] - reach, 0.2)                             # 2 mm of arm above and below the slot
        self.assertGreaterEqual(S['r'] - slot['hw'] - P.OMNI['pivot_hole_r'], 0.3)           # and 3 mm between the slot and the pivot bore
        self.assertGreaterEqual(P.OMNI['arm_len'] * 1.0 - S['r'] - slot['hw'] - 0.3, 0.5)     # far from the axle hole

    def test_the_stop_screw_is_held_at_both_ends(self):
        M, S = P.OMNI_MOUNT, P.OMNI_MOUNT['stop']
        sx, sz = P.omni_stop_pin()
        self.assertGreaterEqual(M['pillar']['x'][1] - (sx + S['hole_r']), 0.15)              # 1.5 mm of pillar in front of the screw's hole
        self.assertGreaterEqual(sx - S['hole_r'] - M['pillar']['x'][0], 0.5)
        self.assertGreaterEqual(M['pillar']['z'][1] - (sz + S['hole_r']), 0.3)               # and above it
        self.assertGreaterEqual(sz - S['hole_r'] - P.Z_FLOOR_TOP, 0.1)                       # clear of the floor under the pocket's edge
        self.assertGreaterEqual(M['ear']['x'][1] - (sx + S['ear_hole_r']), 0.15)               # the ear carries the M3 insert with 1.5 mm of wall in front of it
        self.assertGreaterEqual(M['ear']['z'][1] - (sz + S['ear_hole_r']), 0.3)
        tip = M['pillar']['y'][0] + S['screw_len']                                           # the head is on the pillar's inboard face
        self.assertGreaterEqual(tip - M['ear']['y'][0], 0.5)                                 # 5 mm of thread in the insert
        self.assertLessEqual(tip, M['ear']['y'][0] + S['ear_depth'])
        self.assertGreaterEqual(M['ear']['y'][1] - M['ear']['y'][0], S['ear_depth'] + 0.1)

    def test_the_stop_pin_clears_the_spring_coil(self):
        """A pin 7 mm from the pivot would have grazed the coil (outer radius 5.8 mm); at 8 mm it passes 0.7 mm outside it."""
        S = P.OMNI_MOUNT['stop']
        self.assertGreaterEqual(S['r'] - S['pin_r'] - P.OMNI_SPRING['od'] / 2, 0.05)

    def test_the_arm_slot_and_the_pocket_leave_the_ear_and_the_pillar_on_solid_floor(self):
        M = P.OMNI_MOUNT
        self.assertAlmostEqual(M['pocket']['y'][0], M['pillar']['y'][1], places=6)           # the pocket starts where the pillar ends
        self.assertGreaterEqual(M['ear']['y'][0] - M['pocket']['y'][1], -0.01)               # and ends where the ear starts
        self.assertGreaterEqual(M['pillar']['y'][1] - M['pillar']['y'][0], 0.3)
        self.assertGreaterEqual(M['pocket']['x'][1], P.OMNI_OPENING['wheel']['x'][0] + 0.2)  # the corridor overlaps the wheel's opening (nothing is left of the floor under the stop screw)

    def test_the_spring_s_rear_leg_rests_on_an_adjuster_screw_in_the_pillar_tail(self):
        M, S = P.OMNI_MOUNT, P.OMNI_SPRING
        px, pz = P.OMNI['pivot']
        A, T_ = M['adjuster'], M['tail']
        self.assertAlmostEqual(A['x'], px - S['legs'][0], places=6)                          # the leg ends 10 mm behind the pivot
        self.assertAlmostEqual(A['head_z'][1], pz + S['mean_d'] / 2 - S['wire'] / 2, places=6)    # it leaves the top of the coil and lies on the screw head
        self.assertLessEqual(T_['z'][1], A['head_z'][0] - 0.01)                              # the head stands clear of the tail's top, so a 5.5 mm socket can turn it from the side
        self.assertLessEqual(T_['x'][0], A['x'] - A['head_r'] - 0.1)                         # the tail carries the screw with 1 mm of wall behind its head
        self.assertGreaterEqual(T_['x'][1], M['pillar']['x'][0])                             # and meets the pillar
        self.assertAlmostEqual(A['y'], S['y'][0] + S['wire'] / 2, places=9)                  # the rear leg is the coil's first turn run straight out: the adjuster stands under it, in the same plane
        self.assertEqual(T_['y'][0], M['pillar']['y'][0])                                    # the tail starts where the pillar does and is wide enough for the head (a 90 degree bend in the leg to reach an adjuster inboard of its plane was the 9 Oct form)
        self.assertGreaterEqual(T_['y'][1] - (A['y'] + A['head_r']), 0.0)
        self.assertGreaterEqual((A['y'] - A['head_r']) - T_['y'][0], 0.2)
        self.assertGreaterEqual(P.CRADLE['seat_y'][0] - T_['y'][1], 0.08)                    # and stays 0.8 mm off the left motor's seat
        insert_bottom = T_['z'][1] - 0.57                                                    # an M3 x 5.7 heat-set insert in the tail, flush with its top
        self.assertAlmostEqual(A['head_z'][0] - A['bolt_len'], insert_bottom, delta=0.02)    # the M3 x 6 hex bolt (head 2 mm) reaches the insert's bottom: its whole length is in the insert
        self.assertGreaterEqual(A['bolt_len'], 0.57)
        self.assertGreaterEqual(insert_bottom - P.Z_BELLY, 0.3)                              # and 3 mm of printed floor stays under it

    def test_the_forward_leg_runs_along_the_arm_on_a_seat_pin_and_clears_the_stop_screw_and_the_wheel(self):
        """10 Oct: the 9 Oct model left the forward leg out ('hooked on the arm's top edge'). A tangent leg of the 10 mm coil lies 5 mm from the pivot, along the arm (it turns with the arm, so it never
        slides), and the coil is inboard of the arm's face, so a pin pressed into the arm carries it: the leg rests on the pin's top. It is splayed 4 degrees up from the arm so that it passes 1 mm over the stop
        screw (parallel it passed 0.5 mm over it), stays 1 mm off the wheel's rim and keeps its pin clear of the screw, over the whole travel. Lowering the slot instead put the screw's head 0.4 mm into the floor."""
        S, K, M = P.OMNI_SPRING, P.OMNI_SEAT, P.OMNI_MOUNT
        r_leg = S['wire'] / 2
        sx, sz = P.omni_stop_pin()
        for k in range(101):
            t = P.OMNI['travel'] * k / 100
            (x0, z0), (x1, z1) = P.omni_leg(t)
            d = Point(sx, sz).distance(LineString([(x0, z0), (x1, z1)]))
            self.assertGreaterEqual(d - r_leg - M['stop']['pin_r'], 0.10, 'leg over the stop screw, travel %d' % k)
            cx, cz = P.omni_seat(t)
            self.assertGreaterEqual(math.hypot(cx - sx, cz - sz) - K['r'] - M['stop']['pin_r'], 0.08, 'seat pin to the stop screw, travel %d' % k)
            ox, oz = P.omni_at(t)
            self.assertGreaterEqual(math.hypot(x1 - ox, z1 - oz) - r_leg - P.OMNI['r'], 0.10, 'leg tip to the rim, travel %d' % k)
            self.assertGreaterEqual(math.hypot(cx - ox, cz - oz) - K['r'] - P.OMNI['r'], 0.10, 'seat pin to the rim, travel %d' % k)
        n_seat = P.omni_seat_n()
        self.assertAlmostEqual(n_seat + K['r'], P.omni_leg_n(K['s']) - r_leg / math.cos(math.radians(S['tilt'])), places=9)    # the top of the pin is the leg's underside
        self.assertGreaterEqual(math.hypot(K['s'], n_seat) - K['r'] - S['od'] / 2, 0.2)       # the pin stands outside the coil's footprint, so the coil cannot touch it
        self.assertGreaterEqual(P.OMNI['arm_half'] - (n_seat + K['r']), 0.099)                # 1 mm of bar above the pin's hole
        self.assertAlmostEqual(S['legs'][1], K['s'], places=9)                                # the lever of the leg's force is the pin's distance along the arm
        self.assertGreaterEqual(S['tip'], 0.05)                                               # the leg overhangs the pin by at least 0.5 mm
        y0, y1 = K['y']
        self.assertLessEqual(y0, S['y'][1] - S['wire'] - 0.02)                                # the pin starts inboard of the last turn, which the leg continues
        self.assertGreaterEqual(P.OMNI['arm_y'][0] - y0, 0.3)                                 # 3 mm of pin stand out of the arm's inboard face
        self.assertGreaterEqual(P.OMNI['arm_y'][1] - y1, 0.03)                                # and the pin ends inside the arm: nothing toward the ear
        self.assertGreaterEqual(y1 - P.OMNI['arm_y'][0], 0.2)                                 # 2.5 mm of it in the bar

    def test_the_seat_hole_leaves_metal_round_it_and_the_slot_keeps_its_wall(self):
        K, M = P.OMNI_SEAT, P.OMNI_MOUNT
        slot = P.omni_stop_slot()
        # the arm's frame: s along the arm from the pivot, n up from its axis; the slot as the thick arc it is
        arcs = [(math.radians(a)) for a in [slot['a_lo'] + (slot['a_hi'] - slot['a_lo']) * k / 40 for k in range(41)]]
        shape = LineString([(slot['r'] * math.cos(a), slot['r'] * math.sin(a)) for a in arcs]).buffer(slot['hw'])
        hole = Point(K['s'], P.omni_seat_n()).buffer(K['hole_r'])
        self.assertGreaterEqual(shape.distance(hole), 0.10)                                   # 1 mm of aluminium between the seat hole and the slot's end (21 MPa at the 70 N impact)
        top = max(slot['r'] * math.sin(math.radians(a)) for a in (slot['a_lo'], slot['a_hi'])) + slot['hw']
        bottom = -min(slot['r'] * math.sin(math.radians(a)) for a in (slot['a_lo'], slot['a_hi'])) + slot['hw']
        self.assertGreaterEqual(P.OMNI['arm_half'] - top, 0.13)                               # 1.3 mm of bar above and below the slot
        self.assertGreaterEqual(P.OMNI['arm_half'] - bottom, 0.13)

    def test_the_coil_sits_on_a_printed_arbor_that_leaves_the_tube_and_the_arm_free(self):
        """A torsion spring wants an arbor of 0.8 to 0.9 of its inside diameter. The coil (ID 8.4 mm, about 8.7 when it opens at full travel) hanging on the 5 mm tube would rest 1.7 mm off-centre because
        both legs are pushed up by their supports. A printed sleeve, 7.4 mm outside, on the tube keeps it within 0.5 to 0.65 mm of the middle."""
        S, B, V = P.OMNI_SPRING, P.OMNI_ARBOR, P.OMNI_PIVOT
        px, pz = P.OMNI['pivot']
        self.assertGreaterEqual(S['id'] / 2 - B['r'], 0.04)                                   # the free coil clears the arbor by 0.4 mm
        self.assertAlmostEqual(2 * B['r_in'], 0.52, places=9)                                 # drawn 5.2 mm for the 5.00 mm tube: a printed hole comes out 0.1 to 0.2 mm small
        self.assertGreaterEqual(B['r_in'] - V['tube_r'][1], 0.005)                            # the sleeve slides on the 5 mm tube
        self.assertLessEqual(B['r_in'] - V['tube_r'][1], 0.011)                               # without rattling (0.1 mm of play all round at most)
        self.assertAlmostEqual(B['y'][0], P.OMNI_MOUNT['pillar']['y'][1], places=9)           # it stands on the pillar's face
        self.assertGreaterEqual(P.OMNI['arm_y'][0] - B['y'][1], 0.02)                         # and stops short of the arm
        self.assertLessEqual(B['y'][0], S['y'][0])                                            # it covers the coil's whole length
        self.assertGreaterEqual(B['y'][1], S['y'][1])
        n_open = S['body_turns'] - 44.2 / 360.0                                               # turns left at full travel (12.7 + 31.5 degrees of opening)
        id_open = S['mean_d'] * S['body_turns'] / n_open - S['wire']                          # the wire keeps its length: the mean diameter grows by body_turns / n_open
        ecc = id_open / 2 - B['r']                                                            # how far the loaded coil sits off-centre (pushed up by its supports), cm
        self.assertLessEqual(ecc, 0.07)
        top = pz + ecc + id_open / 2 + S['wire']
        self.assertLessEqual(top, P.TRAY['z'][0] - 0.03)                                      # under the battery tray's height even at full travel and off-centre (the tray does not reach this far in y)
        ecc0 = S['id'] / 2 - B['r']                                                           # unloaded the coil hangs on the arbor instead: its underside stays above the belly line
        self.assertGreaterEqual((pz - ecc0 - S['od'] / 2) - P.Z_BELLY, 0.05)

    def test_the_spring_is_wound_for_the_legs_the_arm_and_the_adjuster_leave_it(self):
        """Both legs leave the top of the coil: the rear one points back (180 degrees), the forward one lies along the arm, 4 degrees up from it (-11.1 degrees at rest). Seen from the right side, with the winding running from the
        forward leg's root to the rear leg's, counterclockwise (a right-hand coil), the sweep between the roots is -(phi + tilt) (+ whole turns). The coil is pre-opened by the preload angle, so the free coil has
        body_turns = whole turns + (-(phi + tilt) + preload) / 360; with the rounding of body_turns to 0.01 turn the legs sit within 2 degrees of where the supports hold them. At full travel the arm has turned
        31.5 degrees and the coil is opened by that much more: the load OPENS this coil (the supports are under the legs, so it cannot be wound the other way)."""
        import omni_mount
        s = omni_mount.spring(*omni_mount.MODEL, quiet=True)
        ox, oz = P.OMNI['rest']
        px, pz = P.OMNI['pivot']
        phi = math.degrees(math.atan2(oz - pz, ox - px))
        want = (-(phi + P.OMNI_SPRING['tilt']) + s['pre_deg_chosen']) % 360.0
        have = (P.OMNI_SPRING['body_turns'] * 360.0) % 360.0
        self.assertLess(abs(want - have), 2.0)
        self.assertAlmostEqual(P.OMNI_SPRING['active_turns'], P.OMNI_SPRING['body_turns'] + sum(P.OMNI_SPRING['legs']) / (3 * math.pi * P.OMNI_SPRING['mean_d']), places=9)

    def test_the_forward_leg_and_its_pin_carry_the_spring_torque_with_room_to_spare(self):
        """The forward leg is 11 mm long to its pin (the lever), so the 374 N.mm at full travel is 34 N on a 2 mm steel pin 1.5 mm from its root, which is bending of about 60 MPa, and the pin's bearing on the
        arm's bar is about 7 MPa."""
        import omni_mount
        s = omni_mount.spring(*omni_mount.MODEL, quiet=True)
        K = P.OMNI_SEAT
        lever = K['s'] * 10.0                                                                 # mm
        force = s['m1'] / lever                                                               # N at full travel
        self.assertLess(force, 40.0)
        arm_face = P.OMNI['arm_y'][0]
        stick = (arm_face - P.OMNI_SPRING['y'][1] + P.OMNI_SPRING['wire'] / 2) * 10.0         # mm from the arm's face to the leg's centre line, the leg's y being the last turn's
        d = 2 * K['r'] * 10.0
        sigma = 32 * force * max(stick, 1.0) / (math.pi * d ** 3)
        self.assertLess(sigma, 100.0)                                                         # a stainless 2 mm pin yields at about 500 MPa
        self.assertLess(force / (d * (K['y'][1] - arm_face) * 10.0), 15.0)                    # bearing pressure on the bore over the pin's length in the bar (aluminium yields at 240)

    def test_axle_screw_ends_inside_the_arm_and_its_head_sits_in_the_wheel_s_bore(self):
        """An M4 nut outboard of the arm would sweep through the floor wall (the axle rises through the floor's thickness as the wheel compresses) and toward the floor port 4.5 mm further out.
        A head standing 4 mm out of the wheel's inboard face (first draft) swept through the floor beside the bay and would have needed the bay to cut into the GIGA post H1's footprint:
        the head and its washer sit in the bore, 5 mm behind the face."""
        A = P.OMNI_AXLE
        w0 = P.OMNI['y_wheel'][0]
        y_under = w0 + A['seat'] - A['washer_h']                                             # underside of the head: on a 0.5 mm washer on the inboard bearing's inner ring
        tip = y_under + A['screw_len']
        self.assertLessEqual(tip, P.OMNI['arm_y'][1])                                        # the tip stops inside the arm
        self.assertGreaterEqual(tip - P.OMNI['arm_y'][0], 0.25)                              # with 2.5 mm of thread in the bar
        self.assertGreaterEqual((y_under - A['head_h']) - w0, 0.02)                          # the head's outer face is behind the wheel's face
        self.assertLessEqual(A['head_r'], P.OMNI_WHEEL['bore_r'] - 0.1)                      # the 7 mm head fits the 12 mm hole with 2.5 mm to spare each side
        post_edge = P.giga_hole_xy(0)[1] + P.GIGA_POST['r']                                  # the +y edge of the GIGA post H1
        self.assertGreaterEqual(P.OMNI_BAY_Y[0] - post_edge, 0.1)                            # the floor opening stays 1.4 mm clear of the post's footprint
        self.assertGreaterEqual(w0 - P.OMNI_BAY_Y[0], 0.1)                                   # and 1.5 mm off the wheel's face

    def test_the_floor_opening_is_two_plan_cuts_that_hold_the_wheel_and_the_arm_over_the_whole_travel_and_stay_off_the_wall(self):
        """The stadium cuts of 8 Oct, extruded along y over the whole height, nicked the front wall and the +12 degree frame boss. Now: the wheel's opening (y -1.13 to 1.73, its rim at the floor level
        reaches x 10.003 at 41 percent of the travel) and the arm's corridor (y 0.75 to 2.10, x to 8.0), both cut through the floor only. The wall's inner face is x 10.08 at y 2.10, so the
        corridor must not run forward: that is why the opening is not one rectangle to y 2.10."""
        Op = P.OMNI_OPENING
        r = P.OMNI['r']
        wo, ar = Op['wheel'], Op['arm']
        for k in range(101):
            xc, zc = P.omni_at(P.OMNI['travel'] * k / 100)
            for z in (P.Z_BELLY, P.Z_FLOOR_TOP):
                if abs(z - zc) < r:
                    d = math.sqrt(r * r - (z - zc) ** 2)
                    self.assertGreaterEqual((xc - d) - wo['x'][0], 0.05, 'rear, travel %d' % k)
                    self.assertGreaterEqual(wo['x'][1] - (xc + d), 0.05, 'front, travel %d' % k)
            self.assertGreaterEqual(ar['x'][1] - (xc + P.OMNI['arm_half']), 0.3, 'the arm\'s round end, travel %d' % k)
        self.assertLessEqual(math.hypot(wo['x'][1], wo['y'][1]), P.R_INT - 0.05)             # the wheel opening's front outer corner stays off the wall's inner face
        self.assertLessEqual(math.hypot(ar['x'][1], ar['y'][1]), P.R_INT - 1.0)
        self.assertGreaterEqual(wo['y'][0], P.OMNI['y_wheel'][0] - 0.2)                      # 1 to 2 mm of play each side of the wheel, no more
        self.assertGreaterEqual(wo['y'][1] - P.OMNI['y_wheel'][1], 0.1)
        self.assertEqual((wo['y'][0], ar['y'][1]), P.OMNI_BAY_Y)                             # together they span y -1.13 to 2.10
        fp = P.FLOOR_FRONT
        self.assertGreaterEqual(fp['y'] - (fp['w'] + 0.1) / 2 - ar['y'][1], 0.3)             # a 3 mm floor wall between the corridor and the floor port's hole

    def test_the_pivot_and_stop_screw_heads_clear_the_floor_so_that_they_can_be_pulled_out_inboard(self):
        """The dry run (9 Oct) pulled the pivot screw out along -y in the teardown path: its 7 mm M4 head (axis z 4.2) reaches z 3.85 and scraped the floor (top z 3.9) along the whole 3.6 cm. A relief under
        the head would have had to be a trench as long as the pull; M3 screws (5.5 mm heads) clear the floor by 0.25 mm and need nothing."""
        V, S = P.OMNI_PIVOT, P.OMNI_MOUNT['stop']
        sx, sz = P.omni_stop_pin()
        self.assertGreaterEqual((P.OMNI['pivot'][1] - V['head_r']) - P.Z_FLOOR_TOP, 0.02)
        self.assertGreaterEqual((sz - S['head_r']) - P.Z_FLOOR_TOP, 0.02)
        self.assertEqual(V['screw_r'], S['pin_r'])                                           # one screw size for both: M3 x 25

    def test_the_ear_holds_an_m3_insert_and_the_pivot_screw_reaches_it(self):
        M, V = P.OMNI_MOUNT, P.OMNI_PIVOT
        ear = M['ear']['y']
        self.assertGreaterEqual(ear[1] - ear[0], V['hole_depth'] + 0.1)                      # the blind hole and a skin behind it
        tip = M['pillar']['y'][0] + V['screw_len']                                           # the screw's head sits on the pillar's inboard face
        self.assertGreaterEqual(V['insert_len'], 0.5)                                        # an M3 x 5.7 mm insert: 5.7 mm of thread (1.9 diameters)
        self.assertGreaterEqual(tip - ear[0], V['insert_len'])                               # the screw is through the whole insert
        self.assertLessEqual(tip, ear[0] + V['hole_depth'])                                  # and its tip stops in the blind hole beyond it
        self.assertAlmostEqual(V['tube_len'], M['pocket']['y'][1] - M['pillar']['y'][1], places=6)    # the tube is clamped between the pillar and the ear

    def test_spring_gives_the_rate_the_ramp_needs_and_stays_under_half_the_tensile_strength(self):
        import omni_mount
        s = omni_mount.spring(*omni_mount.MODEL, quiet=True)
        S = P.OMNI_SPRING
        rate = S['E'] * (S['wire'] * 10) ** 4 / (64.1 * S['mean_d'] * 10 * S['active_turns'])             # N.mm/rad
        self.assertLess(abs(rate / s['k_theta'] - 1.0), 0.05)
        self.assertLessEqual(s['sigma'], 0.5 * S['uts'])
        self.assertAlmostEqual(S['od'], S['mean_d'] + S['wire'], places=6)
        gap = P.OMNI['arm_y'][0] - P.OMNI_MOUNT['pillar']['y'][1]
        self.assertGreaterEqual(gap - S['length'], 0.05)                                     # the coil fits between the pillar and the arm with 0.5 mm to spare

    def test_a_pet_g_arm_is_good_for_a_prototype_only(self):
        """Torsion from the wheel's 15 mm offset: an aluminium 3 x 12 mm bar twists the wheel 0.15 degrees at the full-travel load, a printed 4 x 12 mm PETG bar 2.2 degrees."""
        import omni_mount
        s = omni_mount.spring(*omni_mount.MODEL, quiet=True)
        lines = omni_mount.strength_numbers(s)
        self.assertLess(lines['aluminium 3 x 12']['twist_deg'], 0.3)
        self.assertGreater(lines['PETG 4 x 12']['twist_deg'], 1.0)


class OmniArmDrawing(unittest.TestCase):
    """The arm's cut file and drawing (omni_arm.py, Figure 13) say what the model says: a machinist who cuts the DXF gets the arm the Fusion model has."""

    @classmethod
    def setUpClass(cls):
        import tempfile
        import omni_arm
        cls.omni_arm = omni_arm
        cls.dir = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.dir.name, 'arm.dxf')
        omni_arm.write_dxf(cls.path)
        cls.ents = omni_arm.read_dxf(cls.path)

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    def test_the_file_round_trips_and_holds_only_lines_arcs_and_circles(self):
        want = self.omni_arm.dxf_entities()
        self.assertEqual(len(self.ents), len(want))
        for got, ref in zip(self.ents, want):
            self.assertEqual(got[0], ref[0])
            for a, b in zip(got[1:], ref[1:]):
                self.assertAlmostEqual(a, b, places=3)
        self.assertEqual(sorted(set(e[0] for e in self.ents)), ['ARC', 'CIRCLE', 'LINE'])
        with open(self.path) as fh:
            text = fh.read()
        self.assertIn('AC1009', text)                                                         # R12: every CAD program and cutting service reads it
        self.assertTrue(text.rstrip().endswith('EOF'))

    def test_the_outline_is_the_12_by_3_bar_with_the_arm_length_of_the_model(self):
        lines = [e for e in self.ents if e[0] == 'LINE']
        arcs = [e for e in self.ents if e[0] == 'ARC' and abs(e[3] - P.OMNI['arm_half'] * 10) < 1e-6]
        self.assertEqual(len(lines), 2)
        for e in lines:
            self.assertAlmostEqual(abs(e[4] - e[2]) + abs(e[3] - e[1]), 46.0, places=6)         # 46 mm straight, so 58 mm over the round ends
            self.assertAlmostEqual(abs(e[2]), 6.0, places=6)
        self.assertEqual(len(arcs), 2)
        self.assertAlmostEqual(self.omni_arm.features()['length'], 58.0, places=6)
        self.assertAlmostEqual(self.omni_arm.features()['thick'], P.OMNI['arm_t'] * 10, places=6)
        self.assertAlmostEqual(P.OMNI['arm_t'], 0.3, places=9)                                 # 3 mm bar
        self.assertAlmostEqual(2 * P.OMNI['arm_half'], 1.2, places=9)                          # 12 mm bar

    def test_the_outline_and_the_slot_are_closed_loops_of_joined_lines_and_arcs(self):
        """A cutting service needs closed contours: every end of every line and arc of the outline and of the slot must meet exactly one other end."""
        ends = {}

        def add(x, y):
            key = (round(x, 3), round(y, 3))
            ends[key] = ends.get(key, 0) + 1
        for e in self.ents:
            if e[0] == 'LINE':
                add(e[1], e[2])
                add(e[3], e[4])
            elif e[0] == 'ARC':
                cx, cy, r, a0, a1 = e[1:]
                add(cx + r * math.cos(math.radians(a0)), cy + r * math.sin(math.radians(a0)))
                add(cx + r * math.cos(math.radians(a1)), cy + r * math.sin(math.radians(a1)))
        self.assertTrue(ends)
        self.assertEqual(sorted(set(ends.values())), [2], {k: v for k, v in ends.items() if v != 2})
        self.assertEqual(len(ends), 8)                                                         # 4 corners of the outline's straights and arcs, 4 where the slot's arcs meet its round ends

    def test_the_slot_ends_are_half_circles_that_bulge_away_from_the_slot(self):
        """The end arcs share their end points with the long arcs whichever side they are drawn on, so the closed-loop test cannot see a cap drawn the wrong way round (a notch instead of a round end):
        the middle of each cap must lie beyond the last centre of the slot, along the arc."""
        f = self.omni_arm.features()
        caps = [e for e in self.ents if e[0] == 'ARC' and abs(e[3] - f['slot_hw']) < 1e-6]
        self.assertEqual(len(caps), 2)
        seen = {}
        for cx, cy, r, a0, a1 in (c[1:] for c in caps):
            span = (a1 - a0) % 360.0
            self.assertAlmostEqual(span, 180.0, places=3)                                      # a half circle
            m = math.radians(a0 + span / 2.0)
            mid = math.degrees(math.atan2(cy + r * math.sin(m), cx + r * math.cos(m)))         # where the cap's middle is, as an angle about the pivot
            centre = math.degrees(math.atan2(cy, cx))
            seen['rest' if centre > 0 else 'compression'] = (mid, centre)
        self.assertGreater(seen['rest'][0], seen['rest'][1])                                   # the rest end bulges toward larger angles
        self.assertLess(seen['compression'][0], seen['compression'][1])                        # the compression end toward smaller ones

    def test_the_holes_are_where_the_model_has_them(self):
        circles = sorted((e[1], e[2], 2 * e[3]) for e in self.ents if e[0] == 'CIRCLE')
        kx, kz = P.omni_arm_pt(P.OMNI_SEAT['s'], P.omni_seat_n())                                # the seat pin, back in the arm's own frame
        px, pz = P.OMNI['pivot']
        phi = math.atan2(P.OMNI['rest'][1] - pz, P.OMNI['rest'][0] - px)
        sx = (kx - px) * math.cos(phi) + (kz - pz) * math.sin(phi)
        sy = -(kx - px) * math.sin(phi) + (kz - pz) * math.cos(phi)
        want = sorted([(0.0, 0.0, 2 * P.OMNI['pivot_hole_r'] * 10), (P.OMNI['arm_len'] * 10, 0.0, 3.3), (sx * 10, sy * 10, 2 * P.OMNI_SEAT['hole_r'] * 10)])
        for got, ref in zip(circles, want):
            for a, b in zip(got, ref):
                self.assertAlmostEqual(a, b, places=3)
        self.assertGreaterEqual(2 * P.OMNI['pivot_hole_r'] * 10 - 2 * P.OMNI_PIVOT['tube_r'][1] * 10, 0.04)    # reamed 0.05 over the tube, which turns in it
        self.assertAlmostEqual(2 * P.OMNI['axle_r'] * 10, 4.0, places=6)                        # the M4 axle screw's thread, tapped with a 3.3 drill

    def test_the_slot_in_the_file_holds_the_stop_screw_over_the_whole_travel_with_0_2_mm_play_at_both_ends(self):
        r_out = max(e[3] for e in self.ents if e[0] == 'ARC' and e[3] > 6.5)
        r_in = min(e[3] for e in self.ents if e[0] == 'ARC' and 6.1 < e[3] < 6.5)
        a_lo, a_hi = [(e[4], e[5]) for e in self.ents if e[0] == 'ARC' and abs(e[3] - r_out) < 1e-6][0]
        hw, r = (r_out - r_in) / 2, (r_out + r_in) / 2
        centre = LineString([(r * math.cos(math.radians(a_lo + (a_hi - a_lo) * k / 60)), r * math.sin(math.radians(a_lo + (a_hi - a_lo) * k / 60))) for k in range(61)])
        shape = centre.buffer(hw, cap_style=1, quad_segs=32)
        px, pz = P.OMNI['pivot']
        sx, sz = P.omni_stop_pin()
        screw_r = P.OMNI_MOUNT['stop']['pin_r'] * 10
        inside = []
        for k in range(101):
            t = P.OMNI['travel'] * k / 100
            xc, zc = P.omni_at(t)
            phi = math.atan2(zc - pz, xc - px)
            sa = (sx - px) * math.cos(phi) + (sz - pz) * math.sin(phi)                              # the screw in the arm's frame at this travel
            sn = -(sx - px) * math.sin(phi) + (sz - pz) * math.cos(phi)
            self.assertTrue(shape.contains(Point(sa * 10, sn * 10).buffer(screw_r - 1e-6)), 'screw leaves the slot at travel %.2f' % t)
            inside.append((sa * 10, sn * 10))
        th_rest = math.degrees(math.atan2(inside[0][1], inside[0][0]))                              # where the screw sits on the arc at the two stops
        th_full = math.degrees(math.atan2(inside[-1][1], inside[-1][0]))
        gap_rest = math.radians(a_hi - th_rest) * r + hw - screw_r                                  # the free path along the arc from the screw's surface to the slot's end
        gap_full = math.radians(th_full - a_lo) * r + hw - screw_r
        self.assertAlmostEqual(gap_rest, 0.2, delta=0.01)                                           # 0.2 mm at the rest stop (file this end to the measured ride height) ...
        self.assertAlmostEqual(gap_full, 0.2, delta=0.01)                                           # ... and at the compression stop

    def test_the_drawing_quotes_the_ride_height_effect_of_a_slot_end(self):
        """Figure 13 and the spec say that a slot end filed 0.2 mm moves the wheel 1.1 mm: the arm's horizontal reach over the stop radius is 5.5 times."""
        self.assertAlmostEqual(0.2 * self.omni_arm.ride_gain(), 1.1, delta=0.03)

    def test_the_slot_and_the_seat_hole_leave_the_bar_whole(self):
        f = self.omni_arm.features()
        self.assertGreaterEqual(f['width'] / 2 - (f['slot_r'] * math.sin(math.radians(f['slot_a_hi'])) + f['slot_hw']), 1.0)                  # 2 mm of bar above and below the slot
        self.assertGreaterEqual(f['axle_x'] - (f['slot_r'] + f['slot_hw']) - f['axle_tap'] / 2, 20.0)       # the long free bar between the slot and the axle end
        self.assertGreaterEqual(self.omni_arm.mass_g()[0], 4.0)
        self.assertLessEqual(sum(self.omni_arm.mass_g()), P.BOUGHT_G['Omni arm'])                      # the model's 6 g budget covers the arm and its pin


class Chute(unittest.TestCase):
    def test_square_channel_lowest_point(self):
        p0, e = P.chute_ends(1)
        down = P.CHUTE['out_w'] / 2 - P.CHUTE['lift']        # the outer floor is this far below the axis; the section is out_w wide
        low = chute_exit.tube_min_z(p0, e, 'square', down, half_lat=P.CHUTE['out_w'] / 2)
        self.assertAlmostEqual(low, 2.77, delta=0.02)        # the 19 mm channel's outer floor is where the 18 mm one's was (2.78); both hang 2 mm lower at the wall than the 13 mm one did (3.00)
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
        diagonal (14.57 mm). A 17 mm slot, hopper void and bore take the cube lying at any turn (face diagonal 14.57 mm, with 2.43 mm to spare) and, since 9 Oct, tipped any way: standing
        on a corner it needs 16.25 mm (cube_slot.py; the 16 mm slot of 8 Oct left out the 0.9 % of attitudes within 4.5 degrees of it)."""
        diag = 1.03 * math.sqrt(2)
        for name, width in (('slot', P.PLATE['slot']), ('hopper void', P.CHUTE['hopper_in']), ('bore', P.CHUTE['in_w'])):
            self.assertGreaterEqual(width, diag + 0.12, name)
            self.assertGreaterEqual(width, 1.625 + 0.05, name)                           # a corner stand needs 16.25 mm: any attitude passes
        self.assertGreaterEqual(P.CHUTE['in_w'], P.CHUTE['hopper_in'])                  # no wall end catches a kit between the hopper and the channel
        self.assertGreaterEqual(P.CHUTE['hopper_in'], P.PLATE['slot'])
        self.assertAlmostEqual(P.CHUTE['out_w'], P.CHUTE['in_w'] + 2 * P.CHUTE['wall'], places=6)
        self.assertGreaterEqual(P.CHUTE['hopper_out'] - P.CHUTE['hopper_in'], 0.4)      # hopper walls at least 2 mm
        self.assertGreaterEqual(P.CHUTE['flange'] - P.CHUTE['hopper_out'], 0.3)

    def test_bore_is_wider_and_taller_than_a_tumbling_cube(self):
        """With bouncy impacts the chute simulation wedged tumbling kits between the two side walls and between the floor and the ceiling of a 16 mm bore: a cube in a general attitude is
        up to its space diagonal, 17.84 mm, wide. The 19 mm bore (9 Oct) clears it by 1.16 mm. The square section is centred `lift` (1.5 mm) above the axis, so the floor is 8 mm below the axis and the
        ceiling 11 mm above it: the floor, which sets the lowest point at the wall, stays where the 16 and 18 mm designs had it. The 18 mm bore of 8 Oct had only 0.16 mm at the nominal 10.3 mm
        kit; a 10.5 mm kit (the rev 3 tolerance: space diagonal 18.19 mm) or a bore printed 0.4 mm small wedged some bouncing kits (94 to 97 % got out): the 19 mm bore costs 0.5 mm of the
        chute-to-wheel gap (spec 6.2, D21)."""
        self.assertGreaterEqual(P.CHUTE['in_w'], 1.03 * math.sqrt(3) + 0.005)
        self.assertAlmostEqual(P.CHUTE['in_w'] / 2 - P.CHUTE['lift'], 0.8, places=6)

    def test_bore_margin_covers_the_kit_and_print_tolerance(self):
        """Reliability pick (D21, 9 Oct): the 18 mm bore had 0.16 mm over the space diagonal of the nominal 10.3 mm kit and was 0.19 mm short of a 10.5 mm kit's (18.19 mm), so a kit that is a
        little big or a bore printed a little small wedged 2 to 5 % of the bouncing kits. The bore must clear the space diagonal of a 10.5 mm kit by 0.5 mm (a bore printed 0.4 mm small
        still has 0.1 mm), which is 19 mm; the section stays centred so that its floor is 8 mm under the axis (test above)."""
        self.assertGreaterEqual(P.CHUTE['in_w'] - 1.05 * math.sqrt(3), 0.05)

    def test_parking_allowance_is_what_the_slot_leaves_over_the_pocket(self):
        """A kit lying corner to corner in its pocket is as wide along the ring as the pocket, so the plate may be parked off by (slot - pocket) / 2 before the kit's corner overhangs the slot's
        edge; the simulation with the pocket walls in (chute_dynamics.py) takes every kit out up to that allowance and loses kits beyond it. 8 Oct: 14 mm pocket, 16 mm slot, 1.0 mm
        allowance. Reliability pick (D22, 9 Oct): 13 mm pocket (a 10.5 mm kit still has 1.25 mm each side), 17 mm slot, 2.0 mm allowance, about 3 degrees of the plate at the ring. This
        number is the dropper's parking requirement (spec 6.2); change the slot or the pocket and the spec's figure has to be re-derived."""
        self.assertAlmostEqual((P.PLATE['slot'] - P.PLATE['pocket']) / 2, 0.2, places=6)
        self.assertGreaterEqual(P.PLATE['pocket'] - 1.05, 0.2)                        # a 10.5 mm kit goes into the pocket with 1 mm to spare each side

    def test_drop_sequences_with_the_bigger_slot(self):
        """Parked, no pocket that holds a kit may be within `park_margin_mm` of a slot (else a parking error drops a second kit), and the margin should be about as big as the parking allowance
        (else widening the slot only trades one failure for the other): 8 Oct design 1.74 mm margin for 1.0 mm allowance, the 9 Oct design 1.92 mm for 2.0 mm."""
        import platesim
        import platesim2
        pl = platesim.Plate(r=38.6, pocket=P.PLATE['pocket'] * 10.0, slot=P.PLATE['slot'] * 10.0)
        res, viol = platesim2.check(pl)
        self.assertEqual(res['violations'], 0, viol[:3])
        self.assertGreaterEqual(res['park_margin_mm'], 1.8)
        self.assertGreaterEqual(res['wall_mm'], 3.5)                                   # the web between two pockets, printed 1.2 mm thick

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
        """Cutting the channel's own section (22.2 mm wide, 21.2 on 8 Oct, topped at its ceiling plane) out of the hopper left the hopper's downhill corner, above the channel, hanging free: Fusion split
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
    def test_tof_positions_are_mirror_images_and_keep_the_firmware_offsets(self):
        """The left and right sensors mirror each other; the numbers the firmware needs stay as in the spec: F 9.5 cm ahead, the side sensors 6.0 cm out (TOF_SIDE_OUT_MM 60), the rear ones 8.4 cm behind. Moved on
        9 Oct for the real board: the front diagonal ones 1 mm and 1 degree, the side ones from x +-7.18 to +-6.85, the rear ones from y +-4.5 to +-3.6."""
        pos = {n: (x, y, a) for n, x, y, a in P.TOF}
        for left, right in (('FL', 'FR'), ('SFL', 'SFR'), ('SRL', 'SRR'), ('RL', 'RR')):
            self.assertAlmostEqual(pos[left][0], pos[right][0], places=9)
            self.assertAlmostEqual(pos[left][1], -pos[right][1], places=9)
            self.assertEqual((pos[left][2] + pos[right][2]) % 360, 0)                  # +25 and -25, +90 and -90, 180 and 180
        self.assertEqual(pos['F'], (9.5, 0.0, 0))
        for n in ('SFL', 'SRL'):
            self.assertAlmostEqual(pos[n][1], 6.0, places=9)
        self.assertAlmostEqual(pos['RL'][0], -8.4, places=9)

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
        wheel = box(P.OMNI['rest'][0] - P.OMNI['r'], P.OMNI['y_wheel'][0], P.OMNI['rest'][0] + P.OMNI['r'], P.OMNI['y_wheel'][1])
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

    def test_wifi_antenna_is_stuck_on_the_bumper_recess_wall_clear_of_the_switch_the_socket_and_the_boss(self):
        """9 Oct: at -17.5 degrees and z 5.5 to 7.04 the strip lay inside the right bumper's recess wall (z 3.9 to 6.6, angles -62 to -8 degrees, inner face r 10.15): 0.036 cm3 of overlap with the tub
        in the dry run. It stands on that wall now, inside the band, 8 degrees clear of the bumper switch and 5.6 mm under the USB-C socket."""
        A, Bp = P.ANTENNA, P.BUMPER
        r_wall = Bp['r_out'] - Bp['t'] - 0.4 - 0.2                                           # the recess wall's inner face
        self.assertLessEqual(A['r_face'], r_wall)
        self.assertGreaterEqual(A['z'][0], Bp['z0'])
        self.assertLessEqual(A['z'][1], Bp['z1'] + 0.09)                                     # the recess wall reaches z 6.6
        half = math.degrees(A['w'] / 2 / A['r_face'])
        self.assertGreaterEqual(A['angle'] - half, -(Bp['a1'] + 4) + 1.0)                    # inside the recess wall's angles with 1 degree to spare
        self.assertLessEqual(A['angle'] + half, -(Bp['a0'] - 4) - 1.0)
        sw = -P.BUMPER_SW_DEG
        self.assertGreaterEqual(abs(A['angle'] - sw) - half - math.degrees(0.275 / 10.2), 3.0)    # 3 degrees (5 mm) from the switch pocket's edge
        self.assertGreaterEqual(P.USB['z'][0] - A['z'][1], 0.5)                              # the socket's metal shell is 5 mm above the strip
        S = P.SCREW
        self.assertGreaterEqual(S['boss_z0'] - A['z'][1], 0.5)                               # below the frame boss at z 7.7
        self.assertGreaterEqual(A['z'][0] - P.Z_FLOOR_TOP, 1.0)                              # well above the floor and the floor modules
        cx, cy, ang = P.antenna_pose()
        a = math.radians(ang)
        ux, uy = math.cos(a), math.sin(a)
        vx, vy = -uy, ux
        corners = [(cx + sx * A['t'] / 2 * ux + sy * A['w'] / 2 * vx, cy + sx * A['t'] / 2 * uy + sy * A['w'] / 2 * vy) for sx in (1, -1) for sy in (1, -1)]
        self.assertLess(max(math.hypot(x, y) for x, y in corners), P.R_INT)
        bx, by = P.polar(S['r'], -12.0)
        rx, ry = P.polar((S['r'] + S['rib_to']) / 2, -12.0)
        half_l = (S['rib_to'] - S['r']) / 2
        boss = Point(bx, by).buffer(S['boss_r']).union(affinity.rotate(box(rx - half_l, ry - S['rib_w'] / 2, rx + half_l, ry + S['rib_w'] / 2), -12.0, origin=(rx, ry)))
        self.assertGreaterEqual(Polygon(corners).distance(boss), 0.3)                        # 3 mm from the boss rib at -12 degrees in plan (the rib starts at z 7.7)
        w0, w1 = P.OMNI['y_wheel']
        nearest = 1e9
        for k in range(0, 101):                                                              # the aluminium wheel over its whole travel: a metal body within a few mm detunes a 2.4 GHz antenna, so keep 15 mm
            wx, wz = P.omni_at(P.OMNI['travel'] * k / 100)
            for x, y in corners:
                for z in A['z']:
                    rho = math.hypot(x - wx, z - wz)
                    nearest = min(nearest, math.hypot(max(rho - P.OMNI['r'], 0.0), max(w0 - y, 0.0, y - w1)))
        self.assertGreaterEqual(nearest, 1.5, 'antenna to the aluminium omni wheel')
        jx, jy = P.giga_j14()
        run = math.sqrt((cx - jx) ** 2 + (cy - jy) ** 2 + ((A['z'][0] + A['z'][1]) / 2 - P.J14['z']) ** 2)
        self.assertLessEqual(run * 1.3 + 3.0, A['cable'])                                    # routed run (x1.3 + 3 cm, as for the ToF cables) fits the 100 mm cable


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

    def test_the_omni_module_is_weighed_as_bought_parts(self):
        """The 73 g aluminium wheel (budget before: 30 g), a 3 mm aluminium arm (it was a printed plate), the two M4 screws, bearings and spring."""
        for nm in ('Omni wheel', 'Omni arm', 'Omni pins', 'Omni pivot', 'Omni spring', 'Omni adjuster'):
            self.assertIn(nm, P.BOUGHT_G, nm)
        self.assertNotIn('Omni arm', P.PRINTED)
        self.assertGreaterEqual(P.BOUGHT_G['Omni wheel'], P.OMNI_WHEEL['mass_g'])



class TofBoardsAndMounts(unittest.TestCase):
    """The nine ToF pockets for the real Adafruit board (spec 8.3): numbers from tof_mount.py and v3_params4.TOF_BOARD / TOF_MOUNT."""

    def test_board_is_the_adafruit_qt_board(self):
        B = P.TOF_BOARD
        self.assertAlmostEqual(B['long'], 2.54, places=6)               # 25.4 mm
        self.assertAlmostEqual(B['short'], 1.778, places=6)             # 17.78 mm
        self.assertAlmostEqual(B['hole_long'] * 2, 2.032, places=6)    # holes 20.32 mm apart along the board ...
        self.assertAlmostEqual(B['hole_short'] * 2, 1.27, places=6)    # ... and 12.7 mm across it
        self.assertAlmostEqual(B['long'] / 2 - B['hole_long'], 0.254, places=6)    # 2.54 mm from the end
        self.assertLess(B['conn_end'] + B['conn_len'], B['long'] / 2 - B['chip_w'] / 2)       # a connector (6.1 x 4.25) sits between the end and the chip
        self.assertGreater(B['hole_short'] - B['hole_d'] / 2, B['conn_w'] / 2)               # the holes are beside the connectors, not under them

    def test_every_pocket_leaves_2_5_mm_of_ring_in_front_and_every_block_stays_in_the_body(self):
        M = P.TOF_MOUNT
        for nm in T.sensors():
            self.assertLessEqual(T.ring_corner_radius(nm), P.FRAME['r_out'] - M['wall_min'] + 1e-9, nm)
            self.assertLessEqual(max(math.hypot(x, y) for x, y in T.block_plan(nm)), P.R_BODY + 1e-9, nm)

    def test_pockets_blocks_and_tunnels_keep_3_mm_apart(self):
        pg = {nm: T.polygons(nm) for nm in T.sensors()}
        gap = P.TOF_MOUNT['gap_min']
        for a, b in itertools.combinations(pg, 2):
            self.assertGreaterEqual(unary_union([pg[a]['pocket'], pg[a]['block']]).distance(unary_union([pg[b]['pocket'], pg[b]['block']])), gap - 1e-9, '%s and %s' % (a, b))
            self.assertGreaterEqual(pg[a]['cut'].distance(pg[b]['cut']), gap - 1e-9, '%s and %s (cuts)' % (a, b))

    def test_pockets_keep_3_mm_from_the_camera_windows_and_the_cage_inserts(self):
        G, C = P.CAGE, P.CAMERA
        window = box(P.CAM_X - C['window_w'] / 2, 8.0, P.CAM_X + C['window_w'] / 2, 11.0)
        inserts = [Point(P.CAM_X + e * G['ear_x'], 8.7).buffer(G['insert_d'] / 2 + 0.2) for e in (1, -1)]
        for nm in T.sensors():
            cut = T.polygons(nm)['cut']
            if nm.endswith('R') or nm == 'F':
                continue                                                    # the left half is listed; the right camera and its sensors are mirror images
            self.assertGreaterEqual(cut.distance(window), P.TOF_MOUNT['gap_min'] - 1e-9, nm)
            for ins in inserts:
                self.assertGreaterEqual(cut.distance(ins), P.TOF_MOUNT['gap_min'] - 1e-9, nm)

    def test_board_stands_on_the_floor_and_stays_under_the_ring_top(self):
        M, F = P.TOF_MOUNT, P.FRAME
        bottom, top = T.board_z()
        self.assertAlmostEqual(bottom, M['floor_z'] + 0.01, places=6)                      # the board's bottom edge 0.1 mm over the pocket floor
        self.assertGreaterEqual(M['floor_z'] - F['z0'], 0.3 - 1e-9)                         # a 3 mm floor under the pocket
        self.assertGreaterEqual(F['z1'] - top, 0.15 - 1e-9)                                 # the lid is 1.5 mm over the board's top edge
        self.assertGreaterEqual(F['z1'] - (P.TOF_Z + M['tunnel_w']), 0.5)                   # and the ring keeps 5 mm over the beam tunnel
        self.assertGreaterEqual((P.TOF_Z - M['tunnel_w']) - M['floor_z'], 0.3)

    def test_posts_clear_the_connectors_and_the_screws_line_up_with_the_access_holes(self):
        B, M = P.TOF_BOARD, P.TOF_MOUNT
        self.assertGreaterEqual(B['hole_short'] - M['post_r'] - B['conn_w'] / 2, 0.05)       # beside the upper connector (it stands 3 mm out of the board, between the posts)
        self.assertGreaterEqual(M['rear_gap'] - B['screw_head'][1], 0.05)                    # the screw heads fit between the PCB and the rear wall
        self.assertGreaterEqual(M['access_r'] * 2, 0.4)                                      # a 2 mm hex key or a PH0 driver gets through the access hole
        for nm in T.sensors():
            for p, a in zip(T.post_plans(nm), T.access_plans(nm)):
                cp = (sum(q[0] for q in p) / 4, sum(q[1] for q in p) / 4)
                ca = (sum(q[0] for q in a) / 4, sum(q[1] for q in a) / 4)
                x, y, aim = T.sensors()[nm]
                n = (-math.sin(math.radians(aim)), math.cos(math.radians(aim)))
                self.assertAlmostEqual((cp[0] - ca[0]) * n[0] + (cp[1] - ca[1]) * n[1], 0.0, places=9, msg=nm)     # on the same line along the beam

    def test_beam_tunnel_holds_the_whole_cone(self):
        """The VL53L0X's 25 degree cone leaves the chip's 4.4 x 2.4 mm window; at the ring's outer face (r 10.5, which the oblique side sensors reach after 2.5 cm) it must still lie inside the 1.8 x 1.8 cm tunnel."""
        B, M = P.TOF_BOARD, P.TOF_MOUNT
        half = math.tan(math.radians(12.5))
        worst = 0.0
        for nm, (x, y, aim) in T.sensors().items():
            ox, oy, oz = T.beam_origin(nm)
            a = math.radians(aim)
            ux, uy = math.cos(a), math.sin(a)
            b = ox * ux + oy * uy
            d = -b + math.sqrt(b * b - (ox * ox + oy * oy - P.R_BODY ** 2))                # distance along the aim from the chip to the outer face
            need_v = B['chip_v'] / 2 + d * half
            need_w = B['chip_w'] / 2 + d * half
            worst = max(worst, need_v, need_w)
            self.assertLessEqual(need_v, M['tunnel_v'], '%s: cone half width %.2f at the outer face, tunnel %.2f' % (nm, need_v, M['tunnel_v']))
            self.assertLessEqual(need_w, M['tunnel_w'], nm)
        self.assertGreater(worst, 0.5)                                                       # the check is not vacuous: the longest tunnel needs more than 5 mm of half width

    def test_plug_shafts_drop_inside_the_tub_clear_of_the_bosses(self):
        """The plug of the lower connector and its cable go down through the ring into the tub: inside the wall (r 10.3), 3 mm from the insert bosses and their ribs."""
        S = P.SCREW
        half_l = (S['rib_to'] - S['r']) / 2
        for a in P.screw_angles():
            bx, by = P.polar(S['r'], a)
            rx, ry = P.polar((S['r'] + S['rib_to']) / 2, a)
            boss = Point(bx, by).buffer(S['boss_r']).union(affinity.rotate(box(rx - half_l, ry - S['rib_w'] / 2, rx + half_l, ry + S['rib_w'] / 2), a, origin=(rx, ry)))
            for nm in T.sensors():
                shaft = Polygon(T.shaft_plan(nm))
                self.assertLessEqual(max(math.hypot(x, y) for x, y in T.shaft_plan(nm)), P.R_INT - 0.05, nm)
                self.assertGreaterEqual(shaft.distance(boss), 0.3, '%s shaft against the boss at %s degrees' % (nm, a))


class CameraAndCage(unittest.TestCase):
    """The OpenMV Cam H7 Plus on its cage (spec 8.4). The lens axis, holder size and tip height are ESTIMATES (v3_params4.CAMERA): these tests keep the design inside what a wrong estimate by a few mm would still allow,
    and the model's checks (3D) say the rest."""

    def world(self, s, lx, ly, lz):
        """World point of a local camera point (the frame of v4_model.camera_frame)."""
        t = math.radians(P.CAM['tilt'])
        tip = (P.CAM_X, s * P.CAM['tip_r'] * math.sin(math.radians(P.CAM['psi'])), P.CAM['zl'])
        ez = (0.0, -s * math.cos(t), math.sin(t))
        ey = (0.0, s * math.sin(t), math.cos(t))
        return tuple(tip[i] + (s * lx if i == 0 else 0.0) + ly * ey[i] + lz * ez[i] for i in range(3))

    def test_board_and_holes_are_the_drawing(self):
        C = P.CAMERA
        bw, bl, bt = C['board']
        self.assertAlmostEqual(bw, 3.556, places=3)
        self.assertAlmostEqual(bl, 4.445, places=3)
        for u, v, d in C['holes']:
            self.assertLess(v, 0.9)                                    # all four holes are in the 9 mm next to the camera end
            self.assertTrue(0.2 < u < bw - 0.2)
            self.assertGreater(d, 0.27)
        spans = sorted(u for u, v, d in C['holes'])
        self.assertAlmostEqual(spans[-1] - spans[0], 3.09, delta=0.2)  # about 30 mm between the two rows

    def test_board_clears_the_lid_hump_the_tyre_and_the_battery(self):
        C, G = P.CAMERA, P.CAGE
        bw, bl, bt = C['board']
        top = max(self.world(1, 0.0, bw / 2, C['tip'] + bt + G['screw25'][2])[2], self.world(1, 0.0, G['frame_y'], C['tip'])[2])
        bottom = min(self.world(1, 0.0, -bw / 2, C['tip'])[2], self.world(1, 0.0, -G['frame_y'], C['tip'] - G['frame_t'])[2])
        self.assertGreaterEqual(P.Z_LID - top, 0.2)                    # the hump's pocket ceiling over the board and the screw heads
        self.assertGreaterEqual(self.world(1, 0.0, -bw / 2, C['tip'])[2] - 7.6, 0.8)       # the placeholder battery's top is at 7.6 (the tail points over it); the 3D report measures the cage too
        W = P.WHEEL
        low = []                                                       # every low corner of the holder, the flange, the rails and the frame that lies above the tyre (y 7 to 9, 1.2 cm more when the wheel slides out)
        for lx, ly, lz in ((C['holder'][1] / 2, -C['holder'][0] / 2, C['tip'] - C['holder'][2]), (G['flange_x'], -G['rail_y'], G['flange_z'][1]), (G['rail_x'][1], -G['rail_y'], G['flange_z'][1]),
                           (G['frame_x'], -G['frame_y'], C['tip'] - G['frame_t']), (G['frame_x'], -G['frame_y'], C['tip']), (bw / 2, -bw / 2, C['tip'])):
            for sx in (1, -1):
                x, y, z = self.world(1, sx * lx, ly, lz)
                if W['y0'] - 0.2 <= y <= W['y1'] + 1.2 and abs(x) <= W['r']:
                    low.append(z)
        self.assertTrue(low)
        self.assertGreaterEqual(min(low) - (P.AXLE_Z + W['r']), 0.4)   # 4 mm over the tyre's top (z 8.0) wherever the camera is above it
        tail = P.CAM_X + (bl - C['lens_v'])
        self.assertGreaterEqual(C['pocket_x'][1] - tail, 0.4)          # the lid pocket reaches 4 mm past the tail end
        self.assertGreaterEqual((P.CAM_X - C['holder'][1] / 2) - C['pocket_x'][0], 0.2)           # and 2 mm past the lens holder's overhang and the cage frame
        self.assertGreaterEqual((P.CAM_X - G['frame_x']) - C['pocket_x'][0], 0.15)

    def test_lid_humps_stay_inside_the_body_radius_and_hold_the_board_and_the_cage(self):
        """The hump's outer corner (x 4.9, y 7.9) must not leave the body radius (the 3D envelope report found a corner at r 10.512 when the hump ran to y 9.3), and the pocket under it must hold everything of the
        camera that stands above the lid's underside (z 11.7): the board's upper part, its screw heads and the cage frame's top bar."""
        C = P.CAMERA
        hx0, hx1 = C['pocket_x'][0] - 0.2, C['pocket_x'][1] + 0.2
        for x in (hx0, hx1):
            for y in C['hump_y']:
                self.assertLessEqual(math.hypot(x, y), P.R_BODY - 0.05, (x, y))
        self.assertGreaterEqual(C['pocket_y'][0] - C['hump_y'][0], 0.19)
        self.assertGreaterEqual(C['hump_y'][1] - C['pocket_y'][1], 0.19)
        G = P.CAGE
        bw = C['board'][0]
        for ly, lz in ((bw / 2, C['tip'] + C['board'][2] + G['screw25'][2]), (G['frame_y'], C['tip']), (G['frame_y'], C['tip'] - G['frame_t'])):
            x, y, z = self.world(1, 0.0, ly, lz)
            if z > P.LID['z0'] - 0.05:
                self.assertTrue(C['pocket_y'][0] + 0.1 <= y <= C['pocket_y'][1] - 0.1, 'point at y %.2f, z %.2f stands up into the lid outside the pocket' % (y, z))

    def test_flange_leaves_the_barrel_and_the_cone_free(self):
        C, G = P.CAMERA, P.CAGE
        self.assertGreaterEqual(G['hole_d'] / 2 - C['barrel_d'] / 2, 0.15)               # 1.5 mm round the barrel
        self.assertGreater(G['flange_z'][0], 0.0)                                         # the flange is behind the lens tip: nothing in front of the view
        self.assertGreaterEqual(G['open_x'] - C['holder'][1] / 2, 0.1)                    # the frame's opening has 1 mm round the holder in x ...
        self.assertGreaterEqual(G['open_y'] - C['holder'][0] / 2, 0.2)                    # ... and 2 mm in y
        self.assertGreaterEqual(G['rail_x'][0] - C['holder'][1] / 2, 0.1)                 # the rails run past the holder

    def test_cage_flange_stays_off_the_ring_and_the_ears_land_on_it(self):
        G = P.CAGE
        r_in = P.FRAME['r_in']
        for s in (1, -1):
            for x_off in (-G['flange_x'], G['flange_x']):
                for ly in (-G['rail_y'], G['rail_y']):
                    x, y, z = self.world(s, s * x_off, ly, G['flange_z'][0])
                    self.assertLessEqual(math.hypot(x, y), r_in - 0.1, 'flange corner s=%d x=%.1f y=%.2f' % (s, x_off, ly))
        for e in (1, -1):
            xe = P.CAM_X + e * G['ear_x']
            y_face = math.sqrt(r_in ** 2 - (abs(xe) + G['ear_w'] / 2) ** 2)
            self.assertGreater(y_face, 8.0)                            # the ear box (from y 8.0) has some length
            self.assertLess(y_face, math.sqrt(r_in ** 2 - xe ** 2) + 1e-9)

    def test_insert_holes_keep_3_mm_from_the_window_and_the_screws(self):
        G, C = P.CAGE, P.CAMERA
        S = P.SCREW
        for e in (1, -1):
            xe = P.CAM_X + e * G['ear_x']
            ins = box(xe - G['insert_d'] / 2, 8.4, xe + G['insert_d'] / 2, 9.2)
            self.assertGreaterEqual(ins.distance(box(P.CAM_X - C['window_w'] / 2, 8.0, P.CAM_X + C['window_w'] / 2, 11.0)), 0.3)
            for a in P.screw_angles():
                if a > 0:
                    self.assertGreaterEqual(ins.distance(Point(*P.polar(S['r'], a)).buffer(S['cbore_r'])), 0.3, 'screw at %s' % a)


class ImuAndFloorPort(unittest.TestCase):
    def test_imu_board_sits_on_four_posts_in_the_free_column_behind_the_axle(self):
        """BNO055 breakout (Adafruit 2472: 20 x 27 mm, holes 20 x 12 mm apart): on the centre line 5 cm behind the axle, between the N20 and the start of the rear chamfer, header edge forward."""
        I = P.IMU
        length, width, t = I['size']
        self.assertAlmostEqual(length, 2.7, places=6)
        self.assertAlmostEqual(width, 2.0, places=6)
        self.assertAlmostEqual(2 * I['holes'][0], 2.0, places=6)                       # holes 20 mm apart along the board
        self.assertAlmostEqual(2 * I['holes'][1], 1.2, places=6)                       # and 12 mm across it
        front, rear = I['x'] + length / 2, I['x'] - length / 2
        n20_rear = P.PLATE['cx'] - P.PLATE['n20_w'][1] / 2
        self.assertGreaterEqual(n20_rear - front, 0.5)                                 # 5 mm behind the N20 (its body is 1.2 cm across)
        self.assertGreaterEqual(rear - P.CHAMFER[0][0], 0.1)                          # the board and its posts stand on floor that is still 4 mm thick (the underside rises behind x -6.5)
        self.assertLess(abs(I['y']) + width / 2, P.OMNI_BAY_Y[1])                      # nothing else: it is on the centre line
        self.assertGreaterEqual(I['post_h'], 0.4)                                      # the 1.6 mm board's solder joints and the header's pins clear the floor

    def test_front_floor_port_is_the_real_tcs34725_board(self):
        fp = P.FLOOR_FRONT
        self.assertAlmostEqual(fp['w'], 2.03, places=2)                                # 20.3 mm square (20.44 x 20.28 on the product page)
        y0, y1 = P.OMNI_BAY_Y
        self.assertGreaterEqual(fp['y'] - (fp['w'] + 0.1) / 2 - y1, 0.3)               # 3 mm of floor between its hole and the omni bay


if __name__ == '__main__':
    unittest.main()
