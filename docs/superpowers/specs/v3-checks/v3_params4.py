"""Revision 4 parameters (docs/superpowers/specs/2026-10-07-theseus-v3-mechanical-design.md): the single source for the rev 4 Fusion model and its tests.
cm and degrees; x forward, y left, z up; origin at the midpoint of the drive axle on the floor line (axle z 4.0).
Everything that did not change is re-exported from v3_params (rev 3). No Fusion imports here, so the tests can run anywhere."""
import math

from v3_params import *  # noqa: F401,F403  R_BODY R_INT R_SWEPT Z_BELLY Z_ROOF Z_LID AXLE_Z WHEEL MOTOR NUB CHAMFER BUMPER FLOOR_FRONT SILVER BUMPER_SW_DEG TOF_Z TOF_W TOF_H TOF_T CAM PLATE
from v3_params import PART_SIZES, PLATE, R_BODY, load_pack, slot_xy
PLATE = dict(PLATE, pocket=1.3, slot=1.7)       # rev 4 changes the pocket and the slot (a copy, the rev 3 dict stays as it was). 8 Oct: slot 16 mm instead of 14.5, so that a kit that tips on its way in cannot wedge across its 14.57 mm diagonal.
# 9 Oct, reliability pick (spec 6.2, D22): pocket 13 mm instead of 14 and slot 17 mm: the plate may be parked 2.0 mm off ((slot - pocket) / 2; it was 1.0 mm) before a kit stays behind, and 1.92 mm off before the next
# pocket opens over the slot (platesim: park margin); a 17 mm slot also passes a cube standing on a corner (16.25 mm). 'R' (the plate's outer radius, 4.913) is kept as it was computed for the 14 mm pocket: the rim is 0.06 cm thicker.

Z_TUB_TOP = 8.7             # top of the tub wall = underside of the upper frame
Z_FLOOR_TOP = 3.9           # tub floor z 3.5 to 3.9
FLOOR_T = 0.4
KIT = 1.03                  # kit 10.3 mm
CAM_X = CAM['tip_r'] * math.cos(math.radians(CAM['psi']))      # x of both camera lens tips (0.307): centre of the camera windows and humps
Z_LID = 12.3                # ceiling of the camera humps' pockets under the lid (it was 12.1: the real 35.6 mm tall board, tilted 20 degrees, reaches z 12.0 with its screw heads)
Z_ROOF = 12.0               # the lid's top: the ring (top 11.7, 5 mm higher than rev 3: the standing ToF boards) and the 3 mm lid


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


# ---- front omni wheel and its mount, inside the body (spec section 4, Figure 12). 9 Oct: the wheel is the Nexus 14145 (its datasheet, read at vstone.co.jp: 60 mm, 25.6 mm wide, 12 mm axle hole, 73 g, 3 kg load,
# 10 rollers), not the 20 mm wide placeholder of 8 Oct. It spins on two 604 bearings (4 x 12 x 4 mm) pressed into its 12 mm hole, on an M4 axle screw clamped in a 3 mm aluminium arm [the bore depth is to be measured].
# At 25.6 mm the wheel on the centre line would touch the GIGA's front-inner post H1 (post edge y -1.27, wheel face -1.28), so it sits 3 mm to the left (yc 0.30), the arm is outboard of it (y 1.68 to 1.98) and the
# floor port moves out to y 3.5. The pivot is raised from z 3.6 to 4.2: a 4 mm pin at 3.6 hung 1 mm under the belly line (3.5), and the spring coil (OD 11.6 mm) needs the room. With the pivot higher the axle's arc bows
# forward, so the rest position moves back from x 7.0 to 6.85 and the arm grows from 4.49 to 4.6 cm: the rim stays 1.7 mm inside the wall over the whole travel (omni_mount.py).
OMNI_WHEEL = dict(name='Nexus 14145', r=3.0, w=2.56, bore_r=0.6, mass_g=73.0)
OMNI_BEARING = dict(bore_r=0.2, od_r=0.6, w=0.4)             # 604-2RS 4 x 12 x 4 mm: bore radius, outer radius, width; two of them in the wheel's hole
OMNI = dict(r=OMNI_WHEEL['r'], w=OMNI_WHEEL['w'], yc=0.30, rest=(6.85, 3.0), travel=2.5, arm_len=4.6, pz=4.2, arm_t=0.3, arm_gap=0.1, arm_half=0.6, pin_r=0.15, axle_r=0.2, pivot_hole_r=0.2525)   # pivot bore 5.05 mm (reamed) for the 5.00 mm tube (10 Oct; 9 Oct had them equal)
OMNI['pivot'] = (OMNI['rest'][0] - math.sqrt(OMNI['arm_len'] ** 2 - (OMNI['pz'] - OMNI['rest'][1]) ** 2), OMNI['pz'])          # (2.409, 4.2)
OMNI['y_wheel'] = (OMNI['yc'] - OMNI['w'] / 2, OMNI['yc'] + OMNI['w'] / 2)                                                      # y -0.98 to 1.58
OMNI['arm_y'] = (OMNI['y_wheel'][1] + OMNI['arm_gap'], OMNI['y_wheel'][1] + OMNI['arm_gap'] + OMNI['arm_t'])                   # y 1.68 to 1.98: a 3 mm bar, 1 mm outboard of the wheel
OMNI_BAY_Y = (OMNI['y_wheel'][0] - 0.15, OMNI['arm_y'][1] + 0.12)                                                               # floor opening for the wheel and the arm: y -1.13 to 2.10
OMNI_ARM_SLOT_Y = (OMNI['arm_y'][0] - 0.1, OMNI['arm_y'][1] + 0.12)                                                             # the arm's part of that opening, y 1.58 to 2.10 (the pocket behind it is one cut with it, 9 Oct)
_PX, _PZ = OMNI['pivot']
# the opening through the floor, two plan cuts (9 Oct; the 8 Oct stadium cuts, extruded over the whole height, nicked the wall and the frame boss): the wheel's (its rim at floor level reaches x 10.003) and the arm's corridor
OMNI_OPENING = dict(wheel=dict(x=(3.65, 10.1), y=(OMNI['y_wheel'][0] - 0.15, OMNI['y_wheel'][1] + 0.15)), arm=dict(x=(_PX - 0.75, 8.0), y=(0.75, OMNI['arm_y'][1] + 0.12)))
# the axle: an M4 x 25 socket screw whose head and 0.5 mm washer sit INSIDE the wheel's 12 mm bore (the inboard bearing is seated 5 mm in), through a 12.6 mm sleeve between the two inner rings and a 1 mm spacer on
# the outboard side, tapped into the 3 mm arm (Loctite 243). Its tip stops 0.1 mm inside the arm's outer face: a nut out there would sweep through the floor wall as the axle rises through the floor's thickness and come
# within 1.5 mm of the floor port; a head standing out of the wheel's face (first draft) swept through the floor beside the bay and the GIGA post H1's footprint.
OMNI_AXLE = dict(screw_len=2.5, head_r=0.35, head_h=0.4, washer_r=0.3, washer_h=0.05, sleeve_r=(0.205, 0.3), seat=0.5)
# the torsion spring on the pivot (spec 4.3): music wire 1.6 mm, mean coil 10 mm, right-hand wound, the load OPENS it (both legs leave the top of the coil, each held from below). 10 Oct: the legs were given their
# supports. legs = (rear, forward): the straight length from the coil's tangent point to the load point, i.e. the lever of each leg's force; the rear leg lies on the adjuster head 10 mm behind the pivot, the forward
# one runs along the arm, splayed `tilt` degrees up from it (it starts 5 mm above the arm's axis, tangent to the coil, and has to pass over the stop screw), and rests on a 2 mm pin in the arm, 11 mm out
# (OMNI_SEAT), overhanging it by `tip`. body_turns 4.07 is what the geometry needs (the legs sit at -(phi_rest + tilt) + the preload angle round the coil, omni_mount.spring), the 4.29 active turns (legs included:
# + (L1 + L2) / (3 pi D)) give 493 N.mm/rad against the 484 the ramp needs. 9 Oct had 4.21 body turns and an 8 mm forward leg 'hooked on the arm's top edge', which the coil's position 5 mm inboard of the
# arm and the leg's height made impossible.
OMNI_SPRING = dict(wire=0.16, mean_d=1.0, body_turns=4.07, legs=(1.0, 1.1), tip=0.1, tilt=4.0, E=207000.0, uts=2200.0)
OMNI_SPRING['active_turns'] = OMNI_SPRING['body_turns'] + sum(OMNI_SPRING['legs']) / (3 * math.pi * OMNI_SPRING['mean_d'])
OMNI_SPRING['od'] = OMNI_SPRING['mean_d'] + OMNI_SPRING['wire']
OMNI_SPRING['id'] = OMNI_SPRING['mean_d'] - OMNI_SPRING['wire']
OMNI_SPRING['length'] = OMNI_SPRING['wire'] * (OMNI_SPRING['body_turns'] + 1.0)                                         # close wound
OMNI_SPRING['y'] = (0.80, 0.80 + OMNI_SPRING['length'])                                                                 # the coil: 0.5 mm off the pillar, 0.7 mm short of the arm
# the spring's seat on the arm: a 2 mm stainless dowel pin (ISO 2338 2 m6 x 6) in a 2.0 H7 hole in the arm (retaining compound), 3.5 mm standing out of its inboard face and 2.5 mm in the bar, s mm out along the arm from the
# pivot; the leg rests on its top (n from the arm's axis = the leg's underside minus the pin's radius). It stands 1 mm clear of the stop screw's surface and outside the coil's footprint
OMNI_SEAT = dict(s=OMNI_SPRING['legs'][1], r=0.10, hole_r=0.10, y=(OMNI['arm_y'][0] - 0.35, OMNI['arm_y'][0] + 0.25))
# the spring's arbor: a printed sleeve on the pivot tube, 7.4 mm outside and drawn 5.2 mm inside for the 5.00 mm tube (a printed hole comes out small), 9 mm long (the coil's inside diameter is 8.4 mm, about 8.7 at full travel), from the pillar's face to 0.3 mm short of the arm. Without it the
# coil hangs on the 5 mm tube 1.7 mm off-centre, pushed up by its two supports
OMNI_ARBOR = dict(r=0.37, r_in=0.26, y=(0.75, 1.65))
# the pivot: an M3 x 25 socket screw (the same as the stop screw), head on the pillar's inboard face, through the pillar (3.2 mm hole), a 5 x 3.1 x 13.5 mm tube that the arm turns on (clamped between the pillar and the
# ear), threaded into an M3 x 5.7 mm heat-set insert in the outer ear, its tip in the blind hole behind the insert (7.6 mm deep). An M4 screw (first draft) had a 7 mm head that reached z 3.85, 0.5 mm into the floor, and
# scraped it when pulled out along y in the teardown path of the dry run; the 5.5 mm M3 head clears the floor by 0.25 mm. The tube, not the screw, carries the arm's load (96 N in the impact case: 30 MPa in the tube).
OMNI_PIVOT = dict(screw_r=0.15, screw_len=2.5, head_r=0.275, head_h=0.3, tube_r=(0.155, 0.25), tube_len=1.35, insert_r=0.23, insert_len=0.57, hole_depth=0.76)
_LEG_Z = _PZ + OMNI_SPRING['mean_d'] / 2 - OMNI_SPRING['wire'] / 2                                                      # underside of the rear leg where it leaves the top of the coil: 4.62
OMNI_MOUNT = dict(
    hole_r=0.16,                                                                                                       # 3.2 mm holes in the pillar for the M3 pivot and stop screws
    pillar=dict(x=(_PX - 0.63, 3.55), y=(0.35, 0.75), z=(Z_FLOOR_TOP, 4.95)),                                           # inner pillar on the floor: the pivot screw's and the stop screw's head side
    tail=dict(x=(_PX - OMNI_SPRING['legs'][0] - 0.42, _PX - 0.63), y=(0.35, 1.2), z=(Z_FLOOR_TOP, 4.4)),               # its tail behind the pivot, carries the spring's adjuster screw under the rear leg (y to 1.2: 1 mm off the left motor's seat)
    adjuster=dict(x=_PX - OMNI_SPRING['legs'][0], y=OMNI_SPRING['y'][0] + OMNI_SPRING['wire'] / 2, shank_r=0.15, head_r=0.318, head_z=(_LEG_Z - 0.2, _LEG_Z), hole_r=0.21, bolt_len=0.6),    # M3 x 6 hex bolt (head 5.5 across flats, 2 mm) in an M3 x 5.7 insert flush with the tail's top: the rear leg lies on its head, one turn = 0.55 N at the axle
    ear=dict(x=(_PX - 0.5, 3.65), y=(OMNI['arm_y'][1] + 0.12, OMNI['arm_y'][1] + 1.0), z=(Z_FLOOR_TOP, 4.7)),           # outer ear, 8.8 mm thick for the M3 insert and the 7.6 mm blind holes, 1.2 mm outboard of the arm, under the battery tray (4.9), long enough to hold the stop screw too
    pocket=dict(x=OMNI_OPENING['arm']['x'], y=OMNI_OPENING['arm']['y']),                                               # through cut in the floor between the two, joined to the wheel bay: the coil (bottom z 3.62), the arm and the stop pin
    stop=dict(r=0.8, pin_r=0.15, hw=0.165, play=0.02, hole_r=0.16, ear_hole_r=0.13, ear_depth=0.76, screw_len=2.5, head_r=0.275, head_h=0.3),
    # the stop: an M3 x 25 screw from the pillar (head inboard) across the pocket and through the arm's slot into the ear (7.5 mm of thread in a 2.6 mm pilot hole), 8 mm ahead of the pivot (it clears the coil's 5.8 mm
    # radius by 0.7 mm); the arm's slot is an arc about the pivot, 3.3 mm wide, whose two ends are the rest stop and the compression stop
)


def omni_stop_psi():
    """Angle of the stop screw about the pivot, degrees: the middle of the arm's swing, so that the slot in the arm is centred on the arm's axis. (A slot cut 5 degrees below the axis, to give the forward spring leg
    more room over the screw, put the screw's head 0.4 mm into the floor on the pull-out path: the leg is tilted instead, 10 Oct.)"""
    ox, oz = OMNI['rest']
    xc, zc = omni_at(OMNI['travel'])
    return 0.5 * (math.degrees(math.atan2(oz - _PZ, ox - _PX)) + math.degrees(math.atan2(zc - _PZ, xc - _PX)))


def omni_stop_pin():
    """World (x, z) of the stop screw's axis."""
    a = math.radians(omni_stop_psi())
    return _PX + OMNI_MOUNT['stop']['r'] * math.cos(a), _PZ + OMNI_MOUNT['stop']['r'] * math.sin(a)


def omni_stop_slot():
    """The arc slot in the arm, in the arm's own frame (angles in degrees from the arm's axis about the pivot, positive = above it): r, half width hw, and the angles of the centres of its two round ends
    (a_hi = the rest stop, the screw at the top of the slot; a_lo = the compression stop). The slot ends are `play` (0.2 mm) beyond the screw at rest and at full travel."""
    S = OMNI_MOUNT['stop']
    ox, oz = OMNI['rest']
    xc, zc = omni_at(OMNI['travel'])
    psi = math.radians(omni_stop_psi())
    th_rest = psi - math.atan2(oz - _PZ, ox - _PX)
    th_full = psi - math.atan2(zc - _PZ, xc - _PX)
    d = (S['play'] - (S['hw'] - S['pin_r'])) / S['r']
    return dict(r=S['r'], hw=S['hw'], a_hi=math.degrees(th_rest + d), a_lo=math.degrees(th_full - d))


def omni_at(travel):
    """Axle position when the sprung arm has been pushed up by `travel` cm (the axle moves on an arc about the pivot)."""
    ox, oz = OMNI['rest']
    px, pz = OMNI['pivot']
    zc = oz + travel
    return px + math.sqrt((ox - px) ** 2 + (oz - pz) ** 2 - (zc - pz) ** 2), zc


def omni_arm_pt(s, n, travel=0.0):
    """World (x, z) of the point at s cm along the arm from the pivot and n cm up from its axis, with the arm pushed up by `travel` (the arm turns, the point turns with it)."""
    px, pz = OMNI['pivot']
    xc, zc = omni_at(travel)
    phi = math.atan2(zc - pz, xc - px)
    return px + s * math.cos(phi) - n * math.sin(phi), pz + s * math.sin(phi) + n * math.cos(phi)


def omni_leg_n(s):
    """Height of the forward spring leg's centre line above the arm's axis, cm, s cm along the arm: a straight line tangent to the coil's mean circle (mean_d / 2 from the pivot) and splayed `tilt` degrees up from
    the arm's direction."""
    al = math.radians(OMNI_SPRING['tilt'])
    return OMNI_SPRING['mean_d'] / 2 / math.cos(al) + s * math.tan(al)


def omni_seat_n():
    """The seat pin's centre above the arm's axis, cm: its top is the underside of the forward leg at the pin."""
    S = OMNI_SPRING
    return omni_leg_n(OMNI_SEAT['s']) - S['wire'] / 2 / math.cos(math.radians(S['tilt'])) - OMNI_SEAT['r']


def omni_seat(travel=0.0):
    """World (x, z) of the seat pin's axis (it runs along y)."""
    return omni_arm_pt(OMNI_SEAT['s'], omni_seat_n(), travel)


def omni_leg(travel=0.0):
    """The forward spring leg's centre line in the world x-z plane, from where it clears the coil's footprint to its tip: ((x0, z0), (x1, z1)). It lies along the arm, splayed `tilt` degrees up from it, and ends `tip`
    past the seat pin; it turns with the arm."""
    S = OMNI_SPRING
    s0 = math.sqrt(max((S['od'] / 2) ** 2 - (S['mean_d'] / 2 - S['wire'] / 2) ** 2, 0.0)) + 0.06     # where the leg's lower edge leaves the coil's outer circle (4.0 mm), plus 0.6 mm
    s1 = S['legs'][1] + S['tip']
    return omni_arm_pt(s0, omni_leg_n(s0), travel), omni_arm_pt(s1, omni_leg_n(s1), travel)


# ---- ToF boards (spec 8.3). 9 Oct: the board is the current Adafruit VL53L0X board with two STEMMA QT connectors (since July 2020): 25.4 x 17.78 mm, not the 21 x 18 mm of the product page, which still shows the old board.
# Source: Adafruit's EAGLE file "Adafruit VL530X STEMMA QT.brd" (github.com/adafruit/Adafruit-VL53L0X-ToF-Distance-Sensor-PCB): outline 25.4 x 17.78 mm with 2.54 mm corner radius, four 2.5 mm plated holes (3.2 mm pad) at 2.54 mm
# from the edges (a 20.32 x 12.7 mm grid), the chip at the centre (its long side across the board), a JST SH 4-pin connector (SM04B-SRSS-TB: 6.1 wide, 4.25 deep, about 3 mm tall) 0.58 mm in from each short end, mid-width, all on the
# chip side. The connector height is from JLCPCB and Farnell listings of the part (2.9 to 3.05 mm), the PCB thickness of 1.6 mm is the usual Adafruit value (not in the file): both [measure]. The pre-July-2020 board (20.32 x 17.78 mm,
# two holes, no connectors) does not fit this mount.
# The board STANDS on its short end (25.4 mm vertical): lying down it needs 3.2 to 3.7 cm of ring and the pockets of the front-left and side sensors overlap; standing it needs 2.4 cm, and the frame grows from 25 to 30 mm
# (the board is 25.4 mm tall and sits on a 3 mm floor). Local frame of one sensor: u along the beam, v to its left, w up, origin at the centre of the PCB's front (chip) face = the position (x, y) of the list below.
TOF_BOARD = dict(long=2.54, short=1.778, t=0.16, corner_r=0.254, hole_short=0.635, hole_long=1.016, hole_d=0.25, pad_d=0.32,
                 conn_h=0.305, conn_len=0.425, conn_w=0.61, conn_end=0.058, chip_u=0.10, chip_v=0.44, chip_w=0.24, screw_head=(0.45, 0.15), screw_len=0.6)
# The mount (all in the frame, spec 8.3): a pocket open at the top that the board stands in on a 3 mm floor, a rear wall with two access holes for the screws, two posts in front of the board's upper holes (M2.5 x 6 screws from
# behind pull the board onto them), a recess in front for the chip and the connectors, the beam tunnel, a shaft down through the floor for the plug of the lower connector, and a printed block that fills the bore side where
# the pocket hangs in the bore (the side sensors).
TOF_MOUNT = dict(rear_gap=0.23, rear_wall=0.30, rec=0.36, side_gap=0.08, side_wall=0.25, front_wall=0.25, shaft_v=0.38, tunnel_v=0.9, tunnel_w=0.9,
                 post_r=0.26, pilot_r=0.11, access_r=0.25, wall_min=0.25, gap_min=0.30, floor_z=9.0)
TOF_Z = 10.28               # the chip's height: the board's bottom edge stands on the floor (z 9.0), its top edge is at 11.55 (rev 3: chip at 10.0, board 2.1 cm tall)
# name, x, y, aim (deg, 0 = forward, 90 = left): the position of the chip plane. Moved on 9 Oct (only what the real board needs: the recess corners stay inside r 10.25, 2.5 mm of ring in front of them, and no two pockets
# closer than 3 mm): FL and FR 1 mm along their aim (rho 9.9 -> 9.8) and 1 degree round, the side sensors from x +-7.18 to +-6.85 (baseline 14.4 -> 13.7 cm, y 6.0 stays: TOF_SIDE_OUT_MM stays 60.0), the rear two from
# y +-4.5 to +-3.6; F stays.
TOF = [('F', 9.5, 0.0, 0), ('FL', 9.8 * math.cos(math.radians(24)), 9.8 * math.sin(math.radians(24)), 25), ('FR', 9.8 * math.cos(math.radians(24)), -9.8 * math.sin(math.radians(24)), -25),
       ('SFL', 6.85, 6.0, 90), ('SFR', 6.85, -6.0, -90), ('SRL', -6.85, 6.0, 90), ('SRR', -6.85, -6.0, -90),
       ('RL', -8.4, 3.6, 180), ('RR', -8.4, -3.6, 180)]

# ---- cameras (spec 8.4). 9 Oct: what is known and what is not about the OpenMV Cam H7 Plus.
# KNOWN [src] openmv.io: the board is 35.56 x 44.45 mm (1.40 x 1.75 in), 45 x 36 x 29 mm with the lens; the dimension drawing gives the four mounting holes, all in the 8 mm next to the camera end of the board:
# 3.0 and 2.4 mm from the left edge / 32.5 and 33.3 mm, 3.1 and 8.2 mm from the camera end, drilled 2.8 and 3.05 mm (read from the drawing, +-0.5 mm). The back of the board carries the micro-SD socket (about 2 mm tall).
# NOT PUBLISHED, so estimated from the product photo [measure with a caliper on the real board, spec 8.4 and Figure 10]: the lens axis is centred across the board and about 5 mm from the camera end, the lens holder is
# about 17.4 mm square and overhangs the end by about 4 mm, the M12 barrel is about 12.5 mm across, and the lens tip stands 27.4 mm above the PCB's front face (29 mm in all over its rear face, less the 1.6 mm PCB;
# if the 29 mm includes the SD socket the tip is 2 mm nearer). The PCB thickness 1.6 mm is the usual value. All the numbers marked (est) are guesses and the model, the cage and the window rest on them.
CAMERA = dict(board=(3.556, 4.445, 0.16),                  # u (short side, up in the robot) x v (long side, along x) x PCB thickness, cm
              holes=((0.30, 0.31, 0.28), (3.25, 0.31, 0.28), (0.24, 0.82, 0.305), (3.33, 0.82, 0.305)),       # u from the left edge, v from the camera end, drill diameter
              lens_u=1.78, lens_v=0.50,                      # (est) lens axis: centred across, 5 mm from the camera end
              holder=(1.74, 1.74, 1.20), barrel_d=1.25,      # (est) lens holder u x v x height above the PCB's front face; M12 barrel diameter
              tip=2.74,                                      # (est) lens tip above the PCB's front face
              tail=1,                                        # the long tail of the board points to +x on both sides (the lens end is rear of the lens axis by lens_v)
              sd=(1.4, 1.2, 0.2),                            # micro-SD socket on the back: u x v x height (est)
              window_w=2.8,                                  # width of the window through the ring (spec 8.4)
              pocket_y=(5.6, 7.7), hump_y=(5.4, 7.9),         # the lid pocket's and the hump's extent across the robot (the hump wider than the pocket by the 2 mm skin; its corner at x 4.9 must stay inside the body radius)
              pocket_x=(-1.3, 4.7))                          # the lid's pocket over the board: from the cage frame's edge (x -1.09) to the tail end (x 4.25), 2 and 4.5 mm to spare
# The cage (printed, one per side): a frame lying on the board's front face round the lens holder with four M2.5 screws from behind, two rails along the sides of the holder, a flange behind the lens tip with the barrel
# opening, and two ears that reach the ring's inner face beside the window, each with an M3 screw from the bore side into an insert in the ring. The board is screwed to it on the bench; the unit goes into the robot by
# the two ear screws. Local frame of one camera: origin at the lens tip, z along the view axis backwards (towards the board), y up in the board's plane, x along the robot's x (tail side positive for the left camera).
CAGE = dict(frame_t=0.50, frame_x=1.40, frame_y=1.85, open_x=1.00, open_y=1.15,       # the frame: half widths of the outer edge and of the opening for the lens holder, thickness (5 mm: the M2.5 screws, 6 mm long, bite 4.6 mm into it)
            rail_x=(1.00, 1.40), rail_y=0.55, flange_z=(0.80, 1.10), flange_x=3.0, hole_d=1.70,       # rails, flange (z behind the tip, half width, barrel opening): the flange's top front edge stays 1.4 mm off the ring's inner face
            ear_x=2.5, ear_w=1.0, ear_z=(9.0, 10.0), m3_d=0.32, insert_d=0.40, insert_depth=0.6, pilot_d=0.22,
            screw25=(0.25, 0.45, 0.15), screw3=(0.30, 0.55, 0.20))                      # M2.5 and M3 screws: shaft, head diameter, head height

# ---- hopper and 19 mm square channel (spec section 6.2; redesigned on 8 Oct after the chute simulation and enlarged on 9 Oct for reliability: the first design, a 14.5 mm slot and hopper over a 13 mm bore whose open trough was
# cut flat at z 7.7, jammed every kit that arrived square to the slot). The slot and the hopper void are 17 mm (16 on 8 Oct), so that a cube lying at any turn (face diagonal 14.57 mm) passes, and one tipped any way, even standing on a
# corner (16.25 mm, cube_slot.py); the bore is 19 mm square (18 on 8 Oct), wider than the void and 1.16 mm wider than the cube's space diagonal (17.84 mm; 0.81 mm over a 10.5 mm kit's), so that a kit that
# tumbles on its bounce cannot wedge between two walls or between floor and ceiling, even with a bore printed 0.4 mm small (spec 6.2, D21).
# The trough is cut down to the bore floor inside the hopper: a 25 mm square cut (`trough`, larger than the void: it must go right through the side walls where the tube starts, or loose
# fragments of wall are left behind, and the walls then end in a V that leads a kit into the bore) removes everything of the tube inside it, and the floor slab is laid back afterwards
# (`slab_end` cm along the axis from the slot centre; the cut's vertical wall meets the sloping floor at 2.55 cm (top face) to 2.65 cm (underside), not at the 2.04 cm of its plan corner, so 2.8). The square sections of the tube, the bore, the hopper socket and the wall hole are centred
# `lift` above the axis, so the bore floor is 8 mm below the axis (as in the 16 and 18 mm designs) and the ceiling 11 mm above it. The hopper's socket (what the channel's section cuts out of the
# hopper where the channel leaves it) is `socket_up` taller than the channel, upward: cut to the channel's own ceiling it left the hopper's downhill corner above the channel hanging
# free, an 85 mm3 lintel that Fusion split off as a second body of each hopper.
CHUTE = dict(in_w=1.9, wall=0.16, exit_x=-6.0, exit_axis_z=4.15, start_z=7.95, hopper_out=2.2, hopper_in=1.7, flange=2.6, flange_t=0.15, hopper_z0=7.2, hole_clear=0.04,
             trough=2.5, trough_z0=5.0, slab_end=2.8, lift=0.15, socket_up=0.8)           # 9 Oct (D21, D22): bore 19 mm (it was 18) and lift 1.5 mm (was 1.0) so that the floor stays 8 mm under the axis; hopper void 17 mm = the slot (was 16)
CHUTE['out_w'] = CHUTE['in_w'] + 2 * CHUTE['wall']          # 2.22 (2.12 on 8 Oct)
CHUTE['exit_y'] = math.sqrt(R_BODY ** 2 - CHUTE['exit_x'] ** 2)


def chute_ends(side):
    """Axis from the slot centre (z 7.95) to the exit on the body radius (z 4.15). side +1 = left (slot B), -1 = right (slot A)."""
    sx, sy = slot_xy('B' if side > 0 else 'A')
    return (sx, sy, CHUTE['start_z']), (CHUTE['exit_x'], side * CHUTE['exit_y'], CHUTE['exit_axis_z'])


# ---- upper frame, dropper unit, lid, handle, front controls (spec sections 3.1, 6.1, 7)
FRAME = dict(r_in=9.0, r_out=10.5, z0=8.7, z1=11.7, floor_t=0.3,
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
HOOK = dict(r_in=10.34, r_out=10.5, skirt_z0=10.9, half_deg=5.5, groove_half_deg=4.5, groove_r_in=10.24, bump_z=(11.05, 11.35))       # 9 Oct: 5 mm higher with the ring top (the standing ToF boards)
LID = dict(z0=11.7, z1=12.0, slot_x=(2.95, 9.05), slot_half=0.9, front_notch=(4.9, 9.2, 0.7, 3.5), finger_deg=12.0, hump_skin=0.2)       # front_notch: x0, x1, y0, y1, over the control deck; it joins the handle slot
CONTROLS = (('Start button', 7.55, 2.15, ('round', 0.6), 9.0, 9.7, '#639922'), ('Power switch', 6.0, 2.15, ('rect', 1.3, 1.9), 9.0, 10.3, '#3B6D11'),
            ('Status LED 1', 7.5, 0.0, ('round', 0.2), 9.8, 10.2, '#888780'), ('Status LED 2', 6.6, 0.0, ('round', 0.2), 9.8, 10.2, '#888780'))              # name, x, y, shape (round: radius; rect: x size, y size), z0, z1, colour: the button and switch on the +y side of the deck, the two status LEDs in a row on top of the bridge rib (z 9.8), all reached through the lid's slot and front notch; the victim LED is on the handle bar (HANDLE['led'])
USB = dict(angle=-24.0, r_face=10.45, depth=1.3, half=0.7, z=(7.1, 7.9))   # USB-C service socket in the front-right wall above the bumper band: front face flush (r 10.45 on the axis, 10.47 at the edges), 13 mm deep; at -24 degrees it is 8 mm from the 12 degree boss rib


def usb_pose():
    """Centre of the socket body in plan (x, y) and its direction in degrees."""
    r = USB['r_face'] - USB['depth'] / 2
    a = math.radians(USB['angle'])
    return r * math.cos(a), r * math.sin(a), USB['angle']


# ---- floor sensors, moved forward for earlier detection (spec 5.3): one each side of the omni bay, 7.5 cm ahead of the axle (rev 3: FP at 6.5, SM at the axle)
FLOOR_FRONT = dict(FLOOR_FRONT, x=7.5, y=3.5, w=2.03)       # 9 Oct: the front port is the Adafruit TCS34725 board, 20.32 x 20.32 mm with two 2.5 mm holes 15.24 mm apart at its top edge (EAGLE file, github.com/adafruit/Adafruit-TCS34725-Color-Sensor-Breakout-PCB), not the 16 mm placeholder
SILVER = dict(SILVER, x=7.5, y=-2.7)                          # the silver module is still a placeholder (parts not chosen: the silver bench test decides)


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


# ---- BNO055 IMU (spec 8.6). Adafruit 2472: 20 x 27 x 4 mm, four mounting holes 20 mm apart along the board and 12 mm across, header along one short edge (product page). On four printed posts 5 mm over the tub floor, on the
# centre line 5 cm behind the axle (free column: the hoppers and chutes are beside it, the N20 in front, the spoke above); header edge forward. The yaw rate does not depend on where the board is; the magnetometer does (motors, battery,
# the N20), so the firmware should run it in IMU mode (no magnetometer) until it is calibrated in place. Positions of the chip and the header are not modelled.
IMU = dict(x=-5.0, y=0.0, size=(2.7, 2.0, 0.16), holes=(1.0, 0.6), hole_d=0.25, post=0.50, post_h=0.5, pilot=0.20, chip=(0.52, 0.38, 0.11), screw_head=(0.45, 0.15))


# ---- Wi-Fi/Bluetooth antenna (spec 8.5). The GIGA has no on-board antenna: only the u.FL socket J14 at the front-inner corner, beside the reset button, with a flat flex antenna in the box
# (Molex 206994-0100, about 15.4 x 6.4 mm on a 100 mm cable). It is stuck on the inside of the tub's front wall just right of centre.
J14 = dict(from_header=0.5, from_edge=0.22, z=6.4)             # read from the datasheet picture (+-1 mm); the socket top is about 1 mm over the board
ANTENNA = dict(angle=-22.0, r_face=10.14, w=0.64, t=0.05, z=(5.0, 6.54), cable=10.0)       # 9 Oct: on the right bumper's recess wall (inner face r 10.15, z 3.9 to 6.6, angles -62 to -8), 8 degrees from the switch, 5.6 mm under the USB-C socket; w tangential, t radial, z range = the 15.4 mm strip


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
           'Face plate L', 'Face plate R', 'N20 face plate', 'GIGA posts', 'Bumper L', 'Bumper R', 'Camera cage L', 'Camera cage R')
BOUGHT_G = {'Wheel L': 55.0, 'Wheel R': 55.0, 'Motor L': 95.0, 'Motor R': 95.0, 'Omni wheel': 77.0, 'Omni arm': 6.0, 'Omni pins': 3.0, 'Omni pivot': 3.0, 'Omni spring': 4.0, 'Omni adjuster': 1.0, 'N20 motor': 12.0,
            'Arduino GIGA R1': 55.0, 'Main PCB': 25.0, 'Battery': 110.0, 'Bumper switch L': 1.0, 'Bumper switch R': 1.0, 'Kits': 48.0,
            'Camera L': 27.5, 'Camera R': 27.5, 'Rear nub': 5.0, 'Power switch': 8.0, 'Start button': 3.0, 'Victim LED': 1.0,
            'Status LED 1': 1.0, 'Status LED 2': 1.0, 'USB-C service socket': 4.0, 'Floor port FP': 4.0, 'Silver module SM': 4.0, 'Wi-Fi antenna': 1.0, 'BNO055 IMU': 3.0}
for _nm, _x, _y, _a in TOF:
    BOUGHT_G['ToF ' + _nm] = 3.3
UNMODELLED_G = (('mux, regulator, wiring', 55.0, 0.0, 6.0), ('LCD, second colour sensor, hardware', 63.0, 0.0, 7.0), ('cables and fasteners', 40.0, 0.0, 8.0))     # name, g, x, z


if __name__ == '__main__':
    print('pivot', [round(v, 3) for v in OMNI['pivot']], 'compressed axle', [round(v, 3) for v in omni_at(OMNI['travel'])])
    print('chute A', [[round(v, 3) for v in p] for p in chute_ends(-1)])
    print('GIGA holes', [[round(v, 3) for v in giga_hole_xy(i)] for i in GIGA_USED], 'J12', [round(v, 3) for v in giga_j12()])
    print('stepper', {k: v for k, v in stepper_geometry().items()})
