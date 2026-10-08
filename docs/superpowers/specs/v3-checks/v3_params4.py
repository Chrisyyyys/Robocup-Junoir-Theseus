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
