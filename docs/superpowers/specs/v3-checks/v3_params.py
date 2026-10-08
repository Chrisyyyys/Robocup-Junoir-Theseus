"""V3 baseline, single source of truth for figures.py (cm, degrees).
Frame: x forward, y left, z up. Origin = midpoint of the drive axle, on the floor line (the axle is at z = 4.0).
Every number here is the one quoted in docs/superpowers/specs/2026-10-06-theseus-v3-robot-design.md."""
import json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))

R_BODY, R_INT, R_SWEPT = 10.5, 10.3, 11.0
Z_BELLY, Z_ROOF, Z_LID = 3.5, 11.5, 12.1
AXLE_Z = 4.0
PATH_NOM, PATH_MIN, WALL_NOM, WALL_MIN = 28.0, 25.2, 14.0, 12.6          # path width and half widths (rule 3.3.3, +-10 % rule 3.8.6)

WHEEL = dict(r=4.0, y0=7.0, y1=9.0)
MOTOR = dict(r=1.0, y_face=6.1, length=4.4, enc=0.3)                      # occupies y = 1.4 .. 6.1
OMNI = dict(r=3.0, w=2.0, rest=(7.95, 3.0), pivot=(3.5, 3.6), travel=2.5, fork_half=1.5)
NUB = dict(x=-8.0, z_low=2.0, r=0.4)
CHAMFER = ((-6.5, 3.5), (-10.5, 5.3))                                       # underside rises from (x, z) to (x, z)
BUMPER = dict(a0=12.0, a1=58.0, z0=4.0, z1=6.5, r_out=11.0, t=0.25)
FLOOR_FRONT = dict(x=6.5, y=3.0, z_face=2.8, w=1.6)                        # port 1: black / blue / red, sees the tile ahead of the wheels
SILVER = dict(x=0.0, y=0.0, z_face=2.8, l=2.0, w=2.2)                       # port 2: silver pair (straight + 20 deg tilted), between the motors at the axle line
BUMPER_SW_DEG = 14.0                                                        # microswitch at the inner end of each plate (outer ends: 2 spare inputs)

R_ = R_BODY - 0.2
TOF_Z, TOF_W, TOF_H, TOF_T, TOF_CONE = 10.0, 1.8, 2.1, 0.45, 25.0
# name, x, y, aim (deg, 0 = forward, 90 = left)
TOF = [('F', R_BODY - 1.0, 0.0, 0), ('FL', R_ * math.cos(math.radians(25)), R_ * math.sin(math.radians(25)), 25),
       ('FR', R_ * math.cos(math.radians(25)), -R_ * math.sin(math.radians(25)), -25),
       ('SFL', R_ * math.cos(math.radians(45)), R_ * math.sin(math.radians(45)), 90),
       ('SFR', R_ * math.cos(math.radians(45)), -R_ * math.sin(math.radians(45)), -90),
       ('SRL', R_ * math.cos(math.radians(135)), R_ * math.sin(math.radians(135)), 90),
       ('SRR', R_ * math.cos(math.radians(135)), -R_ * math.sin(math.radians(135)), -90),
       ('RL', R_ * math.cos(math.radians(154)), R_ * math.sin(math.radians(154)), 180),
       ('RR', R_ * math.cos(math.radians(154)), -R_ * math.sin(math.radians(154)), 180)]

CAM = dict(psi=88.0, tip_r=8.8, zl=9.3, tilt=20.0, hfov=65.9, vfov=51.8, board=(4.5, 3.6), lens=(2.3, 2.0, 2.0))  # board w x h, lens depth x w x h

PLATE = dict(cx=-2.0, cy=0.0, r_ring=3.86, pocket=1.40, slot=1.45, pitch=30.0, n=8, z0=9.0, t=1.2, floor_t=0.3, rim=0.3,
             slotA_robot_deg=225.0, slotB_robot_deg=135.0, plate_deg_A=0.0, plate_deg_B=270.0, first_pocket_plate_deg=30.0,
             n20_len=4.15, n20_w=(1.0, 1.2))
PLATE['R'] = math.hypot(PLATE['r_ring'] + PLATE['pocket'] / 2, PLATE['pocket'] / 2) + PLATE['rim']
EXIT = dict(x=-6.0, z_nose=3.0, bore=1.5, wall=0.15)
EXIT['y'] = math.sqrt(R_BODY ** 2 - EXIT['x'] ** 2)
KIT_LAND = (-6.6, 11.3)                                                     # mu 0.35, see kit_final.py

def slot_xy(which):
    a = math.radians(PLATE['slotA_robot_deg'] if which == 'A' else PLATE['slotB_robot_deg'])
    return PLATE['cx'] + PLATE['r_ring'] * math.cos(a), PLATE['cy'] + PLATE['r_ring'] * math.sin(a)

def pocket_xy(i, rot_deg=0.0):
    """i = 1..8, plate rotation rot_deg (CCW seen from above) from the parked position. Robot angle = 225 + 30 i + rot."""
    ang = math.radians(PLATE['slotA_robot_deg'] + PLATE['first_pocket_plate_deg'] + PLATE['pitch'] * (i - 1) + rot_deg - PLATE['plate_deg_A'])
    return PLATE['cx'] + PLATE['r_ring'] * math.cos(ang), PLATE['cy'] + PLATE['r_ring'] * math.sin(ang), math.degrees(ang)

def load_pack():
    for fn in ('pack_v3.json',):
        p = os.path.join(HERE, fn)
        if os.path.exists(p):
            return json.load(open(p))['parts'], fn
    raise FileNotFoundError('run cam_recess.py first')

PART_SIZES = {'battery (placeholder 7.0x3.5x2.5)': (7.0, 3.5), 'GIGA R1 + main PCB stack': (10.152, 5.334)}

if __name__ == '__main__':
    print('plate R', round(PLATE['R'], 2), 'exit', round(EXIT['x'], 2), round(EXIT['y'], 2), 'slotA', [round(v, 2) for v in slot_xy('A')], 'slotB', [round(v, 2) for v in slot_xy('B')])
    for i in (1, 8): print('pocket', i, [round(v, 2) for v in pocket_xy(i)])
    print(load_pack())
