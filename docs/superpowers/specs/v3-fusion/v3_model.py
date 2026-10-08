"""Theseus V3 baseline as a Fusion assembly, built from docs/superpowers/specs/2026-10-06-theseus-v3-robot-design.md.
Run inside Fusion (see README.md).  Units cm.  Frame: x forward, y left, z up, origin at the drive-axle midpoint on the floor line.
Numbers come from ../v3-checks/v3_params.py, the single source of truth of the 2D checks; anything chosen here that the spec does not give
is marked [not in spec]."""
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
import v3_params as prm

R_BODY, R_INT, R_SWEPT = prm.R_BODY, prm.R_INT, prm.R_SWEPT
Z_BELLY, Z_ROOF, Z_LID = prm.Z_BELLY, prm.Z_ROOF, prm.Z_LID
FLOOR_T = 0.4                       # printed floor thickness [not in spec]
Z_RIM = 11.2                        # top of the wall; the lid plate (0.3) sits on it, so the roof is at 11.5 as in the spec
HUMP_SKIN = 0.2                     # lid skin over the camera boards; the spec's lid 12.1 is the clearance over the board top (12.05), so the outer top is 12.3 [not in spec]
RING_Z0, RING_Z1, RING_RIN = 8.7, 11.15, 9.0    # ToF ring [not in spec: only "one printed ring at z 10.0"]
HANDLE_X, HANDLE_Y, HANDLE_Z = -4.5, 9.04, 14.6  # placeholder arch handle on the chassis [not in spec]
OMNI_TRAVEL_MECH = prm.OMNI['travel']   # 2.5 cm hard stop, the value the 2D terrain checks used (spec text says about 3 cm; at 2.8 and 3.0 the arm plate clips the GIGA board corner, see README)
CAM_WINDOW_W, CAM_WINDOW_H = 2.8, 2.4   # camera window (spec: about 2 x 2 cm; 3D rays show 2.8 x 2.4 is needed to pass the 65.9 x 51.8 degree field, corners included, from 1.7 cm behind the surface)
OMNI_BAY_HALF = 1.8                 # half width of the omni and arm bay in the chassis (arm plates reach y 1.5, pivot pin 1.7)
TOF_MODE = 'fixed'                  # 'spec' = positions exactly as in the spec (side modules stick out of the shell), 'fixed' = moved inward


# ---------------------------------------------------------------------------------------------------- ToF positions
def tof_positions(mode):
    spec = {nm: (x, y, aim) for nm, x, y, aim in prm.TOF}
    if mode == 'spec':
        return spec
    fixed = dict(spec)
    c25, s25 = math.cos(math.radians(25)), math.sin(math.radians(25))
    fixed['FL'] = (9.9 * c25, 9.9 * s25, 25)
    fixed['FR'] = (9.9 * c25, -9.9 * s25, -25)
    fixed['SFL'] = (7.18, 6.0, 90)
    fixed['SFR'] = (7.18, -6.0, -90)
    fixed['SRL'] = (-7.18, 6.0, 90)
    fixed['SRR'] = (-7.18, -6.0, -90)
    fixed['RL'] = (-8.4, 4.5, 180)
    fixed['RR'] = (-8.4, -4.5, 180)
    return fixed


def tof_corner_r(x, y, aim):
    a = math.radians(aim)
    c, s = math.cos(a), math.sin(a)
    return max(math.hypot(x + dx * c - dy * s, y + dx * s + dy * c) for dx in (-prm.TOF_T / 2, prm.TOF_T / 2) for dy in (-prm.TOF_W / 2, prm.TOF_W / 2))


def t_exit(x, y, aim, r_out=10.8):
    a = math.radians(aim)
    ux, uy = math.cos(a), math.sin(a)
    b = x * ux + y * uy
    c = x * x + y * y - r_out ** 2
    return -b + math.sqrt(b * b - c)


def tof_rect(x, y, aim, t0, t1, w):
    """Rectangle along the aim direction from t0 to t1 (distance from the module centre), width w."""
    a = math.radians(aim)
    ux, uy = math.cos(a), math.sin(a)
    nx, ny = -uy, ux
    c0 = (x + ux * t0, y + uy * t0)
    c1 = (x + ux * t1, y + uy * t1)
    return poly([(c0[0] + nx * w / 2, c0[1] + ny * w / 2), (c1[0] + nx * w / 2, c1[1] + ny * w / 2),
                 (c1[0] - nx * w / 2, c1[1] - ny * w / 2), (c0[0] - nx * w / 2, c0[1] - ny * w / 2)])


class Ctx:
    def __init__(self, tof_mode=TOF_MODE):
        self.app = adsk.core.Application.get()
        self.design = adsk.fusion.Design.cast(self.app.activeProduct)
        if self.design.designIntent == adsk.fusion.DesignIntentTypes.PartDesignIntentType:
            self.design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType    # a part design holds one component only
        self.root = self.design.rootComponent
        self.pal = L.Palette(self.app, self.design)
        self.tof = tof_positions(tof_mode)
        self.tof_mode = tof_mode


def omni_at(travel):
    """Omni axle position when the sprung arm has been pushed up by `travel` cm (the axle moves on an arc about the pivot)."""
    ox, oz = prm.OMNI['rest']
    px, pz = prm.OMNI['pivot']
    zc = oz + travel
    xc = px + math.sqrt((ox - px) ** 2 + (oz - pz) ** 2 - (zc - pz) ** 2)
    return xc, zc


def omni_compressed():
    return omni_at(OMNI_TRAVEL_MECH)


def chute_ends(side):
    """Chute axis from the slot (centre z 7.95, spec) to the exit on the wall (nose z 3.0 = axis end z 3.75). side +1 = left (slot B)."""
    sx, sy = prm.slot_xy('B' if side > 0 else 'A')
    p0 = (sx, sy, prm.PLATE['z0'] - prm.PLATE['floor_t'] - prm.EXIT['bore'] / 2)
    e = (prm.EXIT['x'], side * prm.EXIT['y'], prm.EXIT['z_nose'] + prm.EXIT['bore'] / 2)
    return p0, e


# ---------------------------------------------------------------------------------------------------- chassis
def build_chassis(ctx):
    occ, comp = L.new_part(ctx.root, 'Chassis')
    shell = L.prism(comp, 'Chassis shell', 'xy', [circle((0, 0), R_BODY)], Z_BELLY, Z_RIM)

    def cut(nm, plane, shapes, a0, a1):
        return L.prism(comp, nm, plane, shapes, a0, a1, op=CUT, targets=[shell])

    def join(nm, plane, shapes, a0, a1):
        return L.prism(comp, nm, plane, shapes, a0, a1, op=JOIN, targets=[shell])

    cut('Cavity', 'xy', [circle((0, 0), R_INT)], Z_BELLY + FLOOR_T, Z_RIM + 0.5)
    # rear underside chamfer: from (x -6.5, z 3.5) rising to (x -10.5, z 5.3), cut across the full width
    (cx0, cz0), (cx1, cz1) = prm.CHAMFER
    slope = (cz1 - cz0) / (cx1 - cx0)
    xe = -11.2
    cut('Rear chamfer', 'xz', [poly([(cx0, cz0), (xe, cz0 + slope * (xe - cx0)), (xe, 2.9), (cx0, 2.9)])], -11.0, 11.0)
    # openings in the floor for the wheels and the motors
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = (6.9, 9.1) if s > 0 else (-9.1, -6.9)
        cut('Wheel opening ' + tag, 'xy', [rect(0, (y0 + y1) / 2, 8.4, y1 - y0)], Z_BELLY - 0.1, Z_BELLY + FLOOR_T + 0.1)
        cut('Motor and shaft slot ' + tag, 'xy', [rect(0, s * 5.225, 2.1, 7.75)], Z_BELLY - 0.1, Z_BELLY + FLOOR_T + 0.1)   # y 1.35 .. 9.1: motor, shaft and the wheel opening
    # omni and arm bay (through the floor and the front wall), swept envelope between rest and fully compressed
    ox, oz = prm.OMNI['rest']
    px, pz = prm.OMNI['pivot']
    xc, zc = omni_compressed()
    xm, zm = omni_at(OMNI_TRAVEL_MECH / 2)     # the axle moves on an arc about the pivot, which bulges 2.7 mm ahead of the straight line from rest to compressed
    cut('Omni bay 1', 'xz', [stadium((ox, oz), (xm, zm), prm.OMNI['r'] + 0.15)], -OMNI_BAY_HALF, OMNI_BAY_HALF)
    cut('Omni bay 2', 'xz', [stadium((xm, zm), (xc, zc), prm.OMNI['r'] + 0.15)], -OMNI_BAY_HALF, OMNI_BAY_HALF)
    cut('Arm bay rest', 'xz', [stadium((px, pz), (ox, oz), 0.8)], -OMNI_BAY_HALF, OMNI_BAY_HALF)
    cut('Arm bay mid', 'xz', [stadium((px, pz), (xm, zm), 0.8)], -OMNI_BAY_HALF, OMNI_BAY_HALF)
    cut('Arm bay compressed', 'xz', [stadium((px, pz), (xc, zc), 0.8)], -OMNI_BAY_HALF, OMNI_BAY_HALF)
    # floor-sensor holes
    fp, sm = prm.FLOOR_FRONT, prm.SILVER
    cut('Floor port FP hole', 'xy', [rect(fp['x'], fp['y'], fp['w'] + 0.1, fp['w'] + 0.1)], Z_BELLY - 0.1, Z_BELLY + FLOOR_T + 0.1)
    cut('Silver module SM hole', 'xy', [rect(sm['x'], sm['y'], sm['l'] + 0.1, sm['w'] + 0.1)], Z_BELLY - 0.1, Z_BELLY + FLOOR_T + 0.1)
    # chute holes through floor and wall (sloped tool cylinders)
    for s, tag in ((1, 'B left'), (-1, 'A right')):
        p0, e = chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        tool = L.axis_tool(comp, 'Chute hole tool ' + tag, p0, L.vadd(e, L.vmul(d, 0.5)), prm.EXIT['bore'] / 2 + prm.EXIT['wall'])
        L.combine(comp, shell, [tool])
    # ToF windows through the wall, one per module, along its aim
    for nm, (x, y, aim) in ctx.tof.items():
        cut('ToF window ' + nm, 'xy', [tof_rect(x, y, aim, 0.0, t_exit(x, y, aim), 1.8)], 9.1, 10.9)
    # camera windows, about 2 x 2 cm [spec]; 2.3 x 2.2 here so the 65.9 degree view is not cropped at 1.7 cm from the lens
    cam_x = prm.CAM['tip_r'] * math.cos(math.radians(prm.CAM['psi']))
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = (8.9, 10.8) if s > 0 else (-10.8, -8.9)
        cut('Camera window ' + tag, 'xz', [rect(cam_x, 8.6, CAM_WINDOW_W, CAM_WINDOW_H)], y0, y1)     # z 7.4 .. 9.8: the lowest view ray leaves at z 7.55
    # bumper recess: the body behind the plate is recessed by the 4 mm travel (plate inner face 10.75, body 10.35), wall kept 0.2 thick
    bp = prm.BUMPER
    r_in_plate = bp['r_out'] - bp['t']
    r_body = r_in_plate - 0.4
    for s, tag in ((1, 'L'), (-1, 'R')):
        a0, a1 = (bp['a0'] - 4, bp['a1'] + 4) if s > 0 else (-bp['a1'] - 4, -bp['a0'] + 4)
        cut('Bumper recess ' + tag, 'xy', [sector(r_body, R_BODY + 0.3, a0, a1)], bp['z0'], bp['z1'])
        join('Bumper recess wall ' + tag, 'xy', [sector(r_body - 0.2, R_INT + 0.02, a0, a1)], bp['z0'] - 0.1, bp['z1'] + 0.1)
        a = math.radians(s * prm.BUMPER_SW_DEG)
        rc = 10.225
        cut('Bumper switch pocket ' + tag, 'xy', [rect(rc * math.cos(a), rc * math.sin(a), 0.35, 0.55, s * prm.BUMPER_SW_DEG)], 4.65, 5.85)
    # handle: arch on the chassis [not in spec; spec only says the handle is fixed to the chassis, not the lid]
    for s in (1, -1):
        join('Handle post %s' % ('L' if s > 0 else 'R'), 'xy', [circle((HANDLE_X, s * HANDLE_Y), 0.4)], Z_RIM - 0.05, HANDLE_Z)
    join('Handle bar', 'xz', [circle((HANDLE_X, HANDLE_Z), 0.4)], -HANDLE_Y, HANDLE_Y)
    ctx.pal.paint(shell, '#C8C6BE', 0.45)
    return occ


# ---------------------------------------------------------------------------------------------------- drivetrain
def build_drive(ctx):
    for s, tag in ((1, 'L'), (-1, 'R')):
        # drive wheel: 80 mm, 20 mm wide, centre z 4.0 (spec)
        occ, comp = L.new_part(ctx.root, 'Wheel ' + tag)
        y0, y1 = (prm.WHEEL['y0'], prm.WHEEL['y1']) if s > 0 else (-prm.WHEEL['y1'], -prm.WHEEL['y0'])
        w = L.prism(comp, 'Wheel ' + tag, 'xz', [circle((0, prm.AXLE_Z), prm.WHEEL['r'])], y0, y1)
        L.prism(comp, 'Shaft bore', 'xz', [circle((0, prm.AXLE_Z), 0.15)], y0 - 0.1, y1 + 0.1, op=CUT, targets=[w])
        ctx.pal.paint(w, '#2C2C2A')
        # motor: Pololu 20D 44L gearmotor with 3 mm encoder, occupying y 1.4 .. 6.1 (spec); shaft out of the gearbox face into the wheel
        occ, comp = L.new_part(ctx.root, 'Motor ' + tag)
        yf = prm.MOTOR['y_face']
        yenc0 = yf - prm.MOTOR['length'] - prm.MOTOR['enc']
        yenc1 = yf - prm.MOTOR['length']
        sgn = (lambda a, b: (a, b)) if s > 0 else (lambda a, b: (-b, -a))
        enc = L.prism(comp, 'Encoder ' + tag, 'xz', [circle((0, prm.AXLE_Z), prm.MOTOR['r'] - 0.1)], *sgn(yenc0, yenc1))
        mot = L.prism(comp, 'Gearmotor ' + tag, 'xz', [circle((0, prm.AXLE_Z), prm.MOTOR['r'])], *sgn(yenc1, yf))
        sh = L.prism(comp, 'Motor shaft ' + tag, 'xz', [circle((0, prm.AXLE_Z), 0.15)], *sgn(yf, 8.0))
        ctx.pal.paint(mot, '#888780')
        ctx.pal.paint(enc, '#0F6E56')
        ctx.pal.paint(sh, '#C8C6BE')


def build_front_support(ctx):
    ox, oz = prm.OMNI['rest']
    px, pz = prm.OMNI['pivot']
    r, hw = prm.OMNI['r'], prm.OMNI['w'] / 2
    # omni wheel, simplified solid with hub and bore; drawn in the rest (flat-floor) pose
    occ, comp = L.new_part(ctx.root, 'Omni wheel')
    wh = L.prism(comp, 'Omni wheel', 'xz', [circle((ox, oz), r)], -hw, hw)
    hub = L.prism(comp, 'Omni hub', 'xz', [circle((ox, oz), 1.0)], -hw - 0.1, hw + 0.1, op=JOIN, targets=[wh])
    L.prism(comp, 'Axle bore', 'xz', [circle((ox, oz), 0.2)], -hw - 0.2, hw + 0.2, op=CUT, targets=[wh])
    ctx.pal.paint(wh, '#1D9E75')
    # sprung arm: two plates either side of the wheel (fork half width 1.5), pivot at (3.5, 3.6)
    occ, comp = L.new_part(ctx.root, 'Omni arm')
    bodies = []
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = (hw + 0.1, prm.OMNI['fork_half']) if s > 0 else (-prm.OMNI['fork_half'], -hw - 0.1)
        pl = L.prism(comp, 'Arm plate ' + tag, 'xz', [stadium((px, pz), (ox, oz), 0.55)], y0, y1)
        L.prism(comp, 'Pivot hole ' + tag, 'xz', [circle((px, pz), 0.2)], y0 - 0.1, y1 + 0.1, op=CUT, targets=[pl])
        L.prism(comp, 'Axle hole ' + tag, 'xz', [circle((ox, oz), 0.2)], y0 - 0.1, y1 + 0.1, op=CUT, targets=[pl])
        bodies.append(pl)
    ctx.pal.paint(bodies, '#0F6E56')
    occ, comp = L.new_part(ctx.root, 'Omni pins')
    pv = L.prism(comp, 'Pivot pin', 'xz', [circle((px, pz), 0.2)], -prm.OMNI['fork_half'] - 0.2, prm.OMNI['fork_half'] + 0.2)
    ax = L.prism(comp, 'Axle pin', 'xz', [circle((ox, oz), 0.2)], -prm.OMNI['fork_half'], prm.OMNI['fork_half'])
    ctx.pal.paint([pv, ax], '#5F5E5A')


def build_rear_nub(ctx):
    occ, comp = L.new_part(ctx.root, 'Rear nub')
    nub = L.prism(comp, 'Rear nub', 'xy', [circle((prm.NUB['x'], 0.0), prm.NUB['r'])], prm.NUB['z_low'], 4.6)
    (cx0, cz0), (cx1, cz1) = prm.CHAMFER
    slope = (cz1 - cz0) / (cx1 - cx0)
    xe = -11.2
    L.prism(comp, 'Seat on chamfer', 'xz', [poly([(cx0, cz0), (xe, cz0 + slope * (xe - cx0)), (xe, 7.0), (cx0, 7.0)])], -1.0, 1.0, op=CUT, targets=[nub])
    ctx.pal.paint(nub, '#5F5E5A')


# ---------------------------------------------------------------------------------------------------- bumpers
def build_bumpers(ctx):
    bp = prm.BUMPER
    for s, tag in ((1, 'L'), (-1, 'R')):
        a0, a1 = (bp['a0'], bp['a1']) if s > 0 else (-bp['a1'], -bp['a0'])
        occ, comp = L.new_part(ctx.root, 'Bumper ' + tag)
        b = L.prism(comp, 'Bumper plate ' + tag, 'xy', [sector(bp['r_out'] - bp['t'], bp['r_out'], a0, a1)], bp['z0'], bp['z1'])
        ctx.pal.paint(b, '#D85A30')
        occ, comp = L.new_part(ctx.root, 'Bumper switch ' + tag)
        a = math.radians(s * prm.BUMPER_SW_DEG)
        rc = 10.425
        sw = L.prism(comp, 'Microswitch ' + tag, 'xy', [rect(rc * math.cos(a), rc * math.sin(a), 0.65, 0.45, s * prm.BUMPER_SW_DEG)], 4.75, 5.75)
        ctx.pal.paint(sw, '#222222')


# ---------------------------------------------------------------------------------------------------- ToF ring and sensors
def build_tof(ctx):
    occ, comp = L.new_part(ctx.root, 'ToF ring')
    ring = L.ring_prism(comp, 'ToF ring', 'xy', circle((0, 0), R_INT), circle((0, 0), RING_RIN), RING_Z0, RING_Z1)
    t_half = prm.TOF_T / 2 + 0.06
    for nm, (x, y, aim) in ctx.tof.items():
        L.prism(comp, 'Pocket ' + nm, 'xy', [tof_rect(x, y, aim, -t_half, t_half, prm.TOF_W + 0.12)], prm.TOF_Z - prm.TOF_H / 2 - 0.05, prm.TOF_Z + prm.TOF_H / 2 + 0.05, op=CUT, targets=[ring])
        L.prism(comp, 'Beam ' + nm, 'xy', [tof_rect(x, y, aim, 0.0, t_exit(x, y, aim), 1.8)], 9.1, 10.9, op=CUT, targets=[ring])
    cam_x = prm.CAM['tip_r'] * math.cos(math.radians(prm.CAM['psi']))
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = (8.4, 10.8) if s > 0 else (-10.8, -8.4)     # starts inside the ring bore (r 9.0) so no sliver is left at the window sides
        # the tilted lens block reaches r 9.14 and z 10.24 at its upper front edge, so the ring is opened up to z 10.6 there
        L.prism(comp, 'Camera window ' + tag, 'xz', [rect(cam_x, (RING_Z0 - 0.1 + 10.6) / 2, CAM_WINDOW_W, 10.6 - (RING_Z0 - 0.1))], y0, y1, op=CUT, targets=[ring])
    ctx.pal.paint(ring, '#B4B2A9', 0.55)
    for nm, (x, y, aim) in ctx.tof.items():
        occ, comp = L.new_part(ctx.root, 'ToF ' + nm)
        z0, z1 = prm.TOF_Z - prm.TOF_H / 2, prm.TOF_Z + prm.TOF_H / 2
        mod = L.prism(comp, 'VL53L0X module ' + nm, 'xy', [rect(x, y, prm.TOF_T, prm.TOF_W, aim)], z0, z1)
        a = math.radians(aim)
        off = prm.TOF_T / 2 + 0.05
        chip = L.prism(comp, 'VL53L0X chip ' + nm, 'xy', [rect(x + off * math.cos(a), y + off * math.sin(a), 0.1, 0.44, aim)], prm.TOF_Z - 0.12, prm.TOF_Z + 0.12)
        ctx.pal.paint(mod, '#378ADD')
        ctx.pal.paint(chip, '#042C53')


# ---------------------------------------------------------------------------------------------------- cameras
def build_cameras(ctx):
    t = math.radians(prm.CAM['tilt'])
    psi = math.radians(prm.CAM['psi'])
    bw, bh = prm.CAM['board']
    ld, lw, lh = prm.CAM['lens']
    for s, tag in ((1, 'L'), (-1, 'R')):
        tip = (prm.CAM['tip_r'] * math.cos(psi), s * prm.CAM['tip_r'] * math.sin(psi), prm.CAM['zl'])
        ez = (0.0, -s * math.cos(t), math.sin(t))      # from the lens tip back towards the board (the view direction is -ez: outwards and 20 degrees down)
        occ, comp = L.new_part(ctx.root, 'Camera ' + tag, L.frame_from_zy(tip, ez, (0.0, 0.0, 1.0)))
        lens = L.prism(comp, 'Lens block ' + tag, 'xy', [rect(0, 0, lw, lh)], 0.0, ld)
        hold = L.prism(comp, 'Lens holder ' + tag, 'xy', [rect(0, 0, lw, lh)], ld, 2.5, op=JOIN, targets=[lens])
        board = L.prism(comp, 'OpenMV H7 Plus board ' + tag, 'xy', [rect(0, 0, bw, bh)], 2.5, 3.1)
        ctx.pal.paint(lens, '#3C3489')
        ctx.pal.paint(board, '#7F77DD')


# ---------------------------------------------------------------------------------------------------- lid
def build_lid(ctx):
    occ, comp = L.new_part(ctx.root, 'Lid')
    lid = L.prism(comp, 'Lid', 'xy', [circle((0, 0), R_BODY)], Z_RIM, Z_ROOF)
    cam_x = prm.CAM['tip_r'] * math.cos(math.radians(prm.CAM['psi']))
    for s in (1, -1):
        y0, y1 = (5.3, 9.3) if s > 0 else (-9.3, -5.3)
        L.prism(comp, 'Camera hump', 'xy', [rect(cam_x, (y0 + y1) / 2, 5.0, y1 - y0)], Z_ROOF - 0.05, Z_LID + HUMP_SKIN, op=JOIN, targets=[lid])
    for s in (1, -1):
        y0, y1 = (5.5, 9.1) if s > 0 else (-9.1, -5.5)
        L.prism(comp, 'Camera pocket', 'xy', [rect(cam_x, (y0 + y1) / 2, 4.6, y1 - y0)], Z_RIM - 0.1, Z_LID, op=CUT, targets=[lid])
        L.prism(comp, 'Handle notch', 'xy', [circle((HANDLE_X, s * HANDLE_Y), 0.55)], Z_RIM - 0.1, Z_ROOF + 0.1, op=CUT, targets=[lid])
    ctx.pal.paint(lid, '#F1EFE8', 0.55)
    occ, comp = L.new_part(ctx.root, 'Start button')
    b = L.prism(comp, 'Start button', 'xy', [circle((-7.0, 0.0), 0.9)], Z_ROOF, Z_ROOF + 0.5)
    ctx.pal.paint(b, '#639922')
    occ, comp = L.new_part(ctx.root, 'Victim LED')
    d = L.prism(comp, 'Victim LED', 'xy', [circle((-7.0, -3.0), 0.35)], Z_ROOF, Z_ROOF + 0.35)
    ctx.pal.paint(d, '#E24B4A')


# ---------------------------------------------------------------------------------------------------- dropper
def build_dropper(ctx):
    pl = prm.PLATE
    cx, cy, Rp = pl['cx'], pl['cy'], pl['R']
    z_floor0, z_floor1 = pl['z0'] - pl['floor_t'], pl['z0']
    occ, comp = L.new_part(ctx.root, 'Dropper floor')
    fl = L.prism(comp, 'Dropper floor', 'xy', [circle((cx, cy), Rp + 0.2)], z_floor0, z_floor1)
    for nm in 'AB':
        sx, sy = prm.slot_xy(nm)
        L.prism(comp, 'Slot ' + nm, 'xy', [rect(sx, sy, pl['slot'], pl['slot'], 45)], z_floor0 - 0.1, z_floor1 + 0.1, op=CUT, targets=[fl])
    L.prism(comp, 'Shaft hole', 'xy', [circle((cx, cy), 0.2)], z_floor0 - 0.1, z_floor1 + 0.1, op=CUT, targets=[fl])
    ctx.pal.paint(fl, '#854F0B')
    occ, comp = L.new_part(ctx.root, 'Dropper plate')
    plate = L.prism(comp, 'Dropper plate', 'xy', [circle((cx, cy), Rp)], pl['z0'], pl['z0'] + pl['t'])
    for i in range(1, pl['n'] + 1):
        x, y, a = prm.pocket_xy(i)
        L.prism(comp, 'Pocket %d' % i, 'xy', [rect(x, y, pl['pocket'], pl['pocket'], a)], pl['z0'] - 0.1, pl['z0'] + pl['t'] + 0.1, op=CUT, targets=[plate])
    L.prism(comp, 'Shaft bore', 'xy', [circle((cx, cy), 0.15)], pl['z0'] - 0.1, pl['z0'] + pl['t'] + 0.1, op=CUT, targets=[plate])
    ctx.pal.paint(plate, '#BA7517')
    occ, comp = L.new_part(ctx.root, 'N20 motor')
    nz0 = z_floor0 - pl['n20_len']
    n20 = L.prism(comp, 'N20 gearmotor with encoder', 'xy', [rect(cx, cy, pl['n20_w'][0], pl['n20_w'][1])], nz0, z_floor0)
    shaft = L.prism(comp, 'N20 shaft', 'xy', [circle((cx, cy), 0.15)], z_floor0, pl['z0'] + pl['t'] - 0.4)
    ctx.pal.paint(n20, '#888780')
    ctx.pal.paint(shaft, '#C8C6BE')
    occ, comp = L.new_part(ctx.root, 'Kits')
    kit = 1.03
    kits = []
    for i in range(1, pl['n'] + 1):
        x, y, a = prm.pocket_xy(i)
        kits.append(L.prism(comp, 'Kit %d' % i, 'xy', [rect(x, y, kit, kit, a)], pl['z0'], pl['z0'] + kit))
    ctx.pal.paint(kits, '#444441')
    # chutes: 15 mm bore, 1.5 mm wall, straight from the slot to the exit on the wall; the exit is trimmed flush with the body radius
    for s, tag in ((1, 'B left'), (-1, 'A right')):
        p0, e = chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        occ, comp = L.new_part(ctx.root, 'Chute ' + tag)
        tube = L.axis_tool(comp, 'Chute tube ' + tag, p0, L.vadd(e, L.vmul(d, 0.6)), pl_bore_outer())
        bore = L.axis_tool(comp, 'Chute bore tool', L.vsub(p0, L.vmul(d, 0.1)), L.vadd(e, L.vmul(d, 0.7)), prm.EXIT['bore'] / 2)
        L.combine(comp, tube, [bore])
        trim = L.ring_prism(comp, 'Exit trim tool', 'xy', circle((0, 0), 14.0), circle((0, 0), R_BODY), 0.0, 9.0)
        L.combine(comp, tube, [trim])
        ctx.pal.paint(tube, '#993C1D')


def pl_bore_outer():
    return prm.EXIT['bore'] / 2 + prm.EXIT['wall']


# ---------------------------------------------------------------------------------------------------- electronics and floor modules
def build_electronics(ctx):
    parts, _ = prm.load_pack()
    g = parts['GIGA R1 + main PCB stack']
    b = parts['battery (placeholder 7.0x3.5x2.5)']
    gl, gw = prm.PART_SIZES['GIGA R1 + main PCB stack']
    # stack z 6.15 .. 8.05 [placeholder: PCB not designed]: board, two female header strips, shield board, component envelope
    occ, comp = L.new_part(ctx.root, 'Arduino GIGA R1')
    z = g['z0']
    board = L.prism(comp, 'GIGA board', 'xy', [rect(g['x'], g['y'], gl, gw, g['angle'])], z, z + 0.16)
    hdr = []
    for s in (1, -1):
        hdr.append(L.prism(comp, 'Header strip %s' % ('+y' if s > 0 else '-y'), 'xy', [rect(g['x'], g['y'] + s * (gw / 2 - 0.25), gl - 0.8, 0.25, g['angle'])], z + 0.16, z + 1.01))
    ctx.pal.paint(board, '#0F6E56')
    ctx.pal.paint(hdr, '#222222')
    occ, comp = L.new_part(ctx.root, 'Main PCB')
    z2 = z + 1.01
    pcb = L.prism(comp, 'Main PCB shield (placeholder)', 'xy', [rect(g['x'], g['y'], gl, gw, g['angle'])], z2, z2 + 0.16)
    env = L.prism(comp, 'PCB component envelope (placeholder)', 'xy', [rect(g['x'], g['y'], gl - 0.6, gw - 0.6, g['angle'])], z2 + 0.16, g['z1'])
    ctx.pal.paint(pcb, '#1D9E75')
    ctx.pal.paint(env, '#04342C', 0.3)
    occ, comp = L.new_part(ctx.root, 'Battery')
    bat = L.prism(comp, 'Battery (placeholder 7.0 x 3.5 x 2.5)', 'xy', [rect(b['x'], b['y'], 7.0, 3.5, b['angle'])], b['z0'], b['z1'])
    ctx.pal.paint(bat, '#EF9F27')
    fp, sm = prm.FLOOR_FRONT, prm.SILVER
    occ, comp = L.new_part(ctx.root, 'Floor port FP')
    f1 = L.prism(comp, 'Floor sensor FP (black, blue, red)', 'xy', [rect(fp['x'], fp['y'], fp['w'], fp['w'])], fp['z_face'], fp['z_face'] + 1.4)
    ctx.pal.paint(f1, '#F0997B')
    occ, comp = L.new_part(ctx.root, 'Silver module SM')
    f2 = L.prism(comp, 'Silver module SM (straight + 20 deg tilted pair)', 'xy', [rect(sm['x'], sm['y'], sm['l'], sm['w'])], sm['z_face'], sm['z_face'] + 1.4)
    ctx.pal.paint(f2, '#F0997B')


# ---------------------------------------------------------------------------------------------------- checks on the finished model
def occ_name(entity):
    ctxo = entity.assemblyContext
    return ctxo.component.name if ctxo else entity.name


def verify(ctx, interference=True):
    root = ctx.root
    occs = [root.occurrences.item(i) for i in range(root.occurrences.count)]
    lo, hi = [1e9] * 3, [-1e9] * 3
    rows = []
    for o in occs:
        l, h = L.world_bbox(o)
        for k in range(3):
            lo[k] = min(lo[k], l[k])
            hi[k] = max(hi[k], h[k])
        rmax, vol = 0.0, 0.0
        for i in range(o.bRepBodies.count):
            b = o.bRepBodies.item(i)
            vol += b.volume
            calc = b.meshManager.createMeshCalculator()
            calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
            nc = calc.calculate().nodeCoordinatesAsDouble
            for k in range(0, len(nc), 3):
                rmax = max(rmax, math.hypot(nc[k], nc[k + 1]))
        rows.append((o.component.name, rmax, l, h, vol))
    print('model bbox (cm): x %.2f .. %.2f, y %.2f .. %.2f, z %.2f .. %.2f' % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    print('components', len(occs), ' farthest radius from the axle per component (swept circle %.1f, body %.1f):' % (R_SWEPT, R_BODY))
    for nm, r, l, h, vol in sorted(rows, key=lambda t: -t[1])[:22]:
        print('  %-22s r_max %6.3f  z %5.2f .. %5.2f  vol %7.2f' % (nm, r, l[2], h[2], vol))
    if not interference:
        return 'ok'
    ents = adsk.core.ObjectCollection.create()
    for o in occs:
        ents.add(o)
    ii = ctx.design.createInterferenceInput(ents)
    ii.areCoincidentFacesConsideredInterference = False
    res = ctx.design.analyzeInterference(ii)
    out = []
    if res is not None:
        for i in range(res.count):
            r = res.item(i)
            out.append((r.interferenceBody.volume, occ_name(r.entityOne), occ_name(r.entityTwo)))
    print('interfering pairs:', len(out))
    for v, a, b in sorted(out, reverse=True):
        print('  %8.4f cm3  %s  x  %s' % (v, a, b))
    return 'ok'


# ---------------------------------------------------------------------------------------------------- driver
def show_only(ctx, hide=()):
    """Hide the named components (everything else shown)."""
    for i in range(ctx.root.occurrences.count):
        o = ctx.root.occurrences.item(i)
        o.isLightBulbOn = o.component.name not in hide
    return 'ok'


STAGES = [('chassis', build_chassis), ('drive', build_drive), ('front', build_front_support), ('nub', build_rear_nub), ('bumpers', build_bumpers),
          ('tof', build_tof), ('cameras', build_cameras), ('lid', build_lid), ('dropper', build_dropper), ('electronics', build_electronics)]


def build(stages=None, reset=False, tof_mode=TOF_MODE):
    ctx = Ctx(tof_mode)
    if reset:
        L.reset_design(ctx.design)
    names = [s[0] for s in STAGES]
    wanted = names if stages in (None, 'all') else (stages if isinstance(stages, (list, tuple)) else [stages])
    for nm, fn in STAGES:
        if nm in wanted:
            fn(ctx)
            print('stage', nm, 'done; timeline', ctx.design.timeline.count)
    return 'ok'
