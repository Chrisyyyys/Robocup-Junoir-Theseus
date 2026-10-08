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
