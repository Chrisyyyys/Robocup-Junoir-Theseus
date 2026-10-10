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
import tof_mount as T

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
    ctx.tof = {nm: (x, y, aim) for nm, x, y, aim in P.TOF}      # 9 Oct: the rev 4 positions (the wider real board moved the front-left, side and rear sensors), not the rev 3 'fixed' ones
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
    # the floor opening for the omni, 9 Oct: two plan cuts through the floor only (the stadium cuts of 8 Oct, extruded over the whole height, nicked the front wall and the +12 degree frame boss): the wheel's
    # opening (y -1.13 to 1.73, x 3.65 to 10.1) and the arm's corridor (y 0.75 to 2.10, x 1.66 to 8.0), which holds the spring coil, the arm and the stop screw. Pillar, tail and ear stand on the floor beside them (spec 4.1)
    ox, oz = P.OMNI['rest']
    px, pz = P.OMNI['pivot']
    OM = P.OMNI_MOUNT
    Op = P.OMNI_OPENING
    for nm, o in (('Omni wheel opening', Op['wheel']), ('Omni arm corridor', Op['arm'])):
        cut(nm, 'xy', [rect((o['x'][0] + o['x'][1]) / 2, (o['y'][0] + o['y'][1]) / 2, o['x'][1] - o['x'][0], o['y'][1] - o['y'][0])], P.Z_BELLY - 0.1, zf + 0.1)
    for nm, key in (('Omni pillar', 'pillar'), ('Omni pillar tail', 'tail'), ('Omni ear', 'ear')):
        bx = OM[key]
        join(nm, 'xy', [rect((bx['x'][0] + bx['x'][1]) / 2, (bx['y'][0] + bx['y'][1]) / 2, bx['x'][1] - bx['x'][0], bx['y'][1] - bx['y'][0])], zf - 0.05, bx['z'][1])
    stx, stz = P.omni_stop_pin()
    cut('Pivot hole pillar', 'xz', [circle((px, pz), OM['hole_r'])], OM['pillar']['y'][0] - 0.1, OM['pillar']['y'][1] + 0.1)
    cut('Pivot thread hole ear', 'xz', [circle((px, pz), P.OMNI_PIVOT['screw_r'])], OM['ear']['y'][0] - 0.05, OM['ear']['y'][0] + P.OMNI_PIVOT['hole_depth'])    # the M3 insert's thread and the blind hole behind it, modelled at the screw's radius
    cut('Stop hole pillar', 'xz', [circle((stx, stz), OM['stop']['hole_r'])], OM['pillar']['y'][0] - 0.1, OM['pillar']['y'][1] + 0.1)
    cut('Stop thread hole ear', 'xz', [circle((stx, stz), OM['stop']['pin_r'])], OM['ear']['y'][0] - 0.05, OM['ear']['y'][0] + OM['stop']['ear_depth'])         # pilot hole 2.6 mm, the screw cuts its thread: modelled at the screw's radius
    cut('Adjuster hole tail', 'xy', [circle((OM['adjuster']['x'], OM['adjuster']['y']), OM['adjuster']['shank_r'])], OM['adjuster']['head_z'][0] - OM['adjuster']['bolt_len'], OM['tail']['z'][1] + 0.1)    # the M3 x 5.7 insert's thread, modelled at the bolt's radius
    fp, sm = P.FLOOR_FRONT, P.SILVER
    cut('Floor port FP hole', 'xy', [rect(fp['x'], fp['y'], fp['w'] + 0.1, fp['w'] + 0.1)], P.Z_BELLY - 0.1, zf + 0.1)
    cut('Silver module SM hole', 'xy', [rect(sm['x'], sm['y'], sm['l'] + 0.1, sm['w'] + 0.1)], P.Z_BELLY - 0.1, zf + 0.1)
    # square chute holes through the wall (the channel crosses the cylinder obliquely, so the hole is cut with the channel's own profile)
    w_hole = P.CHUTE['out_w'] + 2 * P.CHUTE['hole_clear']
    lift = P.CHUTE['lift']                            # the channel's square section is centred this far above the axis (the floor stays 8 mm below it)
    for s, tag in ((1, 'B left'), (-1, 'A right')):
        p0, e = P.chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        tool = L.axis_prism(comp, 'Chute hole tool ' + tag, rect(0, lift, w_hole, w_hole), p0, L.vadd(e, L.vmul(d, 0.8)))
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
    I = P.IMU                                                  # four posts for the BNO055 breakout, 5 mm tall, with pilot holes for M2.5 self-tapping screws (square in the model)
    posts = [(I['x'] + sx * I['holes'][0], I['y'] + sy * I['holes'][1]) for sx in (1, -1) for sy in (1, -1)]
    join('IMU posts', 'xy', [rect(px, py, I['post'], I['post']) for px, py in posts], zf - 0.05, zf + I['post_h'])
    cut('IMU post pilots', 'xy', [rect(px, py, I['pilot'], I['pilot']) for px, py in posts], zf + I['post_h'] - 0.45, zf + I['post_h'] + 0.1)
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


def _stop_slot_polygon():
    """The arm's arc slot at the rest position, as a world polygon in the x-z plane: a capsule about the arc of radius r from the compression-stop end to the rest-stop end, round ends of the slot's half width."""
    px, pz = P.OMNI['pivot']
    ox, oz = P.OMNI['rest']
    sl = P.omni_stop_slot()
    phi = math.atan2(oz - pz, ox - px)
    r, hw = sl['r'], sl['hw']
    lo, hi = math.radians(sl['a_lo']), math.radians(sl['a_hi'])

    def pt(rad, ang):
        return (px + rad * math.cos(phi + ang), pz + rad * math.sin(phi + ang))

    n = 10
    pts = [pt(r + hw, lo + (hi - lo) * k / n) for k in range(n + 1)]
    for k in range(1, 8):                                      # round end at the rest-stop end (outer side -> tangent direction -> inner side)
        s_ = math.pi * k / 8
        cx, cz = pt(r, hi)
        er, et = (math.cos(phi + hi), math.sin(phi + hi)), (-math.sin(phi + hi), math.cos(phi + hi))
        pts.append((cx + hw * (math.cos(s_) * er[0] + math.sin(s_) * et[0]), cz + hw * (math.cos(s_) * er[1] + math.sin(s_) * et[1])))
    pts += [pt(r - hw, hi - (hi - lo) * k / n) for k in range(n + 1)]
    for k in range(1, 8):                                      # round end at the compression-stop end (inner -> minus tangent -> outer)
        s_ = math.pi * k / 8
        cx, cz = pt(r, lo)
        er, et = (math.cos(phi + lo), math.sin(phi + lo)), (-math.sin(phi + lo), math.cos(phi + lo))
        pts.append((cx - hw * (math.cos(s_) * er[0] + math.sin(s_) * et[0]), cz - hw * (math.cos(s_) * er[1] + math.sin(s_) * et[1])))
    return pts


@stage('omni')
def build_omni(ctx):
    """Front omni wheel and its mount (spec 4.1): the Nexus 14145 on two 604 bearings and an M4 axle screw tapped into ONE 3 x 12 mm aluminium arm on the +y side; the arm turns on a tube on the M4 pivot screw
    and carries an arc slot for the fixed M3 stop screw; a torsion spring on the tube, its rear leg on an M3 adjuster screw in the pillar's tail. Moving in the sweep: wheel, arm, axle set."""
    O, W, B, X, V, SP = P.OMNI, P.OMNI_WHEEL, P.OMNI_BEARING, P.OMNI_AXLE, P.OMNI_PIVOT, P.OMNI_SPRING
    OM, S = P.OMNI_MOUNT, P.OMNI_MOUNT['stop']
    ox, oz = O['rest']
    px, pz = O['pivot']
    w0, w1 = O['y_wheel']
    a0, a1 = O['arm_y']
    # wheel: a solid cylinder (the rollers are not modelled), the 12 mm hole, a 604 bearing flush with each face
    occ, comp = L.new_part(ctx.root, 'Omni wheel')
    wh = L.prism(comp, 'Omni wheel', 'xz', [circle((ox, oz), O['r'])], w0, w1)
    L.prism(comp, 'Wheel bore', 'xz', [circle((ox, oz), W['bore_r'])], w0 - 0.1, w1 + 0.1, op=CUT, targets=[wh])
    y_b1 = w0 + X['seat']                                      # the inboard bearing is seated 5 mm in: the screw's head and its washer sit in front of it, inside the bore
    b1 = L.ring_prism(comp, 'Bearing inboard', 'xz', circle((ox, oz), B['od_r']), circle((ox, oz), B['bore_r']), y_b1, y_b1 + B['w'])
    b2 = L.ring_prism(comp, 'Bearing outboard', 'xz', circle((ox, oz), B['od_r']), circle((ox, oz), B['bore_r']), w1 - B['w'], w1)
    sl = L.ring_prism(comp, 'Axle sleeve', 'xz', circle((ox, oz), X['sleeve_r'][1]), circle((ox, oz), X['sleeve_r'][0]), y_b1 + B['w'], w1 - B['w'])
    sp = L.ring_prism(comp, 'Axle spacer', 'xz', circle((ox, oz), X['sleeve_r'][1]), circle((ox, oz), X['sleeve_r'][0]), w1, a0)
    ctx.pal.paint(wh, '#1D9E75')
    ctx.pal.paint([b1, b2], '#8A8A84')
    ctx.pal.paint([sl, sp], '#B7B6B0')
    # the arm: a 3 x 12 mm bar, 6 mm pivot bore for the tube, M4 axle hole, the arc slot for the stop screw
    occ, comp = L.new_part(ctx.root, 'Omni arm')
    arm = L.prism(comp, 'Arm bar', 'xz', [stadium((px, pz), (ox, oz), O['arm_half'])], a0, a1)
    L.prism(comp, 'Pivot bore', 'xz', [circle((px, pz), O['pivot_hole_r'])], a0 - 0.1, a1 + 0.1, op=CUT, targets=[arm])
    L.prism(comp, 'Axle hole', 'xz', [circle((ox, oz), O['axle_r'])], a0 - 0.1, a1 + 0.1, op=CUT, targets=[arm])
    L.prism(comp, 'Stop slot', 'xz', [poly(_stop_slot_polygon())], a0 - 0.1, a1 + 0.1, op=CUT, targets=[arm])
    # the spring's seat (10 Oct): a 2 mm stainless pin in a 2.0 mm hole, 3.5 mm standing out of the arm's inboard face and 2.5 mm in the bar, and on it the forward spring leg, a wire that lies on the pin's top, runs
    # along the arm splayed 4 degrees up from it and turns with it (so it is part of this moving component here, though it is the spring's)
    K = P.OMNI_SEAT
    kx, kz = P.omni_seat(0.0)
    L.prism(comp, 'Seat hole', 'xz', [circle((kx, kz), K['hole_r'])], a0 - 0.1, a1 + 0.1, op=CUT, targets=[arm])
    seat = L.prism(comp, 'Seat pin', 'xz', [circle((kx, kz), K['r'])], K['y'][0], K['y'][1])
    (lx0, lz0), (lx1, lz1) = P.omni_leg(0.0)
    leg_f = L.prism(comp, 'Spring forward leg', 'xz', [rect((lx0 + lx1) / 2, (lz0 + lz1) / 2, math.hypot(lx1 - lx0, lz1 - lz0), SP['wire'], math.degrees(math.atan2(lz1 - lz0, lx1 - lx0)))],
                    SP['y'][1] - SP['wire'], SP['y'][1])
    ctx.pal.paint(arm, '#0F6E56')
    ctx.pal.paint([seat, leg_f], '#B4B2A9')
    # axle set: the M4 x 25 screw with its head and 0.5 mm washer in the wheel's bore (head underside 0.5 mm in front of the inboard bearing), tip 0.3 mm inside the arm's outer face
    occ, comp = L.new_part(ctx.root, 'Omni pins')
    y_under = w0 + X['seat'] - X['washer_h']
    ax = L.prism(comp, 'Axle screw', 'xz', [circle((ox, oz), O['axle_r'])], y_under, y_under + X['screw_len'])
    hd = L.prism(comp, 'Axle screw head', 'xz', [circle((ox, oz), X['head_r'])], y_under - X['head_h'], y_under)
    wa = L.ring_prism(comp, 'Axle washer', 'xz', circle((ox, oz), X['washer_r']), circle((ox, oz), X['sleeve_r'][0]), y_under, w0 + X['seat'])
    ctx.pal.paint([ax, hd, wa], '#5F5E5A')
    # pivot hardware (fixed): the M4 pivot screw from the pillar's inboard face into the ear's insert, the M3 stop screw alongside it
    occ, comp = L.new_part(ctx.root, 'Omni pivot')
    py0 = OM['pillar']['y'][0]
    pv = L.prism(comp, 'Pivot screw', 'xz', [circle((px, pz), V['screw_r'])], py0, py0 + V['screw_len'])
    pvh = L.prism(comp, 'Pivot screw head', 'xz', [circle((px, pz), V['head_r'])], py0 - V['head_h'], py0)
    stx, stz = P.omni_stop_pin()
    st = L.prism(comp, 'Stop screw', 'xz', [circle((stx, stz), S['pin_r'])], py0, py0 + S['screw_len'])
    sth = L.prism(comp, 'Stop screw head', 'xz', [circle((stx, stz), S['head_r'])], py0 - S['head_h'], py0)
    ctx.pal.paint([pv, pvh, st, sth], '#5F5E5A')
    # the spring on its tube: the tube (5 x 3.1 mm, pillar to ear), the printed arbor on it, the coil on the arbor and the rear leg, a straight wire that lies on the adjuster head; the forward leg is in the arm's
    # component (it turns with the arm)
    occ, comp = L.new_part(ctx.root, 'Omni spring')
    tube = L.ring_prism(comp, 'Pivot tube', 'xz', circle((px, pz), V['tube_r'][1]), circle((px, pz), V['tube_r'][0]), OM['pillar']['y'][1], OM['ear']['y'][0])
    coil = L.ring_prism(comp, 'Spring coil', 'xz', circle((px, pz), SP['od'] / 2), circle((px, pz), SP['id'] / 2), *SP['y'])
    zl = pz + SP['mean_d'] / 2
    wr = SP['wire']
    ad = OM['adjuster']
    x_tip = ad['x'] - 0.2                                                                       # the leg ends 2 mm past the middle of the adjuster head (12 mm in all)
    leg1 = L.prism(comp, 'Spring rear leg', 'xz', [rect((px - 0.46 + x_tip) / 2, zl, (px - 0.46) - x_tip, wr)], ad['y'] - wr / 2, ad['y'] + wr / 2)
    AB = P.OMNI_ARBOR
    arbor = L.ring_prism(comp, 'Spring arbor', 'xz', circle((px, pz), AB['r']), circle((px, pz), AB['r_in']), *AB['y'])
    ctx.pal.paint([tube, coil, leg1], '#B4B2A9')
    ctx.pal.paint(arbor, '#1D9E75')
    # adjuster: M3 hex screw in the tail (insert), the rear leg lies on its head
    occ, comp = L.new_part(ctx.root, 'Omni adjuster')
    hexpts = [(ad['x'] + ad['head_r'] * math.cos(math.radians(60 * k)), ad['y'] + ad['head_r'] * math.sin(math.radians(60 * k))) for k in range(6)]
    ah = L.prism(comp, 'Adjuster screw head', 'xy', [poly(hexpts)], *ad['head_z'])
    asx = L.prism(comp, 'Adjuster screw', 'xy', [circle((ad['x'], ad['y']), ad['shank_r'])], ad['head_z'][0] - ad['bolt_len'], ad['head_z'][0])
    ctx.pal.paint([ah, asx], '#5F5E5A')


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
    for nm in T.sensors():                     # a box of wall round each ToF pocket: where the pocket hangs in the bore it needs a rear wall, side walls and a front wall (the pocket is cut out of it below)
        join('ToF block ' + nm, [poly(T.block_plan(nm))], z0, z1)
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
    TM, TB = P.TOF_MOUNT, P.TOF_BOARD
    hz = T.hole_z()
    for nm in T.sensors():
        cut('ToF pocket ' + nm, 'xy', [poly(T.pocket_plan(nm))], TM['floor_z'], z1 + 0.1)                       # open at the top: the board drops in from above, the lid covers it
        cut('ToF plug shaft ' + nm, 'xy', [poly(T.shaft_plan(nm))], z0 - 0.1, TM['floor_z'] + 0.05)             # the plug of the lower connector and its cable go straight down through the floor
        cut('ToF tunnel ' + nm, 'xy', [poly(T.tunnel_plan(nm))], P.TOF_Z - TM['tunnel_w'], P.TOF_Z + TM['tunnel_w'])
        join('ToF posts ' + nm, [poly(q) for q in T.post_plans(nm)], hz - TM['post_r'], hz + TM['post_r'])      # two posts in front of the upper holes (square in the model)
        cut('ToF pilots ' + nm, 'xy', [poly(q) for q in T.pilot_plans(nm)], hz - TM['pilot_r'], hz + TM['pilot_r'])
        cut('ToF screw access ' + nm, 'xy', [poly(q) for q in T.access_plans(nm)], hz - TM['access_r'], hz + TM['access_r'])    # the screwdriver reaches the screws through the rear wall
    for s, tag in ((1, 'L'), (-1, 'R')):
        y0, y1 = ymirror(s, 8.4, 10.8)
        cut('Camera window ' + tag, 'xz', [rect(P.CAM_X, (z0 - 0.1 + 10.6) / 2, P.CAMERA['window_w'], 10.6 - (z0 - 0.1))], y0, y1)
    CG = P.CAGE
    for s, tag in ((1, 'L'), (-1, 'R')):                       # the two M3 inserts of the camera cage, in the ring's inner face beside the window (square holes in the model, 6 mm deep)
        shapes = []
        for e in (1, -1):
            xe = P.CAM_X + e * CG['ear_x']
            yf = math.sqrt(P.FRAME['r_in'] ** 2 - xe ** 2)
            shapes.append(rect(xe, s * (yf + 0.25), CG['insert_d'], 0.7))
        zc = (CG['ear_z'][0] + CG['ear_z'][1]) / 2
        cut('Camera insert holes ' + tag, 'xy', shapes, zc - CG['insert_d'] / 2, zc + CG['insert_d'] / 2)
    ctx.pal.paint(ring, '#B4B2A9', 0.55)


@stage('tof')
def build_tof_modules(ctx):
    """Nine Adafruit VL53L0X boards (the STEMMA QT version: 25.4 x 17.78 x 1.6 mm, standing on the short end, the chip in the middle and a JST SH connector at each end on the front face) in their pockets, each held on two
    posts by two M2.5 screws from behind (spec 8.3). Holes and screws are square in the model (a sketch cannot draw a hole along the aim without a tool body); the connectors are boxes, the plugs and cables are not modelled."""
    B = P.TOF_BOARD
    zc = P.TOF_Z
    hz = T.hole_z()
    hd, hh = B['screw_head']
    S, Lh = B['short'] / 2, B['long'] / 2
    for nm, (x, y, aim) in T.sensors().items():
        occ, comp = L.new_part(ctx.root, 'ToF ' + nm)
        fr = T.frame(x, y, aim)
        pcb = L.prism(comp, 'VL53L0X board ' + nm, 'xy', [poly(T.rect_pts(fr, -B['t'], 0.0, -S, S))], zc - Lh, zc + Lh)
        for dz in (B['hole_long'], -B['hole_long']):                    # the four M2.5 holes (2.5 mm across, drawn as squares)
            L.prism(comp, 'Board holes', 'xy', [poly(T.rect_pts(fr, -B['t'] - 0.05, 0.05, v * B['hole_short'] - B['hole_d'] / 2, v * B['hole_short'] + B['hole_d'] / 2)) for v in (1, -1)],
                    zc + dz - B['hole_d'] / 2, zc + dz + B['hole_d'] / 2, op=CUT, targets=[pcb])
        e0, e1 = Lh - B['conn_end'] - B['conn_len'], Lh - B['conn_end']
        conn = []
        for lo, hi in ((e0, e1), (-e1, -e0)):                           # the connector at the top end and the one at the bottom end
            conn.append(L.prism(comp, 'QT connector ' + nm, 'xy', [poly(T.rect_pts(fr, 0.0, B['conn_h'], -B['conn_w'] / 2, B['conn_w'] / 2))], zc + lo, zc + hi))
        chip = L.prism(comp, 'VL53L0X chip ' + nm, 'xy', [poly(T.rect_pts(fr, 0.0, B['chip_u'], -B['chip_v'] / 2, B['chip_v'] / 2))], zc - B['chip_w'] / 2, zc + B['chip_w'] / 2)
        heads, shafts = T.screw_plans(nm)
        sh = L.prism(comp, 'M2.5 screw heads ' + nm, 'xy', [poly(q) for q in heads], hz - hd / 2, hz + hd / 2)
        sf = L.prism(comp, 'M2.5 screw shafts ' + nm, 'xy', [poly(q) for q in shafts], hz - 0.10, hz + 0.10)
        ctx.pal.paint(pcb, '#378ADD')
        ctx.pal.paint(conn, '#EEEEEE')
        ctx.pal.paint(chip, '#042C53')
        ctx.pal.paint(list(sh) + list(sf), '#5F5E5A')


def camera_frame(s):
    """Lens tip, the view axis backwards (from the tip towards the board) and the placement matrix of a camera: local z along that axis, local y up in the board's plane, local x = world x for the left camera, -x for the right."""
    t = math.radians(P.CAM['tilt'])
    psi = math.radians(P.CAM['psi'])
    tip = (P.CAM_X, s * P.CAM['tip_r'] * math.sin(psi), P.CAM['zl'])
    ez = (0.0, -s * math.cos(t), math.sin(t))
    return tip, ez, L.frame_from_zy(tip, ez, (0.0, 0.0, 1.0))


def camera_hole_positions(s):
    """Local (x, y) of the four mounting holes with their drill diameters. The board's long tail points to world +x on both sides; v is measured from the camera end, which lies lens_v behind the lens axis."""
    C = P.CAMERA
    out = []
    for u, v, d in C['holes']:
        out.append((s * (v - C['lens_v']), u - C['lens_u'], d))
    return out


@stage('cameras')
def build_cameras4(ctx):
    """Two OpenMV Cam H7 Plus boards (35.56 x 44.45 x 1.6 mm, four holes at the camera end, lens holder and M12 barrel on the front face) tilted 20 degrees behind the windows, each on a printed cage (spec 8.4).
    The lens axis, the holder size and the tip height are estimates (v3_params4.CAMERA); the board's long tail points to +x on both sides."""
    C, G = P.CAMERA, P.CAGE
    bw, bl, bt = C['board']
    tip_h = C['tip']
    for s, tag in ((1, 'L'), (-1, 'R')):
        tip, ez, mat = camera_frame(s)
        occ, comp = L.new_part(ctx.root, 'Camera ' + tag, mat)
        x0 = s * (-C['lens_v'])                         # the camera end
        x1 = s * (bl - C['lens_v'])                     # the tail end
        pcb = L.prism(comp, 'OpenMV H7 Plus board ' + tag, 'xy', [rect((x0 + x1) / 2, 0.0, abs(x1 - x0), bw)], tip_h, tip_h + bt)
        holes = camera_hole_positions(s)
        L.prism(comp, 'Board holes', 'xy', [rect(hx, hy, d, d) for hx, hy, d in holes], tip_h - 0.05, tip_h + bt + 0.05, op=CUT, targets=[pcb])
        hold = L.prism(comp, 'Lens holder ' + tag, 'xy', [rect(0.0, 0.0, C['holder'][1], C['holder'][0])], tip_h - C['holder'][2], tip_h)
        barrel = L.prism(comp, 'Lens barrel ' + tag, 'xy', [circle((0.0, 0.0), C['barrel_d'] / 2)], 0.0, tip_h - C['holder'][2])
        sd = L.prism(comp, 'micro-SD socket ' + tag, 'xy', [rect(s * (-C['lens_v'] + 1.8), 0.7, C['sd'][1], C['sd'][0])], tip_h + bt, tip_h + bt + C['sd'][2])
        d25, h25d, h25h = G['screw25']
        heads = L.prism(comp, 'M2.5 screw heads ' + tag, 'xy', [rect(hx, hy, h25d, h25d) for hx, hy, d in holes], tip_h + bt, tip_h + bt + h25h)
        shafts = L.prism(comp, 'M2.5 screw shafts ' + tag, 'xy', [rect(hx, hy, 0.19, 0.19) for hx, hy, d in holes], tip_h - G['frame_t'] + 0.06, tip_h + bt)
        ctx.pal.paint(pcb, '#7F77DD')
        ctx.pal.paint([hold, barrel], '#3C3489')
        ctx.pal.paint(sd, '#C8C6BE')
        ctx.pal.paint(list(heads) + list(shafts), '#5F5E5A')

        # the cage, in world coordinates: tilted bodies are drawn in the camera's frame and moved there, the ears in the world frame
        occ2, cage = L.new_part(ctx.root, 'Camera cage ' + tag)

        def tilted(name, shapes, z0, z1):
            b = L.prism(cage, name, 'xy', shapes, z0, z1)
            bs = list(b) if isinstance(b, (list, tuple)) else [b]
            for one in bs:
                L.move_body(cage, one, mat)
            return bs

        fz0, fz1 = tip_h - G['frame_t'], tip_h
        fr = L.ring_prism(cage, 'Cage frame ' + tag, 'xy', rect(0.0, 0.0, 2 * G['frame_x'], 2 * G['frame_y']), rect(0.0, 0.0, 2 * G['open_x'], 2 * G['open_y']), fz0, fz1)
        L.move_body(cage, fr, mat)
        rails = tilted('Cage rails ' + tag, [rect(e * (G['rail_x'][0] + G['rail_x'][1]) / 2, 0.0, G['rail_x'][1] - G['rail_x'][0], 2 * G['rail_y']) for e in (1, -1)], G['flange_z'][0], fz0 + 0.05)
        L.combine(cage, fr, rails, op=JOIN)
        fl = tilted('Cage flange ' + tag, [rect(0.0, 0.0, 2 * G['flange_x'], 2 * G['rail_y'])], G['flange_z'][0], G['flange_z'][1])
        L.combine(cage, fr, fl, op=JOIN)
        bore = tilted('Cage barrel opening ' + tag, [circle((0.0, 0.0), G['hole_d'] / 2)], G['flange_z'][0] - 0.1, G['flange_z'][1] + 0.1)
        L.combine(cage, fr, bore, op=CUT)
        pil = tilted('Cage pilots ' + tag, [rect(hx, hy, 0.20, 0.20) for hx, hy, d in holes], fz0 - 0.0 + 0.04, fz1 + 0.02)
        L.combine(cage, fr, pil, op=CUT)
        # the ears: world-aligned boxes from the flange to the ring's inner face, 3 mm thick walls cut by the face itself; each ear carries one M3 hole along world y
        ears = []
        for e in (1, -1):
            xe = P.CAM_X + e * G['ear_x']
            y_face = math.sqrt(P.FRAME['r_in'] ** 2 - (abs(xe) + G['ear_w'] / 2) ** 2)          # the ring's inner face at the ear's outer edge: the ear's flat face never reaches into the ring
            ears.append(rect(xe, s * ((8.0 + y_face) / 2), G['ear_w'], y_face - 8.0))
        ear_b = L.prism(cage, 'Cage ears ' + tag, 'xy', ears, G['ear_z'][0], G['ear_z'][1])
        L.combine(cage, fr, ear_b if isinstance(ear_b, (list, tuple)) else [ear_b], op=JOIN)
        for e in (1, -1):
            xe = P.CAM_X + e * G['ear_x']
            y_face = math.sqrt(P.FRAME['r_in'] ** 2 - (abs(xe) + G['ear_w'] / 2) ** 2)
            zc = (G['ear_z'][0] + G['ear_z'][1]) / 2
            L.prism(cage, 'Cage M3 hole %+d' % e, 'xy', [rect(xe, s * (y_face - 0.45), G['m3_d'], 0.9)], zc - G['m3_d'] / 2, zc + G['m3_d'] / 2, op=CUT, targets=[fr])
        ctx.pal.paint(fr, '#B4B2A9')


@stage('lid')
def build_lid(ctx):
    """Snap-on lid: plate on the ring, camera humps, handle slot, front notch over the control deck, finger notches, four skirt segments with hook bumps (spec 7.1)."""
    Ld, Hk = P.LID, P.HOOK
    occ, comp = L.new_part(ctx.root, 'Lid')
    lid = L.prism(comp, 'Lid plate', 'xy', [circle((0, 0), P.R_BODY)], Ld['z0'], Ld['z1'])
    for s in (1, -1):
        y0, y1 = ymirror(s, *P.CAMERA['hump_y'])               # the board's top part, the screw heads and the cage frame's top bar (y 6.1 to 7.1) stand up into the pocket: 2 cm deep; wider and the hump's corner leaves the body radius
        px0, px1 = P.CAMERA['pocket_x']
        L.prism(comp, 'Camera hump', 'xy', [rect((px0 + px1) / 2, (y0 + y1) / 2, px1 - px0 + 0.4, y1 - y0)], P.Z_ROOF - 0.05, P.Z_LID + Ld['hump_skin'], op=JOIN, targets=[lid])
    for s in (1, -1):
        y0, y1 = ymirror(s, *P.CAMERA['pocket_y'])
        px0, px1 = P.CAMERA['pocket_x']                       # the board's tail (to x 4.25) and the lens holder's overhang (to x -0.56) stand up into the lid: 4.4 mm to spare at each end
        L.prism(comp, 'Camera pocket', 'xy', [rect((px0 + px1) / 2, (y0 + y1) / 2, px1 - px0, y1 - y0)], Ld['z0'] - 0.1, P.Z_LID, op=CUT, targets=[lid])
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
    """Per side: a hopper sealed to the underside of the dropper floor under the 17 mm slot (with a socket cut for the channel), and the 19 mm square channel from the slot axis to the
    body radius. Inside the hopper the channel is an open trough: everything of the tube inside the hopper's void is cut away and only the floor slab is laid back, so that a kit lands
    on the inclined floor with no wall end, ledge or ceiling edge to catch it (spec 6.2; the first design left low side walls inside the void and jammed kits in the chute simulation).
    The hopper and the channel do not overlap, so the channel can slide out."""
    Ch, F = P.CHUTE, P.FRAME
    lift = Ch['lift']                                 # the square section is centred this far above the axis: the bore floor is 8 mm below it, the ceiling 10 mm above (spec 6.2)
    for s, side in ((1, 'B left'), (-1, 'A right')):
        p0, e = P.chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        sx, sy = p0[0], p0[1]
        far = L.vadd(e, L.vmul(d, 0.8))
        occ, comp = L.new_part(ctx.root, 'Hopper ' + side)
        hop = L.prism(comp, 'Hopper', 'xy', [rect(sx, sy, Ch['hopper_out'], Ch['hopper_out'], 45)], Ch['hopper_z0'], F['z0'])
        L.prism(comp, 'Hopper flange', 'xy', [rect(sx, sy, Ch['flange'], Ch['flange'], 45)], F['z0'] - Ch['flange_t'], F['z0'], op=JOIN, targets=[hop])
        L.prism(comp, 'Hopper void', 'xy', [rect(sx, sy, Ch['hopper_in'], Ch['hopper_in'], 45)], Ch['hopper_z0'] + 0.16, F['z0'] + 0.1, op=CUT, targets=[hop])
        up = Ch['socket_up']                          # the socket is taller than the channel: cut to the channel's ceiling it left a loose lintel of the hopper's downhill corner above it
        sock = L.axis_prism(comp, 'Socket tool', rect(0, lift + up / 2, Ch['out_w'], Ch['out_w'] + up), L.vsub(p0, L.vmul(d, 0.5)), L.vadd(p0, L.vmul(d, 3.0)))
        L.combine(comp, hop, [sock])
        ctx.pal.paint(hop, '#993C1D', 0.6)
        occ, comp = L.new_part(ctx.root, 'Chute ' + side)
        tube = L.axis_prism(comp, 'Channel', rect(0, lift, Ch['out_w'], Ch['out_w']), p0, far)
        bore = L.axis_prism(comp, 'Channel void', rect(0, lift, Ch['in_w'], Ch['in_w']), L.vsub(p0, L.vmul(d, 0.2)), L.vadd(far, L.vmul(d, 0.2)))
        L.combine(comp, tube, [bore])
        L.prism(comp, 'Trough cut', 'xy', [rect(sx, sy, Ch['trough'], Ch['trough'], 45)], Ch['trough_z0'], F['z0'] + 0.3, op=CUT, targets=[tube])
        hi = Ch['in_w'] / 2
        slab = L.axis_prism(comp, 'Floor slab', rect(0, lift - (hi + Ch['wall'] / 2), Ch['out_w'], Ch['wall']), p0, L.vadd(p0, L.vmul(d, Ch['slab_end'])))
        L.combine(comp, tube, [slab], op=JOIN)
        # the tall section's top corners just outside the trough, near the slot centre, would reach into the dropper floor (up to 2 mm above its underside): they are cut at z 8.7
        L.prism(comp, 'Dropper floor clearance cut', 'xy', [circle((0, 0), 14.0)], F['z0'], F['z0'] + 1.0, op=CUT, targets=[tube])
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


@stage('imu')
def build_imu(ctx):
    """Adafruit BNO055 breakout (20 x 27 x 1.6 mm board, 4 mm with the header) on the four tub posts, header edge forward, four M2.5 screws (spec 8.6)."""
    I = P.IMU
    z0 = P.Z_FLOOR_TOP + I['post_h']
    length, width, t = I['size']
    pts = [(I['x'] + sx * I['holes'][0], I['y'] + sy * I['holes'][1]) for sx in (1, -1) for sy in (1, -1)]
    occ, comp = L.new_part(ctx.root, 'BNO055 IMU')
    pcb = L.prism(comp, 'BNO055 board', 'xy', [rect(I['x'], I['y'], length, width)], z0, z0 + t)
    L.prism(comp, 'Board holes', 'xy', [rect(px, py, I['hole_d'], I['hole_d']) for px, py in pts], z0 - 0.05, z0 + t + 0.05, op=CUT, targets=[pcb])
    chip = L.prism(comp, 'BNO055 chip', 'xy', [rect(I['x'], I['y'], I['chip'][0], I['chip'][1])], z0 + t, z0 + t + I['chip'][2])
    hd, hh = I['screw_head']
    heads = L.prism(comp, 'M2.5 screw heads', 'xy', [rect(px, py, hd, hd) for px, py in pts], z0 + t, z0 + t + hh)
    shafts = L.prism(comp, 'M2.5 screw shafts', 'xy', [rect(px, py, 0.19, 0.19) for px, py in pts], z0 - 0.4, z0 + t)
    ctx.pal.paint(pcb, '#0F6E56')
    ctx.pal.paint(chip, '#222222')
    ctx.pal.paint(list(heads) + list(shafts), '#5F5E5A')


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
