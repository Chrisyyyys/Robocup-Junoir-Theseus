"""Checks on the rev 4 Fusion model (run inside Fusion through v4_run.py): per-stage probe tests and local interference now, whole-model reports in Task 9.
A probe asks "is there material at this point of this component"; the probes pin the features each build stage must produce."""
import math

import adsk.core
import adsk.fusion

import fusion_lib as L
import v3_checks3d as K3
import tof_mount as T
import v3_params4 as P

P3, V3 = L.P3, L.V3
PROBES = {}                       # stage name -> function returning rows (component, (x, y, z), expect_solid, label)
STAGE_COMPONENTS = {}             # stage name -> components the stage creates (for the local interference test)
ALLOW = {}                        # frozenset({component a, component b}) -> largest accepted overlap in cm3; every entry needs a comment saying why
ONE_BODY = {}                     # stage name -> components that must be a single body: a cut that leaves a loose chip splits a part in two, and no probe point sees that (the hopper lintel)


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
    for comp in ONE_BODY.get(stage, []):
        try:
            n = len(_bodies(ctx, comp))
        except KeyError:
            ok = False
            lines.append('  FAIL %-26s component missing (one body)' % comp)
            continue
        ok = ok and n == 1
        lines.append('  %s %-26s %-46s expected 1 body, got %d' % ('ok  ' if n == 1 else 'FAIL', comp, 'one body, no loose fragment', n))
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
    in_wall = P.polar(10.4, ang + 9.0) + (e[2],)                          # 9 degrees (16 mm) round the wall from the hole centre (the 22 mm wide hole reaches about 6.5 degrees)
    a_b = P.SCREW_DEG[0]                                                      # the front boss of the left side (12 degrees)
    bx, by = P.polar(S['r'], a_b)
    ux, uy = math.cos(math.radians(a_b)), math.sin(math.radians(a_b))        # radial direction at that boss, vx/vy tangential
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
        ('Tub', (3.0, 1.3, 3.7), False, 'pocket in the floor beside the pivot (spring coil and arm)'),
        ('Tub', (3.9, 1.3, 3.7), False, 'the pocket joins the omni bay'),
        ('Tub', (3.0, 0.0, 3.7), True, 'plain floor inboard of the pillar'),
        ('Tub', (2.9, 0.55, 4.5), True, 'omni pillar, above and beside the pivot hole'),
        ('Tub', (P.OMNI['pivot'][0], 0.55, P.OMNI['pivot'][1]), False, 'pivot screw hole through the pillar'),
        ('Tub', (P.omni_stop_pin()[0], 0.55, P.omni_stop_pin()[1]), False, 'stop screw hole through the pillar'),
        ('Tub', (1.65, 0.55, 4.1), True, 'pillar tail behind the pivot'),
        ('Tub', (P.OMNI_MOUNT['adjuster']['x'] + 0.3, 1.0, 4.1), True, 'the tail is wide enough (y to 1.2) for the adjuster under the rear leg'),
        ('Tub', (P.OMNI_MOUNT['adjuster']['x'] + 0.3, 1.3, 4.1), False, 'and stops 1 mm short of the left motor seat (y 1.3)'),
        ('Tub', (P.OMNI_MOUNT['adjuster']['x'], P.OMNI_MOUNT['adjuster']['y'], 4.1), False, 'hole for the adjuster screw in the tail'),
        ('Tub', (2.4, 2.5, 4.55), True, 'outer ear, above the pivot thread hole'),
        ('Tub', (P.OMNI['pivot'][0], 2.5, P.OMNI['pivot'][1]), False, 'thread hole for the M3 pivot screw in the ear'),
        ('Tub', (P.omni_stop_pin()[0], 2.4, P.omni_stop_pin()[1]), False, 'pilot hole for the stop screw in the ear'),
        ('Tub', (P.FLOOR_FRONT['x'], P.FLOOR_FRONT['y'], 3.7), False, 'front floor port hole, 7.5 cm ahead of the axle'),
        ('Tub', (P.SILVER['x'], P.SILVER['y'], 3.7), False, 'silver module hole, 7.5 cm ahead of the axle beside the omni bay'),
        ('Tub', (0.0, 0.0, 3.7), True, 'plain floor at the axle line (the silver module moved forward)'),
        ('Tub', in_hole, False, 'chute hole through the wall'),
        ('Tub', in_wall, True, 'wall beside the chute hole'),
        ('Tub', (bx, by, 7.9), True, 'insert boss at %d degrees' % a_b),
        ('Tub', (bx, by, 8.5), False, 'insert hole in the boss'),
        ('Tub', (rib[0], rib[1], 8.2), True, 'rib from the %d degree boss to the wall' % a_b),
        ('Tub', (beside[0], beside[1], 8.2), False, 'beside the rib'),
        ('Tub', (bat[0], bat[1], 5.0), True, 'battery tray'),
        ('Tub', (0.9, 5.0, 5.0), False, 'notch in the tray over the left motor'),
        ('Tub', (1.6, 5.0, 5.0), True, 'tray beside the notch'),
        ('Tub', (usb_in[0], usb_in[1], 7.5), False, 'USB-C socket hole in the front-right wall'),
        ('Tub', (usb_side[0], usb_side[1], 7.5), True, 'wall beside the socket hole'),
        ('Tub', (-10.4, 0.0, 7.5), True, 'rear wall is plain now'),
        ('Tub', (P.IMU['x'] + P.IMU['holes'][0] + 0.18, P.IMU['y'] + P.IMU['holes'][1] + 0.18, P.Z_FLOOR_TOP + 0.25), True, 'IMU post (beside its pilot hole)'),
        ('Tub', (P.IMU['x'] + P.IMU['holes'][0], P.IMU['y'] + P.IMU['holes'][1], P.Z_FLOOR_TOP + 0.3), False, 'pilot hole in the IMU post'),
        ('Tub', (P.IMU['x'], P.IMU['y'], P.Z_FLOOR_TOP + 0.25), False, 'no floor boss under the middle of the IMU board'),
    ]


STAGE_COMPONENTS.update({'drive': ['Wheel L', 'Motor L', 'Wheel R', 'Motor R'], 'cartridges': ['Face plate L', 'Face plate R'], 'omni': ['Omni wheel', 'Omni arm', 'Omni pins', 'Omni pivot', 'Omni spring', 'Omni adjuster'],
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
    O, B, X, M, SP = P.OMNI, P.OMNI_BEARING, P.OMNI_AXLE, P.OMNI_MOUNT, P.OMNI_SPRING
    ox, oz = O['rest']
    px, pz = O['pivot']
    w0, w1 = O['y_wheel']
    a0, a1 = O['arm_y']
    ym = (a0 + a1) / 2
    xm = 4.8
    zm = pz + (oz - pz) * (xm - px) / (ox - px)          # centre line of the arm at x 4.8
    stx, stz = P.omni_stop_pin()
    ad = M['adjuster']
    zl = pz + SP['mean_d'] / 2
    y_b1 = w0 + P.OMNI_AXLE['seat']
    y_under = y_b1 - P.OMNI_AXLE['washer_h']
    kx, kz = P.omni_seat(0.0)
    (lx0, lz0), (lx1, lz1) = P.omni_leg(0.0)
    lmx, lmz = (lx0 + lx1) / 2, (lz0 + lz1) / 2
    y_leg = SP['y'][1] - SP['wire'] / 2
    return [
        ('Omni wheel', (ox, 0.5, oz + 2.5), True, 'wheel above the hub'),
        ('Omni wheel', (ox + 0.25, 0.5, oz), True, 'sleeve between the inner rings'),
        ('Omni wheel', (ox, 0.5, oz), False, 'the hole of the sleeve (the screw is in the axle set)'),
        ('Omni wheel', (ox + 0.4, y_b1 + 0.2, oz), True, 'inboard bearing ring'),
        ('Omni wheel', (ox, y_b1 + 0.2, oz), False, 'inboard bearing bore'),
        ('Omni wheel', (ox + 0.45, w0 + 0.2, oz), False, 'the bore in front of the inboard bearing (the screw head sits here)'),
        ('Omni wheel', (ox + 0.25, (w1 + a0) / 2, oz), True, '1 mm spacer between the wheel and the arm'),
        ('Omni wheel', (ox + 2.9, 0.5, oz), True, 'front edge of the wheel (x 9.75)'),
        ('Omni wheel', (ox + 3.1, 0.5, oz), False, 'beyond the front edge'),
        ('Omni wheel', (ox, w0 - 0.05, oz + 2.0), False, 'inboard of the wheel face (y -1.03)'),
        ('Omni arm', (xm, ym, zm), True, 'single arm bar on +y'),
        ('Omni arm', (xm, -1.3, zm), False, 'no second arm bar on -y'),
        ('Omni arm', (px, ym, pz), False, 'pivot bore'),
        ('Omni arm', (ox, ym, oz), False, 'axle hole'),
        ('Omni arm', (stx, ym, stz), False, 'the arc slot, at the stop screw'),
        ('Omni arm', (px + 1.5, ym, pz - 0.2), True, 'solid bar between the pivot and the slot'),
        ('Omni arm', (kx, 1.5, kz), True, 'seat pin standing out of the arm\'s inboard face'),
        ('Omni arm', (kx, ym, kz), True, 'seat pin inside the bar'),
        ('Omni arm', (kx, a1 - 0.02, kz), False, 'the seat hole is open for the last 0.5 mm'),
        ('Omni arm', (lmx, y_leg, lmz), True, 'forward spring leg'),
        ('Omni arm', (lmx, y_leg - 0.3, lmz), False, 'nothing but the leg in this plane: the leg is one wire thick'),
        ('Omni pins', (ox, 0.5, oz), True, 'axle screw'),
        ('Omni pins', (ox + 0.2, y_under - 0.2, oz), True, 'axle screw head, in the wheel\'s bore'),
        ('Omni pins', (ox, w0 - 0.05, oz), False, 'nothing of the axle set in front of the wheel\'s inboard face'),
        ('Omni pins', (ox, a1 - 0.05, oz), True, 'axle screw tip inside the arm'),
        ('Omni pins', (ox, a1 + 0.05, oz), False, 'nothing beyond the arm\'s outer face'),
        ('Omni pivot', (px, 1.3, pz), True, 'pivot screw'),
        ('Omni pivot', (px, 2.5, pz), True, 'pivot screw in the ear'),
        ('Omni pivot', (stx, 1.3, stz), True, 'stop screw'),
        ('Omni pivot', (px, 1.3, pz + 0.4), False, 'nothing above the pivot screw'),
        ('Omni spring', (px, 1.3, pz + SP['mean_d'] / 2), True, 'spring coil (top)'),
        ('Omni spring', (px + 0.2, 1.3, pz), True, 'pivot tube'),
        ('Omni spring', (px, 1.3, pz), False, 'the screw hole in the tube'),
        ('Omni spring', (ad['x'], ad['y'], zl), True, 'rear leg over the middle of the adjuster head'),
        ('Omni spring', (ad['x'] - 0.15, ad['y'], zl), True, 'rear leg ends 2 mm past it'),
        ('Omni spring', (ad['x'] - 0.25, ad['y'], zl), False, 'and no further'),
        ('Omni spring', (px + 0.31, 1.2, pz), True, 'spring arbor between the tube and the coil'),
        ('Omni spring', (px + 0.395, 1.2, pz), False, 'half a millimetre of air between the arbor and the coil'),
        ('Omni adjuster', (ad['x'], ad['y'], 0.5 * (ad['head_z'][0] + ad['head_z'][1])), True, 'adjuster screw head'),
        ('Omni adjuster', (ad['x'], ad['y'], 4.2), True, 'adjuster screw shank'),
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


STAGE_COMPONENTS.update({'frame': ['Upper frame'], 'tof': ['ToF %s' % n for n, x, y, a in P.TOF], 'cameras': ['Camera L', 'Camera R', 'Camera cage L', 'Camera cage R']})
ONE_BODY['cameras'] = ['Camera cage L', 'Camera cage R']          # a cut through a rail or an ear must not leave a loose piece


def _tof_pt(nm, u, v, w=0.0):
    """World point of the local point (u, v, w) of a ToF sensor (u along the beam from the PCB's front face, v to the left, w up from z 10.0)."""
    x, y, aim = T.sensors()[nm]
    px, py = T.uv(T.frame(x, y, aim), u, v)
    return (px, py, P.TOF_Z + w)


def _cam_pt(s, lx, ly, lz):
    """World point of the local point (x, y, z) of the camera on side s (the frame of build_cameras4: origin at the lens tip, z back along the view axis, y up in the board's plane, x = world x for the left camera)."""
    t = math.radians(P.CAM['tilt'])
    tip = (P.CAM_X, s * P.CAM['tip_r'] * math.sin(math.radians(P.CAM['psi'])), P.CAM['zl'])
    ez = (0.0, -s * math.cos(t), math.sin(t))
    ey = (0.0, s * math.sin(t), math.cos(t))
    ex = (s * 1.0, 0.0, 0.0)
    return L.vadd(L.vadd(L.vadd(tip, L.vmul(ex, lx)), L.vmul(ey, ly)), L.vmul(ez, lz))


@probes('frame')
def _frame_probes():
    S, TB, TM = P.SCREW, P.TOF_BOARD, P.TOF_MOUNT
    Fr, Hk = P.FRAME, P.HOOK
    a_b = P.SCREW_DEG[0]
    free = P.polar(S['r'], 66.0)                      # between the screw at 60 and the hook at 71 degrees: no window, no screw
    hole = P.polar(S['r'], a_b)
    zh = Hk['skirt_z0'] + 0.4                          # inside the hook rebate and the ring beside it
    reb = P.polar(10.45, 71.0)
    groove = P.polar(10.3, 71.0)
    ring_ok = P.polar(10.45, 62.0)
    hz = T.hole_z()
    ur = T.u_rear()
    face_l = math.sqrt(P.FRAME['r_in'] ** 2 - (abs(P.CAM_X + P.CAGE['ear_x']) + P.CAGE['ear_w'] / 2) ** 2)
    zb = P.TOF_Z - TB['long'] / 2 - 0.0                # the board's bottom edge
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
        ('Upper frame', (hole[0], hole[1], 10.0), False, 'screw hole at %d degrees' % a_b),
        ('Upper frame', (hole[0], hole[1], Fr['z1'] - 0.15), False, 'counterbore at %d degrees' % a_b),
        ('Upper frame', (reb[0], reb[1], zh), False, 'hook rebate at 71 degrees'),
        ('Upper frame', (groove[0], groove[1], (Hk['bump_z'][0] + Hk['bump_z'][1]) / 2), False, 'hook groove at 71 degrees'),
        ('Upper frame', (ring_ok[0], ring_ok[1], zh), True, 'ring beside the rebate'),
        ('Upper frame', _tof_pt('F', -0.05, 0.0), False, 'ToF pocket F (the board stands in it)'),
        ('Upper frame', _tof_pt('F', -0.05, 0.0, 1.4), False, 'ToF pocket F is open to the top (the board drops in from above, the lid covers it)'),
        ('Upper frame', _tof_pt('F', 0.7, 0.0), False, 'ToF beam tunnel F'),
        ('Upper frame', _tof_pt('F', 0.7, 1.05), True, 'wall beside the beam tunnel F'),
        ('Upper frame', _tof_pt('F', 0.2, TB['hole_short'] + 0.18, TB['hole_long']), True, 'ToF post F, upper left (beside its pilot hole)'),
        ('Upper frame', _tof_pt('F', 0.2, -TB['hole_short'] - 0.18, TB['hole_long']), True, 'ToF post F, upper right (beside its pilot hole)'),
        ('Upper frame', _tof_pt('F', 0.2, TB['hole_short'] + 0.18, -TB['hole_long']), False, 'no post at the lower holes'),
        ('Upper frame', _tof_pt('F', 0.25, TB['hole_short'], TB['hole_long']), False, 'pilot hole in the post'),
        ('Upper frame', _tof_pt('F', ur - 0.15, TB['hole_short'], TB['hole_long']), False, 'screwdriver access hole through the rear wall'),
        ('Upper frame', _tof_pt('F', ur - 0.15, 0.0, TB['hole_long']), True, 'rear wall between the access holes'),
        ('Upper frame', (T.sensors()['F'][0] + 0.1, 0.0, 8.85), False, 'plug shaft under the lower connector, down through the floor'),
        ('Upper frame', (T.sensors()['F'][0] - 0.2, 0.0, 8.85), True, 'floor under the pocket beside the shaft'),
        ('Upper frame', _tof_pt('SFL', ur - 0.15, 0.0), True, 'rear wall of the SFL pocket: the block that fills the bore side'),
        ('Upper frame', _tof_pt('SFL', 0.9, 0.0), False, 'beam tunnel SFL (the tunnel runs through the bore and the ring)'),
        ('Upper frame', (0.307, 9.6, 9.3), False, 'camera window, left'),
        ('Upper frame', (P.CAM_X + P.CAGE['ear_x'], face_l + 0.2, 9.5), False, 'M3 insert hole for the camera cage (left, front ear)'),
        ('Upper frame', (P.CAM_X + P.CAGE['ear_x'], face_l + 1.0, 9.5), True, 'ring behind the insert hole'),
    ]


@probes('tof')
def _tof_probes():
    TB = P.TOF_BOARD
    top = TB['long'] / 2 - TB['conn_end'] - TB['conn_len'] / 2
    rows = []
    for nm in ('F', 'SFL', 'RR'):
        rows += [('ToF ' + nm, _tof_pt(nm, -0.08, 0.0), True, 'board %s' % nm),
                 ('ToF ' + nm, _tof_pt(nm, 0.05, 0.0), True, 'chip %s' % nm),
                 ('ToF ' + nm, _tof_pt(nm, 0.15, 0.0, top), True, 'QT connector %s (top end)' % nm),
                 ('ToF ' + nm, _tof_pt(nm, 0.15, 0.0, -top), True, 'QT connector %s (bottom end, the plug goes here)' % nm),
                 ('ToF ' + nm, _tof_pt(nm, 0.15, 0.5, 0.0), False, 'nothing but the chip in front of the board %s' % nm),
                 ('ToF ' + nm, _tof_pt(nm, -0.08, 0.0, TB['long'] / 2 + 0.1), False, 'above the board %s (top z %.2f)' % (nm, P.TOF_Z + TB['long'] / 2))]
    rows += [('ToF F', _tof_pt('F', -0.08, TB['hole_short'], TB['hole_long']), True, 'M2.5 screw in the upper left hole'),
             ('ToF F', _tof_pt('F', -0.08, TB['hole_short'], -TB['hole_long']), False, 'lower left hole is empty'),
             ('ToF F', _tof_pt('F', -0.23, TB['hole_short'], TB['hole_long']), True, 'screw head behind the board')]
    return rows


@probes('cameras')
def _camera_probes():
    G, C = P.CAGE, P.CAMERA
    rows = []
    for s, tag in ((1, 'L'), (-1, 'R')):
        hx = s * (C['holes'][0][1] - C['lens_v'])
        hy = C['holes'][0][0] - C['lens_u']
        xe = P.CAM_X + G['ear_x']
        face = math.sqrt(P.FRAME['r_in'] ** 2 - (abs(xe) + G['ear_w'] / 2) ** 2)
        rows += [('Camera ' + tag, _cam_pt(s, 0.0, 0.0, 1.0), True, 'lens barrel, 1 cm behind the tip'),
                 ('Camera ' + tag, _cam_pt(s, 0.0, 0.0, C['tip'] - 0.4), True, 'lens holder'),
                 ('Camera ' + tag, _cam_pt(s, 0.0, 0.0, C['tip'] + 0.08), True, 'board in line with the lens axis'),
                 ('Camera ' + tag, _cam_pt(s, s * 3.0, 0.0, C['tip'] + 0.08), True, "the board's tail 3 cm behind the lens axis"),
                 ('Camera ' + tag, _cam_pt(s, -s * 1.5, 0.0, C['tip'] + 0.08), False, 'nothing beyond the camera end (the board ends 5 mm behind the axis)'),
                 ('Camera ' + tag, _cam_pt(s, hx, hy, C['tip'] + 0.25), True, 'M2.5 screw head behind the board'),
                 ('Camera cage ' + tag, _cam_pt(s, 0.0, 1.5, C['tip'] - G['frame_t'] / 2), True, 'cage frame, upper bar'),
                 ('Camera cage ' + tag, _cam_pt(s, 0.0, 0.0, C['tip'] - G['frame_t'] / 2), False, 'cage frame opening round the lens holder'),
                 ('Camera cage ' + tag, _cam_pt(s, 1.2, 0.0, 1.8), True, 'cage rail'),
                 ('Camera cage ' + tag, _cam_pt(s, 2.0, 0.0, (G['flange_z'][0] + G['flange_z'][1]) / 2), True, 'cage flange'),
                 ('Camera cage ' + tag, _cam_pt(s, 0.0, 0.0, (G['flange_z'][0] + G['flange_z'][1]) / 2), False, 'barrel opening in the flange'),
                 ('Camera cage ' + tag, (xe + 0.3, s * (face - 0.25), 9.5), True, 'cage ear against the ring'),
                 ('Camera cage ' + tag, (xe, s * (face - 0.25), 9.5), False, 'M3 hole in the ear')]
    return rows


STAGE_COMPONENTS.update({'lid': ['Lid'], 'handle': ['Handle bar', 'Victim LED'], 'controls': [c[0] for c in P.CONTROLS] + ['USB-C service socket']})


@probes('lid')
def _lid_probes():
    sk = P.polar(10.42, 71.0)
    bump = P.polar(10.30, 71.0)
    between = P.polar(10.42, 90.0)
    finger = P.polar(10.3, 12.0)
    zl = P.LID['z0'] + 0.15                                # inside the lid plate
    zs = (P.HOOK['skirt_z0'] + P.LID['z0']) / 2            # inside the hook skirts
    zu = P.LID['z0'] - 0.1                                 # just under the lid plate: the space the camera humps open up
    return [
        ('Lid', (-5.0, 0.0, zl), True, 'lid plate'),
        ('Lid', (4.0, 0.0, zl), False, 'handle slot'),
        ('Lid', (-9.5, 0.0, zl), True, 'lid plate at the rear (the panel moved to the front, no rear notch)'),
        ('Lid', (7.0, 2.0, zl), False, 'front notch over the control deck'),
        ('Lid', (7.0, 3.8, zl), True, 'plate beside the front notch'),
        ('Lid', (7.0, -1.6, zl), True, 'plate on the -y side of the slot: the notch is on the +y side only'),
        ('Lid', (sk[0], sk[1], zs), True, 'hook skirt at 71 degrees'),
        ('Lid', (bump[0], bump[1], (P.HOOK['bump_z'][0] + P.HOOK['bump_z'][1]) / 2), True, 'hook bump at 71 degrees'),
        ('Lid', (between[0], between[1], zs), False, 'no skirt between the hooks'),
        ('Lid', (finger[0], finger[1], zl), False, 'finger notch at 12 degrees'),
        ('Lid', (0.307, 7.3, 12.4), True, 'camera hump skin (pocket ceiling 12.3, skin 2 mm)'),
        ('Lid', (0.307, 7.3, zl), False, 'space under the camera hump (the pocket goes through the lid plate up to z %.1f)' % P.Z_LID),
        ('Lid', (4.4, 7.3, zl), False, 'the hump pocket reaches the board tail (x 4.25) and the lens holder overhang (x -0.56)'),
        ('Lid', (-0.8, 7.3, zl), False, 'pocket beyond the lens holder overhang'),
        ('Lid', (5.0, 7.3, zl), True, 'lid plate beyond the hump pocket'),
        ('Lid', (-1.5, 7.3, zl), True, 'lid plate behind the hump pocket (it ends at x -1.3)'),
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


STAGE_COMPONENTS.update({'dropper': ['Dropper floor', 'Dropper plate', 'N20 motor', 'N20 face plate', 'Kits'], 'chutes': ['Hopper A right', 'Chute A right', 'Hopper B left', 'Chute B left']})
ONE_BODY['chutes'] = ['Hopper A right', 'Chute A right', 'Hopper B left', 'Chute B left']


@probes('dropper')
def _dropper_probes():
    cx, cy = P.PLATE['cx'], P.PLATE['cy']
    p1 = P.pocket_xy(1)
    sx, sy = P.slot_xy('A')
    return [
        ('Dropper floor', (-2.0, 3.0, 8.85), True, 'floor disc'),
        ('Dropper floor', (sx, sy, 8.85), False, 'slot A through the floor'),
        ('Dropper floor', (sx + 0.80 * math.cos(math.radians(45)), sy + 0.80 * math.sin(math.radians(45)), 8.85), False, 'slot A is 17 mm: 8.0 mm from its centre along a side normal is open'),
        ('Dropper floor', (sx + 0.90 * math.cos(math.radians(45)), sy + 0.90 * math.sin(math.radians(45)), 8.85), True, 'floor beside the 17 mm slot A (9.0 mm from its centre)'),
        ('Dropper floor', (cx, cy, 8.85), False, 'N20 pocket'),
        ('Dropper floor', (cx, 0.8, 8.95), False, 'N20 face plate recess'),
        ('Dropper floor', (cx, 0.8, 8.75), True, 'floor under the recess'),
        ('Dropper floor', (3.5, 1.0, 8.95), True, 'front tab, top half of the floor thickness'),
        ('Dropper floor', (3.5, 1.0, 8.75), False, 'nothing under the tab: the bridge web is the ledge'),
        ('Dropper floor', (-7.3, 0.0, 8.95), True, 'rear tab, beyond the disc rim'),
        ('Dropper floor', (-7.3, 0.0, 8.75), False, 'nothing under the rear tab: the spoke web is the ledge'),
        ('Dropper plate', (cx, cy + 0.5, 9.6), True, 'plate near the centre'),
        ('Dropper plate', (cx, cy, 9.6), False, 'shaft bore'),
        ('Dropper plate', (p1[0], p1[1], 9.6), False, 'pocket 1 (13 mm)'),
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
    Ch = P.CHUTE
    wall = Ch['in_w'] / 2 + Ch['wall'] / 2                                            # the middle of a side wall (0.98 from the axis)
    floor = -(Ch['in_w'] / 2 - Ch['lift'] + Ch['wall'] / 2)                           # the middle of the floor (0.88 below the axis)
    roof = Ch['in_w'] / 2 + Ch['lift'] + Ch['wall'] / 2                               # the middle of the ceiling (1.08 above the axis)
    for s, side in ((1, 'B left'), (-1, 'A right')):
        p0, e = P.chute_ends(s)
        d = L.unit(L.vsub(e, p0))
        lat = L.unit((-d[1], d[0], 0.0))
        up = L.unit(L.vsub((0.0, 0.0, 1.0), L.vmul(d, d[2])))                            # across the axis in the vertical plane, as the channel prism is built
        mid = L.vadd(p0, L.vmul(d, 3.5))
        under = L.vadd(p0, L.vmul(d, 0.8))                                                # inside the open trough, 8 mm along the axis from the slot centre
        rows += [
            ('Hopper ' + side, (p0[0], p0[1], 8.3), False, 'hopper void under the slot'),
            ('Hopper ' + side, L.vadd((p0[0], p0[1], 8.0), L.vmul(lat, 1.2)), True, 'hopper wall beside the channel socket'),
            ('Chute ' + side, mid, False, 'channel bore'),
            ('Chute ' + side, L.vadd(mid, L.vmul(lat, wall)), True, 'channel wall'),
            ('Chute ' + side, L.vadd(mid, L.vmul(up, floor)), True, 'channel floor'),
            ('Chute ' + side, L.vadd(mid, L.vmul(up, roof)), True, 'channel ceiling'),
            ('Chute ' + side, L.vadd(e, L.vmul(d, 0.3)), False, 'channel trimmed at the body radius'),
            # the open trough under the slot: only the floor slab is there, so that a kit lands on the inclined floor and no wall end, ledge or ceiling edge can catch it
            ('Chute ' + side, L.vadd(under, L.vmul(up, floor)), True, 'floor slab under the slot'),
            ('Chute ' + side, L.vadd(under, L.vmul(up, roof)), False, 'no ceiling over the trough'),
            ('Chute ' + side, L.vadd(L.vadd(under, L.vmul(lat, wall)), L.vmul(up, -0.3)), False, 'no low side wall in the trough (the first design left one, and kits stuck on it)'),
            # the vertical trough cut meets the sloping floor at 2.55 cm (top face) to 2.65 cm (underside) along the axis, not at the 2.04 cm of its plan corner: a slab that ended at 2.5 cm left a notch
            ('Chute ' + side, L.vadd(L.vadd(L.vadd(p0, L.vmul(d, 2.57)), L.vmul(lat, -0.37)), L.vmul(up, floor)), True, 'floor slab reaches past the trough cut (no notch at 2.57 cm, one side)'),
            ('Chute ' + side, L.vadd(L.vadd(L.vadd(p0, L.vmul(d, 2.57)), L.vmul(lat, 0.37)), L.vmul(up, floor)), True, 'floor slab reaches past the trough cut (no notch at 2.57 cm, other side)'),
            ('Chute ' + side, L.vadd(L.vadd(p0, L.vmul(d, 3.2)), L.vmul(lat, wall)), True, 'side wall beyond the trough'),
            ('Chute ' + side, L.vadd(L.vadd(p0, L.vmul(d, 3.2)), L.vmul(up, roof)), True, 'ceiling beyond the trough'),
            ('Chute ' + side, L.vadd(L.vadd(p0, L.vmul(d, 3.2)), L.vmul(up, Ch['in_w'] / 2 + Ch['lift'] - 0.05)), False, 'bore is 19 mm tall: open 0.5 mm under the ceiling'),
        ]
    return rows


STAGE_COMPONENTS.update({'posts': ['GIGA posts'], 'electronics': ['Arduino GIGA R1', 'Main PCB', 'Battery', 'Floor port FP', 'Silver module SM'], 'antenna': ['Wi-Fi antenna'], 'imu': ['BNO055 IMU']})


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


@probes('imu')
def _imu_probes():
    I = P.IMU
    z0 = P.Z_FLOOR_TOP + I['post_h']
    px, py = I['x'] + I['holes'][0], I['y'] + I['holes'][1]
    return [('BNO055 IMU', (I['x'], I['y'] + 0.45, z0 + 0.08), True, 'IMU board on its posts'),
            ('BNO055 IMU', (I['x'], I['y'], z0 + I['size'][2] + 0.05), True, 'BNO055 chip'),
            ('BNO055 IMU', (px, py, z0 + I['size'][2] + 0.07), True, 'M2.5 screw head'),
            ('BNO055 IMU', (I['x'] + 0.6, I['y'], z0 + I['size'][2] + 0.3), False, 'nothing above the board but the chip and the screw heads'),
            ('BNO055 IMU', (I['x'] + I['size'][0] / 2 + 0.2, I['y'], z0 + 0.08), False, 'nothing beyond the board edge')]


@probes('stepper')
def _stepper_probes():
    g = P.stepper_geometry()
    return [('Stepper bay 28BYJ-48', (g['centre'][0], g['centre'][1], 7.5), True, 'bay body'),
            ('Stepper bay 28BYJ-48', (g['ears'][0][0], g['ears'][0][1], 8.4), True, 'ear 1'),
            ('Stepper bay 28BYJ-48', (g['block'][0], g['block'][1], 7.5), True, 'wire block')]


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
    z_tube = P.OMNI['pivot'][1] - P.OMNI_PIVOT['tube_r'][1]
    z_coil = P.OMNI['pivot'][1] - P.OMNI_SPRING['od'] / 2
    lines.append('pivot tube (5 mm) lowest point z %.2f: %+.1f mm against the belly line; spring coil lowest point z %.2f: %+.1f mm (the pivot was raised to z 4.2 on 9 Oct: a 4 mm pin at 3.6 hung 1 mm under it)' %
                 (z_tube, (z_tube - z_belly) * 10, z_coil, (z_coil - z_belly) * 10))
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
    frame = ['Upper frame', 'Handle bar', 'Victim LED', 'Camera L', 'Camera R', 'Camera cage L', 'Camera cage R'] + tof + [c[0] for c in P.CONTROLS]          # what the six screws hold; the control parts sit on its bridge
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
         'right cartridge up (prongs flex), pivot and stop screws out inboard, axle screw unscrewed 3.5 mm out of the arm, wheel set with the screw still in it down, arm with the spring on its tube up',
         [lid] + chutes + [unit_up, (frame, [(0.0, 0.0, 12.0)]),
          (['Battery'], [(0.0, 0.0, 8.0)]),
          (['Wheel L'], [(0.0, 1.2, 0.0), (0.0, 0.0, -9.0)]), (['Motor L', 'Face plate L'], [(0.0, 0.0, 10.0)]),
          (['Arduino GIGA R1', 'Main PCB'], [(0.0, 0.0, 10.0)]),
          (['Wheel R'], [(0.0, -1.2, 0.0), (0.0, 0.0, -9.0)]), (['Motor R', 'Face plate R'], [(0.0, 0.0, 10.0)]),
          (['Omni pivot'], [(0.0, -3.6, 0.0)]),
          (['Omni pins'], [(0.0, -0.35, 0.0)]),
          (['Omni wheel', 'Omni pins'], [(0.0, 0.0, -9.0)]),
          (['Omni arm', 'Omni spring'], [(0.0, 0.0, 12.0)])]),
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


def report_tof(ctx, half_angle=12.5, reach=12.0):
    """Nine 25 degree cones, 17 rays each (the axis and two rings of eight), from 1 mm in front of the chip's face, against every part but the board's own. Expect no ray blocked: the tunnels, the recess and the ring
    must leave the cone free, and so must the lid, the cameras and the dropper."""
    root = ctx.root
    print('ToF cones (rev 4 positions), half angle %.1f deg, blocked rays / 17 and the first blocker:' % half_angle)
    for nm, (x, y, aim) in T.sensors().items():
        a = math.radians(aim)
        u = (math.cos(a), math.sin(a), 0.0)
        n = (-math.sin(a), math.cos(a), 0.0)
        o = T.beam_origin(nm)
        dirs = [u]
        for rho in (half_angle / 2, half_angle):
            for k in range(8):
                ph = math.radians(45 * k)
                side = L.vadd(L.vmul(n, math.cos(ph)), (0.0, 0.0, math.sin(ph)))
                dirs.append(L.unit(L.vadd(L.vmul(u, math.cos(math.radians(rho))), L.vmul(side, math.sin(math.radians(rho))))))
        blocked, first = 0, None
        for d in dirs:
            hits = [h for h in K3._cast(root, o, d, own=('ToF ' + nm,)) if h[0] < reach]
            if hits:
                blocked += 1
                if first is None or hits[0][0] < first[0]:
                    first = (round(hits[0][0], 2), hits[0][1])
        print('  %-4s blocked %2d / %d  first blocker %s' % (nm, blocked, len(dirs), first))
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
    ('Chute B left', 'Wheel L', 5.6, 5.0, 'chute to the left wheel (rev 3 6.7 mm; 8.2 mm with the first rev 4 chute, 5.6 mm with the 18 mm channel)'),
    ('Chute A right', 'Wheel R', 5.6, 5.0, 'chute to the right wheel (mirror of the left)'),
    ('Omni wheel', 'GIGA posts', 2.9, 2.0, 'omni wheel to GIGA post H1 (spec 2.9 mm)'),
    ('Wi-Fi antenna', 'Bumper switch R', None, 3.0, 'Wi-Fi antenna to the right bumper switch'),
    ('Wi-Fi antenna', 'USB-C service socket', None, 3.0, 'Wi-Fi antenna to the USB-C socket above it'),
    ('Omni wheel', 'Tub', 1.5, 1.0, 'omni wheel to the floor opening and the pillar at rest (bay clearance 1.5 mm)'),
    ('Omni wheel', 'Omni pivot', None, 3.0, 'omni wheel to the stop screw head at rest'),
    ('Omni arm', 'Battery', None, 1.5, 'omni arm to the battery at rest'),
    ('Arduino GIGA R1', 'Tub', 1.7, 1.0, 'GIGA stack to the tub (3.1 mm plain wall, 1.7 mm behind the right bumper)'),
    ('Battery', 'Motor L', 1.0, 0.5, 'battery above the left drive motor (spec 1.0 mm)'),
    ('Dropper plate', 'Lid', None, 5.0, 'kit plate to the lid'),
    ('Camera L', 'Wheel L', 3.6, 2.5, 'camera board and holder to the wheel top (rev 3 3.6 mm with the lens block; the real board is taller)'),
    ('Camera cage L', 'Wheel L', None, 2.5, 'camera cage to the wheel top'),
    ('Camera L', 'Lid', None, 1.5, 'camera board and screw heads to the lid hump ceiling'),
    ('Camera cage L', 'Lid', None, 1.5, 'camera cage to the lid'),
    ('Camera L', 'Battery', None, 3.0, 'left camera tail to the battery'),
    ('Camera R', 'Arduino GIGA R1', None, 3.0, 'right camera tail to the GIGA board (the tail points forward on both sides)'),
    ('Camera L', 'Dropper plate', None, 5.0, 'left camera to the kit plate'),
    ('BNO055 IMU', 'N20 motor', None, 5.0, 'IMU board to the N20 gearmotor above and ahead of it'),
    ('BNO055 IMU', 'Hopper A right', None, 10.0, 'IMU board to the right hopper above it'),
    ('Floor port FP', 'Omni arm', None, 3.0, 'front floor port (the real 20.3 mm board) to the omni arm'),
    ('Floor port FP', 'GIGA posts', None, 3.0, 'front floor port to the GIGA post H1'),
    (GHOST, 'Main PCB', 5.0, 3.0, 'stepper bay to the GIGA stack, nearest at the shield (spec 5.0 mm in plan view; 3D is larger where the heights differ)'),
    (GHOST, 'Hopper B left', 4.0, 3.0, 'stepper bay to hopper B (spec 4.0 mm; 5.2 mm with the first hopper)'),
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


def _named_body(ctx, comp, body):
    for b in _bodies(ctx, comp):
        if b.name == body:
            return b
    raise KeyError((comp, body))


LEG_PAIRS = [   # (component, body, component, body, smallest acceptable mm, label): the forward spring leg and its seat pin turn with the arm, so the model (at rest) is their closest position to the stop screw's rest end
    ('Omni arm', 'Spring forward leg', 'Omni pivot', 'Stop screw', 1.0, 'forward spring leg over the stop screw (at rest, the screw is at the rest end of the slot: the closest position; 1.07 mm by the 2D check over the travel)'),
    ('Omni arm', 'Seat pin', 'Omni pivot', 'Stop screw', 1.0, 'seat pin to the stop screw (rest)'),
    ('Omni arm', 'Spring forward leg', 'Omni wheel', 'Omni wheel', 1.5, 'forward spring leg tip to the wheel rim (the wheel is a plain cylinder here, its rollers stand out of its plates)'),
    ('Omni arm', 'Seat pin', 'Omni wheel', 'Omni wheel', 1.5, 'seat pin to the wheel rim'),
    ('Omni spring', 'Spring arbor', 'Omni spring', 'Spring coil', 0.4, 'arbor to the coil (0.5 mm of air all round when the coil is free)'),
    ('Omni spring', 'Spring rear leg', 'Omni adjuster', 'Adjuster screw head', 0.0, 'rear leg on the adjuster head (they touch by design: the leg lies on it)'),
]


def report_legs(ctx):
    """10 Oct: the forward spring leg and its seat pin (parts of the arm's component, they turn with it), the arbor and the rear leg, body by body, at rest. The sweep report covers the motion."""
    mm = ctx.app.measureManager
    lines, ok = [], True
    for ca, ba, cb, bb, minimum, label in LEG_PAIRS:
        d = mm.measureMinimumDistance(_named_body(ctx, ca, ba), _named_body(ctx, cb, bb)).value * 10.0
        good = d >= minimum - 1e-6
        ok = ok and good
        lines.append('  %s %6.2f mm  (at least %.1f)  %-22s | %-18s  %s' % ('ok  ' if good else 'FAIL', d, minimum, ba, bb, label))
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
    lines.append('front load %.1f %% at the omni %.2f cm ahead (rev 3: 11.9 %% at 7.95)' % (100.0 * X / P.OMNI['rest'][0], P.OMNI['rest'][0]))
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
