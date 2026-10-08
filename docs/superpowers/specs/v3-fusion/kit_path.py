"""Kit path check on the finished rev 4 model: can a 10.3 mm rescue-kit cube get from its plate pocket to the outside? Static, geometry only (whether a real kit tumbles into the channel
and slides at 32 degrees is the bench test, spec section 12). Made after the plan's reports; not part of the plan's code and not in the dry-run emulator.

1. Drop: the cube, turned like the slot (a 14.5 mm square at 45 degrees), falls straight down the slot and the hopper from where it rests in the plate pocket (z 9.0) to the bottom of the
   channel's open trough (z 7.7; below that the cube has to be in the bore, turned along it, which is step 2).
2. Turning room: the channel runs at an angle to the slot square, so the cube must turn in the trough on its way into the bore. The line states how far the cube can turn inside the 14.5 mm
   square against how far the channel needs, and how far it may be off the channel axis and still enter the 13 mm bore. Arithmetic, not a sweep.
3. Slide: the cube, turned like the channel (faces along and across the 13 mm bore), moves along the channel axis from the hopper to 2 cm beyond the exit, centred and shifted 1.2 mm
   to each side and up and down (the bore leaves 1.35 mm).
Steps 1 and 3 are tested against every body except the kits, the plate and the hidden stepper bay (temporary B-rep intersections, overlaps under 1e-3 cm3 ignored). Two controls, a cube too
big for the slot and a cube too big for the bore, must be blocked, or the check could not see a blockage and fails. Whether a real kit tumbles into the channel and keeps sliding at 32 degrees
is not geometry: it is the bench test.

Inside a Fusion script (see README.md, Rebuild):
    import sys; sys.path.insert(0, r"<repo>\\docs\\superpowers\\specs\\v3-fusion"); import kit_path
    def run(context): print(kit_path.report(sides=(1,)))        # 1 = channel B (left), -1 = channel A (right); one side per call stays inside the 60 s tool limit
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(os.path.dirname(HERE), 'v3-checks')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import adsk.core
import adsk.fusion
import fusion_lib as L
import v3_params4 as P

SKIP = ('Kits', 'Dropper plate', 'Stepper bay 28BYJ-48')      # the kits are the thing that moves; the plate is only the pocket they start in
FLOOR = 1e-3
TROUGH_Z = 7.7                                                  # the channel's top is cut open over the 14.5 mm square above this height (v4_model.build_chutes, 'Trough cut')


def _targets(root):
    """(label, temporary copy, bounding box) of every body that can block the kit."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    out = []
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name in SKIP:
            continue
        for j in range(o.bRepBodies.count):
            b = o.bRepBodies.item(j)
            out.append(('%s / %s' % (o.component.name, b.name), tbm.copy(b), b.boundingBox))
    return out


def _worst(cube, targets):
    """Largest overlap volume of the temporary box `cube` with any target, as (cm3, label)."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    bb = cube.boundingBox
    worst = (0.0, None)
    for label, tool, tb in targets:
        if (bb.maxPoint.x < tb.minPoint.x or bb.minPoint.x > tb.maxPoint.x or bb.maxPoint.y < tb.minPoint.y or bb.minPoint.y > tb.maxPoint.y
                or bb.maxPoint.z < tb.minPoint.z or bb.minPoint.z > tb.maxPoint.z):
            continue
        c = tbm.copy(cube)
        try:
            v = c.volume if tbm.booleanOperation(c, tool, adsk.fusion.BooleanTypes.IntersectionBooleanType) else 0.0
        except Exception:
            v = 0.0
        if v > worst[0]:
            worst = (v, label)
    return worst


def _box(center, ex, ey, size):
    obb = adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(*center), adsk.core.Vector3D.create(*ex), adsk.core.Vector3D.create(*ey), size, size, size)
    return adsk.fusion.TemporaryBRepManager.get().createBox(obb)


def drop(side, targets, size=P.KIT, step=0.1):
    """Cube falling down the slot and the hopper. Returns (worst overlap, number of steps)."""
    pl = P.PLATE
    sx, sy = P.slot_xy('B' if side > 0 else 'A')
    yaw = math.atan2(sy - pl['cy'], sx - pl['cx'])               # the pocket's turn when it is over the slot
    ex, ey = (math.cos(yaw), math.sin(yaw), 0.0), (-math.sin(yaw), math.cos(yaw), 0.0)
    z_top, z_end = pl['z0'] + size / 2, TROUGH_Z + size / 2
    worst, n, z = (0.0, None), 0, z_top
    while z >= z_end - 1e-9:
        w = _worst(_box((sx, sy, z), ex, ey, size), targets)
        n += 1
        if w[0] > worst[0]:
            worst = w
        z -= step
    return worst, n


def slide(side, targets, size=P.KIT, offsets=((0.0, 0.0), (0.12, 0.0), (-0.12, 0.0), (0.0, 0.12), (0.0, -0.12)), step=0.3):
    """Cube sliding along the channel axis. Returns (worst overlap, number of positions)."""
    p0, e = P.chute_ends(side)
    d = L.unit(L.vsub(e, p0))
    ey0 = L.unit(L.vsub((0.0, 0.0, 1.0), L.vmul(d, L.dot((0.0, 0.0, 1.0), d))))      # the channel's frame (fusion_lib.axis_prism): local y as vertical as possible
    ex0 = L.cross(ey0, d)
    length = L.vlen(L.vsub(e, p0)) + 2.0
    worst, n = (0.0, None), 0
    for ox, oy in offsets:
        t = size / 2 + 0.2
        while t <= length:
            c = L.vadd(L.vadd(p0, L.vmul(d, t)), L.vadd(L.vmul(ex0, ox), L.vmul(ey0, oy)))
            w = _worst(_box(c, ex0, ey0, size), targets)
            n += 1
            if w[0] > worst[0]:
                worst = w
            t += step
    return worst, n


def turning_room(side, size=P.KIT):
    """(angle the channel needs, angle the cube can turn inside the slot square, angle it may be off the channel axis and still enter the bore), degrees."""
    p0, e = P.chute_ends(side)
    chan = math.degrees(math.atan2(e[1] - p0[1], e[0] - p0[0]))
    delta = (chan - 45.0) % 90.0                                 # the slot square has sides at 45 + 90 k degrees
    need = min(delta, 90.0 - delta)                              # angle from the nearest side to the channel direction
    can = math.degrees(math.asin(min(1.0, P.PLATE['slot'] / (size * math.sqrt(2))))) - 45.0
    enter = math.degrees(math.asin(min(1.0, P.CHUTE['in_w'] / (size * math.sqrt(2))))) - 45.0
    return need, can, enter


def report(sides=(1, -1)):
    root = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
    targets = _targets(root)
    lines, ok = [], True
    for side in sides:
        nm = 'B (left)' if side > 0 else 'A (right)'
        (v, who), n = drop(side, targets)
        good = v <= FLOOR
        ok = ok and good
        lines.append('  %s kit %.1f mm falls from the plate pocket through slot %s and its hopper to the trough floor (z %.1f): %d positions, worst overlap %.4f cm3%s' % ('ok  ' if good else 'FAIL', P.KIT * 10, nm, TROUGH_Z, n, v, '' if good else '  with ' + who))
        need, can, enter = turning_room(side)
        good = need < can
        ok = ok and good
        lines.append('  %s channel %s runs %.1f degrees off the slot square; the cube can turn %.1f degrees inside the 14.5 mm square (room %.1f degrees) and enter the 13 mm bore up to %.1f degrees off its axis' % ('ok  ' if good else 'FAIL', nm, need, can, can - need, enter))
        (v, who), n = slide(side, targets)
        good = v <= FLOOR
        ok = ok and good
        lines.append('  %s kit %.1f mm slides down channel %s to 2 cm beyond the exit: %d positions (centred, 1.2 mm to each side and up and down), worst overlap %.4f cm3%s' % ('ok  ' if good else 'FAIL', P.KIT * 10, nm, n, v, '' if good else '  with ' + who))
    side = sides[0]
    (v, who), n = drop(side, targets, size=1.47, step=0.5)
    good = v > FLOOR
    ok = ok and good
    lines.append('  %s control: a 14.7 mm cube in the slot is %s (%.4f cm3 with %s)' % ('ok  ' if good else 'FAIL', 'blocked, as it must be' if good else 'NOT blocked: the check cannot see a blockage', v, who))
    (v, who), n = slide(side, targets, size=1.40, offsets=((0.0, 0.0),), step=0.5)
    good = v > FLOOR
    ok = ok and good
    lines.append('  %s control: a 14.0 mm cube in the 13 mm channel is %s (%.4f cm3 with %s)' % ('ok  ' if good else 'FAIL', 'blocked, as it must be' if good else 'NOT blocked: the check cannot see a blockage', v, who))
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)
