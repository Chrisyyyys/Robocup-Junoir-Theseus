"""3D checks on the finished Fusion model (run inside Fusion, read-only): ToF cones, camera fields of view, omni sweep.
Each check reports what the robot's own parts do to a sensor view or to the moving omni; the maze walls are not modelled."""
import math
import adsk.core
import adsk.fusion
import fusion_lib as L
import v3_model as M

P3, V3 = L.P3, L.V3
prm = M.prm


def _cast(root, origin, direction, own=()):
    """Faces hit by a ray, nearest first, as (distance, component name); hits on components named in `own` are skipped."""
    hp = adsk.core.ObjectCollection.create()
    ents = root.findBRepUsingRay(P3(*origin), V3(*direction), adsk.fusion.BRepEntityTypes.BRepFaceEntityType, -1.0, False, hp)
    rows = []
    for i in range(ents.count):
        e = ents.item(i)
        pt = hp.item(i)
        nm = e.assemblyContext.component.name if e.assemblyContext else e.body.name
        if nm in own:
            continue
        rows.append((math.sqrt((pt.x - origin[0]) ** 2 + (pt.y - origin[1]) ** 2 + (pt.z - origin[2]) ** 2), nm))
    rows.sort()
    return rows


def tof_cones(ctx, half_angle=12.5, reach=12.0):
    """17 rays per sensor inside its 25 degree cone, from the front face of the chip. Reports rays that hit a robot part within `reach` cm."""
    root = ctx.root
    print('ToF cones (%s positions), half angle %.1f deg, blocked rays / 17 and the first blocker:' % (ctx.tof_mode, half_angle))
    for nm, (x, y, aim) in ctx.tof.items():
        a = math.radians(aim)
        u = (math.cos(a), math.sin(a), 0.0)
        n = (-math.sin(a), math.cos(a), 0.0)
        o = (x + u[0] * (prm.TOF_T / 2 + 0.11), y + u[1] * (prm.TOF_T / 2 + 0.11), prm.TOF_Z)
        dirs = [u]
        for rho in (half_angle / 2, half_angle):
            for k in range(8):
                ph = math.radians(45 * k)
                side = L.vadd(L.vmul(n, math.cos(ph)), (0.0, 0.0, math.sin(ph)))
                dirs.append(L.unit(L.vadd(L.vmul(u, math.cos(math.radians(rho))), L.vmul(side, math.sin(math.radians(rho))))))
        blocked, first = 0, None
        for d in dirs:
            hits = [h for h in _cast(root, o, d, own=('ToF ' + nm,)) if h[0] < reach]
            if hits:
                blocked += 1
                if first is None or hits[0][0] < first[0]:
                    first = (round(hits[0][0], 2), hits[0][1])
        print('  %-4s blocked %2d / %d  first blocker %s' % (nm, blocked, len(dirs), first))
    return 'ok'


def camera_fov(ctx, reach=3.0):
    """Rays to the corners, edge midpoints and centre of each camera's 65.9 x 51.8 degree field of view, from the lens front face.
    Reports hits within `reach` cm: that is the window, ring, wall and lid around the lens."""
    root = ctx.root
    t = math.radians(prm.CAM['tilt'])
    psi = math.radians(prm.CAM['psi'])
    th, tv = math.tan(math.radians(prm.CAM['hfov'] / 2)), math.tan(math.radians(prm.CAM['vfov'] / 2))
    for s, tag in ((1, 'L'), (-1, 'R')):
        tip = (prm.CAM['tip_r'] * math.cos(psi), s * prm.CAM['tip_r'] * math.sin(psi), prm.CAM['zl'])
        u = (0.0, s * math.cos(t), -math.sin(t))
        up = (0.0, s * math.sin(t), math.cos(t))
        right = (1.0, 0.0, 0.0)
        blocked, total, first = 0, 0, None
        for a in (-1, -0.5, 0, 0.5, 1):
            for b in (-1, -0.5, 0, 0.5, 1):
                d = L.unit(L.vadd(L.vadd(u, L.vmul(right, a * th)), L.vmul(up, b * tv)))
                o = L.vadd(tip, L.vmul(u, 0.02))
                hits = [h for h in _cast(root, o, d, own=('Camera ' + tag,)) if h[0] < reach]
                total += 1
                if hits:
                    blocked += 1
                    if first is None or hits[0][0] < first[0]:
                        first = (round(hits[0][0], 2), hits[0][1], round(a, 1), round(b, 1))
        print('camera %s: %d / %d view rays hit a robot part within %.1f cm of the lens; first: %s' % (tag, blocked, total, reach, first))
    return 'ok'


def _bodies(ctx, name):
    for i in range(ctx.root.occurrences.count):
        o = ctx.root.occurrences.item(i)
        if o.component.name == name:
            return [o.bRepBodies.item(j) for j in range(o.bRepBodies.count)]
    raise KeyError(name)


def clearances(ctx):
    """Nearest distance between pairs of parts, against the gaps the spec quotes (cm; 0 = touching)."""
    mm = ctx.app.measureManager
    pairs = [('Chute B left', 'Wheel L', 'chute to wheel (spec: 6.7 mm)'), ('Chute A right', 'Wheel R', 'chute to wheel'),
             ('Camera L', 'Wheel L', 'camera lens block to wheel top (spec: 3.6 mm)'), ('Camera R', 'Wheel R', 'camera to wheel'),
             ('Arduino GIGA R1', 'Chassis', 'GIGA stack to wall (packing: 3.1 mm)'), ('Main PCB', 'Chassis', 'PCB shield to wall'),
             ('Battery', 'Motor L', 'battery to motor (packing: 0.86 mm)'), ('Battery', 'Chassis', 'battery to wall'),
             ('Dropper plate', 'Lid', 'plate to lid'), ('Dropper plate', 'Camera L', 'plate to camera'), ('Chute B left', 'Wheel R', 'chute B to the other wheel'),
             ('Chute A right', 'Arduino GIGA R1', 'chute A to GIGA'), ('Chute A right', 'Main PCB', 'chute A to PCB'),
             ('Omni wheel', 'ToF F', 'omni at rest to the front ToF'), ('Omni arm', 'Arduino GIGA R1', 'arm to GIGA at rest'),
             ('N20 motor', 'Motor L', 'N20 to the left drive motor'), ('N20 motor', 'Silver module SM', 'N20 to silver module'),
             ('Camera L', 'Lid', 'camera board to lid hump'), ('Camera L', 'ToF ring', 'camera to ring')]
    for a, b, label in pairs:
        best = None
        for ba in _bodies(ctx, a):
            for bb in _bodies(ctx, b):
                d = mm.measureMinimumDistance(ba, bb).value
                best = d if best is None else min(best, d)
        print('  %5.2f mm   %-26s | %-22s  %s' % (best * 10.0, a, b, label))
    return 'ok'


def omni_sweep(ctx, steps=13, omni=None, omni_at=None, travel=None):
    """Push the sprung omni, its arm plates and pins through the full mechanical travel and intersect with every other body (temporary B-reps, nothing is changed).
    The defaults are the rev 3 omni (prm.OMNI, M.omni_at, M.OMNI_TRAVEL_MECH); rev 4 passes its own."""
    omni = omni or prm.OMNI
    omni_at = omni_at or M.omni_at
    travel = M.OMNI_TRAVEL_MECH if travel is None else travel
    tbm = adsk.fusion.TemporaryBRepManager.get()
    movers, others = [], []
    for i in range(ctx.root.occurrences.count):
        o = ctx.root.occurrences.item(i)
        for j in range(o.bRepBodies.count):
            (movers if o.component.name in ('Omni wheel', 'Omni arm', 'Omni pins') else others).append((o.component.name, o.bRepBodies.item(j)))
    px, pz = omni['pivot']
    ox, oz = omni['rest']
    a_rest = math.atan2(oz - pz, ox - px)
    print('omni sweep: %d moving bodies against %d others' % (len(movers), len(others)))
    worst = {}
    for k in range(steps):
        trav = travel * k / (steps - 1)
        xc, zc = omni_at(trav)
        rot = math.atan2(zc - pz, xc - px) - a_rest
        m = adsk.core.Matrix3D.create()
        m.setToRotation(-rot, V3(0, 1, 0), P3(px, 0, pz))
        probe = P3(ox, 0, oz)
        probe.transformBy(m)
        if abs(probe.x - xc) > 1e-6 or abs(probe.z - zc) > 1e-6:
            raise RuntimeError('rotation sign wrong: probe (%.3f, %.3f) expected (%.3f, %.3f)' % (probe.x, probe.z, xc, zc))
        for mn, mb in movers:
            t1 = tbm.copy(mb)
            tbm.transform(t1, m)
            bb1 = t1.boundingBox
            for on, ob in others:
                bb2 = ob.boundingBox
                if (bb1.maxPoint.x < bb2.minPoint.x or bb1.minPoint.x > bb2.maxPoint.x or bb1.maxPoint.y < bb2.minPoint.y or bb1.minPoint.y > bb2.maxPoint.y
                        or bb1.maxPoint.z < bb2.minPoint.z or bb1.minPoint.z > bb2.maxPoint.z):
                    continue
                t2 = tbm.copy(t1)
                t3 = tbm.copy(ob)
                try:
                    ok = tbm.booleanOperation(t2, t3, adsk.fusion.BooleanTypes.IntersectionBooleanType)
                    v = t2.volume if ok else 0.0
                except Exception:
                    v = 0.0
                if v > 1e-4:
                    key = (mn, on)
                    if key not in worst or v > worst[key][0]:
                        worst[key] = (v, trav)
    if not worst:
        print('  no interference at any of the %d positions between 0 and %.1f cm' % (steps, travel))
    for (mn, on), (v, trav) in sorted(worst.items(), key=lambda kv: -kv[1][0]):
        print('  %s x %s: %.4f cm3 at travel %.2f' % (mn, on, v, trav))
    return 'ok'
