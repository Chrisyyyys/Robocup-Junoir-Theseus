"""Does the emulator find a known overlap, a known cut, a placed component, a body along a sloped axis and a rotation? Run before trusting a 'no overlap' from the dry run.
Usage: python selftest.py (exits 1 on a failure)."""
import math
import sys

import numpy as np

import fake_fusion as F

F.install(axis_prism_source=None if hasattr(F.L, 'axis_prism') else (
    "def axis_prism(comp, name, shape, p_start, p_end):\n"
    "    d = vsub(p_end, p_start)\n"
    "    body = prism(comp, name, 'xy', [shape], 0.0, vlen(d))\n"
    "    move_body(comp, body, frame_from_zy(p_start, d, (0.0, 0.0, 1.0) if abs(unit(d)[2]) < 0.99 else (1.0, 0.0, 0.0)))\n"
    "    return body\n"))
L = F.L


def run():
    ok = True

    def check(label, got, want, tol=None):
        nonlocal ok
        good = (abs(got - want) <= tol) if tol is not None else (got == want)
        ok = ok and good
        print('  %s %-46s got %-10s want %s' % ('ok  ' if good else 'FAIL', label, ('%.4f' % got) if isinstance(got, float) else got, want))

    root = F.Root()
    oa, ca = L.new_part(root, 'A')
    ob, cb = L.new_part(root, 'B')
    a = L.prism(ca, 'box', 'xy', [L.rect(0, 0, 2.0, 2.0)], 0.0, 2.0)                       # x -1..1, y -1..1, z 0..2
    b = L.prism(cb, 'box', 'xy', [L.rect(1.0, 0.0, 2.0, 2.0)], 1.0, 3.0)                   # x 0..2, y -1..1, z 1..3: overlap x 0..1, z 1..2 = 2 cm3
    check('overlap of two boxes (cm3)', F.pair_overlap(a, b), 2.0, 0.1)
    oc, cc = L.new_part(root, 'C')
    c = L.prism(cc, 'shell', 'xy', [L.circle((0, 0), 5.0)], 0.0, 1.0)
    L.prism(cc, 'cavity', 'xy', [L.circle((0, 0), 4.0)], 0.2, 2.0, op='CUT', targets=[c])
    od, cd = L.new_part(root, 'D')
    d = L.prism(cd, 'bar', 'xz', [L.rect(0.0, 0.5, 2.0, 0.5)], -1.0, 1.0)                  # x -1..1, y -1..1, z 0.25..0.75: inside the cavity
    check('a bar in the cavity does not overlap (cm3)', F.pair_overlap(c, d), 0.0, 1e-9)
    oe, ce = L.new_part(root, 'E')
    e = L.prism(ce, 'low', 'xz', [L.rect(0.0, 0.1, 2.0, 0.1)], -1.0, 1.0)                  # z 0.05..0.15: inside the floor of the shell, 2 x 2 x 0.1 = 0.4 cm3
    check('a bar in the shell floor (cm3)', F.pair_overlap(c, e), 0.4, 0.02)
    P3 = L.P3
    inside = F.fus.PointContainment.PointInsidePointContainment
    check('pointContainment inside', a.pointContainment(P3(0, 0, 1)) == inside, True)
    check('pointContainment outside', a.pointContainment(P3(3, 0, 1)) == inside, False)
    check('pointContainment in a cut cavity', c.pointContainment(P3(0, 0, 0.5)) == inside, False)
    of, cf = L.new_part(root, 'F')
    t = L.axis_prism(cf, 'tube', L.rect(0, 0, 1.0, 1.0), (4.0, 0.0, 2.0), (7.0, 0.0, 5.0))
    check('sloped body: point on its axis', t.pointContainment(P3(5.5, 0, 3.5)) == inside, True)
    check('sloped body: point off its axis', t.pointContainment(P3(5.5, 0.6, 3.5)) == inside, False)
    check('sloped body: point beyond its end', t.pointContainment(P3(7.5, 0, 5.5)) == inside, False)
    m = L.frame_from_zy((10.0, 0.0, 5.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    og, cg = L.new_part(root, 'G', m)
    g = L.prism(cg, 'lens', 'xy', [L.rect(0, 0, 1.0, 1.0)], 0.0, 2.0)                    # local z 0..2 -> world x 10..12
    check('placed component: inside', g.pointContainment(P3(11.0, 0.0, 5.0)) == inside, True)
    check('placed component: outside', g.pointContainment(P3(9.0, 0.0, 5.0)) == inside, False)
    rot = F.core.Matrix3D.create()
    rot.setToRotation(math.pi / 2, F.core.Vector3D.create(0, 1, 0), F.core.Point3D.create(0, 0, 0))   # right-handed about +y: (1, 0, 0) -> (0, 0, -1)
    p = F.core.Point3D.create(1, 0, 0)
    p.transformBy(rot)
    check('rotation about +y by 90 degrees: x', p.x, 0.0, 1e-9)
    check('rotation about +y by 90 degrees: z', p.z, -1.0, 1e-9)
    tbm = F.fus.TemporaryBRepManager.get()
    t1, t2 = tbm.copy(a), tbm.copy(b)
    mv = F.core.Matrix3D.create()
    mv.translation = F.core.Vector3D.create(0, 5.0, 0)
    tbm.transform(t1, mv)
    tbm.booleanOperation(t1, t2, F.fus.BooleanTypes.IntersectionBooleanType)
    check('temporary B-rep moved away: intersection (cm3)', t1.volume, 0.0, 1e-9)
    return ok


if __name__ == '__main__':
    good = run()
    print('RESULT:', 'PASS' if good else 'FAIL')
    sys.exit(0 if good else 1)
