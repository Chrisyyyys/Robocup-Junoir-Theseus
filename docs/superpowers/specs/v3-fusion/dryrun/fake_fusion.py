"""A small geometry emulator for the part of the Autodesk Fusion API that fusion_lib, v3_model, v4_model and v4_checks3d use, so the rev 4 build and its checks can be dry-run
without Fusion (see README.md in this folder).

A body is an ordered list of operations (an extruded 2D shape or a boolean tool body, each with a placement frame). Point containment is exact (strictly inside or not, like Fusion's
pointContainment); overlap volumes are Monte-Carlo estimates; distances are refined sampling. Not emulated: real boolean topology, meshes, the interference analysis, sketches.
Importing this module installs fake `adsk` modules, so import it before fusion_lib / v3_model / v4_*, then call install()."""
import math
import os
import sys
import types
import zlib

import numpy as np
import shapely
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
FUSION_DIR = os.path.dirname(HERE)                                  # docs/superpowers/specs/v3-fusion
CHECKS_DIR = os.path.join(os.path.dirname(FUSION_DIR), 'v3-checks')
for _p in (FUSION_DIR, CHECKS_DIR):
    if _p not in sys.path:
        sys.path.append(_p)

WARN = []                      # (category, message): problems the emulator noticed while building (a cut that removes nothing, a join that touches nothing)
RNG = np.random.default_rng(12345)


# ------------------------------------------------------------------------------------------ fake adsk
def _apply(m, pts):
    return (m[:3, :3] @ pts.T).T + m[:3, 3]


class _P3:
    def __init__(self, x=0.0, y=0.0, z=0.0): self.x, self.y, self.z = float(x), float(y), float(z)
    @staticmethod
    def create(x=0.0, y=0.0, z=0.0): return _P3(x, y, z)
    def asArray(self): return (self.x, self.y, self.z)
    def transformBy(self, m):
        self.x, self.y, self.z = (float(v) for v in _apply(m.m, np.array([[self.x, self.y, self.z]]))[0])
        return True


class _V3(_P3):
    @staticmethod
    def create(x=0.0, y=0.0, z=0.0): return _V3(x, y, z)


class _M3:
    def __init__(self): self.m = np.eye(4)
    @staticmethod
    def create(): return _M3()
    def setWithCoordinateSystem(self, origin, ex, ey, ez):
        self.m = np.eye(4)
        self.m[:3, 0], self.m[:3, 1], self.m[:3, 2], self.m[:3, 3] = ex.asArray(), ey.asArray(), ez.asArray(), origin.asArray()
        return True
    @property
    def translation(self): return _V3(*self.m[:3, 3])
    @translation.setter
    def translation(self, v): self.m[:3, 3] = v.asArray()
    def setToRotation(self, angle, axis, origin):
        """Right-handed rotation by `angle` about `axis` through `origin` (Rodrigues)."""
        a = np.array(axis.asArray(), float)
        a /= np.linalg.norm(a)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
        o = np.array(origin.asArray(), float)
        self.m = np.eye(4)
        self.m[:3, :3] = R
        self.m[:3, 3] = o - R @ o
        return True


class _VI:
    @staticmethod
    def createByReal(v): return v


class _Ops:
    NewBodyFeatureOperation, JoinFeatureOperation, CutFeatureOperation = 'NEW', 'JOIN', 'CUT'


core = types.ModuleType('adsk.core')
core.Point3D, core.Vector3D, core.Matrix3D, core.ValueInput = _P3, _V3, _M3, _VI
fus = types.ModuleType('adsk.fusion')
fus.FeatureOperations = _Ops
fus.PointContainment = types.SimpleNamespace(PointInsidePointContainment='in', PointOnPointContainment='on', PointOutsidePointContainment='out')
fus.BooleanTypes = types.SimpleNamespace(IntersectionBooleanType='intersect', UnionBooleanType='union', DifferenceBooleanType='difference')
adsk = types.ModuleType('adsk')
adsk.core, adsk.fusion = core, fus
sys.modules.update({'adsk': adsk, 'adsk.core': core, 'adsk.fusion': fus})

import fusion_lib as L  # noqa: E402  (the real one: the 2D shapes and the vector maths are pure Python)


# ------------------------------------------------------------------------------------------ 2D shapes -> shapely
def _arc(a, m, b, n=64):
    (ax, ay), (bx, by), (cx, cy) = a, m, b
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    assert abs(d) > 1e-12, 'collinear arc points'
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    t0, t1, t2 = (math.atan2(p[1] - uy, p[0] - ux) for p in (a, m, b))
    two = 2 * math.pi
    sweep = ((t2 - t0) % two) if ((t1 - t0) % two) < ((t2 - t0) % two) else -((t0 - t2) % two)
    return [(ux + r * math.cos(t0 + sweep * k / n), uy + r * math.sin(t0 + sweep * k / n)) for k in range(n + 1)]


def shape_poly(sh, name=''):
    if sh['t'] == 'C':
        assert sh['r'] > 0, 'circle radius <= 0 in ' + name
        return Point(sh['c']).buffer(sh['r'], 128)
    pts = []
    for seg in sh['segs']:
        if seg[0] == 'L':
            pts.append(tuple(seg[1]))
        else:
            pts += _arc(seg[1], seg[2], seg[3])[:-1]
    poly = Polygon(pts)
    assert poly.is_valid and poly.area > 1e-9, 'bad polygon in %s (valid %s, area %.3g)' % (name, poly.is_valid, poly.area)
    return poly


def shapes_poly(shapes, name=''):
    return unary_union([shape_poly(s, name) for s in shapes])


# ------------------------------------------------------------------------------------------ bodies as operation lists
class Body:
    def __init__(self, name, comp):
        self.name, self.comp, self.ops = name, comp, []
        self.opacity = self.appearance = None
        self._mp = None

    # an op: {'kind': NEW|JOIN|CUT, 'frame': 4x4 (op-local -> body coordinates), 'plane', 'geom', 'a0', 'a1', 'name'} or {..., 'tool': Body}
    def _op_box(self, op):
        """Axis-aligned box of one op in body coordinates (cached; move_body clears the cache)."""
        if 'bb' not in op:
            if 'tool' in op:
                l2, h2 = op['tool'].bbox_all()
                corners = np.array([[x, y, z] for x in (l2[0], h2[0]) for y in (l2[1], h2[1]) for z in (l2[2], h2[2])])
            else:
                x0, y0, x1, y1 = op['geom'].bounds
                pl = op['plane']
                corners = []
                for (u, v) in [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]:
                    for w in (op['a0'], op['a1']):
                        corners.append((u, v, w) if pl == 'xy' else ((u, w, v) if pl == 'xz' else (w, u, v)))
                corners = np.array(corners)
            wc = _apply(op['frame'], corners)
            op['bb'] = (wc.min(axis=0), wc.max(axis=0))
        return op['bb']

    def inside(self, pts):
        """pts: (n, 3) in component coordinates -> bool array (strictly inside)."""
        state = np.zeros(len(pts), bool)
        if len(pts) == 0:
            return state
        plo, phi = pts.min(axis=0), pts.max(axis=0)
        for op in self.ops:
            lo, hi = self._op_box(op)
            if np.any(phi < lo) or np.any(plo > hi):
                continue                                   # the op cannot touch any of these points
            q = _apply(np.linalg.inv(op['frame']), pts)
            if 'tool' in op:
                ins = op['tool'].inside(q)
            else:
                pl = op['plane']
                u, v, w = (q[:, 0], q[:, 1], q[:, 2]) if pl == 'xy' else ((q[:, 0], q[:, 2], q[:, 1]) if pl == 'xz' else (q[:, 1], q[:, 2], q[:, 0]))
                ins = (w > op['a0']) & (w < op['a1'])
                if ins.any():
                    ins &= shapely.contains_xy(op['geom'], u, v)
            if op['kind'] in ('NEW', 'JOIN'):
                state |= ins
            else:
                state &= ~ins
        return state

    def bbox_all(self):
        """Box of every NEW and JOIN op (cuts ignored), in component coordinates: a safe over-estimate of the body."""
        lo, hi = np.full(3, 1e9), np.full(3, -1e9)
        for op in self.ops:
            if op['kind'] == 'CUT':
                continue
            l2, h2 = self._op_box(op)
            lo, hi = np.minimum(lo, l2), np.maximum(hi, h2)
        return lo, hi

    # ---- the bits of the Fusion body API that the report code reads
    @property
    def boundingBox(self):
        a, b = world_box(self)
        return types.SimpleNamespace(minPoint=_P3(*a), maxPoint=_P3(*b))

    def _mass(self):
        if self._mp is None:
            rng = np.random.default_rng(zlib.crc32((self.comp.name + self.name).encode()))
            lo, hi = self.bbox_all()
            vol_box = float(np.prod(hi - lo))
            n_in, acc, sx = 0, 0, np.zeros(3)
            for _ in range(8):
                p = rng.uniform(lo, hi, size=(100000, 3))
                ok = self.inside(p)
                n_in += int(ok.sum())
                acc += 100000
                sx += _apply(self.comp.matrix, p[ok]).sum(axis=0)
            self._mp = (vol_box * n_in / acc, sx / max(n_in, 1))
        return self._mp

    @property
    def volume(self):
        return self._mass()[0]

    @property
    def physicalProperties(self):
        return types.SimpleNamespace(centerOfMass=_P3(*self._mass()[1]))

    def sample(self, n, rng=RNG, tries=40):
        """n points strictly inside the body, in component coordinates."""
        lo, hi = self.bbox_all()
        out, got = [], 0
        for _ in range(tries):
            p = rng.uniform(lo, hi, size=(max(20000, 4 * n), 3))
            p = p[self.inside(p)]
            out.append(p)
            got += len(p)
            if got >= n:
                break
        pts = np.vstack(out) if out else np.zeros((0, 3))
        return pts[:n]

    def pointContainment(self, p):
        loc = _apply(np.linalg.inv(self.comp.matrix), np.array([[p.x, p.y, p.z]]))
        return fus.PointContainment.PointInsidePointContainment if self.inside(loc)[0] else fus.PointContainment.PointOutsidePointContainment


class Coll:
    def __init__(self, items): self._items = items
    @property
    def count(self): return len(self._items)
    def item(self, i): return self._items[i]


class Comp:
    def __init__(self, name, matrix=None, parent=None):
        self.name, self.bodies, self.parent = name, [], parent
        self.matrix = np.eye(4) if matrix is None else matrix
        self._occ = []
    @property
    def occurrences(self): return Coll(self._occ)
    @property
    def bRepBodies(self): return Coll(self.bodies)


class Occ:
    def __init__(self, comp, parent):
        self.component, self._parent, self.isLightBulbOn = comp, parent, True
    @property
    def bRepBodies(self): return Coll(self.component.bodies)
    @property
    def name(self): return self.component.name
    def deleteMe(self): self._parent._occ.remove(self)


class Root(Comp):
    def __init__(self):
        super().__init__('root')


# ------------------------------------------------------------------------------------------ contact tests (Fusion rejects a cut that removes nothing and a join that touches nothing)
def _to3(plane, u, v, w):
    if plane == 'xy':
        return np.stack([u, v, w], 1)
    if plane == 'xz':
        return np.stack([u, w, v], 1)
    return np.stack([w, u, v], 1)


def _contact(target, plane, poly, a0, a1, n=80000):
    """(overlaps, touches): random points inside the extrusion (overlap), points just outside its end faces and just outside its side faces (contact) against the target body."""
    e = 0.004
    x0, y0, x1, y1 = poly.bounds
    u = RNG.uniform(x0, x1, n)
    v = RNG.uniform(y0, y1, n)
    ok = shapely.contains_xy(poly, u, v)
    u, v = u[ok], v[ok]
    w = RNG.uniform(a0, a1, len(u))
    over = bool(len(u)) and bool(target.inside(_to3(plane, u, v, w)).any())
    m = min(len(u), 4000)
    touch = bool(target.inside(_to3(plane, u[:m], v[:m], np.full(m, a0 - e))).any()) or bool(target.inside(_to3(plane, u[:m], v[:m], np.full(m, a1 + e))).any())
    if not (over or touch):
        buf = poly.buffer(e)
        bnd = buf.exterior if buf.geom_type == 'Polygon' else buf.geoms[0].exterior
        ts = np.linspace(0.0, bnd.length, max(200, int(bnd.length / 0.03)))
        pts = np.array([bnd.interpolate(t).coords[0] for t in ts])
        for wl in np.linspace(a0 + 0.01, a1 - 0.01, 9):
            if target.inside(_to3(plane, pts[:, 0], pts[:, 1], np.full(len(pts), wl))).any():
                touch = True
                break
    return over, touch


# ------------------------------------------------------------------------------------------ fusion_lib replacements
LOG = []                       # (component, operation, name) of every feature built


def fake_prism(comp, name, plane, shapes, a0, a1, op='NEW', targets=None):
    assert plane in ('xy', 'xz', 'yz'), plane
    assert a1 > a0 + 1e-9, 'non-positive extrusion %s: %s .. %s' % (name, a0, a1)
    assert shapes, 'no shapes for ' + name
    geom = shapes_poly(shapes, name)
    if op == 'NEW':
        parts = list(geom.geoms) if geom.geom_type == 'MultiPolygon' else [geom]
        bodies = []
        for i, g in enumerate(parts):
            b = Body(name if len(parts) == 1 else '%s %d' % (name, i + 1), comp)
            b.ops.append({'kind': 'NEW', 'frame': np.eye(4), 'plane': plane, 'geom': g, 'a0': a0, 'a1': a1, 'name': name})
            comp.bodies.append(b)
            bodies.append(b)
        LOG.append((comp.name, 'NEW', name))
        return bodies[0] if len(bodies) == 1 else bodies
    assert targets, 'JOIN/CUT without targets: ' + name
    for t in targets:
        assert t in comp.bodies, 'target body of %s is not in component %s' % (name, comp.name)
        over, touch = _contact(t, plane, geom, a0, a1)
        if op == 'CUT' and not over:
            WARN.append(('cut removes nothing', '%s / %s' % (comp.name, name)))
        if op == 'JOIN' and not (over or touch):
            WARN.append(('join touches nothing', '%s / %s' % (comp.name, name)))
        t.ops.append({'kind': op, 'frame': np.eye(4), 'plane': plane, 'geom': geom, 'a0': a0, 'a1': a1, 'name': name})
    LOG.append((comp.name, op, name))
    return types.SimpleNamespace(name=name)


def fake_ring_prism(comp, name, plane, outer, inner, a0, a1):
    assert a1 > a0
    g = shape_poly(outer, name).difference(shape_poly(inner, name))
    assert g.geom_type == 'Polygon' and g.area > 1e-9, 'ring_prism %s did not give one ring' % name
    b = Body(name, comp)
    b.ops.append({'kind': 'NEW', 'frame': np.eye(4), 'plane': plane, 'geom': g, 'a0': a0, 'a1': a1, 'name': name})
    comp.bodies.append(b)
    LOG.append((comp.name, 'NEW', name))
    return b


def fake_combine(comp, target, tools, op='CUT', keep=False):
    assert target in comp.bodies, 'combine target not in component'
    for t in tools:
        assert t in comp.bodies, 'combine tool not in component'
        hit = pair_overlap(target, t, n=200000) > 0.0
        if op == 'CUT' and not hit:
            WARN.append(('combine cut removes nothing', '%s / %s from %s' % (comp.name, t.name, target.name)))
        if op == 'JOIN' and not hit:
            WARN.append(('combine join does not overlap', '%s / %s into %s' % (comp.name, t.name, target.name)))
        target.ops.append({'kind': op, 'frame': np.eye(4), 'tool': t, 'name': t.name})
        if not keep:
            comp.bodies.remove(t)
    LOG.append((comp.name, 'COMBINE-' + str(op), target.name))


def fake_move_body(comp, body, matrix):
    for op in body.ops:
        op['frame'] = matrix.m @ op['frame']
        op.pop('bb', None)
    body._mp = None


def fake_new_part(parent, name, matrix=None):
    comp = Comp(name, None if matrix is None else matrix.m.copy(), parent)
    occ = Occ(comp, parent)
    parent._occ.append(occ)
    return occ, comp


def fake_find_occ(comp, name):
    for o in comp._occ:
        if o.component.name == name:
            return o
    return None


class Pal:
    """Stands in for fusion_lib.Palette: only checks that a body is being painted."""
    def paint(self, bodies, colour, opacity=None):
        for b in (bodies if isinstance(bodies, (list, tuple)) else [bodies]):
            assert hasattr(b, 'name'), 'paint target is not a body'

    def get(self, c): return c


def install(axis_prism_source=None):
    """Replace the Fusion-touching functions of fusion_lib with the emulated ones. If fusion_lib does not have axis_prism yet (the plan's Task 2 Step 2 has not been applied),
    pass the plan's replacement for axis_tool as source text."""
    L.prism, L.ring_prism, L.combine, L.new_part, L.find_occ, L.move_body = fake_prism, fake_ring_prism, fake_combine, fake_new_part, fake_find_occ, fake_move_body
    if not hasattr(L, 'axis_prism'):
        assert axis_prism_source, 'fusion_lib has no axis_prism: pass the source of the plan patch'
        exec(compile(axis_prism_source, 'fusion_lib_patch.py', 'exec'), L.__dict__)


# ------------------------------------------------------------------------------------------ overlap between bodies
def world_box(b, xf=None):
    """Axis-aligned world box of a body (all eight corners of its component-space box, through the component matrix and an optional extra world transform xf)."""
    lo, hi = b.bbox_all()
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = _apply(b.comp.matrix, c)
    if xf is not None:
        w = _apply(xf, w)
    return w.min(axis=0), w.max(axis=0)


def pair_overlap(ba, bb, n=40000, xf_a=None, xf_b=None):
    """Monte-Carlo overlap volume (cm3) of two bodies in world coordinates: points drawn uniformly in the intersection of their boxes.
    xf_a / xf_b (4x4, world -> world) move body A / B first (used for the removal paths and the omni sweep)."""
    la, ha = world_box(ba, xf_a)
    lb, hb = world_box(bb, xf_b)
    lo, hi = np.maximum(la, lb), np.minimum(ha, hb)
    if np.any(hi <= lo):
        return 0.0
    vol = float(np.prod(hi - lo))
    pts = RNG.uniform(lo, hi, size=(n, 3))
    inv_a = np.linalg.inv(ba.comp.matrix if xf_a is None else xf_a @ ba.comp.matrix)
    ia = ba.inside(_apply(inv_a, pts))
    if not ia.any():
        return 0.0
    inv_b = np.linalg.inv(bb.comp.matrix if xf_b is None else xf_b @ bb.comp.matrix)
    ib = bb.inside(_apply(inv_b, pts[ia]))
    return vol * float(ib.sum()) / n


# ------------------------------------------------------------------------------------------ temporary B-reps: a body, a placement, and the estimated volume of an intersection
class TempBody:
    def __init__(self, body, xf=None):
        self.body, self.xf, self.volume = body, (np.eye(4) if xf is None else xf), 0.0

    @property
    def boundingBox(self):
        lo, hi = world_box(self.body, self.xf)
        return types.SimpleNamespace(minPoint=_P3(*lo), maxPoint=_P3(*hi))


class _TBM:
    def copy(self, b):
        return TempBody(b.body, b.xf.copy()) if isinstance(b, TempBody) else TempBody(b)

    def transform(self, t, m):
        t.xf = m.m @ t.xf
        return True

    def booleanOperation(self, t1, t2, op):
        assert op == fus.BooleanTypes.IntersectionBooleanType, 'only the intersection is emulated'
        t1.volume = pair_overlap(t1.body, t2.body, n=40000, xf_a=t1.xf, xf_b=t2.xf)
        return True


fus.TemporaryBRepManager = types.SimpleNamespace(get=lambda: _TBM())
