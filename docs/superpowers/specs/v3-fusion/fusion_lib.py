"""Helpers for building the Theseus V3 model in Autodesk Fusion (run inside Fusion's Python, see README.md).

Units: cm and degrees (the Fusion API works in cm). Frame: x forward, y left, z up, origin at the midpoint of the drive axle on the floor line.
Everything is built from base-plane sketches (xy, xz, yz) extruded with an offset start, so no construction planes are needed.
Tilted or sloped parts are placed with a frame matrix (component transform) or a free move.
"""
import math
import adsk.core
import adsk.fusion

P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
_OPS = adsk.fusion.FeatureOperations
NEW, JOIN, CUT = _OPS.NewBodyFeatureOperation, _OPS.JoinFeatureOperation, _OPS.CutFeatureOperation


# ---------------------------------------------------------------------------------------------------- plain-tuple vector maths
def vadd(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def vsub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def vmul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def vlen(a): return math.sqrt(dot(a, a))
def unit(a):
    n = vlen(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def frame_matrix(origin, ex, ey, ez):
    """Matrix3D that maps a part's local axes onto the world vectors ex, ey, ez (right-handed) with the local origin at `origin`."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P3(*origin), V3(*ex), V3(*ey), V3(*ez))
    return m


def frame_from_zy(origin, ez, ey_hint=(0.0, 0.0, 1.0)):
    """Right-handed frame with local z = ez and local y as close to ey_hint as possible."""
    ez = unit(ez)
    ey = unit(vsub(ey_hint, vmul(ez, dot(ey_hint, ez))))
    ex = cross(ey, ez)
    return frame_matrix(origin, ex, ey, ez)


# ---------------------------------------------------------------------------------------------------- 2D shapes in plane coordinates
# plane 'xy': (u, v) = (x, y), extruded along +z.  plane 'xz': (u, v) = (x, z), extruded along +y.  plane 'yz': (u, v) = (y, z), extruded along +x.
def circle(c, r):
    return {'t': 'C', 'c': c, 'r': r}


def poly(pts):
    pts = list(pts)
    n = len(pts)
    return {'t': 'S', 'segs': [('L', pts[i], pts[(i + 1) % n]) for i in range(n)]}


def rect(cx, cy, sx, sy, ang=0.0):
    """Rectangle of size sx (along the direction `ang`, degrees) by sy, centred on (cx, cy)."""
    a = math.radians(ang)
    c, s = math.cos(a), math.sin(a)
    pts = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
    return poly([(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts])


def stadium(p1, p2, r):
    """Slot shape: the convex hull of two circles of radius r centred on p1 and p2."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    nx, ny = -uy, ux
    a1 = (p1[0] + nx * r, p1[1] + ny * r)
    a2 = (p2[0] + nx * r, p2[1] + ny * r)
    b2 = (p2[0] - nx * r, p2[1] - ny * r)
    b1 = (p1[0] - nx * r, p1[1] - ny * r)
    tip2 = (p2[0] + ux * r, p2[1] + uy * r)
    tip1 = (p1[0] - ux * r, p1[1] - uy * r)
    return {'t': 'S', 'segs': [('L', a1, a2), ('A', a2, tip2, b2), ('L', b2, b1), ('A', b1, tip1, a1)]}


def sector(r_in, r_out, a0, a1, center=(0.0, 0.0)):
    """Annular sector between radii r_in and r_out, angles a0 -> a1 degrees (counter-clockwise)."""
    ra0, ra1, am = math.radians(a0), math.radians(a1), math.radians((a0 + a1) / 2.0)

    def pt(r, a):
        return (center[0] + r * math.cos(a), center[1] + r * math.sin(a))
    return {'t': 'S', 'segs': [('A', pt(r_out, ra0), pt(r_out, am), pt(r_out, ra1)), ('L', pt(r_out, ra1), pt(r_in, ra1)),
                               ('A', pt(r_in, ra1), pt(r_in, am), pt(r_in, ra0)), ('L', pt(r_in, ra0), pt(r_out, ra0))]}


# ---------------------------------------------------------------------------------------------------- sketching and extruding
def _plane_obj(comp, plane):
    return {'xy': comp.xYConstructionPlane, 'xz': comp.xZConstructionPlane, 'yz': comp.yZConstructionPlane}[plane]


def _to3d(plane, u, v):
    return {'xy': P3(u, v, 0.0), 'xz': P3(u, 0.0, v), 'yz': P3(0.0, u, v)}[plane]


def _draw(sk, plane, shape):
    def S(p):
        return sk.modelToSketchSpace(_to3d(plane, p[0], p[1]))
    if shape['t'] == 'C':
        sk.sketchCurves.sketchCircles.addByCenterRadius(S(shape['c']), shape['r'])
        return
    segs = shape['segs']
    first, prev = None, None
    for i, seg in enumerate(segs):
        a = S(seg[1])
        b = S(seg[2] if seg[0] == 'L' else seg[3])
        start = prev if prev is not None else a
        last = (i == len(segs) - 1)
        end = first if last else b
        if seg[0] == 'L':
            ent = sk.sketchCurves.sketchLines.addByTwoPoints(start, end)
        else:
            ent = sk.sketchCurves.sketchArcs.addByThreePoints(start, S(seg[2]), end)
        # Fusion stores an arc counter-clockwise, so its start/end points can be swapped relative to the order given: pick by position
        sp0, sp1 = ent.startSketchPoint, ent.endSketchPoint
        if sp0.geometry.distanceTo(a) <= sp1.geometry.distanceTo(a):
            s_pt, e_pt = sp0, sp1
        else:
            s_pt, e_pt = sp1, sp0
        if first is None:
            first = s_pt
        prev = e_pt


def prism(comp, name, plane, shapes, a0, a1, op=NEW, targets=None):
    """Extrude the 2D `shapes` (non-nested) on a base plane from offset a0 to a1 along the plane normal.
    NEW returns the body (or a list of bodies); JOIN and CUT return the feature and act on `targets` (list of bodies in comp)."""
    sk = comp.sketches.add(_plane_obj(comp, plane))
    sk.name = name + ' sketch'
    for sh in shapes:
        _draw(sk, plane, sh)
    n = sk.profiles.count
    if n == 0:
        raise RuntimeError('no closed profile for ' + name)
    col = adsk.core.ObjectCollection.create()
    for i in range(n):
        col.add(sk.profiles.item(i))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(col, op)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(VI(a0))
    inp.setDistanceExtent(False, VI(a1 - a0))
    if op != NEW and targets:
        inp.participantBodies = list(targets)
    feat = ext.add(inp)
    feat.name = name
    sk.isLightBulbOn = False
    if op == NEW:
        bodies = [feat.bodies.item(i) for i in range(feat.bodies.count)]
        for i, b in enumerate(bodies):
            b.name = name if len(bodies) == 1 else '%s %d' % (name, i + 1)
        return bodies[0] if len(bodies) == 1 else bodies
    return feat


def ring_prism(comp, name, plane, outer, inner, a0, a1):
    """Extrude the region between two nested shapes (outer minus inner) as one new body."""
    sk = comp.sketches.add(_plane_obj(comp, plane))
    sk.name = name + ' sketch'
    _draw(sk, plane, outer)
    _draw(sk, plane, inner)
    prof = None
    for i in range(sk.profiles.count):
        if sk.profiles.item(i).profileLoops.count == 2:
            prof = sk.profiles.item(i)
    if prof is None:
        raise RuntimeError('no ring profile for ' + name)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, NEW)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(VI(a0))
    inp.setDistanceExtent(False, VI(a1 - a0))
    feat = ext.add(inp)
    feat.name = name
    sk.isLightBulbOn = False
    body = feat.bodies.item(0)
    body.name = name
    return body


def move_body(comp, body, matrix):
    col = adsk.core.ObjectCollection.create()
    col.add(body)
    mv = comp.features.moveFeatures
    mi = mv.createInput2(col)
    mi.defineAsFreeMove(matrix)
    return mv.add(mi)


def combine(comp, target, tools, op=CUT, keep=False):
    col = adsk.core.ObjectCollection.create()
    for t in tools:
        col.add(t)
    ci = comp.features.combineFeatures.createInput(target, col)
    ci.operation = op
    ci.isKeepToolBodies = keep
    return comp.features.combineFeatures.add(ci)


def axis_prism(comp, name, shape, p_start, p_end):
    """Extrude a 2D `shape` (drawn on a local xy plane, centred on the axis) along the straight line p_start -> p_end; returns the new body.
    The local y axis points as close to world z as possible, so a rectangle has two faces that stay vertical-ish along a sloped axis."""
    d = vsub(p_end, p_start)
    body = prism(comp, name, 'xy', [shape], 0.0, vlen(d))
    move_body(comp, body, frame_from_zy(p_start, d, (0.0, 0.0, 1.0) if abs(unit(d)[2]) < 0.99 else (1.0, 0.0, 0.0)))
    return body


def axis_tool(comp, name, p_start, p_end, r):
    """Solid cylinder of radius r from p_start to p_end (a straight tool body for cutting sloped holes)."""
    return axis_prism(comp, name, circle((0.0, 0.0), r), p_start, p_end)


# ---------------------------------------------------------------------------------------------------- assembly structure
def new_part(parent_comp, name, matrix=None):
    occ = parent_comp.occurrences.addNewComponent(matrix if matrix is not None else adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ, occ.component


def reset_design(design):
    root = design.rootComponent
    for i in reversed(range(root.occurrences.count)):
        root.occurrences.item(i).deleteMe()
    for i in reversed(range(root.sketches.count)):
        root.sketches.item(i).deleteMe()
    for i in reversed(range(root.constructionPlanes.count)):
        root.constructionPlanes.item(i).deleteMe()
    for i in reversed(range(root.bRepBodies.count)):
        root.bRepBodies.item(i).deleteMe()


def find_occ(comp, name):
    for i in range(comp.occurrences.count):
        o = comp.occurrences.item(i)
        if o.component.name == name:
            return o
    return None


def world_bbox(occ):
    """Bounding box (min, max) over all bodies of an occurrence, in the coordinates of the occurrence's parent."""
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for i in range(occ.bRepBodies.count):
        bb = occ.bRepBodies.item(i).boundingBox
        for k, (a, b) in enumerate(((bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y), (bb.minPoint.z, bb.maxPoint.z))):
            lo[k] = min(lo[k], a)
            hi[k] = max(hi[k], b)
    return lo, hi


# ---------------------------------------------------------------------------------------------------- colours
class Palette:
    """Colour appearances, copied from the Fusion appearance library and recoloured."""

    def __init__(self, app, design):
        self.design = design
        lib = app.materialLibraries.itemByName('Fusion Appearance Library')
        self.base = lib.appearances.itemByName('ABS (White)')
        self.cache = {}

    def get(self, hex_color):
        key = hex_color.lstrip('#').upper()
        if key in self.cache:
            return self.cache[key]
        name = 'V3 ' + key
        ap = None
        for i in range(self.design.appearances.count):
            if self.design.appearances.item(i).name == name:
                ap = self.design.appearances.item(i)
        if ap is None:
            ap = self.design.appearances.addByCopy(self.base, name)
            r, g, b = int(key[0:2], 16), int(key[2:4], 16), int(key[4:6], 16)
            prop = adsk.core.ColorProperty.cast(ap.appearanceProperties.itemByName('Color'))
            prop.value = adsk.core.Color.create(r, g, b, 255)
        self.cache[key] = ap
        return ap

    def paint(self, bodies, hex_color, opacity=None):
        if not isinstance(bodies, (list, tuple)):
            bodies = [bodies]
        for b in bodies:
            b.appearance = self.get(hex_color)
            if opacity is not None:
                b.opacity = opacity
