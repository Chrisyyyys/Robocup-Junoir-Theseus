"""Convex-piece model of the kit chute for the rev 4 layout (spec section 6.2): the dropper floor around slot B, the hopper under it and the 18 mm square channel, built from the same
parameters and the same cuts as v4_model.build_chutes, so that a physics engine can run on it. Units cm, frame x forward, y left, z up. Pure numpy, no Fusion and no physics engine.

A convex piece is a list of half-spaces n.x <= b. A solid with a hole is a union of convex pieces: `difference(A, B)` cuts a convex B out of a convex A and returns convex pieces
(A minus B = the union of A and the inside of B's first i-1 faces and the outside of its i-th face). `contains` tests a point against a list of pieces. The check against the real Fusion
body (random points, containment compared) is chute_geometry_check.py; its result is in results/chute_geometry.txt.

The mirror-image channel A (right) is not built: the layout is symmetric about y = 0 and side = -1 only flips y.
"""
import itertools
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import v3_params4 as P

EPS = 1e-9


class Hs:
    """The half-space n.x <= b (n is a unit vector)."""
    __slots__ = ('n', 'b')

    def __init__(self, n, b):
        n = np.asarray(n, float)
        k = np.linalg.norm(n)
        self.n, self.b = n / k, float(b) / k


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def box(origin, ex, ey, ez, x0, x1, y0, y1, z0, z1):
    """Box in the frame (origin, ex, ey, ez), local ranges [x0, x1] x [y0, y1] x [z0, z1]."""
    o = np.asarray(origin, float)
    out = []
    for ax, lo, hi in ((ex, x0, x1), (ey, y0, y1), (ez, z0, z1)):
        ax = unit(ax)
        out.append(Hs(ax, o @ ax + hi))
        out.append(Hs(-ax, -(o @ ax + lo)))
    return out


def rect_prism(cx, cy, w, h, angle_deg, z0, z1):
    """Vertical prism over the w x h rectangle centred (cx, cy), turned by angle_deg about z, from z0 to z1."""
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    corners = [(cx + ca * dx - sa * dy, cy + sa * dx + ca * dy) for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]
    out = []
    for i in range(4):
        (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % 4]
        n = (y1 - y0, -(x1 - x0), 0.0)                       # outward for a counter-clockwise polygon
        out.append(Hs(n, n[0] * x0 + n[1] * y0))
    out.append(Hs((0, 0, 1), z1))
    out.append(Hs((0, 0, -1), -z0))
    return out


def difference(A, B):
    """Convex pieces whose union is A minus B (A, B convex)."""
    pieces, inside = [], []
    for h in B:
        pieces.append(list(A) + inside + [Hs(-h.n, -h.b)])
        inside.append(h)
    return pieces


def cut(pieces, B):
    out = []
    for p in pieces:
        out += difference(p, B)
    return out


def clip(pieces, h):
    """Intersect every piece with a half-space (or a list of them)."""
    extra = h if isinstance(h, list) else [h]
    return [p + extra for p in pieces]


def _dedupe(hs, tol=1e-7):
    out = []
    for h in hs:
        if not any(np.allclose(h.n, g.n, atol=1e-9) and abs(h.b - g.b) < tol for g in out):
            out.append(h)
    return out


def vertices(hs, tol=1e-7):
    """Corners of the convex polytope (empty array if it has no volume)."""
    hs = _dedupe(hs)
    N = np.array([h.n for h in hs])
    b = np.array([h.b for h in hs])
    pts = []
    for i, j, k in itertools.combinations(range(len(hs)), 3):
        M = N[[i, j, k]]
        if abs(np.linalg.det(M)) < 1e-9:
            continue
        x = np.linalg.solve(M, b[[i, j, k]])
        if np.all(N @ x <= b + tol):
            if not any(np.linalg.norm(x - q) < 1e-6 for q in pts):
                pts.append(x)
    return np.array(pts)


def volume(hs, verts=None, tol=1e-6):
    """Volume of the convex polytope from its faces (0 when degenerate)."""
    hs = _dedupe(hs)
    v = vertices(hs) if verts is None else verts
    if len(v) < 4:
        return 0.0
    c = v.mean(axis=0)
    vol = 0.0
    for h in hs:
        on = v[np.abs(v @ h.n - h.b) < tol]
        if len(on) < 3:
            continue
        fc = on.mean(axis=0)
        a = np.cross(h.n, [1.0, 0.0, 0.0]) if abs(h.n[0]) < 0.9 else np.cross(h.n, [0.0, 1.0, 0.0])
        a = unit(a)
        bb = np.cross(h.n, a)
        ang = np.arctan2((on - fc) @ bb, (on - fc) @ a)
        q = on[np.argsort(ang)]
        area = 0.0
        for i in range(len(q)):
            area += 0.5 * np.linalg.norm(np.cross(q[i] - fc, q[(i + 1) % len(q)] - fc))
        vol += area * (h.b - h.n @ c) / 3.0
    return abs(vol)


def solid(pieces, min_vol=1e-4):
    """Convex pieces as vertex arrays, with the slivers (under min_vol cm3) dropped."""
    out = []
    for p in pieces:
        v = vertices(p)
        if len(v) >= 4 and volume(p, v) > min_vol:
            out.append((v, p))
    return out


def contains(solids, pt, tol=0.0):
    pt = np.asarray(pt, float)
    return any(all(h.n @ pt <= h.b + tol for h in p) for v, p in solids)


def channel_frame(side=1):
    """(p0, d, ex, ey, ez, exit point e) of the channel; the frame is the one fusion_lib.axis_prism builds: local z along the axis, local y as vertical as possible."""
    p0, e = P.chute_ends(side)
    p0, e = np.array(p0, float), np.array(e, float)
    ez = unit(e - p0)
    ey = unit(np.array([0.0, 0.0, 1.0]) - ez * ez[2])
    ex = np.cross(ey, ez)
    return p0, ez, ex, ey, ez, e


def build(side=1):
    """Dict of name -> list of (vertices, half-spaces): 'floor' (dropper floor round the slot), 'hopper', 'channel'. Side 1 is channel B (left)."""
    if side != 1:
        raise ValueError('only side +1 (channel B) is built; the layout is mirrored about y = 0')
    Ch, F, pl = P.CHUTE, P.FRAME, P.PLATE
    p0, d, ex, ey, ez, e = channel_frame(side)
    sx, sy = p0[0], p0[1]
    out = {}
    # dropper floor: a 3.8 x 3.8 cm patch round the slot (clear of the N20 recess and the seat tabs), z 8.7 to 9.0, with the square slot (PLATE slot, 16 mm) turned 45 degrees
    patch = box((sx, sy, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), -1.9, 1.9, -1.9, 1.9, F['z0'], F['z0'] + P.DROPPER_FLOOR['t'])
    rim = [Hs((math.cos(a), math.sin(a), 0.0), P.DROPPER_FLOOR['r'] + pl['cx'] * math.cos(a) + pl['cy'] * math.sin(a)) for a in (2 * math.pi * k / 48 for k in range(48))]
    out['floor'] = solid(clip(difference(patch, rect_prism(sx, sy, pl['slot'], pl['slot'], 45.0, F['z0'] - 0.2, F['z0'] + 0.5)), rim))
    # hopper: box (hopper_out) from hopper_z0 to the floor underside plus a flange, minus the void (hopper_in, from hopper_z0 + 1.6 mm up) and the channel socket
    hop = [rect_prism(sx, sy, Ch['hopper_out'], Ch['hopper_out'], 45.0, Ch['hopper_z0'], F['z0']),
           rect_prism(sx, sy, Ch['flange'], Ch['flange'], 45.0, F['z0'] - Ch['flange_t'], F['z0'])]
    void = rect_prism(sx, sy, Ch['hopper_in'], Ch['hopper_in'], 45.0, Ch['hopper_z0'] + 0.16, F['z0'] + 0.1)
    w = Ch['out_w'] / 2
    lift = Ch['lift']                                        # the square sections are centred this far above the axis
    socket = box(p0, ex, ey, ez, -w, w, lift - w, lift + w + Ch['socket_up'], -0.5, 3.0)       # taller than the channel, so that no lintel of the hopper hangs free above it
    pcs = []
    for h in hop:
        pcs += difference(h, void)
    pcs = cut(pcs, socket)
    out['hopper'] = solid(pcs)
    # channel: square tube (outer out_w) from the slot centre to 0.8 cm past the exit, minus the bore (in_w), minus the trough (a 25 mm square, 45 degrees, from z trough_z0 up: everything
    # of the tube inside it goes, walls, ceiling and floor), plus the floor slab laid back (outer width, wall thick, from the slot centre to slab_end), kept inside the body radius
    far = np.linalg.norm(e - p0) + 0.8
    wi = Ch['in_w'] / 2
    tube = box(p0, ex, ey, ez, -w, w, lift - w, lift + w, 0.0, far)
    bore = box(p0, ex, ey, ez, -wi, wi, lift - wi, lift + wi, -0.2, far + 0.2)
    trough = rect_prism(sx, sy, Ch['trough'], Ch['trough'], 45.0, Ch['trough_z0'], F['z0'] + 0.3)
    pcs = cut(difference(tube, bore), trough)
    pcs.append(box(p0, ex, ey, ez, -w, w, lift - w, lift - wi, 0.0, Ch['slab_end']))
    pcs = clip(pcs, Hs((0.0, 0.0, 1.0), F['z0']))                  # the section's top corners just outside the trough would reach into the dropper floor: cut at its underside
    # trimmed flush with the body radius: the true cut is a cylinder about the axle; the tube is +-5.8 degrees wide there (+-1.06 cm at r 10.5), so a fan of tangent planes 2 degrees apart
    # over +-8 degrees of the exit direction follows it to 0.02 mm (one tangent plane at the exit would be 0.5 mm off at the tube's edges)
    a_exit, r_exit = math.atan2(e[1], e[0]), math.hypot(e[0], e[1])
    pcs = clip(pcs, [Hs((math.cos(a_exit + math.radians(k)), math.sin(a_exit + math.radians(k)), 0.0), r_exit) for k in range(-8, 9, 2)])
    out['channel'] = solid(pcs)
    return out


def summary(geo):
    return ', '.join('%s %d pieces %.2f cm3' % (k, len(v), sum(volume(p, vv) for vv, p in v)) for k, v in geo.items())


if __name__ == '__main__':
    g = build(1)
    print(summary(g))
