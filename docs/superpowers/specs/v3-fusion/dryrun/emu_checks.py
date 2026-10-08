"""The Fusion primitives that the whole-model reports of v4_checks3d need and that fake_fusion does not already provide: ray casting, mesh extremes, the nearest distance between two
solids, and a context object to build the model into. The reports themselves (removal_sequence, omni_sweep, local_interference, ...) run unchanged on the fake temporary B-rep manager."""
import math
import sys
import time
import types

import numpy as np

import fake_fusion as F

_apply = F._apply
VERBOSE = False


def make_ctx(tof_positions):
    """A stand-in for v3_model.Ctx: a root component, a palette, the 'fixed' ToF positions, and a measure manager."""
    return types.SimpleNamespace(root=F.Root(), pal=F.Pal(), design=None, app=types.SimpleNamespace(measureManager=_Measure()), tof=tof_positions('fixed'), tof_mode='fixed')


def _inside_world(b, pts):
    return b.inside(_apply(np.linalg.inv(b.comp.matrix), pts))


def max_radius(body, n=60000):
    """Largest distance from the vertical axis through the origin (sampled; replaces the mesh vertices of the Fusion version)."""
    w = _apply(body.comp.matrix, body.sample(n))
    return float(np.hypot(w[:, 0], w[:, 1]).max())


def cast(root, origin, direction, own=(), reach=14.0, step=0.005, ghost='Stepper bay 28BYJ-48'):
    """Replaces v3_checks3d._cast: faces hit by a ray, nearest first, as (distance, component name); found by marching along the ray and noting every in/out change of every body."""
    o = np.array(origin, float)
    d = np.array(direction, float)
    d /= np.linalg.norm(d)
    t = np.arange(0.0, reach, step)
    pts = o + t[:, None] * d
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    rows = []
    for occ in root._occ:
        nm = occ.component.name
        if nm in own or nm == ghost:
            continue
        for b in occ.component.bodies:
            wl, wh = F.world_box(b)
            if np.any(hi < wl) or np.any(lo > wh):
                continue
            ins = _inside_world(b, pts)
            for i in np.nonzero(ins[1:] != ins[:-1])[0]:
                rows.append(((i + 0.5) * step, nm))
    rows.sort()
    return rows


def min_dist(ba, bb, n=6000):
    """Nearest distance (cm) between two solids by sampling: coarse samples of each body clipped to the other body's box grown by D, then boxes shrunk around the closest pair."""
    def pts_in(b, n, lo, hi, tries=40):
        wl, wh = F.world_box(b)
        wl, wh = np.maximum(wl, lo), np.minimum(wh, hi)
        if np.any(wh <= wl):
            return np.zeros((0, 3))
        out, got = [], 0
        inv = np.linalg.inv(b.comp.matrix)
        for _ in range(tries):
            p = F.RNG.uniform(wl, wh, size=(100000, 3))
            p = p[b.inside(_apply(inv, p))]
            out.append(p)
            got += len(p)
            if got >= n:
                break
        return np.vstack(out)[:n] if out else np.zeros((0, 3))

    def nearest(A, B):
        best, ia, ib = np.inf, 0, 0
        b2 = (B * B).sum(1)
        for i0 in range(0, len(A), 250):
            a = A[i0:i0 + 250]
            d2 = (a * a).sum(1)[:, None] + b2[None, :] - 2.0 * a @ B.T
            j = np.unravel_index(np.argmin(d2), d2.shape)
            if d2[j] < best:
                best, ia, ib = d2[j], i0 + j[0], j[1]
        return math.sqrt(max(best, 0.0)), ia, ib

    la, ha = F.world_box(ba)
    lb, hb = F.world_box(bb)
    gap = np.maximum(0.0, np.maximum(la - hb, lb - ha))
    D = float(np.linalg.norm(gap)) + 0.3
    d = None
    for _ in range(8):
        A = pts_in(ba, n, lb - D, hb + D)
        B = pts_in(bb, n, la - D, ha + D)
        if len(A) and len(B):
            d, ia, ib = nearest(A, B)
            if d <= D:
                break
        D *= 2.0
    if d is None:
        return float('nan')
    for r in range(4):
        mid = (A[ia] + B[ib]) / 2.0
        h = d / 2.0 + 0.2 / (r + 1)
        A2, B2 = pts_in(ba, 12000, mid - h, mid + h), pts_in(bb, 12000, mid - h, mid + h)
        if len(A2) == 0 or len(B2) == 0:
            break
        d2_, ia, ib = nearest(A2, B2)
        A, B = A2, B2
        d = min(d, d2_)
    return d


class _Measure:
    """Stands in for app.measureManager."""
    def measureMinimumDistance(self, ba, bb):
        t0 = time.time()
        d = min_dist(ba, bb)
        if VERBOSE:
            print('  [min distance %-24s %-18s %7.2f mm  %.1f s]' % (ba.comp.name + '/' + ba.name[:10], bb.comp.name + '/' + bb.name[:10], d * 10, time.time() - t0), file=sys.stderr, flush=True)
        return types.SimpleNamespace(value=d)


def install_report_patches(K, K3):
    """K = v4_checks3d, K3 = v3_checks3d: replace the two primitives that cannot run on the fake temporary B-rep manager (mesh extremes and ray casting)."""
    K._max_radius = max_radius
    K3._cast = cast
