"""Which attitudes of a cube can fall through a square slot? (mechanical design section 6.2; the independent review of 8 Oct found that '16 mm passes the cube in any attitude' was too strong.)
The cube (side a, the kit: 10.3 mm) may turn freely about the vertical but does not tip on the way, so its horizontal shadow must fit the square. The width of a cube of side a along a unit vector n is
a (|nx| + |ny| + |nz|). u is the vertical in the cube's frame; s_min(u) is the smallest square that holds the shadow. Flat on a face: a. Tipped onto an edge: the face diagonal a sqrt 2 (14.57 mm).
Standing on a corner (space diagonal vertical): the shadow is a regular hexagon, and the square that holds it has the side a (1 + sqrt 3) / sqrt 3 = 16.25 mm. A slot of 16 mm lets every attitude through
except those within a few degrees of standing on a corner. A kit that tips while it falls (the plate parked a little off, a corner on the slot's edge) can be wider than this static count: that is the
simulation's job (chute_dynamics.py). Only numpy is needed. Monte Carlo over random attitudes with a fixed seed."""
import math

import numpy as np

A = 10.3                                               # the kit, mm


def s_min(u, nphi=181):
    u = np.asarray(u, float) / np.linalg.norm(u)
    t = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    b1 = np.cross(u, t)
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(u, b1)
    phi = np.linspace(0.0, math.pi / 2, nphi)
    n1 = np.outer(np.cos(phi), b1) + np.outer(np.sin(phi), b2)
    n2 = np.outer(-np.sin(phi), b1) + np.outer(np.cos(phi), b2)
    w1 = A * np.abs(n1).sum(axis=1)
    w2 = A * np.abs(n2).sum(axis=1)
    return float(np.max(np.stack([w1, w2]), axis=0).min())


if __name__ == '__main__':
    print('kit %.1f mm: face diagonal %.2f mm, space diagonal %.2f mm, standing on a corner %.2f mm (closed form %.2f)' % (
        A, A * math.sqrt(2), A * math.sqrt(3), s_min((1, 1, 1)), A * (1 + math.sqrt(3)) / math.sqrt(3)))
    for name, u in (('flat on a face', (0, 0, 1)), ('tipped onto an edge', (0, 1, 1)), ('standing on a corner', (1, 1, 1))):
        print('  %-24s smallest square that holds its shadow: %.2f mm' % (name, s_min(u)))
    rng = np.random.default_rng(1)
    s = np.array([s_min(v, 91) for v in rng.normal(size=(20000, 3))])
    for w in (13.0, 14.5, 14.6, 15.0, 15.5, 16.0, 16.25, 17.0):
        print('  slot %5.2f mm: %4.1f %% of 20000 random attitudes do not fit' % (w, 100.0 * np.mean(s > w + 1e-9)))
    d = np.array([1.0, 1.0, 1.0]) / math.sqrt(3.0)
    t1 = np.cross(d, [1.0, 0.0, 0.0])
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(d, t1)
    cone = 0.0
    for ang in np.arange(0.0, 40.0, 0.5):
        need = max(s_min(math.cos(math.radians(ang)) * d + math.sin(math.radians(ang)) * (math.cos(az) * t1 + math.sin(az) * t2), 91) for az in np.linspace(0.0, 2 * math.pi, 73))
        if need > 16.0:
            cone = ang
    print('a 16.0 mm slot is too small somewhere up to %.1f degrees away from standing on a corner' % cone)
