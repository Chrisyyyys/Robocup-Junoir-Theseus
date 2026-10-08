import math, random, sys, json, itertools
from shapely.geometry import Polygon, Point, box
from shapely import affinity
from shapely.ops import unary_union
from shapely.prepared import prep
from layout import *

random.seed(7)
CL = 0.15  # required clearance, cm

def build_bands(F, buf=CL, step=0.25, zmax=10.0):
    bands = []
    z = 0.0
    while z < zmax:
        z2 = z + step
        polys = [f.poly.buffer(buf) for f in F if f.z0 < z2 and f.z1 > z]
        u = unary_union(polys) if polys else Polygon()
        bands.append((z, z2, u, prep(u)))
        z = z2
    return bands

def collides(bands, poly, z0, z1):
    for b0, b1, u, pu in bands:
        if b0 < z1 and b1 > z0 and pu.intersects(poly):
            return True
    return False

def min_dist(bands, poly, z0, z1):
    d = 1e9
    for b0, b1, u, pu in bands:
        if b0 < z1 and b1 > z0 and not u.is_empty:
            d = min(d, u.distance(poly) + CL)
    return d

def mirror(poly):
    return affinity.scale(poly, xfact=1, yfact=-1, origin=(0, 0))

def main(R_body=9.5, z_pf=8.0, verbose=True):
    P['R_body'] = R_body; P['R_int'] = R_body - P['wall']; P['z_pf'] = z_pf
    F = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    bands = build_bands(F)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    outer_cam = Point(0, 0).buffer(R_body + 0.25, 128)
    out = {}
    # ---- cameras (left candidates, mirrored to the right) ----
    cams = []
    for psi in range(30, 151, 2):
        for orient in ('h', 'v'):
            for zc in (6.2, 6.6, 7.0, 7.4):
                parts = camera_parts(1, psi, zc, orient)
                ok = True
                for c in parts:
                    if c.z0 < P['z_belly'] or c.z1 > P['z_lid'] - 0.05: ok = False; break
                    lim = outer_cam if c.name.endswith('_lens') else inner
                    if not c.poly.within(lim): ok = False; break
                    if collides(bands, c.poly, c.z0, c.z1) or collides(bands, mirror(c.poly), c.z0, c.z1): ok = False; break
                if ok:
                    d = min(min_dist(bands, c.poly, c.z0, c.z1) for c in parts)
                    cams.append((psi, orient, zc, d))
    out['cameras'] = cams
    if verbose:
        print(f'R={R_body} z_pf={z_pf}: feasible camera placements: {len(cams)}')
        by = {}
        for psi, o, zc, d in cams: by.setdefault((o, zc), []).append(psi)
        for k, v in sorted(by.items()): print('   ', k, 'psi', v[0], '..', v[-1], ' n=', len(v))
    return F, bands, cams

if __name__ == '__main__':
    for R, zpf in ((9.5, 8.0), (9.5, 7.0), (10.5, 8.0)):
        main(R, zpf)
