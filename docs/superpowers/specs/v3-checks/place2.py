import math, random, itertools, time, json
import numpy as np
from shapely.geometry import Polygon, Point, MultiPoint, box
from shapely import affinity
from shapely.ops import unary_union
from shapely.prepared import prep
from layout import *
random.seed(11)
CL = 0.15

def build_bands(F, buf=CL, step=0.25, zmax=13.0):
    bands = []; z = 0.0
    while z < zmax:
        z2 = z + step
        polys = [f.poly.buffer(buf) for f in F if f.z0 < z2 and f.z1 > z]
        u = unary_union(polys) if polys else Polygon()
        bands.append((z, z2, u, prep(u)))
        z = z2
    return bands

def collides(bands, poly, z0, z1):
    for b0, b1, u, pu in bands:
        if b0 < z1 and b1 > z0 and pu.intersects(poly): return True
    return False

def min_dist(bands, poly, z0, z1):
    d = 1e9
    for b0, b1, u, pu in bands:
        if b0 < z1 and b1 > z0 and not u.is_empty: d = min(d, u.distance(poly) + CL)
    return d

def cam_points(psi_deg, tip_r, zl, tilt_deg, aim_deg, orient='h', n=6):
    L, S, D = P['cam']
    tw, th = (L, S) if orient == 'h' else (S, L)
    ap = math.radians(psi_deg); aa = math.radians(aim_deg); ph = math.radians(tilt_deg)
    tip = np.array([tip_r*math.cos(ap), tip_r*math.sin(ap), zl])
    ax = np.array([math.cos(aa)*math.cos(ph), math.sin(aa)*math.cos(ph), -math.sin(ph)]); back = -ax
    tdir = np.array([-math.sin(aa), math.cos(aa), 0.0])
    hdir = np.cross(ax, tdir); hdir = hdir if hdir[2] > 0 else -hdir
    lens, board = [], []
    for a in np.linspace(0, 2.3, n):
        for t in np.linspace(-1.0, 1.0, n):
            for h in np.linspace(-1.0, 1.0, n): lens.append(tip + back*a + tdir*t + hdir*h)
    for a in np.linspace(2.5, 3.1, 3):
        for t in np.linspace(-tw/2, tw/2, n+3):
            for h in np.linspace(-th/2, th/2, n+3): board.append(tip + back*a + tdir*t + hdir*h)
    return np.array(lens), np.array(board)

def pts_ok(bands, pts, rmax):
    for x, y, z in pts:
        if math.hypot(x, y) > rmax or z < P['z_belly'] or z > P['z_lid'] - 0.03: return False
        for b0, b1, u, pu in bands:
            if b0 <= z < b1:
                if pu.contains(Point(x, y)): return False
                break
    return True

def hull_part(name, pts):
    return Part(name, MultiPoint([(p[0], p[1]) for p in pts]).convex_hull, float(pts[:, 2].min()), float(pts[:, 2].max()), 'cam')

def run():
    F = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    bands = build_bands(F)
    # ---- cameras above the wheels
    cams = []
    for psi, tip_r, zl, tilt in itertools.product((86, 88, 90, 92, 94), (9.3, 9.5), (9.0, 9.1, 9.2, 9.3, 9.4), (15, 20)):
        lens, board = cam_points(psi, tip_r, zl, tilt, 90, 'h')
        lm = lens.copy(); lm[:, 1] *= -1; bm = board.copy(); bm[:, 1] *= -1
        if pts_ok(bands, lens, P['R_sw'] - 0.05) and pts_ok(bands, board, P['R_int']) and pts_ok(bands, lm, P['R_sw'] - 0.05) and pts_ok(bands, bm, P['R_int']):
            cams.append((psi, tip_r, zl, tilt, round(float(board[:, 2].max()), 2)))
    print('camera above the wheel, feasible:', len(cams))
    for c in cams[:6]: print('   ', c)
    if not cams: return None
    # choose: tilt 20, lowest board top
    cams.sort(key=lambda c: (abs(c[0] - 90), -c[3], c[4]))
    psi, tip_r, zl, tilt, top = [c for c in cams if c[3] == 20][0] if any(c[3] == 20 for c in cams) else cams[0]
    lens, board = cam_points(psi, tip_r, zl, tilt, 90, 'h')
    camparts = []
    for s in (1, -1):
        l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
        camparts += [hull_part(f'cam{"L" if s>0 else "R"}_lens', l), hull_part(f'cam{"L" if s>0 else "R"}_board', b)]
    F2 = F + camparts
    bands2 = build_bands(F2)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    items = [('GIGA+shield', 10.152, 5.334, 2.5, [5.15, 5.4, 5.65, 5.9, 6.05]),
             ('battery 3S (placeholder 7.0x3.5x2.5)', 7.0, 3.5, 2.5, [3.55, 4.0, 4.5, 5.15, 5.65, 6.15]),
             ('mux A', 2.5, 2.5, 0.8, [3.55, 5.15, 6.0, 7.0]), ('mux B', 2.5, 2.5, 0.8, [3.55, 5.15, 6.0, 7.0]),
             ('regulator 1.78x2.03x0.88', 2.03, 1.78, 0.88, [3.55, 5.15, 6.0, 7.0])]
    placed = []
    report = {}
    for name, l, w, h, zs in items:
        best = None
        for _ in range(60000 if 'GIGA' in name else 20000):
            x = random.uniform(-8.5, 8.5); y = random.uniform(-8.5, 8.5); ang = random.choice([0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165])
            z0 = random.choice(zs); z1 = z0 + h
            if z1 > P['z_pf'] - 0.3 - 0.1 and name.startswith('GIGA'): continue
            poly = rect(x, y, l, w, ang)
            if not poly.within(inner): continue
            if collides(bands2, poly, z0, z1): continue
            if any((z0 < p[3]+0.0 and z1 > p[2]) and poly.distance(p[4]) < CL for p in placed): continue
            d = min_dist(bands2, poly, z0, z1)
            for p in placed:
                if z0 < p[3] and z1 > p[2]: d = min(d, poly.distance(p[4]))
            if best is None or d > best[0]: best = (d, x, y, ang, z0, z1, poly)
        if best is None:
            print('NO PLACEMENT for', name); report[name] = None; continue
        placed.append((name, None, best[4], best[5], best[6]))
        report[name] = dict(x=round(best[1], 2), y=round(best[2], 2), angle=best[3], z0=best[4], z1=round(best[5], 2), clearance=round(best[0], 2))
        print(f'placed {name}: x={best[1]:.2f} y={best[2]:.2f} angle={best[3]} z {best[4]}-{best[5]:.2f}  clearance {best[0]:.2f} cm')
    return dict(camera=dict(psi=psi, tip_r=tip_r, zl=zl, tilt=tilt, board_top=top), parts=report)

if __name__ == '__main__':
    t0 = time.time()
    r = run()
    print('time', round(time.time() - t0), 's')
    json.dump(r, open('packing.json', 'w'), indent=1)
