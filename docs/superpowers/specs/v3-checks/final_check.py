import math, random, itertools, time, json, sys
from giga import *
from collections import Counter

def build_model(R=10.5, stack_h=1.9):
    P['R_body'] = R; P['R_int'] = R - P['wall']; P['R_sw'] = R + 0.5
    P['omni_x'] = P['R_sw'] - P['omni_r'] - 0.05
    P['arm_pivot'] = (3.5, 3.6)
    P['exit_x'] = -(R - 3.5); P['x_p'] = -2.0
    F = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    return F

def overlaps(F):
    bad = Counter()
    for a, b in itertools.combinations(F, 2):
        if a.name == b.name: continue
        if {a.kind, b.kind} == {'tof'}: continue
        if overlap(a, b) > 1e-4: bad[tuple(sorted((a.name, b.name)))] += 1
    return bad

if __name__ == '__main__':
    R = float(sys.argv[1]) if len(sys.argv) > 1 else 10.5
    F = build_model(R)
    print('R_body', R, 'swept', R + 0.5, 'plate R', round(P['R_plate'], 2), ' omni rest x', round(P['omni_x'], 2))
    print('fixed-part overlaps (omni/omniArm and plateFloor/chute are contacts by design):', dict(overlaps(F)))
    s = setup_R(R)
    F2, bands, cam = s
    print('camera above the wheel:', cam)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    placed = []; report = {}
    items = [('GIGA+shield', 10.152, 5.334, 1.9, [5.15 + 0.1*k for k in range(0, 14)]),
             ('battery (placeholder 7.0x3.5x2.5)', 7.0, 3.5, 2.5, [3.6 + 0.2*k for k in range(0, 14)]),
             ('mux A', 2.5, 2.5, 0.8, [3.6, 5.15, 6.0, 7.0]), ('mux B', 2.5, 2.5, 0.8, [3.6, 5.15, 6.0, 7.0]),
             ('regulator 1.78x2.03', 2.03, 1.78, 0.88, [3.6, 5.15, 6.0, 7.0])]
    random.seed(5)
    for name, l, w, h, zs in items:
        best = None
        for _ in range(60000 if 'GIGA' in name else 25000):
            x = random.uniform(-10, 10); y = random.uniform(-10, 10); ang = random.choice(range(0, 180, 10))
            z0 = random.choice(zs); z1 = z0 + h
            if name.startswith('GIGA') and z1 > P['z_pf'] - 0.4: continue
            poly = rect(x, y, l, w, ang)
            if not poly.within(inner): continue
            if collides(bands, poly, z0, z1): continue
            if any(z0 < q[3] and z1 > q[2] and poly.distance(q[4]) < CL for q in placed): continue
            d = min_dist(bands, poly, z0, z1)
            for q in placed:
                if z0 < q[3] and z1 > q[2]: d = min(d, poly.distance(q[4]))
            if best is None or d > best[0]: best = (d, x, y, ang, z0, z1, poly)
        if best is None: print('NO PLACEMENT for', name); report[name] = None; continue
        placed.append((name, None, best[4], best[5], best[6]))
        report[name] = dict(x=round(best[1], 2), y=round(best[2], 2), angle=best[3], z0=round(best[4], 2), z1=round(best[5], 2), clearance=round(best[0], 2))
        print(f'  {name:<36} x={best[1]:6.2f} y={best[2]:6.2f} angle={best[3]:>3} z {best[4]:.2f}-{best[5]:.2f} clearance {best[0]:.2f} cm')
    json.dump(dict(R=R, camera=cam, parts=report), open(f'pack_R{R}.json', 'w'), indent=1)
