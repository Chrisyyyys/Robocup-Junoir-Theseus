import math, random, itertools, time
from place2 import *
from shapely.geometry import Point

def setup_R(R):
    P['R_body'] = R; P['R_int'] = R - P['wall']; P['R_sw'] = R + 0.5
    P['omni_x'] = P['R_sw'] - P['omni_r'] - 0.05
    P['exit_x'] = -(R - 3.5)
    P['x_p'] = -2.0
    F = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    bands = build_bands(F)
    cams = []
    for psi, tip_r, zl, tilt in itertools.product((88, 90, 92), (R - 0.2, R), (9.1, 9.2, 9.3), (20,)):
        lens, board = cam_points(psi, tip_r, zl, tilt, 90, 'h')
        lm = lens.copy(); lm[:, 1] *= -1; bm = board.copy(); bm[:, 1] *= -1
        if pts_ok(bands, lens, P['R_sw'] - 0.05) and pts_ok(bands, board, P['R_int']) and pts_ok(bands, lm, P['R_sw'] - 0.05) and pts_ok(bands, bm, P['R_int']):
            cams.append((psi, tip_r, zl, tilt))
    if not cams: return None
    psi, tip_r, zl, tilt = cams[0]
    lens, board = cam_points(psi, tip_r, zl, tilt, 90, 'h')
    camparts = []
    for s in (1, -1):
        l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
        camparts += [hull_part(f'cam{"L" if s>0 else "R"}_lens', l), hull_part(f'cam{"L" if s>0 else "R"}_board', b)]
    F2 = F + camparts
    return F2, build_bands(F2), (psi, tip_r, zl, tilt)

def place_one(bands, l, w, h, zs, n=40000):
    inner = Point(0, 0).buffer(P['R_int'], 128)
    best = None
    for _ in range(n):
        x = random.uniform(-9.5, 9.5); y = random.uniform(-9.5, 9.5); ang = random.choice(range(0, 180, 10))
        z0 = random.choice(zs); z1 = z0 + h
        if z1 > P['z_pf'] - 0.4: continue
        poly = rect(x, y, l, w, ang)
        if not poly.within(inner): continue
        if collides(bands, poly, z0, z1): continue
        d = min_dist(bands, poly, z0, z1)
        if best is None or d > best[0]: best = (d, round(x, 2), round(y, 2), ang, z0)
    return best

if __name__ == '__main__':
    for R in (9.5, 10.0, 10.5):
        s = setup_R(R)
        if s is None: print('R', R, 'no camera placement'); continue
        F2, bands, cam = s
        out = []
        for h in (1.5, 1.9, 2.5):
            b = place_one(bands, 10.152, 5.334, h, [5.15 + 0.15*k for k in range(0, 8)])
            out.append(f'stack {h}: ' + (f'FITS x={b[1]} y={b[2]} ang={b[3]} z0={b[4]:.2f} clearance {b[0]:.2f}' if b else 'no fit'))
        print(f'R_body {R} (swept {R+0.5}): camera {cam}; ' + ' | '.join(out))
