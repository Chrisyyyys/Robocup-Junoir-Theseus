"""Stack alone (no battery) at R 9.5 / 10.0 / 10.5 for stack heights 1.5, 1.9, 2.5 cm: does any placement exist? Uses fit_R's consistent geometry. python fit_stack_only.py"""
import random, sys
from fit_R import *

def stack_only(R, h, n=250000, seed=5, bl=10.152, bw=5.334):
    exit_x = None
    for ex in [-4.8 - 0.1 * k for k in range(0, 40)]:
        F, q = build(R, ex)
        if chute_wheel_gap(F) >= WALL_CLEAR: exit_x = ex; break
    F, q = build(R, exit_x)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    lens, board = cam_points(88, 8.8, 9.3, 20, 90, 'h'); parts = []
    for s in (1, -1):
        l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
        parts += [hull_part(f'cam{"L" if s > 0 else "R"}_lens', l), hull_part(f'cam{"L" if s > 0 else "R"}_board', b)]
    bands = build_bands(F + parts)
    random.seed(seed); best = None
    zs = [4.0 + 0.1 * k for k in range(0, 40)]
    for _ in range(n):
        x = random.uniform(-9, 9); y = random.uniform(-9, 9); ang = random.choice(range(0, 180, 5)); z0 = random.choice(zs); z1 = z0 + h
        if z1 > P['z_pf'] - 0.4: continue
        poly = rect(x, y, bl, bw, ang)
        if not poly.within(inner) or chamfer_gap(poly, z0) < 0 or collides(bands, poly, z0, z1): continue
        d = min(min_dist(bands, poly, z0, z1), chamfer_gap(poly, z0))
        if best is None or d > best[0]: best = (d, x, y, ang, z0)
    return best

if __name__ == '__main__':
    print('GIGA R1 (10.15 x 5.33 cm) stack alone, by stack height')
    for R in (9.5, 10.0, 10.5):
        row = []
        for h in (1.5, 1.9, 2.5):
            b = stack_only(R, h)
            row.append(f'h {h}: ' + ('none' if b is None else f'fits (x {b[1]:.1f}, y {b[2]:.1f}, z0 {b[4]:.1f}, clearance {b[0]*10:.1f} mm)'))
        print(f'R {R}: ' + ' | '.join(row), flush=True)
    print('Smaller main board (what a Teensy 4.1 in sockets on a custom board would need), 1.5 cm tall, battery not placed')
    for R in (9.5, 10.0):
        row = []
        for (l, w) in ((8.0, 5.0), (7.0, 4.5), (6.5, 4.0)):
            b = stack_only(R, 1.5, n=150000, bl=l, bw=w)
            row.append(f'{l} x {w}: ' + ('none' if b is None else f'fits (clearance {b[0]*10:.1f} mm)'))
        print(f'R {R}: ' + ' | '.join(row), flush=True)
