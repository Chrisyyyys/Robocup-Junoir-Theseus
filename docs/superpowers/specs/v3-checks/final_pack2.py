"""V3 layout with the approved-draft pocket size (14 mm), the corrected ring radius, the chute exit at x -6.0, the camera recessed behind a window,
the rear chamfer as a constraint, and the main PCB modelled as the GIGA shield stack. Writes pack_v3.json."""
import json, math, random, sys
from final_check import *
from place2 import cam_points, pts_ok, hull_part
from chamfer_pack import chamfer_gap
from com import com

R = 10.5
RING, POCKET, SLOT, EXIT_X = 3.86, 1.40, 1.45, -6.0
CAM_TIP, CAM_Z, CAM_TILT = 8.8, 9.3, 20

def build():
    P['R_body'] = R; P['R_int'] = R - P['wall']; P['R_sw'] = R + 0.5; P['omni_x'] = P['R_sw'] - P['omni_r'] - 0.05
    P['arm_pivot'] = (3.5, 3.6); P['x_p'] = -2.0; P['exit_x'] = EXIT_X; P['r_ring'] = RING; P['pocket'] = POCKET; P['slot'] = SLOT; P['z_lid'] = 12.1
    F = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    return F

def main():
    F = build()
    print('plate radius', round(P['R_plate'], 3), 'cm; fixed-part overlaps (omni/omniArm and plateFloor/chute are contacts by design):', dict(overlaps(F)))
    lens, board = cam_points(88, CAM_TIP, CAM_Z, CAM_TILT, 90, 'h')
    lm = lens.copy(); lm[:, 1] *= -1; bm = board.copy(); bm[:, 1] *= -1
    bands0 = build_bands(F)
    ok = pts_ok(bands0, lens, P['R_sw'] - 0.05) and pts_ok(bands0, board, P['R_int']) and pts_ok(bands0, lm, P['R_sw'] - 0.05) and pts_ok(bands0, bm, P['R_int'])
    print(f'camera tip radius {CAM_TIP}, lens z {CAM_Z}, tilt {CAM_TILT}: feasible against the fixed parts: {ok}; board top {board[:, 2].max():.2f}')
    parts = []
    for s in (1, -1):
        l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
        parts += [hull_part(f'cam{"L" if s > 0 else "R"}_lens', l), hull_part(f'cam{"L" if s > 0 else "R"}_board', b)]
    bands = build_bands(F + parts)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    items = [('battery (placeholder 7.0x3.5x2.5)', 7.0, 3.5, 2.5, [3.7 + 0.2 * k for k in range(0, 10)], (2.5, 10)),
             ('GIGA R1 + main PCB stack', 10.152, 5.334, 1.9, [5.15 + 0.1 * k for k in range(0, 14)], (-10, 10))]
    random.seed(11); placed = []; report = {}
    for name, l, w, h, zs, xr in items:
        best = None
        for _ in range(80000 if 'GIGA' in name else 30000):
            x = random.uniform(*xr); y = random.uniform(-10, 10); ang = random.choice(range(0, 180, 10))
            z0 = random.choice(zs); z1 = z0 + h
            if name.startswith('GIGA') and z1 > P['z_pf'] - 0.4: continue
            poly = rect(x, y, l, w, ang)
            if not poly.within(inner): continue
            if chamfer_gap(poly, z0) < 0: continue
            if collides(bands, poly, z0, z1): continue
            if any(z0 < q[3] and z1 > q[2] and poly.distance(q[4]) < CL for q in placed): continue
            d = min_dist(bands, poly, z0, z1)
            for q in placed:
                if z0 < q[3] and z1 > q[2]: d = min(d, poly.distance(q[4]))
            d = min(d, chamfer_gap(poly, z0))
            if best is None or d > best[0]: best = (d, x, y, ang, z0, z1, poly)
        if best is None:
            print('NO PLACEMENT for', name); continue
        placed.append((name, None, best[4], best[5], best[6]))
        report[name] = dict(x=round(best[1], 2), y=round(best[2], 2), angle=best[3], z0=round(best[4], 2), z1=round(best[5], 2), clearance=round(best[0], 2))
        print(f'   {name:<36} x={best[1]:6.2f} y={best[2]:6.2f} ang={best[3]:>3} z {best[4]:.2f}-{best[5]:.2f} clearance {best[0]:.2f} cm')
    g = report['GIGA R1 + main PCB stack']; b = report['battery (placeholder 7.0x3.5x2.5)']
    M, x, z = com((g['x'], (g['z0'] + g['z1']) / 2), (b['x'], (b['z0'] + b['z1']) / 2))
    print(f'COM (placeholder masses): {M/1000:.2f} kg, x {x:+.2f} cm (front load {100*x/7.95:.1f}%), height {z:.1f} cm')
    json.dump(dict(R=R, camera=[88, CAM_TIP, CAM_Z, CAM_TILT], plate=dict(ring=RING, pocket=POCKET, slot=SLOT, R_plate=round(P['R_plate'], 3)), exit_x=EXIT_X, parts=report), open('pack_v3.json', 'w'), indent=1)

if __name__ == '__main__':
    main()
