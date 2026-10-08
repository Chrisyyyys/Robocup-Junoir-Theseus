"""Can the camera lens be recessed behind a window (tip radius < R-0.2) and the electronics still fit?
Re-runs the placement with the camera hulls and the rear chamfer as constraints. cm."""
import json, math, random, sys
from final_check import *
from place2 import cam_points, pts_ok, hull_part
from chamfer_pack import chamfer_gap, underside

R = 10.5
P['z_lid'] = 12.3     # allow a taller lid for the check; the feasible board top is reported
F = build_model(R)
bands0 = build_bands(F)

def cam_hulls(psi, tip_r, zl, tilt):
    lens, board = cam_points(psi, tip_r, zl, tilt, 90, 'h')
    parts = []
    for s in (1, -1):
        l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
        parts += [hull_part(f'cam{"L" if s > 0 else "R"}_lens', l), hull_part(f'cam{"L" if s > 0 else "R"}_board', b)]
    return lens, board, parts

def feasible(psi, tip_r, zl, tilt):
    lens, board = cam_points(psi, tip_r, zl, tilt, 90, 'h')
    lm = lens.copy(); lm[:, 1] *= -1; bm = board.copy(); bm[:, 1] *= -1
    ok = pts_ok(bands0, lens, P['R_sw'] - 0.05) and pts_ok(bands0, board, P['R_int']) and pts_ok(bands0, lm, P['R_sw'] - 0.05) and pts_ok(bands0, bm, P['R_int'])
    return ok, float(board[:, 2].max()), float(lens[:, 2].min())

print('camera feasibility (lid allowed up to 12.3): ok / board top / lowest lens point (wheel top is 8.0)')
for tilt in (20, 25):
    for tip_r in (10.3, 9.3, 8.8):
        row = []
        for zl in (9.1, 9.2, 9.3, 9.4):
            ok, top, low = feasible(88, tip_r, zl, tilt)
            row.append(f'zl {zl}: {"ok" if ok else "NO"} top {top:4.2f} low {low:4.2f}')
        print(f'  tilt {tilt} tip_r {tip_r}: ' + ' | '.join(row))

def pack(tip_r, zl, tilt=20, seed=11):
    lens, board, parts = cam_hulls(88, tip_r, zl, tilt)
    bands = build_bands(F + parts)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    items = [('battery (placeholder 7.0x3.5x2.5)', 7.0, 3.5, 2.5, [3.7 + 0.2 * k for k in range(0, 10)], (2.5, 10)),
             ('GIGA+shield', 10.152, 5.334, 1.9, [5.15 + 0.1 * k for k in range(0, 14)], (-10, 10)),
             ('mux A', 2.5, 2.5, 0.8, [3.7, 4.2, 4.7, 5.2, 5.7, 6.2, 6.7, 7.0], (-10, 10)),
             ('mux B', 2.5, 2.5, 0.8, [3.7, 4.2, 4.7, 5.2, 5.7, 6.2, 6.7, 7.0], (-10, 10)),
             ('regulator 1.78x2.03', 2.03, 1.78, 0.88, [3.7, 4.2, 4.7, 5.2, 5.7, 6.2, 6.7, 7.0], (-10, 10))]
    random.seed(seed); placed = []; report = {}
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
            print(f'   NO PLACEMENT for {name}'); report[name] = None; continue
        placed.append((name, None, best[4], best[5], best[6]))
        report[name] = dict(x=round(best[1], 2), y=round(best[2], 2), angle=best[3], z0=round(best[4], 2), z1=round(best[5], 2), clearance=round(best[0], 2))
        print(f'   {name:<36} x={best[1]:6.2f} y={best[2]:6.2f} ang={best[3]:>3} z {best[4]:.2f}-{best[5]:.2f} clearance {best[0]:.2f} cm')
    return report

if __name__ == '__main__':
    for tip_r, zl in ((10.3, 9.1), (8.8, 9.3)):
        ok, top, low = feasible(88, tip_r, zl, 20)
        print(f'--- packing with the camera tip radius {tip_r}, lens z {zl}: camera feasible {ok}, board top {top:.2f}, lowest lens point {low:.2f}')
        if ok:
            rep = pack(tip_r, zl)
            json.dump(dict(R=R, camera=[88, tip_r, zl, 20], parts=rep), open(f'pack_cam_{tip_r}_{zl}.json', 'w'), indent=1)
