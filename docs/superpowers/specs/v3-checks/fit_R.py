"""Does the GIGA R1 + main PCB stack (and the battery) fit at body radius 9.5, 10.0 and 10.5, with the wheel track, motors, omni, chute exit and camera adjusted for each radius?
Everything that depends on R is recomputed here (the earlier giga.py sweep kept the 10.5 wheel track at every R, which pushes the wheel corners through the wall at R 9.5).
Run: python fit_R.py [R ...]"""
import json, math, random, sys
from final_check import *
from place2 import cam_points, pts_ok, hull_part
from chamfer_pack import chamfer_gap

RING, POCKET, SLOT = 3.86, 1.40, 1.45
WALL_CLEAR = 0.5            # the draft's rule: at least 5 mm between a chute and the wheel

def params_for(R):
    r_int = R - P['wall']
    y_out = min(9.0, math.sqrt((r_int - 0.1) ** 2 - 16.0))          # wheel outer face, corner of the tyre 1 mm inside the wall
    return dict(R=R, y_out=round(y_out, 2), y_in=round(y_out - 2.0, 2))

def build(R, exit_x):
    q = params_for(R)
    P['R_body'] = R; P['R_int'] = R - P['wall']; P['R_sw'] = R + 0.5; P['omni_x'] = P['R_sw'] - P['omni_r'] - 0.05
    P['arm_pivot'] = (3.5, 3.6); P['x_p'] = -2.0; P['exit_x'] = exit_x; P['r_ring'] = RING; P['pocket'] = POCKET; P['slot'] = SLOT; P['z_lid'] = 12.1
    P['wheel_yin'] = q['y_in']; P['wheel_yout'] = q['y_out']; P['motor_face_y'] = q['y_in'] - 0.9
    return fixed_parts() + plate_parts() + tof_parts() + bumper_parts(), q

def chute_wheel_gap(F):
    chutes = [f for f in F if f.name.startswith('chute')]; wheels = [f for f in F if f.name.startswith('wheel')]
    return min(c.poly.distance(w.poly) for c in chutes for w in wheels if not (w.z1 <= c.z0 or w.z0 >= c.z1))

def fit(R, seed=11, stack=(10.152, 5.334, 1.9)):
    # chute exit: the most forward x that keeps 5 mm to the wheel
    exit_x = None
    for ex in [-4.8 - 0.1 * k for k in range(0, 40)]:
        F, q = build(R, ex)
        if chute_wheel_gap(F) >= WALL_CLEAR: exit_x = ex; break
    F, q = build(R, exit_x)
    out = dict(R=R, y_in=q['y_in'], y_out=q['y_out'], exit_x=round(exit_x, 2), track=round(q['y_in'] + 1.0, 2) * 2)
    inner = Point(0, 0).buffer(P['R_int'], 128)
    # camera above the wheel, lens recessed 1.5 cm behind the window (tip radius R - 1.7), lens z 9.3, tilt 20: search the tip radius down from there
    best_cam = None
    for tip_r in [R - 1.7 + 0.1 * k for k in range(0, 15)]:
        for zl in (9.3, 9.4):
            lens, board = cam_points(88, tip_r, zl, 20, 90, 'h')
            lm = lens.copy(); lm[:, 1] *= -1; bm = board.copy(); bm[:, 1] *= -1
            bands0 = build_bands(F)
            if pts_ok(bands0, lens, P['R_sw'] - 0.05) and pts_ok(bands0, board, P['R_int']) and pts_ok(bands0, lm, P['R_sw'] - 0.05) and pts_ok(bands0, bm, P['R_int']):
                best_cam = (tip_r, zl); break
        if best_cam: break
    out['camera'] = best_cam
    if best_cam is None:
        out['stack'] = None; return out
    tip_r, zl = best_cam
    lens, board = cam_points(88, tip_r, zl, 20, 90, 'h'); parts = []
    for s in (1, -1):
        l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
        parts += [hull_part(f'cam{"L" if s > 0 else "R"}_lens', l), hull_part(f'cam{"L" if s > 0 else "R"}_board', b)]
    bands = build_bands(F + parts)
    random.seed(seed); placed = []; res = {}
    for name, l, w, h, zs, xr in (('battery', 7.0, 3.5, 2.5, [3.7 + 0.2 * k for k in range(0, 10)], (2.5, 10)),
                                  ('stack', stack[0], stack[1], stack[2], [4.2 + 0.1 * k for k in range(0, 30)], (-10, 10))):
        best = None
        for _ in range(80000 if name == 'stack' else 30000):
            x = random.uniform(*xr); y = random.uniform(-10, 10); ang = random.choice(range(0, 180, 10)); z0 = random.choice(zs); z1 = z0 + h
            if name == 'stack' and z1 > P['z_pf'] - 0.4: continue
            poly = rect(x, y, l, w, ang)
            if not poly.within(inner) or chamfer_gap(poly, z0) < 0 or collides(bands, poly, z0, z1): continue
            if any(z0 < q_[3] and z1 > q_[2] and poly.distance(q_[4]) < CL for q_ in placed): continue
            d = min(min_dist(bands, poly, z0, z1), chamfer_gap(poly, z0))
            for q_ in placed:
                if z0 < q_[3] and z1 > q_[2]: d = min(d, poly.distance(q_[4]))
            if best is None or d > best[0]: best = (d, x, y, ang, z0, z1, poly)
        res[name] = None if best is None else dict(x=round(best[1], 2), y=round(best[2], 2), angle=best[3], z0=round(best[4], 2), z1=round(best[5], 2), clearance_mm=round(best[0] * 10, 1))
        if best is not None: placed.append((name, None, best[4], best[5], best[6]))
    out.update(res)
    return out

if __name__ == '__main__':
    Rs = [float(a) for a in sys.argv[1:]] or [9.5, 10.0, 10.5]
    for R in Rs:
        r = fit(R)
        print('(GIGA R1 + shield, 10.15 x 5.33 x 1.9 cm)')
        print(f'R {R}: wheel y {r["y_in"]}..{r["y_out"]} (track {r["track"]:.1f} cm), chute exit x {r["exit_x"]}, camera (tip radius, lens z) {r["camera"]}')
        for k in ('battery', 'stack'):
            v = r.get(k)
            print(f'      {k:<8}', 'NO PLACEMENT' if v is None else f'x {v["x"]:6.2f} y {v["y"]:6.2f} angle {v["angle"]:>3} z {v["z0"]}-{v["z1"]}  clearance {v["clearance_mm"]} mm')

    # the alternative: a small main board (Teensy 4.1 in sockets on a custom board) at R 9.5, battery included
    for dims in ((7.0, 4.5, 1.5), (6.5, 4.0, 1.5)):
        r = fit(9.5, stack=dims)
        print(f'(main board {dims[0]} x {dims[1]} x {dims[2]} cm) R 9.5: wheel track {r["track"]:.1f} cm, chute exit x {r["exit_x"]}, camera {r["camera"]}')
        for k in ('battery', 'stack'):
            v = r.get(k)
            print(f'      {k:<8}', 'NO PLACEMENT' if v is None else f'x {v["x"]:6.2f} y {v["y"]:6.2f} angle {v["angle"]:>3} z {v["z0"]}-{v["z1"]}  clearance {v["clearance_mm"]} mm')
