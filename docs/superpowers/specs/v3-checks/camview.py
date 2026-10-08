"""Camera view of the victim wall, re-derived from the geometry (cm, degrees). (Kit landing moved to kit_final.py.)
Camera: OpenMV H7 Plus, FOV H 65.9 / V 51.8. Lens centre height zl, tilted down by `tilt`, looking sideways (azimuth 90)."""
import math, itertools, sys
from final_check import *
from place2 import cam_points, pts_ok

HF, VF = 65.9, 51.8
th, tv = math.tan(math.radians(HF / 2)), math.tan(math.radians(VF / 2))

def view(d_h, zl=9.1, tilt=20.0):
    """d_h = horizontal distance from the lens to the wall. Returns (z_bottom, z_top, lateral width at the centre row)"""
    t = math.radians(tilt)
    zb = zl - d_h * math.tan(t + math.radians(VF / 2)); zt = zl - d_h * math.tan(t - math.radians(VF / 2))
    lat = 2 * (d_h / math.cos(t)) * th
    return zb, zt, lat

def coverage_table():
    print('lens axis z 9.3 (baseline); wall distance from the lens tip (horizontal); band to see: letter z 5..9 cm (4 cm letter, centre about 7 cm), 4 cm wide')
    print(f'{"lens tip y":>10} {"path":>6} {"d_h":>5} | tilt: ' + ' | '.join(f'{t:>2} deg: z-range (band seen) width' for t in (20, 25, 30)))
    for tip_y in (10.3, 8.8):
        for path in (28.0, 25.2):
            d = path / 2 - tip_y
            cells = []
            for tilt in (20, 25, 30):
                zb, zt, lat = view(d, 9.3, tilt)
                zb2 = max(zb, 0.0)
                seen = max(0.0, min(zt, 9.0) - max(zb2, 5.0)) / 4.0
                cells.append(f'{zb2:4.1f}-{zt:4.1f} ({100*seen:3.0f}%) {lat:3.1f}')
            print(f'{tip_y:>10.1f} {path:>6.1f} {d:>5.2f} | ' + ' | '.join(cells))

def inboard_feasibility(R=10.5):
    F2, bands, cam = setup_R(R)    # includes the camera at tip radius R-0.2; rebuild without it
    F = build_model(R)
    bands0 = build_bands(F)
    print('camera lens tip radius sweep (psi 88, lens z 9.1): feasible against fixed parts, ToF boards, bumpers, plate, chutes')
    for tilt in (20, 25, 30):
        row = []
        for tip_r in (10.3, 9.8, 9.3, 8.8, 8.3, 7.8):
            lens, board = cam_points(88, tip_r, 9.1, tilt, 90, 'h')
            lm = lens.copy(); lm[:, 1] *= -1; bm = board.copy(); bm[:, 1] *= -1
            ok = pts_ok(bands0, lens, P['R_sw'] - 0.05) and pts_ok(bands0, board, P['R_int']) and pts_ok(bands0, lm, P['R_sw'] - 0.05) and pts_ok(bands0, bm, P['R_int'])
            top = float(board[:, 2].max())
            row.append(f'r={tip_r}: {"ok" if ok else "NO"} (top {top:4.1f})')
        print(f'  tilt {tilt}: ' + '  '.join(row))

def kit_landing(mu=0.35, fall_extra=0.0, R=10.5):
    g = 9.81
    sx, sy = -2.0 + 3.62 * math.cos(math.radians(135)), 3.62 * math.sin(math.radians(135))
    ex = -(R - 3.5); ey = math.sqrt(R * R - ex * ex)
    z_start = 9.0 - 0.3 - 0.75; z_end = 3.0 + 0.75
    plan = math.hypot(ex - sx, ey - sy); drop = z_start - z_end; L = math.hypot(plan, drop)
    sl = math.atan2(drop, plan)
    a = g * (math.sin(sl) - mu * math.cos(sl))
    v = math.sqrt(2 * a * L / 100) if a > 0 else 0.0
    vh = v * math.cos(sl); vz = v * math.sin(sl)
    # fall from the centre of the kit at the nose (z_end) to the floor (kit half height 0.5 cm)
    h = (z_end - 0.5 + fall_extra) / 100
    t = (-vz + math.sqrt(vz * vz + 2 * g * h)) / g
    dist = vh * t * 100
    ux, uy = (ex - sx) / plan, (ey - sy) / plan
    lx, ly = ex + ux * dist, ey + uy * dist
    print(f'chute: slot ({sx:.2f},{sy:.2f}) -> exit ({ex:.2f},{ey:.2f}) plan {plan:.2f} cm, drop {drop:.2f} cm, slope {math.degrees(sl):.1f} deg, path {L:.2f} cm')
    print(f'  mu {mu}: exit speed {v:.2f} m/s (horizontal {vh:.2f}), flight {t*1000:.0f} ms, carries {dist:.1f} cm -> lands at ({lx:.1f}, {ly:.1f})  [wall at y=14.0: {14.0-ly:.1f} cm away; 12.6: {12.6-ly:.1f}]')
    return lx, ly

if __name__ == '__main__':
    coverage_table()      # flush lens (tip y 10.3) versus recessed lens (tip y 8.8), tilt 20 / 25 / 30 degrees
    print()
    inboard_feasibility()  # how far the lens can be recessed against the fixed parts (the lid limit z 11.9 rules out 25 and 30 degrees; see cam_recess.py)
