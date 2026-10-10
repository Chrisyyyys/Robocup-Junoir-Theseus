"""Where on the ring can the six frame screws and the four lid hooks go? (mechanical design 2026-10-07, sections 3.1 and 7.1). cm, angles in degrees from +x counter-clockwise. Needs shapely.
Rev 4 of 9 Oct: the windows are what is cut from the ring for each ToF board (tof_mount: the pocket with its cable ear, the beam tunnel and the two screwdriver access holes: the real 25.4 mm board is wider than the
21 x 18 mm of the product page) and the two camera windows. Screw: hole 6.5 mm across at r 9.75 plus 3 mm of material to a window. Hook: a 20 mm wide tongue in the lid skirt at r 10.5 plus 3 mm to a window.
Everything is mirror symmetric about y = 0 as far as the windows go, so only y >= 0 is listed. 9 Oct: the windows are what is cut from the ring for each ToF board (tof_mount: the pocket, the beam tunnel, the plug shaft and the
two screwdriver access holes) and the two camera windows. The real board is 25.4 mm long: lying down it needs a pocket 3.2 to 3.7 cm wide, the front screws at +-12 degrees have no room (the free range at r 9.75 starts at 54 degrees) and
the pockets of FL and SFL overlap; standing on its short end it needs 2.4 cm, and the screws keep their rev 4 angles.
"""
import math

from shapely.geometry import Point, box

import tof_mount as T
import v3_params4 as P

R_IN, R_OUT, R_SCREW = P.FRAME['r_in'], P.FRAME['r_out'], P.SCREW['r']
R_HOOK = 10.42
CAM_X = P.CAM_X


def all_windows():
    """Every opening the ring needs: nine ToF pockets with their tunnels and the two camera windows."""
    wins = {nm: T.polygons(nm)['cut'] for nm in T.sensors()}
    for s in (1, -1):
        wins['CAM' + ('L' if s > 0 else 'R')] = box(CAM_X - P.CAMERA['window_w'] / 2, s * 8.0 if s > 0 else -11.0, CAM_X + P.CAMERA['window_w'] / 2, 11.0 if s > 0 else -8.0)      # through the ring
    return wins


def ang_extent(poly, r):
    """Angular interval (deg, 0 .. 180: the y >= 0 half) that the polygon covers on the circle of radius r."""
    a = [k / 4 for k in range(0, 721) if poly.contains(Point(r * math.cos(math.radians(k / 4)), r * math.sin(math.radians(k / 4))))]
    return (min(a), max(a)) if a else None


def free_ranges(wins, half_cm, r):
    """Free angular intervals (deg) on the upper half where a feature of half-width half_cm (cm along the circle at radius r) fits clear of every window."""
    blocked = []
    for nm, w in wins.items():
        if nm.endswith('R') and nm != 'F' or nm in ('FR', 'SFR', 'SRR', 'RR', 'CAMR'):
            continue
        e = ang_extent(w.buffer(half_cm), r)
        if e:
            blocked.append(e)
    blocked.sort()
    out, a = [], 0.0
    for lo, hi in blocked:
        if lo > a:
            out.append((a, lo))
        a = max(a, hi)
    if a < 180:
        out.append((a, 180.0))
    return [(round(lo, 1), round(hi, 1)) for lo, hi in out if hi - lo > 0.5]


if __name__ == '__main__':
    wins = all_windows()
    print('angular extent of each window at r 9.75 (screw radius) and r 10.5 (outer face), y >= 0 side:')
    for nm, w in wins.items():
        if nm in ('FR', 'SFR', 'SRR', 'RR', 'CAMR'):
            continue
        e1, e2 = ang_extent(w, R_SCREW), ang_extent(w, R_OUT)
        print('  %-6s r 9.75: %6.1f .. %6.1f    r 10.5: %6.1f .. %6.1f' % (nm, e1[0], e1[1], e2[0], e2[1]) if e1 and e2 else '  %-6s (not on the circle)' % nm)
    print('free angular ranges on y >= 0 (centre of the feature may lie in these):')
    print('  screws (hole 6.5 mm + 3 mm):  ', free_ranges(wins, 0.325 + 0.30, R_SCREW))
    print('  hooks (tongue 20 mm + 3 mm):  ', free_ranges(wins, 1.0 + 0.30, R_HOOK))
    print('rev 4 as built: screws at %s, hooks at %s degrees (each +- on both sides); margin of each to the nearest window, mm:' % (P.SCREW_DEG, P.HOOK_DEG))
    y0 = [w for nm, w in wins.items() if nm not in ('FR', 'SFR', 'SRR', 'RR', 'CAMR')]
    for a in P.SCREW_DEG:
        cx, cy = P.polar(R_SCREW, a)
        d = min(w.distance(Point(cx, cy).buffer(P.SCREW['cbore_r'])) for w in y0)
        print('  screw %4.0f deg: counterbore edge to the nearest window %5.1f mm' % (a, d * 10))
    for a in P.HOOK_DEG:
        cx, cy = P.polar(R_HOOK, a)
        d = min(w.distance(Point(cx, cy).buffer(1.0)) for w in y0)
        print('  hook  %4.0f deg: tongue edge (20 mm wide) to the nearest window %5.1f mm' % (a, d * 10))
    print('the rear screw (118) sits within the angular span of the hook at 114 but 2.9 mm inside it radially (counterbore r 10.05, rebate r 10.34)')
