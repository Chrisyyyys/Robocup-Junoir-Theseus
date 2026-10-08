"""Where on the ring can the six frame screws and the four lid hooks go? (mechanical design 2026-10-07, sections 3.1 and 7.1). cm, angles in degrees from +x counter-clockwise. Needs shapely.
The windows are the prisms the nine ToF cones and the two cameras need through the ring (r 9.0 .. 10.5): half-width 0.9 at the board (board 1.8 wide), widening with the 25 degree cone, plus 0.1 clearance.
Screw: hole 6.5 mm across at r 9.75 plus 3 mm of material to a window. Hook: a 20 mm wide tongue in the lid skirt at r 10.5 plus 3 mm to a window. Everything is mirror symmetric about y = 0, so only y >= 0 is listed."""
import math

from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union

R_IN, R_OUT, R_SCREW = 9.0, 10.5, 9.75
CONE = math.radians(12.5)
TOF = {'F': (9.5, 0.0, 0), 'FL': (8.97, 4.18, 25), 'SFL': (7.18, 6.0, 90), 'SRL': (-7.18, 6.0, 90), 'RL': (-8.4, 4.5, 180),
       'FR': (8.97, -4.18, -25), 'SFR': (7.18, -6.0, -90), 'SRR': (-7.18, -6.0, -90), 'RR': (-8.4, -4.5, 180)}

def window(x, y, aim, half0=0.9, clear=0.1, L=3.0, back=1.2):
    """Pocket behind the board (back cm, as wide as the board plus clearance) and the prism the cone needs ahead of it."""
    a = math.radians(aim); dx, dy = math.cos(a), math.sin(a); nx, ny = -dy, dx
    h0 = half0 + clear; h1 = h0 + L * math.tan(CONE)
    return Polygon([(x - dx * back + nx * h0, y - dy * back + ny * h0), (x + dx * L + nx * h1, y + dy * L + ny * h1), (x + dx * L - nx * h1, y + dy * L - ny * h1), (x - dx * back - nx * h0, y - dy * back - ny * h0)])

def ang_extent(poly, r):
    """Angular interval (deg, 0 .. 180: the y >= 0 half) that the polygon covers on the circle of radius r."""
    a = [k / 4 for k in range(0, 721) if poly.contains(Point(r * math.cos(math.radians(k / 4)), r * math.sin(math.radians(k / 4))))]
    return (min(a), max(a)) if a else None

def all_windows():
    """Every opening the ring needs: nine ToF pockets with their cones and the two camera windows (the controls moved to the front bridge, so the rear panel window of the first rev 4 draft is gone)."""
    wins = {nm: window(*v) for nm, v in TOF.items()}
    cam_x = 8.8 * math.cos(math.radians(88))
    for s in (1, -1):
        wins['CAM' + ('L' if s > 0 else 'R')] = box(cam_x - 1.4, s * 8.0 if s > 0 else -11.0, cam_x + 1.4, 11.0 if s > 0 else -8.0)       # camera window 2.8 wide, through the ring
    return wins


if __name__ == '__main__':
    wins = all_windows()
    print('angular extent of each window at r 9.75 (screw radius) and r 10.5 (outer face), y >= 0 side:')
    ext = {}
    for nm, w in wins.items():
        e1, e2 = ang_extent(w, R_SCREW), ang_extent(w, R_OUT)
        if nm in ('FR', 'SFR', 'SRR', 'RR', 'CAMR'):
            continue
        ext[nm] = (e1, e2)
        print(f'  {nm:6s} r 9.75: {e1[0]:6.1f} .. {e1[1]:6.1f}    r 10.5: {e2[0]:6.1f} .. {e2[1]:6.1f}')
    def free(r_idx, half_cm, r):
        """Free angular intervals (deg) on the upper half where a feature of half-width half_cm (cm along the circle at radius r) fits clear of every window."""
        blocked = []
        for nm, w in wins.items():
            e = ang_extent(w.buffer(half_cm), r)
            if e and nm not in ('FR', 'SFR', 'SRR', 'RR', 'CAMR'): blocked.append(e)
        blocked.sort(); out = []; a = 0.0
        for lo, hi in blocked:
            if lo > a: out.append((a, lo))
            a = max(a, hi)
        if a < 180: out.append((a, 180.0))
        return [(round(l, 1), round(h, 1)) for l, h in out if h - l > 0.5]
    print('free angular ranges on y >= 0 (centre of the feature may lie in these):')
    print('  screws (hole 6.5 mm + 3 mm):  ', free(0, 0.325, R_SCREW))
    print('  hooks (tongue 20 mm + 3 mm):  ', free(1, 1.0, R_OUT))
    print('proposed: screws at 12, 60, 118 degrees, hooks at 71 and 114 degrees (each +- on both sides); margin of each to the nearest window, mm along the circle:')
    for nm, a, r in (('screw', 12, R_SCREW), ('screw', 60, R_SCREW), ('screw', 118, R_SCREW), ('hook', 71, R_OUT), ('hook', 114, R_OUT)):
        half = 0.325 if nm == 'screw' else 1.0
        cx, cy = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        feat = Point(cx, cy).buffer(half) if nm == 'screw' else Point(cx, cy).buffer(half)
        d = min(w.distance(feat) for n, w in wins.items() if n not in ('FR', 'SFR', 'SRR', 'RR', 'CAMR'))
        print(f'  {nm:5s} {a:4d} deg: nearest window {d * 10:5.1f} mm')
    print('screw to hook separation: 60 -> 71 degrees = %.1f mm at r 10.1; the rear screw (118) sits within the angular span of the hook at 114 but 2.9 mm inside it radially (counterbore r 10.05, rebate r 10.34)' % (math.radians(11) * 10.1 * 10))
