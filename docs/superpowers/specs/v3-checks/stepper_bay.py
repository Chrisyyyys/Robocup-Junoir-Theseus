"""Does a 28BYJ-48 stepper fit under the dropper floor, in place of the N20? (mechanical design 2026-10-07, section 6.3). cm. Needs shapely.
Datasheet (Kiatronics 28BYJ-48 drawing, page 1): body 28 mm across and 19 mm tall, two ears 35 mm apart (R 3.5, 4.2 mm holes) on the front face, the output shaft 8 mm OFF the body
centre (at right angles to the ear line), the wire block 14.6 mm wide on the far side reaching 17 mm from the body centre. The front face goes against the floor underside (z 8.7)
and the shaft goes up through the floor to the plate, so the shaft sits on the plate axis (-2.0, 0) and the body centre is 8 mm to one side of it. The motor can be turned to any angle.
The check is in plan against every part whose height range overlaps the stepper's (z 6.8..8.7); the drive motors (top z 5.0), the silver module (top z 4.2), the posts (below z 6.15) are below it."""
import math

from shapely.affinity import rotate
from shapely.geometry import LineString, Point, box

AXIS = (-2.0, 0.0)                                   # plate and shaft axis (spec section 7)
Z_TOP, BODY_H = 8.7, 1.9
BODY_R, EAR_SPAN, EAR_R, BLOCK_W, BLOCK_REACH, OFFSET = 1.4, 1.75, 0.35, 1.46, 1.7, 0.8
Z_LO, Z_EAR = Z_TOP - BODY_H, Z_TOP - 0.7             # body bottom 6.8, ears 8.0 .. 8.7

# parts as (plan polygon, z0, z1): GIGA stack and battery from pack_v3.json, hoppers as designed (19.5 mm box with a 23.5 mm flange, 45 degrees, under the slots)
STACK = box(2.32 - 5.076, -4.03 - 2.667, 2.32 + 5.076, -4.03 + 2.667)
BATT = rotate(box(3.9 - 3.5, 4.35 - 1.75, 3.9 + 3.5, 4.35 + 1.75), 10, origin=(3.9, 4.35))
HOPPER_SIDE = 2.35
PARTS = {'GIGA stack': (STACK, 6.15, 8.05), 'battery': (BATT, 5.1, 7.6)}
for nm, (sx, sy) in (('hopper A', (-4.73, -2.73)), ('hopper B', (-4.73, 2.73))):
    PARTS[nm] = (rotate(box(sx - HOPPER_SIDE / 2, sy - HOPPER_SIDE / 2, sx + HOPPER_SIDE / 2, sy + HOPPER_SIDE / 2), 45, origin=(sx, sy)), 7.2, 8.7)
WALL = Point(0, 0).buffer(10.3)                       # tub wall inner face

def stepper(theta_deg):
    """Plan solids of the stepper when the shaft offset direction (body centre -> shaft) points at theta (0 = +x, 90 = +y)."""
    t = math.radians(theta_deg); ux, uy = math.cos(t), math.sin(t); vx, vy = -uy, ux
    cx, cy = AXIS[0] - OFFSET * ux, AXIS[1] - OFFSET * uy
    body = Point(cx, cy).buffer(BODY_R + 0.05, 64)
    ears = [LineString([(cx + vx * 1.2 * s, cy + vy * 1.2 * s), (cx + vx * EAR_SPAN * s, cy + vy * EAR_SPAN * s)]).buffer(EAR_R) for s in (1, -1)]
    blk = LineString([(cx - ux * 1.2, cy - uy * 1.2), (cx - ux * BLOCK_REACH, cy - uy * BLOCK_REACH)]).buffer(BLOCK_W / 2, cap_style=2)
    return (cx, cy), body, ears, blk

def clearances(theta_deg):
    (cx, cy), body, ears, blk = stepper(theta_deg)
    solids = [('body', body, Z_LO, Z_TOP), ('ear', ears[0], Z_EAR, Z_TOP), ('ear', ears[1], Z_EAR, Z_TOP), ('wire block', blk, Z_LO, Z_TOP)]
    out = []
    for pn, (poly, z0, z1) in PARTS.items():
        best = 1e9
        for sn, sp, a, b in solids:
            if min(b, z1) > max(a, z0):                  # heights overlap, so the plan distance counts
                best = min(best, sp.distance(poly) if not sp.intersects(poly) else -sp.intersection(poly).area ** 0.5)
        out.append((pn, best * 10))
    out.append(('tub wall', (10.3 - max(Point(0, 0).hausdorff_distance(s) for _, s, _, _ in solids)) * 10))
    return (cx, cy), out

if __name__ == '__main__':
    print('shaft on the plate axis (-2.0, 0); angle = direction from the body centre to the shaft (0 = +x front, 90 = +y left); clearances in mm in plan against parts at the same height')
    for th in range(0, 360, 15):
        (cx, cy), out = clearances(th)
        worst = min(out, key=lambda r: r[1])
        flag = 'fits' if worst[1] >= 2.0 else ('touches' if worst[1] >= 0 else 'COLLIDES')
        print(f'  {th:3d} deg  body centre ({cx:5.2f}, {cy:5.2f})  tightest: {worst[0]:<10s} {worst[1]:6.1f} mm  -> {flag:9s}  ' + '  '.join(f'{n} {v:5.1f}' for n, v in out))
    print('\nbest orientations (all clearances at least 2 mm):')
    ok = [th for th in range(0, 360, 5) if min(v for _, v in clearances(th)[1]) >= 2.0]
    print('  ' + (', '.join(str(t) for t in ok) if ok else 'none'))
    if ok:
        th = max(ok, key=lambda t: min(v for _, v in clearances(t)[1]))
        (cx, cy), out = clearances(th)
        print(f'  widest margin at {th} deg: body centre ({cx:.2f}, {cy:.2f}); ' + ', '.join(f'{n} {v:.1f} mm' for n, v in out))
