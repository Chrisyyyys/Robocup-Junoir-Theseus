"""Re-test the geometry changes I made against the approved draft: bumper end angle at R=10.5, chute exit x, omni travel 3.0, near-axle floor sensor."""
import math, sys
from final_check import *
from shapely.geometry import Polygon, box

R = 10.5
# 1. bumper end angle vs wheel and the other fixed parts at R = 10.5
F = build_model(R)
from layout import bumper_parts
print('--- bumper plates at R 10.5 (outer radius 11.0), band z 4.0-6.5: overlap with the wheels / motors / omni arm / ToF boards')
for end in (58, 62, 66, 70):
    parts = []
    for s in (1, -1):
        pts = []
        for ang in [12 + (end - 12) * k / 12 for k in range(13)]:
            a = math.radians(ang * s); pts.append((11.0 * math.cos(a), 11.0 * math.sin(a)))
        for ang in [end - (end - 12) * k / 12 for k in range(13)]:
            a = math.radians(ang * s); pts.append((10.75 * math.cos(a), 10.75 * math.sin(a)))
        parts.append(Part(f'bump{s}', Polygon(pts), 4.0, 6.5, 'bumper'))
    others = [f for f in F if not f.name.startswith('bump')]
    worst = 9e9
    for b in parts:
        for o in others:
            if o.z1 <= b.z0 or o.z0 >= b.z1: continue
            worst = min(worst, b.poly.distance(o.poly))
    reach = 11.0 * math.sin(math.radians(end))
    print(f'  ends at {end} deg: min plan distance to any other part {worst:5.2f} cm; plate end reaches y = {reach:5.2f} cm, {12.6 - reach:4.2f} cm from the wall of a 25.2 cm path, {14.0 - reach:4.2f} cm from a 28 cm wall')

# 2. chute exit x: overlaps and distances to the wheel/motor at R = 10.5
print('--- chute exit x (R 10.5): smallest plan distance from the chute strip to the wheel / motor / omni arm (parts that overlap it in z)')
for ex in (-5.5, -6.0, -6.5, -7.0):
    P['R_body'] = R; P['R_int'] = R - P['wall']; P['R_sw'] = R + 0.5; P['omni_x'] = P['R_sw'] - P['omni_r'] - 0.05
    P['exit_x'] = ex; P['x_p'] = -2.0
    Fx = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    chutes = [f for f in Fx if f.name.startswith('chute')]
    others = [f for f in Fx if not f.name.startswith('chute') and not f.name.startswith('plate') and f.kind != 'tof']
    d = 9e9; who = ''
    for c in chutes:
        for o in others:
            if o.z1 <= c.z0 or o.z0 >= c.z1: continue
            dd = c.poly.distance(o.poly)
            if dd < d: d, who = dd, o.name
    ey = math.sqrt(R * R - ex * ex)
    print(f'  exit x {ex}: y {ey:5.2f}  nearest other part {who} at {d:5.2f} cm')
