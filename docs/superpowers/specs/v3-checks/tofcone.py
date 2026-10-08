import math, itertools
import numpy as np
from final_check import *

R = 10.5
F = build_model(R)
setup_R(R)   # sets P for R
# sensor list: (name, position radius, psi, aim, z)
Z = P['tof_zc']
S = [('front centre (recessed 1 cm)', R - 1.0, 0, 0), ('front toed-out L', R - 0.2, 25, 25), ('front toed-out R', R - 0.2, -25, -25),
     ('side front L', R - 0.2, 45, 90), ('side front R', R - 0.2, -45, -90), ('side rear L', R - 0.2, 135, 90), ('side rear R', R - 0.2, -135, -90),
     ('rear L', R - 0.2, 154, 180), ('rear R', R - 0.2, -154, 180)]
# obstacles: all fixed parts except ToF boards, plus cameras, plus the body shell seen from inside
cam = []
from place2 import cam_points, hull_part
lens, board = cam_points(88, R - 0.2, 9.1, 20, 90, 'h')
for s in (1, -1):
    l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
    cam += [hull_part('camL_lens' if s > 0 else 'camR_lens', l), hull_part('camL_board' if s > 0 else 'camR_board', b)]
obst = [f for f in F if f.kind != 'tof'] + cam
bands = build_bands(obst, buf=0.0)
def hit_dist(pos, az_deg, el_deg, dmax=30.0, start=0.3):
    x0, y0 = pos; a = math.radians(az_deg); e = math.radians(el_deg)
    d = start
    while d < dmax:
        x = x0 + d*math.cos(e)*math.cos(a); y = y0 + d*math.cos(e)*math.sin(a); z = Z + d*math.sin(e)
        if z < 0: return d, 'floor'
        for b0, b1, u, pu in bands:
            if b0 <= z < b1:
                if pu.contains(Point(x, y)): return d, 'robot part'
                break
        d += 0.1
    return None, ''
print('Each VL53L0X cone (25 deg full angle) traced as 5x5 rays to 30 cm; the first hit on the ROBOT itself (not the floor) would be a blocked view.')
for name, r, psi, aim in S:
    a = math.radians(psi); pos = (r*math.cos(a), r*math.sin(a))
    hits = []
    for da in (-12.5, -6.25, 0, 6.25, 12.5):
        for de in (-12.5, -6.25, 0, 6.25, 12.5):
            if math.hypot(da, de) > 12.5: continue
            d, what = hit_dist(pos, aim + da, de)
            if d is not None and what == 'robot part': hits.append((round(d, 1), da, de))
    # first floor hit of the lowest ray
    d_floor, _ = hit_dist(pos, aim, -12.5)
    print(f'{name:<30} pos ({pos[0]:5.2f},{pos[1]:5.2f}) aim {aim:>4}: ' + ('NO robot part in the cone' if not hits else f'BLOCKED by robot at {min(hits)}'))
# ramp-foot reading for the front centre sensor at tile centre (ramp foot 15.0 cm from the tile centre, sensor 9.5 cm ahead of the axle)
t25 = math.tan(math.radians(25)); t12 = math.tan(math.radians(12.5))
for zs in (6.5, 8.1, 10.0):
    dfoot = 15.0 - 9.5
    centre = zs/t25 + dfoot
    lower = (zs + dfoot*t25)/(t12 + t25)
    print(f'sensor height {zs:4.1f} cm: ramp-foot reading at tile centre: beam centre {centre*10:5.0f} mm, nearest cone edge {lower*10:5.0f} mm  (wall at tile centre reads {(14.0-9.5)*10:.0f} mm, wall max when stopped 30 mm short {(14.0-9.5)*10+30:.0f} mm)')
