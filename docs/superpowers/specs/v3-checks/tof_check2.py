"""Trace every VL53L0X cone (25 deg full angle, 5 x 5 rays to 30 cm) against the V3 baseline: 14 mm plate, chute exit x -6.0, camera recessed, PCB stack and battery included."""
import json, math
from final_pack2 import *
from final_check import *

F = build()
lens, board = cam_points(88, CAM_TIP, CAM_Z, CAM_TILT, 90, 'h')
cam = []
for s in (1, -1):
    l = lens.copy(); b = board.copy(); l[:, 1] *= s; b[:, 1] *= s
    cam += [hull_part('camL_lens' if s > 0 else 'camR_lens', l), hull_part('camL_board' if s > 0 else 'camR_board', b)]
pk = json.load(open('pack_v3.json'))['parts']
extra = []
for name, d in pk.items():
    l, w, h = (7.0, 3.5, 2.5) if name.startswith('battery') else (10.152, 5.334, 1.9)
    extra.append(Part(name, rect(d['x'], d['y'], l, w, d['angle']), d['z0'], d['z1'], 'elec'))
obst = [f for f in F if f.kind != 'tof'] + cam + extra
bands = build_bands(obst, buf=0.0)
Z = P['tof_zc']
R = 10.5
S = [('F (recessed 1 cm)', R - 1.0, 0, 0), ('FL', R - 0.2, 25, 25), ('FR', R - 0.2, -25, -25), ('SFL', R - 0.2, 45, 90), ('SFR', R - 0.2, -45, -90),
     ('SRL', R - 0.2, 135, 90), ('SRR', R - 0.2, -135, -90), ('RL', R - 0.2, 154, 180), ('RR', R - 0.2, -154, 180)]

def hit_dist(pos, az_deg, el_deg, dmax=30.0, start=0.3):
    x0, y0 = pos; a = math.radians(az_deg); e = math.radians(el_deg); d = start
    while d < dmax:
        x = x0 + d * math.cos(e) * math.cos(a); y = y0 + d * math.cos(e) * math.sin(a); z = Z + d * math.sin(e)
        if z < 0: return d, 'floor'
        for b0, b1, u, pu in bands:
            if b0 <= z < b1:
                if pu.contains(Point(x, y)): return d, 'robot part'
                break
        d += 0.1
    return None, ''

print('ToF cones against the V3 baseline (plate R', round(P['R_plate'], 2), 'cm, chute exit x', P['exit_x'], ')')
allok = True
for name, r, psi, aim in S:
    a = math.radians(psi); pos = (r * math.cos(a), r * math.sin(a)); hits = []
    for da in (-12.5, -6.25, 0, 6.25, 12.5):
        for de in (-12.5, -6.25, 0, 6.25, 12.5):
            if math.hypot(da, de) > 12.5: continue
            d, what = hit_dist(pos, aim + da, de)
            if d is not None and what == 'robot part': hits.append((round(d, 1), da, de))
    ok = not hits; allok &= ok
    print(f'  {name:<20} pos ({pos[0]:5.2f},{pos[1]:5.2f}) aim {aim:>4}: ' + ('no robot part in the cone' if ok else f'BLOCKED by the robot at {min(hits)}'))
print('ALL NINE CLEAR' if allok else 'SOME BLOCKED')
