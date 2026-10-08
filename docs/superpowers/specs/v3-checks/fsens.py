"""Floor-colour sensors ahead of the belly (spec section 5): do they hang lower than the belly and become the first thing to hit a riser or ramp?
A sensor face at height zs above the floor (flat ground), a 1.6 cm wide window centred at x_s ahead of the axle."""
import math, sys
from nubtable import *
from shapely.geometry import LineString

def fsens_clear(T, xd, zd, ph, xs_, zs, half=0.8):
    seg = LineString([world((xs_ - half, zs - AXLE_Z), xd, zd, ph), world((xs_ + half, zs - AXLE_Z), xd, zd, ph)])
    return -T.intersection(seg).length if T.intersects(seg) else seg.distance(T)

def stats_fs(profile, R, xs, xs_, zs):
    T = terrain(profile); worst = 9e9; where = None
    for xd in xs:
        zd = wheel_centre_z(T, xd, R.wheel_r)
        for i in range(11):
            c = R.travel * i / 10
            ph = pitch_for_omni(T, R, xd, zd, c)
            if ph is None or not rear_ok(T, R, xd, zd, ph): continue
            v = fsens_clear(T, xd, zd, ph, xs_, zs)
            if v < worst: worst, where = v, (xd, round(math.degrees(ph), 1), c)
            break          # stiff-spring pose (smallest feasible compression), as in stats()
    return worst, where

if __name__ == '__main__':
    R = mk()
    xs = [x / 1.0 for x in range(-25, 50)]
    cases = ['riser_up', 'riser_down', 'ramp_up', 'ramp_down', 'bump2', 'bump1', 'steps2x1_up']
    for xs_, zs in ((6.5, 3.5), (6.5, 3.0), (6.5, 2.5)):
        row = []
        for n in cases:
            w, where = stats_fs(CASES[n], R, xs, xs_, zs)
            row.append(f'{n} {w:5.2f}')
        print(f'sensor face {zs:.1f} cm above the floor at x={xs_}: min clearance  ' + '  '.join(row), flush=True)
