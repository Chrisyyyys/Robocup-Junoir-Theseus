"""Floor-sensor face clearance on the terrain cases for the rev 4 body (the 9 Oct omni layout: rest axle 6.85, pivot (2.409, 4.2), arm 4.6 cm; 8 Oct: 7.0 and (2.55, 3.6); chute lip z 2.67) with the sensor window x_s ahead of the axle (mechanical design section 5.3).
Same machinery as fsens.py (rev 3: front port at 6.5, silver module at the axle); the window half width is a parameter here. Prints the smallest clearance (cm) between the window and the
terrain over the whole crossing; negative = the sensor would hit. Rows: x_s, face z, half width. A case passes with at least 0.2 cm. Usage: python fsens_front.py  (about 2 minutes)"""
import fsens
import nubtable as N
import omni_mount as M
from omni_terrain9 import mk as mk9
from sidesim2 import terrain, wheel_centre_z, pitch_for_omni, rear_ok

R = mk9(M.NEW)
XS = [x / 1.0 for x in range(-25, 50)]
CASES = ['riser_up', 'riser_down', 'ramp_up', 'ramp_down', 'bump2', 'bump1', 'steps2x1_up', 'steps2x1_down']
EXTRA = ['stairs3x2_25_up', 'stairs3x2_30_up', 'dz_bump2_on_ramp_up']


def smallest(name, x_s, z_s, half):
    T = (N.CASES if name in N.CASES else N.EXTRA)[name]
    Ter = terrain(T)
    worst = 9e9
    for xd in XS:
        zd = wheel_centre_z(Ter, xd, R.wheel_r)
        for i in range(11):
            c = R.travel * i / 10
            ph = pitch_for_omni(Ter, R, xd, zd, c)
            if ph is None or not rear_ok(Ter, R, xd, zd, ph):
                continue
            worst = min(worst, fsens.fsens_clear(Ter, xd, zd, ph, x_s, z_s, half=half))
            break
    return worst


if __name__ == '__main__':
    print('smallest clearance (cm) between the sensor window and the terrain over the whole crossing; negative = the sensor would hit', flush=True)
    print('%-26s' % 'x_s, face z, half width', ' '.join('%12s' % c[:12] for c in CASES + EXTRA), flush=True)
    for x_s, z_s, half in ((0.0, 2.8, 1.0), (6.5, 2.8, 0.8), (7.5, 2.8, 1.0), (8.0, 2.8, 1.0), (8.5, 2.8, 1.0), (7.5, 3.3, 1.0), (8.0, 3.3, 1.0)):
        print('x %.1f z %.1f half %.1f   ' % (x_s, z_s, half) + ' '.join('%12.2f' % smallest(n, x_s, z_s, half) for n in CASES + EXTRA), flush=True)
