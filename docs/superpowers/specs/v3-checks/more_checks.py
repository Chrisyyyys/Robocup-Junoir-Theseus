"""(1) omni travel 3.0 (approved draft) vs 2.5, (2) silver module near the axle: clearance on risers and ramps."""
import math, sys
from nubtable import *
from fsens import stats_fs

if len(sys.argv) > 1 and sys.argv[1] == 'travel':
    for tr in (2.5, 3.0):
        R = mk(travel=tr)
        print(f'--- omni travel {tr} cm', flush=True)
        show(f'travel {tr}', R, ['riser_up', 'ramp_up', 'ramp_down'], CASES)
        show(f'travel {tr}', R, ['stairs3x2_25_up', 'stairs3x2_30_up', 'dz_bump2_on_ramp_up', 'dz_bump1_on_ramp_up'], EXTRA)
else:
    R = mk()
    xs = [x / 1.0 for x in range(-25, 50)]
    cases = ['riser_up', 'riser_down', 'ramp_up', 'ramp_down', 'bump2', 'steps2x1_up']
    for xs_, zs in ((0.0, 2.8), (1.5, 2.8), (0.0, 2.5)):
        row = []
        for n in cases:
            w, where = stats_fs(CASES[n], R, xs, xs_, zs, half=1.0) if 'half' in stats_fs.__code__.co_varnames else stats_fs(CASES[n], R, xs, xs_, zs)
            row.append(f'{n} {w:5.2f}')
        print(f'silver module face {zs} cm above the floor at x={xs_} (2 cm long): min clearance  ' + '  '.join(row), flush=True)
