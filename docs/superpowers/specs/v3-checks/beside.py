"""Camera beside the wheel (board inside the wall, victim band height) at R=10.5: where can the OpenMV board go if the roof is low?"""
import math
from final_check import *
import place
R = 10.5
for roof in (9.4, 10.0, 11.9):
    P['R_body'] = R; P['R_int'] = R - P['wall']; P['R_sw'] = R + 0.5; P['omni_x'] = P['R_sw'] - P['omni_r'] - 0.05
    P['exit_x'] = -(R - 3.5); P['x_p'] = -2.0; P['z_lid'] = roof; P['z_pf'] = 8.0 if roof < 10 else 9.0
    print(f'--- roof {roof} (plate floor {P["z_pf"]})')
    F, bands, cams = place.main(R, P['z_pf'], verbose=False)
    by = {}
    for psi, o, zc, d in cams: by.setdefault((o, zc), []).append(psi)
    print('   feasible placements:', len(cams), {k: (min(v), max(v)) for k, v in by.items()})
