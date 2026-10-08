"""Plate tolerance for the 14 mm pocket variant: python platetol2.py [ring] [pocket] [slot]"""
import sys
from platesim import Plate, exposure
from platesim2 import check
r = float(sys.argv[1]) if len(sys.argv) > 1 else 38.6
pk = float(sys.argv[2]) if len(sys.argv) > 2 else 14.0
sl = float(sys.argv[3]) if len(sys.argv) > 3 else 14.5
pl = Plate(r=r, pocket=pk, slot=sl)
res, viol = check(pl)
print(f'plate ring r={r} pocket {pk} slot {sl}:', res, 'violations:', viol[:3])
print('target pocket 0 over slot A, plate error in degrees -> share of the kit footprint over the opening:')
print('   ' + '  '.join(f'{err}deg {exposure(pl, 0, -(pl.phi[0]) + err, pl.SA):.2f}' for err in (0, 1, 2, 3, 4, 5, 6, 8)))
print('parked, plate error in degrees -> pocket/slot overlap area (mm2), kit exposure:')
for err in (0, 2, 4, 5, 6, 8, 10):
    pA = pl.pocket_poly(0, -err).intersection(pl.SA).area; eA = exposure(pl, 0, -err, pl.SA)
    print(f'   err {err:>2} deg ({pl.r*err*3.14159/180:.1f} mm at the ring): overlap {pA:.1f} mm2, kit over slot {eA:.2f}')
