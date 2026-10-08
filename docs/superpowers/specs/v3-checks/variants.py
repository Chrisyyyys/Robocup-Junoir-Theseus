"""Plate variants: which ring radius does each pocket size need? (v1 approved draft: 14 mm pockets on a 33 mm ring, plate about 89 mm)"""
import sys
from platesim import Plate
from platesim2 import check
rows = []
for pocket, slot in ((14.0, 14.0), (14.0, 14.5), (13.0, 13.5)):
    for r in (33.0, 34.0, 35.0, 36.2, 37.0, 38.0, 38.6):
        pl = Plate(r=r, pocket=pocket, slot=slot)
        res, viol = check(pl)
        import math
        R_plate = math.hypot(r + pocket/2, pocket/2) + 3.0
        print(f'pocket {pocket:4.1f} slot {slot:4.1f} ring r {r:5.1f}: {res}  plate radius {R_plate:5.1f} mm (dia {2*R_plate/10:4.1f} cm)' + (f'  first violation: {viol[0]}' if viol else ''), flush=True)
