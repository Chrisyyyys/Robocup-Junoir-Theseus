"""Dropper plate check with a precomputed exposure table. State = (used from A end, used from B end); full pockets are the contiguous run between them."""
import math, sys
from platesim import Plate, sq, exposure

def table(pl, step=0.5):
    """exposure[k][slot] as a dict theta->(eA,eB) for theta in [-300,300]"""
    th_vals = [i*step for i in range(int(-300/step), int(300/step)+1)]
    tab = {}
    for k in range(pl.n):
        tab[k] = {th: (exposure(pl, k, th, pl.SA), exposure(pl, k, th, pl.SB)) for th in th_vals}
    return tab, th_vals

def check(pl, step=0.5, limit=0.0):
    tab, th_vals = table(pl, step)
    viol = []
    cases = 0
    n = pl.n
    for usedA in range(n+1):
        for usedB in range(n+1-usedA):
            full = list(range(usedA, n-usedB))
            for side in 'LR':
                for N in (1, 2):
                    if len(full) < N: continue
                    if side == 'L':
                        kidx = full[:N]; target = -(pl.phi[kidx[-1]] - pl.slotA); sgn = -1
                    else:
                        kidx = full[::-1][:N]; target = (pl.slotB - pl.phi[kidx[-1]]); sgn = +1
                    cases += 1
                    tmax = abs(target)
                    ths = [sgn*t for t in [i*step for i in range(int(tmax/step)+1)]]
                    for th in ths:
                        thk = round(th/step)*step
                        for k in full:
                            ea, eb = tab[k][thk]
                            right, wrong = (ea, eb) if side == 'L' else (eb, ea)
                            if wrong > limit:
                                viol.append(f'used A{usedA}/B{usedB} {side}{N}: full pocket {k} over WRONG slot at theta={th:.1f} (exposure {wrong:.2f})'); break
                            if right > limit and k not in kidx:
                                viol.append(f'used A{usedA}/B{usedB} {side}{N}: full pocket {k} over the right slot but not meant to drop, theta={th:.1f} (exposure {right:.2f})'); break
                    # every target pocket must be (almost) fully over the slot at some point of the sweep (the first one drops on the fly)
                    for k in kidx:
                        best = max(tab[k][round(th/step)*step][0 if side == 'L' else 1] for th in ths)
                        if best < 0.9: viol.append(f'used A{usedA}/B{usedB} {side}{N}: pocket {k} only reaches {best:.2f} over the slot')
                    # parked after: nothing full over a slot
                    rest = [k for k in full if k not in kidx]
                    for k in rest:
                        ea, eb = tab[k][0.0]
                        if ea > 0 or eb > 0: viol.append(f'parked with full pocket {k} over a slot')
    park = min(min(pl.pocket_poly(k, 0).distance(pl.SA), pl.pocket_poly(k, 0).distance(pl.SB)) for k in range(pl.n))
    wall = min(pl.pocket_poly(i, 0).distance(pl.pocket_poly(i+1, 0)) for i in range(pl.n-1))
    return dict(cases=cases, violations=len(viol), park_margin_mm=round(park, 2), wall_mm=round(wall, 2)), viol

if __name__ == '__main__':
    print('--- the idea in the spec before: slots 180 deg apart, 8 pockets 45 deg apart, half-pitch offset ---')
    old = Plate(r=33.0, pocket=14.0, slot=14.0, pitch=45.0, phi1=22.5, n=8, slotA=0.0, slotB=180.0)
    # exposures at parked / half-step positions
    tabo, ths = table(old, 0.5)
    for th in (0.0, -22.5, -45.0, 22.5):
        eA = max(tabo[k][th][0] for k in range(8)); eB = max(tabo[k][th][1] for k in range(8))
        print(f'  theta={th:>6}: biggest kit exposure over slot A {eA:.2f}, over slot B {eB:.2f}')
    print('--- V3 plate candidates (all kits full at the start, any mix of 1- and 2-kit drops left/right) ---')
    for label, kw in (('pocket 13.0 / slot 13.5 / r 36.2 (spec candidate)', dict()),
                      ('pocket 13.0 / slot 13.5 / r 34.0', dict(r=34.0)),
                      ('pocket 13.0 / slot 13.5 / r 40.0', dict(r=40.0)),
                      ('pocket 14.0 / slot 14.5 / r 38.6', dict(r=38.6, pocket=14.0, slot=14.5)),
                      ('pocket 12.5 / slot 13.0 / r 35.0', dict(r=35.0, pocket=12.5, slot=13.0))):
        pl = Plate(**kw)
        res, viol = check(pl)
        print(label, '->', res)
        for v in viol[:4]: print('      ', v)
