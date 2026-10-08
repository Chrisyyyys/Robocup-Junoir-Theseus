"""Re-derive every side-view number quoted in the V3 spec. Usage: python nubtable.py [which]
which = final | nub | old | extra | all   (results are printed compactly and also appended to nub_results.txt)"""
import math, sys, json
from sidesim2 import *
import sidesim2 as S2

def mk(nub=2.0, belly=3.5, chamfer=True, travel=2.5, skid=True, bumper_bottom=4.0):
    return Robot(belly=belly, bumper_bottom=bumper_bottom, bumper_x=(5.8, 10.8), tip_x=11.0, lip=(-7.0, 3.0),
                 skid=(-8.0, nub, 0.4) if skid else None, rear_corner=(-10.5, 5.3),
                 rear_chamfer=(-6.5, 5.3) if chamfer else None, pivot=(3.5, 3.6),
                 omni_rest=(7.95, 3.0), travel=travel)

def stats(profile, R, xs):
    """pessimistic (stiff spring) pose per drive-wheel position: max body pitch, omni compression needed, worst clearances"""
    T = terrain(profile)
    maxp = -99; minp = 99; cneed = 0.0; inf = []
    worst = {'belly': 9e9, 'bumper': 9e9, 'exit lip': 9e9}
    for xd in xs:
        zd = wheel_centre_z(T, xd, R.wheel_r)
        feas = []
        for i in range(11):
            c = R.travel * i / 10
            ph = pitch_for_omni(T, R, xd, zd, c)
            if ph is None: continue
            if not rear_ok(T, R, xd, zd, ph): continue
            feas.append((c, ph))
        if not feas:
            inf.append(xd); continue
        c0, ph0 = feas[0]
        cl = clearances(T, R, xd, zd, ph0)
        for k in worst: worst[k] = min(worst[k], cl[k])
        maxp = max(maxp, math.degrees(ph0)); minp = min(minp, math.degrees(ph0)); cneed = max(cneed, c0)
    return dict(ok=(not inf) and min(worst.values()) >= 0.2, infeasible=[round(x, 1) for x in inf[:4]], n_inf=len(inf),
                pitch_max=round(maxp, 1), pitch_min=round(minp, 1), cneed=round(cneed, 2),
                belly=round(worst['belly'], 2), bumper=round(worst['bumper'], 2), lip=round(worst['exit lip'], 2))

def ramp_with_bump(angle=25.0, s0=12.0, w=3.0, h=2.0, length=30.0, top=40.0):
    a = math.radians(angle); ca, sa = math.cos(a), math.sin(a); nx, ny = -sa, ca
    P = lambda s, hh: (s * ca + hh * nx, s * sa + hh * ny)
    pts = [(-60, 0), (0, 0), P(s0, 0), P(s0, h), P(s0 + w, h), P(s0 + w, 0), P(length, 0)]
    pts.append((pts[-1][0] + top, pts[-1][1]))
    return pts

def mirror2(pts, x0=-60.0, x1=90.0):
    """descending version of an ascending profile; the high level must run back to x0 (sidesim2.mirror_profile starts at x=0)"""
    xm = max(p[0] for p in pts)
    q = [(xm - p[0], p[1]) for p in pts][::-1]
    q = [(x0, q[0][1])] + q
    return [(min(x, x1), z) for x, z in q]

XS = [x / 2 for x in range(-50, 100)]
CASES = dict(riser_up=prof_step_up(2.0), riser_down=prof_step_down(2.0),
             steps2x1_up=stairs_steps(1.0, 2.5, 2), steps2x1_down=mirror2(stairs_steps(1.0, 2.5, 2)),
             ramp_up=prof_ramp(25.0), ramp_down=prof_ramp_down(25.0),
             bump2=prof_bump(2.0, 3.0), bump1=prof_bump(1.0, 3.0), seam=prof_seam(0.3))
EXTRA = dict(stairs3x2_25_up=stairs_steps(2.0, 2.0 / math.tan(math.radians(25)), 3),
             stairs3x2_25_down=mirror2(stairs_steps(2.0, 2.0 / math.tan(math.radians(25)), 3)),
             stairs3x2_30_up=stairs_steps(2.0, 2.0 / math.tan(math.radians(30)), 3),
             stairs3x2_30_down=mirror2(stairs_steps(2.0, 2.0 / math.tan(math.radians(30)), 3)),
             dz_bump2_on_ramp_up=ramp_with_bump(25, 12, 3, 2.0), dz_bump1_on_ramp_up=ramp_with_bump(25, 12, 3, 1.0))

def show(tag, R, names, table):
    for n in names:
        s = stats(table[n], R, XS)
        line = f'{tag:<34}{n:<22}{"PASS" if s["ok"] else "FAIL"}  pitch {s["pitch_min"]:5.1f}..{s["pitch_max"]:5.1f}  omni {s["cneed"]:4.2f}  belly {s["belly"]:5.2f} bumper {s["bumper"]:5.2f} lip {s["lip"]:5.2f}' + (f'  infeasible at xd {s["infeasible"]} (n={s["n_inf"]})' if s['n_inf'] else '')
        print(line, flush=True)

if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'final'
    if which in ('final', 'all'):
        show('FINAL nub2.0 belly3.5 chamfer', mk(), list(CASES), CASES)
    if which in ('nub', 'all'):
        for nub in (0.5, 1.0, 1.5, 2.0, 2.5):
            show(f'nub {nub}', mk(nub=nub), ['riser_up', 'riser_down', 'ramp_up', 'ramp_down'], CASES)
    if which in ('old', 'all'):
        show('first draft: belly2.5 skid0.5 nochamf', mk(nub=0.5, belly=2.5, chamfer=False), ['riser_up', 'ramp_up', 'ramp_down'], CASES)
        show('belly3.5 only (skid0.5 nochamf)', mk(nub=0.5, belly=3.5, chamfer=False), ['riser_up', 'ramp_up', 'ramp_down'], CASES)
        show('belly2.5 nub2.0 + chamfer', mk(nub=2.0, belly=2.5, chamfer=True), ['riser_up', 'ramp_up', 'ramp_down'], CASES)
        show('belly3.5 nub2.0 NO chamfer', mk(nub=2.0, belly=3.5, chamfer=False), ['riser_up', 'ramp_up', 'ramp_down'], CASES)
    if which == 'fixdown':
        show('FINAL fixed mirror', mk(), ['steps2x1_down'], CASES)
        show('FINAL fixed mirror', mk(), ['stairs3x2_25_down', 'stairs3x2_30_down'], EXTRA)
    if which in ('extra', 'all'):
        show('FINAL (extra cases)', mk(), list(EXTRA), EXTRA)
