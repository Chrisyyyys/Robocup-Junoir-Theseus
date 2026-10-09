"""Scratch: run the chute simulation on a parametric head design.
Usage: python design_run.py "W=1.5,H=1.5,trough=floor[,damp=0.3][,roll=5][,pitch=0][,par=0.2]" [n] [yawmax] [mus]
roll: robot tilted so the chute side (+y) is lower by this many degrees; pitch: nose up; par: plate parking error, a random tangential shift of the pocket in cm."""
import math
import random
import sys

import numpy as np

import chute_geometry as G
import v3_params4 as P
import head
import chute_dynamics as D

spec = sys.argv[1] if len(sys.argv) > 1 else ''
n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
yawmax = float(sys.argv[3]) if len(sys.argv) > 3 else 29.0
mus = tuple(float(v) for v in sys.argv[4].split(',')) if len(sys.argv) > 4 else (0.2, 0.3, 0.4, 0.5)
kw = {}
for item in [s for s in spec.split(',') if s]:
    k, v = item.split('=')
    try:
        kw[k] = float(v)
    except ValueError:
        kw[k] = v
damp = kw.pop('damp', 1.0)
roll = kw.pop('roll', 0.0)
pitch = kw.pop('pitch', 0.0)
par = kw.pop('par', 0.0)
slot = kw.pop('slot', None)
if slot:
    P.PLATE['slot'] = slot
if kw.get('slot_ang') is not None and abs(kw['slot_ang'] - 45.0) > 1e-6:
    P.PLATE['slotB_robot_deg'] = kw['slot_ang'] - 45.0 + 135.0
D.geometry = lambda: head.pieces(**kw)
G.SLOT_ANG = kw.get('slot_ang', 45.0)
D._geo = None
r_, p_ = math.radians(roll), math.radians(pitch)
gvec = tuple(D.GRAV * v for v in (-math.sin(p_), math.cos(p_) * math.sin(r_), -math.cos(p_) * math.cos(r_)))
print('design', kw, '| damp %g roll %g pitch %g par %g | kits per friction: %d | kit turn up to +-%g deg' % (damp, roll, pitch, par, n, yawmax))
tot_out = tot = 0
for mu in mus:
    sim = D.Sim(mu=mu, damp=damp, gvec=gvec)
    rnd = random.Random(5)
    res = {'exit': 0, 'stuck': 0, 'lost': 0, 'timeout': 0}
    by = {'|turn|<10': [0, 0], '10-20': [0, 0], '20+': [0, 0]}
    for k in range(n):
        yaw, dx, dy = D.sample_pocket(rnd, max_yaw=yawmax)
        dx += rnd.uniform(-par, par)
        r = D.drop_trial(sim, mu, yaw, dx, dy)
        res[r['outcome']] += 1
        b = '|turn|<10' if abs(yaw) < 10 else ('10-20' if abs(yaw) < 20 else '20+')
        by[b][1] += 1
        by[b][0] += r['outcome'] == 'exit'
    tot_out += res['exit']
    tot += n
    print('  mu %.2f: out %3d  stuck %3d  lost %3d  timeout %3d   by turn: %s' % (mu, res['exit'], res['stuck'], res['lost'], res['timeout'], '  '.join('%s %d/%d' % (k, v[0], v[1]) for k, v in by.items())))
print('  total out %d of %d' % (tot_out, tot))
