import math, random, sys
import numpy as np
import chute_geometry as G, v3_params4 as P, head, chute_dynamics as D
D.geometry = lambda: head.pieces(W=1.5, H=1.5, trough='floor')
D._geo = None
mu = 0.5
sim = D.Sim(mu=mu)
rnd = random.Random(5)
shown = 0
for k in range(40):
    yaw, dx, dy = D.sample_pocket(rnd)
    dx += rnd.uniform(-0.25, 0.25)
    r = D.drop_trial(sim, mu, yaw, dx, dy)
    if r['outcome'] != 'exit':
        d = sim.data
        w, x, y, z = d.qpos[3:7]
        roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))); pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
        p = sim.pos_cm()
        sx, sy = D.slot_frame()
        c, s = math.cos(math.radians(45)), math.sin(math.radians(45))
        ox, oy = p[0]-sx, p[1]-sy
        sf = (ox*c+oy*s, -ox*s+oy*c)
        print('turn %+5.1f offset (%+.2f %+.2f) -> %s t=%.2f; end pos in slot frame (%.2f, %.2f, z %.2f); roll %.0f pitch %.0f; speed %.3f' % (yaw, dx, dy, r['outcome'], r.get('t') or 0, sf[0], sf[1], p[2], roll, pitch, np.linalg.norm(sim.vel())))
        shown += 1
        if shown > 8: break
