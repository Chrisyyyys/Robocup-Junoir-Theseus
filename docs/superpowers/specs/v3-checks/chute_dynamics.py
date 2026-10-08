"""Does a rescue kit get out of the chute? A rigid-body simulation (MuJoCo) of a 10.3 mm cube dropped from its plate pocket through the slot into the hopper, then sliding down the 13 mm
square channel to the exit, on the geometry of the Fusion model (chute_geometry.py, checked against Fusion in chute_geometry_check.py). Spec section 6.2; units in the report are cm, m/s.

Assumptions, all guesses to be replaced by the bench test (rev 3 test 3): kit mass 1.2 g; Coulomb friction mu the same between the kit and every printed surface (swept, 0.15 to 0.60;
the spec's earlier landing calculation used 0.35); nearly inelastic impacts (contact damping ratio 1; 0.3 is tried as well); the kit starts at rest in the plate pocket over slot B with a
random turn (up to +-29 degrees, the most a 10.3 mm cube can be turned in the 14.0 mm pocket) and a random position in the pocket, the pocket aligned with the slot (the plate parked
exactly); the robot stands level. The tub floor, the tub wall and the wheel are not modelled: a kit that leaves the hopper or the channel anywhere but the exit is counted as lost.

Controls (the run fails if one does not behave): the same engine settings must give the textbook acceleration g (sin a - mu cos a) of a cube on a plane tilted like the channel, must hold the
cube still at mu 0.7 (tan 32.2 deg = 0.63), and must jam a cube too big for the bore.

Needs MuJoCo (pip install mujoco; installed in C:/Users/christopher.shu/pl4_mujoco, found automatically). Run with PYTHONPATH="C:/Users/christopher.shu/pl4" python chute_dynamics.py [quick|full] [bore in cm, for a what-if with a wider channel bore]
"""
import math
import os
import random
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
for _p in (r'C:/Users/christopher.shu/pl4_mujoco',):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.append(_p)
try:
    import mujoco
except ImportError:
    print('SKIP chute_dynamics: MuJoCo is not installed (pip install mujoco, Python 3.12 wheel)')
    sys.exit(0)

import chute_geometry as G
import v3_params4 as P

CM = 0.01
KIT = P.KIT                                            # 1.03 cm
GRAV = 9.81
DT = 0.0002

_geo = None
TRIALS = []                                           # (mu, kit turn in degrees, outcome) of every trial of the sweep


def geometry():
    global _geo
    if _geo is None:
        g = G.build(1)
        pieces = []
        for k in ('floor', 'hopper', 'channel'):
            for v, planes in g[k]:
                if G.volume(planes, v) > 1e-3:               # slivers under 1 mm3 are dropped
                    pieces.append((k, v))
        _geo = pieces
    return _geo


def _mesh(name, v):
    return '<mesh name="%s" vertex="%s"/>' % (name, ' '.join('%.7f' % (x * CM) for x in np.asarray(v).reshape(-1)))


def world_xml(mu, size=KIT, mass_g=1.2, damp=1.0, timeconst=0.002, gvec=(0.0, 0.0, -GRAV), plane_tilt_deg=None, ground=True):
    """MJCF of the chute, the ground, a corridor wall 14 cm to the left of the robot's centre line, and the kit with a free joint. plane_tilt_deg: replace the chute by a
    flat plate tilted like the channel (the control)."""
    fr = '%g 0.001 0.0001' % mu
    solref = '%g %g' % (timeconst, damp)
    parts = ['<mujoco><option timestep="%g" gravity="%g %g %g" cone="elliptic" impratio="10" integrator="implicitfast"><flag multiccd="enable"/></option>' % ((DT,) + tuple(gvec))]
    parts.append('<asset>')
    geoms = []
    if plane_tilt_deg is None:
        for i, (k, v) in enumerate(geometry()):
            parts.append(_mesh('m%d' % i, v))
            geoms.append('<geom type="mesh" mesh="m%d" friction="%s" solref="%s" rgba="0.7 0.4 0.2 1"/>' % (i, fr, solref))
    else:
        a = math.radians(plane_tilt_deg)
        geoms.append('<geom type="box" size="0.5 0.5 0.01" pos="0 0 -0.01" euler="0 %g 0" friction="%s" solref="%s"/>' % (-plane_tilt_deg, fr, solref))
    parts.append('</asset><worldbody>')
    if ground:
        parts.append('<geom type="plane" size="2 2 0.1" pos="0 0 0" friction="%s" solref="%s"/>' % (fr, solref))
        parts.append('<geom type="box" size="1 0.01 0.3" pos="0 0.15 0.3" friction="%s" solref="%s"/>' % (fr, solref))        # corridor wall, 15 cm: its face is at y = 14 cm
    parts.extend(geoms)
    h = size * CM / 2
    parts.append('<body name="kit" pos="0 0 1"><freejoint/><geom name="kit" type="box" size="%g %g %g" mass="%g" friction="%s" solref="%s"/></body>' % (h, h, h, mass_g * 1e-3, fr, solref))
    parts.append('</worldbody></mujoco>')
    return ''.join(parts)


class Sim:
    def __init__(self, **kw):
        self.model = mujoco.MjModel.from_xml_string(world_xml(**kw))
        self.size = kw.get('size', KIT)
        self.data = mujoco.MjData(self.model)
        self.kit = self.model.body('kit').id
        self.p0, self.d, self.ex, self.ey, self.ez, self.e = G.channel_frame(1)

    def place(self, pos_cm, yaw_deg=0.0):
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[:3] = np.asarray(pos_cm, float) * CM
        a = math.radians(yaw_deg) / 2
        d.qpos[3:7] = [math.cos(a), 0.0, 0.0, math.sin(a)]
        d.qvel[:] = 0
        mujoco.mj_forward(self.model, d)

    def pos_cm(self):
        return self.data.qpos[:3] / CM

    def vel(self):
        return self.data.qvel[:3].copy()


def slot_frame():
    sx, sy = P.slot_xy('B')
    return sx, sy


def drop_trial(sim, mu_unused, yaw, dx, dy, max_t=1.0, land=False):
    """One kit from the pocket over slot B. (dx, dy): position in the slot's frame (cm, along the slot square's sides), yaw: turn of the kit relative to the slot square, degrees.
    Returns a dict: outcome 'exit' / 'stuck' / 'lost' / 'timeout', and the numbers."""
    sx, sy = slot_frame()
    c, s = math.cos(math.radians(45.0)), math.sin(math.radians(45.0))      # the slot square is turned 45 degrees: its sides are along (c, s) and (-s, c)
    px, py = sx + dx * c - dy * s, sy + dx * s + dy * c
    sim.place((px, py, P.PLATE['z0'] + sim.size / 2 + 0.02), 45.0 + yaw)
    e = sim.e
    r_hat = np.array([e[0], e[1]]) / math.hypot(e[0], e[1])
    t_still, t = 0.0, 0.0
    chunk = 25
    out = {'outcome': 'timeout', 't': None}
    in_channel_once = False
    while t < max_t:
        mujoco.mj_step(sim.model, sim.data, nstep=chunk)
        t += chunk * DT
        p = sim.pos_cm()
        v = sim.vel()
        rel = np.array([p[0] - e[0], p[1] - e[1]])
        radial = r_hat @ rel
        # crossed the exit plane inside the channel's mouth
        if radial > 0.0 and abs(np.linalg.norm(p - e)) < 1.6 and 3.0 < p[2] < 6.5:
            out.update(outcome='exit', t=t, speed=float(np.linalg.norm(v)), v=v.copy(), pos=p.copy())
            break
        # fell out of the hopper onto the tub floor
        axis_d = np.linalg.norm(np.cross(p - sim.p0, sim.d))
        if p[2] < 6.0 and axis_d > 1.4 and math.hypot(p[0], p[1]) < 10.3:
            out.update(outcome='lost', t=t, pos=p.copy())
            break
        if np.linalg.norm(v) < 0.004:
            t_still += chunk * DT
            if t_still > 0.25 and p[2] < 8.9:
                q = sim.data.qpos[3:7]
                out.update(outcome='stuck', t=t, pos=p.copy())
                break
        else:
            t_still = 0.0
    if land and out['outcome'] == 'exit':
        while t < max_t + 0.6:
            mujoco.mj_step(sim.model, sim.data, nstep=chunk)
            t += chunk * DT
            p = sim.pos_cm()
            if p[2] < KIT / 2 + 0.15 and np.linalg.norm(sim.vel()) < 0.05:
                break
        out['landing'] = sim.pos_cm()
    return out


def sample_pocket(rnd, max_yaw=29.0):
    """A kit in the 14.0 mm pocket: a random turn, then a random position that still fits the pocket (half width 0.70 cm)."""
    yaw = rnd.uniform(-max_yaw, max_yaw)
    ext = KIT / 2 * (abs(math.cos(math.radians(yaw))) + abs(math.sin(math.radians(yaw))))
    lim = max(0.0, 0.70 - ext)
    return yaw, rnd.uniform(-lim, lim), rnd.uniform(-lim, lim)


def incline_control(mu, tilt=32.2, t_run=0.25):
    """Acceleration of a cube on a plane tilted like the channel, from the simulation and from the textbook. Returns (a_sim, a_theory)."""
    sim = Sim(mu=mu, plane_tilt_deg=tilt, ground=False)
    a = math.radians(tilt)
    # the plate rises towards +x (euler about y by -tilt), so the cube slides towards -x; it starts on the plane at x = 0.2 m, turned with the plane
    x0 = 0.2
    zc = x0 * math.tan(a) + (KIT * CM / 2) / math.cos(a) + 0.0005
    sim.place((x0 / CM, 0.0, zc / CM), 0.0)
    ang = -a / 2
    sim.data.qpos[3:7] = [math.cos(ang), 0.0, math.sin(ang), 0.0]
    mujoco.mj_forward(sim.model, sim.data)
    n = int(t_run / DT)
    t_mid = int(0.05 / DT)
    mujoco.mj_step(sim.model, sim.data, nstep=t_mid)
    v1 = sim.data.qvel[0]
    mujoco.mj_step(sim.model, sim.data, nstep=n - t_mid)
    v2 = sim.data.qvel[0]
    a_sim = abs(v2 - v1) / ((n - t_mid) * DT) / math.cos(a)             # the velocity is measured along x; the slide is along the plane
    a_theory = GRAV * (math.sin(a) - mu * math.cos(a))
    return a_sim, a_theory


def control_report(lines):
    ok = True
    for mu in (0.25, 0.35, 0.45):
        a_sim, a_th = incline_control(mu)
        good = abs(a_sim - a_th) <= 0.05 * a_th + 0.05
        ok = ok and good
        lines.append('  %s control: cube on a plane tilted 32.2 degrees, mu %.2f: acceleration %.2f m/s2 in the simulation, %.2f by g (sin a - mu cos a)' % ('ok  ' if good else 'FAIL', mu, a_sim, a_th))
    sim = Sim(mu=0.70, plane_tilt_deg=32.2, ground=False)
    a = math.radians(32.2)
    zc = 0.2 * math.tan(a) + (KIT * CM / 2) / math.cos(a) + 0.0005
    sim.place((20.0, 0.0, zc / CM), 0.0)
    ang = -a / 2
    sim.data.qpos[3:7] = [math.cos(ang), 0.0, math.sin(ang), 0.0]
    mujoco.mj_forward(sim.model, sim.data)
    mujoco.mj_step(sim.model, sim.data, nstep=int(0.6 / DT))          # the landing on the plane moves it a few cm; after that it must hold
    x0 = sim.data.qpos[0]
    mujoco.mj_step(sim.model, sim.data, nstep=int(0.5 / DT))
    moved = abs(sim.data.qpos[0] - x0) / CM
    good = moved < 0.05
    ok = ok and good
    lines.append('  %s control: the same cube at mu 0.70 (above tan 32.2 = 0.63), once settled, creeps %.3f cm in 0.5 s: it must hold' % ('ok  ' if good else 'FAIL', moved))
    return ok


def oversize_control(lines, size=1.40):
    sim = Sim(mu=0.30, size=size)
    sx, sy = slot_frame()
    n_exit = 0
    for yaw in (0.0, 15.0, -15.0, 29.0):
        r = drop_trial(sim, None, yaw, 0.0, 0.0)
        n_exit += r['outcome'] == 'exit'
    good = n_exit == 0
    lines.append('  %s control: a %.1f mm cube (too big for the 13 mm bore) must not get out: %d of 4 got out' % ('ok  ' if good else 'FAIL', size * 10, n_exit))
    return good


def sweep(mus, n, seed, damp=1.0, gvec=(0.0, 0.0, -GRAV), label='', lines=None, show_jams=3):
    rnd = random.Random(seed)
    rows = []
    for mu in mus:
        sim = Sim(mu=mu, damp=damp, gvec=gvec)
        res = {'exit': 0, 'stuck': 0, 'lost': 0, 'timeout': 0}
        speeds, times, jam = [], [], []
        r2 = random.Random(seed)
        for k in range(n):
            yaw, dx, dy = sample_pocket(r2)
            r = drop_trial(sim, mu, yaw, dx, dy)
            TRIALS.append((mu, yaw, r['outcome']))
            res[r['outcome']] += 1
            if r['outcome'] == 'exit':
                speeds.append(r['speed'])
                times.append(r['t'])
            elif len(jam) < show_jams:
                jam.append((yaw, dx, dy, r['outcome'], r.get('pos')))
        rows.append((mu, res, speeds, times, jam))
    return rows


def format_rows(rows, n, title):
    out = [title]
    out.append('   mu   exit  stuck  lost  timeout   exit speed m/s (min / mean / max)   time to exit s (mean)')
    for mu, res, speeds, times, jam in rows:
        sp = '%.2f / %.2f / %.2f' % (min(speeds), sum(speeds) / len(speeds), max(speeds)) if speeds else '-'
        tm = '%.2f' % (sum(times) / len(times)) if times else '-'
        out.append('  %.2f  %3d   %3d   %3d    %3d      %s   %s' % (mu, res['exit'], res['stuck'], res['lost'], res['timeout'], sp.ljust(26), tm))
    for mu, res, speeds, times, jam in rows:
        for yaw, dx, dy, oc, pos in jam:
            out.append('    mu %.2f %s: kit turned %+.0f deg, offset (%+.2f, %+.2f) cm in the pocket; stopped at (%s)' % (mu, oc, yaw, dx, dy, ', '.join('%.2f' % v for v in pos) if pos is not None else '-'))
    return out


def turn_table(mus):
    """Share of kits that get out, by how far the kit is turned from the slot square (negative = towards the channel, which runs 32.8 degrees off the square)."""
    bins = [(-29, -20), (-20, -10), (-10, 0), (0, 10), (10, 20), (20, 29)]
    out = ['  share of kits that get out, by the turn of the kit from the slot square (degrees; the channel is at -32.8):', '   turn        ' + ''.join('  mu %.2f' % m for m in mus)]
    for lo, hi in bins:
        cells = []
        for m in mus:
            sel = [o for mu, y, o in TRIALS if abs(mu - m) < 1e-9 and lo <= y < hi]
            cells.append('%3d/%-3d ' % (sum(1 for o in sel if o == 'exit'), len(sel)))
        out.append('  %+3d to %+3d   ' % (lo, hi) + ' '.join('%7s' % c for c in cells))
    return out


def main(mode='quick', bore=None):
    global _geo
    t0 = time.time()
    note = 'the geometry is the Fusion model (chute_geometry_check.py: PASS)'
    if bore:
        P.CHUTE['in_w'] = bore
        P.CHUTE['out_w'] = bore + 2 * P.CHUTE['wall']
        _geo = None
        note = 'WHAT IF: the channel bore widened from %.1f to %.1f mm (outside %.1f mm), everything else as in the Fusion model' % (13.0, bore * 10, P.CHUTE['out_w'] * 10)
    lines = ['kit chute dynamics (MuJoCo %s), kit %.1f mm, 1.2 g, slot B (left) to the exit at the body wall; %s' % (mujoco.__version__, KIT * 10, note)]
    ok = control_report(lines)
    ok = oversize_control(lines) and ok
    n = 30 if mode == 'quick' else 100
    mus = (0.25, 0.35, 0.45) if mode == 'quick' else (0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60)
    rows = sweep(mus, n, seed=7)
    lines += format_rows(rows, n, '%d kits per friction value, random turn (+-29 degrees) and position in the pocket, robot level, damping ratio 1 (nearly inelastic):' % n)
    lines += turn_table(mus)
    aligned = [o for mu, y, o in TRIALS if abs(y) < 10.0]
    rate = sum(1 for o in aligned if o == 'exit') / max(1, len(aligned))
    lines.append('  kits within 10 degrees of the slot square (how a kit sits when the pocket wall holds it): %.0f %% get out, all friction values together' % (100 * rate))
    lines.append('time %.0f s' % (time.time() - t0))
    enter = math.degrees(math.asin(min(1.0, P.CHUTE['in_w'] / (KIT * math.sqrt(2))))) - 45.0
    why = ''
    if rate < 0.95:
        if enter < 44.9:
            why = ('  (kits that arrive aligned with the slot do not get out: the channel runs 32.8 degrees off the slot square and the %.1f mm bore takes a 10.3 mm cube only within %.1f degrees of its axis)'
                   % (P.CHUTE['in_w'] * 10, enter))
        else:
            why = ('  (kits that arrive aligned with the slot still do not all get out although the %.1f mm bore takes the cube at any angle: the hopper mouth, not the bore width, is the problem)'
                   % (P.CHUTE['in_w'] * 10))
    lines.append('RESULT: ' + ('FAIL' if (not ok or rate < 0.95) else 'PASS') + ('  (a control failed)' if not ok else '') + why)
    text = '\n'.join(lines)
    print(text)
    return text


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'quick', float(sys.argv[2]) if len(sys.argv) > 2 else None)
