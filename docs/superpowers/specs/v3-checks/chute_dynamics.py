"""Does a rescue kit get out of the chute? A rigid-body simulation (MuJoCo) of a 10.3 mm cube in its plate pocket over slot B, falling through the slot into the hopper and sliding down the
square channel (18 mm bore) to the exit, on the geometry of the Fusion model (chute_geometry.py, checked against Fusion in chute_geometry_check.py). Spec section 6.2; units in the report are cm, m/s.
The first design (14.5 mm slot and hopper, 13 mm bore, trough cut flat at z 7.7) failed this check: results/chute_dynamics_first_design*.txt. The first version of this script left the plate out
(the independent review of 8 Oct found that and the 100 poses that every friction value shared): the plate's pocket walls are now in the model, and every kit has its own random pose.

Assumptions, all guesses to be replaced by the bench test (rev 3 test 3): kit mass 1.2 g (the mass budget says 6 g; sliding does not depend on the mass, see the 6 g row); a uniform cube (the real kit
may be weighted); Coulomb friction mu the same between the kit and every printed surface (swept, 0.15 to 0.60; the spec's earlier landing calculation used 0.35); nearly inelastic impacts (contact damping
ratio 1: 0.13 of the impact speed comes back; 0.3, which gives 0.45, is tried as well); the kit starts at rest on the floor in the plate pocket with a random turn (up to +-29 degrees, the most a 10.3 mm cube
can be turned in the 14.0 mm pocket) and a random place in it. The plate is four pocket walls 5 mm thick from z 9.0 to 10.2, PLATE_GAP above the floor (0.3 mm, a guess: it must turn freely; swept), parked
exactly over slot B, parked off along the ring by up to 2 mm (the pocket turns with the plate), or turning onto the slot at a given speed and stopping there at once. The plate's hub and motor and the
other pockets are not modelled; the robot stands level. The tub floor, the tub wall and the wheel are not modelled: a kit that leaves the hopper or the channel anywhere but the exit is counted as lost.

Controls (the run fails if one does not behave): the same engine settings must give the textbook acceleration g (sin a - mu cos a) of a cube on a plane tilted like the channel, must hold the
cube still at mu 0.7 (tan 32.2 deg = 0.63), and must stop a cube too big for the slot.

Needs MuJoCo (pip install mujoco; installed in C:/Users/christopher.shu/pl4_mujoco, found automatically). Run with PYTHONPATH="C:/Users/christopher.shu/pl4" python chute_dynamics.py [quick|full] [noplate]
[bore in cm, for a what-if with another channel bore]; `noplate` reproduces the first version's model (no pocket walls).
"""
import contextlib
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
PL = P.PLATE
POCKET_HALF = PL['pocket'] / 2                         # 0.70: half width of the plate's 14 mm pocket
WALL_T = 0.5                                           # thickness of the pocket walls that are modelled (cm); the plate's webs between pockets are about 6 mm
PLATE_GAP = 0.03                                       # cm: clearance under the plate (a guess)
SLOT_DEG = PL['slotB_robot_deg']                       # 135: the ring angle of slot B, and of the pocket parked over it

_geo = None
TRIALS = []                                           # (mu, kit turn in degrees, outcome, sweep tag, plate parked this far off in cm) of every trial


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


def plate_xml(fr, solref, gap):
    """The plate as a mocap body (moved by Sim.set_plate): the four walls of a 14 mm pocket, z 9.0 + gap to 10.2, in the frame of the pocket's centre, sides along and across the ring."""
    h, t = POCKET_HALF, WALL_T
    z0, z1 = PL['z0'] + gap, PL['z0'] + PL['t']
    zc, hz = (z0 + z1) / 2 * CM, (z1 - z0) / 2 * CM
    walls = [(h + t / 2, 0.0, t / 2, h + t), (-(h + t / 2), 0.0, t / 2, h + t), (0.0, h + t / 2, h, t / 2), (0.0, -(h + t / 2), h, t / 2)]
    geoms = ''.join('<geom type="box" pos="%g %g 0" size="%g %g %g" friction="%s" solref="%s" rgba="0.8 0.6 0.2 0.4"/>' % (x * CM, y * CM, a * CM, b * CM, hz, fr, solref) for x, y, a, b in walls)
    return '<body name="plate" mocap="true" pos="0 0 %g">%s</body>' % (zc, geoms)


def world_xml(mu, size=KIT, mass_g=1.2, damp=1.0, timeconst=0.002, gvec=(0.0, 0.0, -GRAV), plane_tilt_deg=None, ground=True, plate=False, plate_gap=PLATE_GAP):
    """MJCF of the chute, the ground, a corridor wall 14 cm to the left of the robot's centre line, the kit with a free joint and, with plate=True, the plate's pocket walls.
    plane_tilt_deg: replace the chute by a flat plate tilted like the channel (the control)."""
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
        geoms.append('<geom type="box" size="0.5 0.5 0.01" pos="0 0 -0.01" euler="0 %g 0" friction="%s" solref="%s"/>' % (-plane_tilt_deg, fr, solref))
    parts.append('</asset><worldbody>')
    if ground:
        parts.append('<geom type="plane" size="2 2 0.1" pos="0 0 0" friction="%s" solref="%s"/>' % (fr, solref))
        parts.append('<geom type="box" size="1 0.01 0.3" pos="0 0.15 0.3" friction="%s" solref="%s"/>' % (fr, solref))        # corridor wall, 15 cm: its face is at y = 14 cm
    parts.extend(geoms)
    h = size * CM / 2
    parts.append('<body name="kit" pos="0 0 1"><freejoint/><geom name="kit" type="box" size="%g %g %g" mass="%g" friction="%s" solref="%s"/></body>' % (h, h, h, mass_g * 1e-3, fr, solref))
    if plate:
        parts.append(plate_xml(fr, solref, plate_gap))
    parts.append('</worldbody></mujoco>')
    return ''.join(parts)


class Sim:
    def __init__(self, **kw):
        self.model = mujoco.MjModel.from_xml_string(world_xml(**kw))
        self.size = kw.get('size', KIT)
        self.data = mujoco.MjData(self.model)
        self.kit = self.model.body('kit').id
        self.p0, self.d, self.ex, self.ey, self.ez, self.e = G.channel_frame(1)
        self.has_plate = bool(kw.get('plate'))
        self.mid = self.model.body('plate').mocapid[0] if self.has_plate else None

    def place(self, pos_cm, yaw_deg=0.0):
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[:3] = np.asarray(pos_cm, float) * CM
        a = math.radians(yaw_deg) / 2
        d.qpos[3:7] = [math.cos(a), 0.0, 0.0, math.sin(a)]
        d.qvel[:] = 0
        mujoco.mj_forward(self.model, d)

    def set_plate(self, ring_deg):
        """The plate's pocket on the ring at this robot angle (degrees), turned with it; SLOT_DEG is the pocket exactly over slot B. Call it after place(), which resets the mocap body."""
        a = math.radians(ring_deg)
        self.data.mocap_pos[self.mid][:2] = [(PL['cx'] + PL['r_ring'] * math.cos(a)) * CM, (PL['cy'] + PL['r_ring'] * math.sin(a)) * CM]
        self.data.mocap_quat[self.mid] = [math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)]

    def pos_cm(self):
        return self.data.qpos[:3] / CM

    def vel(self):
        return self.data.qvel[:3].copy()


def slot_frame():
    sx, sy = P.slot_xy('B')
    return sx, sy


def drop_trial(sim, mu_unused, yaw, dx, dy, max_t=1.0, land=False, off_cm=0.0, motion=None):
    """One kit over slot B. With a plate in the model (dx, dy) is the kit's place in the pocket's frame (cm, along and across the ring), yaw its turn relative to the pocket (degrees), off_cm how far the plate is
    parked off along the ring (the pocket turns with it), and motion = (omega in degrees/s, ring angle to start from): the plate starts there, turns at omega to the slot and stops there at once, the kit
    starting at rest where it lies in the pocket of the starting position. Without a plate: (dx, dy) in the slot's frame, yaw relative to the slot square, off_cm added to dx.
    Returns a dict: outcome 'exit' / 'stuck' / 'lost' / 'timeout', and the numbers."""
    sx, sy = slot_frame()
    z = PL['z0'] + sim.size / 2 + 0.02
    sign, ring0, omega = 0.0, SLOT_DEG, 0.0
    if sim.has_plate:
        if motion is None:
            ring0 = SLOT_DEG - math.degrees(off_cm / PL['r_ring'])
        else:
            omega, ring0 = motion
            sign = 1.0 if SLOT_DEG > ring0 else -1.0
            max_t += abs(SLOT_DEG - ring0) / omega
        a = math.radians(ring0)
        pc = np.array([PL['cx'] + PL['r_ring'] * math.cos(a), PL['cy'] + PL['r_ring'] * math.sin(a)])
        ur, ut = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        q = pc + dx * ur + dy * ut
        sim.place((q[0], q[1], z), ring0 + yaw)
        sim.set_plate(ring0)
        mujoco.mj_forward(sim.model, sim.data)
    else:
        c, s = math.cos(math.radians(45.0)), math.sin(math.radians(45.0))      # the slot square is turned 45 degrees: its sides are along (c, s) and (-s, c)
        sim.place((sx + (dx + off_cm) * c - dy * s, sy + (dx + off_cm) * s + dy * c, z), 45.0 + yaw)
    e = sim.e
    r_hat = np.array([e[0], e[1]]) / math.hypot(e[0], e[1])
    t_still, t = 0.0, 0.0
    chunk = 5 if omega else 25                                               # a moving plate is stepped more finely
    out = {'outcome': 'timeout', 't': None}
    while t < max_t:
        if omega:
            sim.set_plate(ring0 + sign * min(omega * t, abs(SLOT_DEG - ring0)))
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
        plate_moving = bool(omega) and omega * t < abs(SLOT_DEG - ring0)
        if np.linalg.norm(v) < 0.004 and not plate_moving:
            t_still += chunk * DT
            if t_still > 0.25 and p[2] < 8.9:
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


def sample_pocket(rnd, size=KIT, max_yaw=None):
    """A kit of this size in the 14.0 mm pocket: a random turn (up to the most the pocket allows), then a random place that still fits the pocket (half width 0.70 cm)."""
    if max_yaw is None:
        max_yaw = math.degrees(math.asin(min(1.0, 2 * POCKET_HALF / (size * math.sqrt(2))))) - 45.0
    yaw = rnd.uniform(-max_yaw, max_yaw)
    ext = size / 2 * (abs(math.cos(math.radians(yaw))) + abs(math.sin(math.radians(yaw))))
    lim = max(0.0, POCKET_HALF - ext)
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


def oversize_control(lines, size=1.75):
    sim = Sim(mu=0.30, size=size)                                        # no plate: a 17.5 mm cube does not fit the 14 mm pocket
    n_exit = 0
    for yaw in (0.0, 15.0, -15.0, 29.0):
        r = drop_trial(sim, None, yaw, 0.0, 0.0)
        n_exit += r['outcome'] == 'exit'
    good = n_exit == 0
    lines.append('  %s control: a %.1f mm cube (too big for the %.0f mm slot) must not get out: %d of 4 got out' % ('ok  ' if good else 'FAIL', size * 10, PL['slot'] * 10, n_exit))
    return good


def sweep(mus, n, seed, damp=1.0, gvec=(0.0, 0.0, -GRAV), par=0.0, tag='nominal', show_jams=3, size=KIT, plate=True, gap=PLATE_GAP, mass_g=1.2):
    """n kits for each friction value, every kit with its own random pose (one random stream for the whole sweep); par: the plate parked off along the ring by up to this many cm."""
    rnd = random.Random(seed)
    rows = []
    for mu in mus:
        sim = Sim(mu=mu, damp=damp, gvec=gvec, size=size, plate=plate, plate_gap=gap, mass_g=mass_g)
        res = {'exit': 0, 'stuck': 0, 'lost': 0, 'timeout': 0}
        speeds, times, jam = [], [], []
        for k in range(n):
            yaw, dx, dy = sample_pocket(rnd, size)
            off = rnd.uniform(-par, par) if par else 0.0
            r = drop_trial(sim, mu, yaw, dx, dy, off_cm=off)
            TRIALS.append((mu, yaw, r['outcome'], tag, abs(off)))
            res[r['outcome']] += 1
            if r['outcome'] == 'exit':
                speeds.append(r['speed'])
                times.append(r['t'])
            elif len(jam) < show_jams:
                jam.append((yaw, dx, dy, off, r['outcome'], r.get('pos')))
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
        for yaw, dx, dy, off, oc, pos in jam:
            out.append('    mu %.2f %s: kit turned %+.0f deg, place (%+.2f, %+.2f) cm in the pocket, plate %+.1f mm off along the ring%s' % (
                mu, oc, yaw, dx, dy, off * 10, '; stopped at (%s)' % ', '.join('%.2f' % v for v in pos) if pos is not None else ''))
    return out


def turn_table(mus, tag='nominal'):
    """Number of kits that get out, by the turn of the kit from the pocket's square (the slot square, when the plate is parked exactly)."""
    bins = [(-29, -20), (-20, -10), (-10, 0), (0, 10), (10, 20), (20, 29)]
    out = ['  kits that get out, by the turn of the kit from the slot square (degrees):', '   turn        ' + ''.join('  mu %.2f' % m for m in mus)]
    for lo, hi in bins:
        cells = []
        for m in mus:
            sel = [o for mu, y, o, t, f in TRIALS if t == tag and abs(mu - m) < 1e-9 and lo <= y < hi]
            cells.append('%3d/%-3d ' % (sum(1 for o in sel if o == 'exit'), len(sel)))
        out.append('  %+3d to %+3d   ' % (lo, hi) + ' '.join('%7s' % c for c in cells))
    return out


def rate(tag):
    sel = [o for mu, y, o, t, f in TRIALS if t == tag]
    return sum(1 for o in sel if o == 'exit') / max(1, len(sel)), len(sel)


def offset_bins(tag='parking', edges=(0.0, 0.5, 1.0, 1.5, 2.0)):
    """The kits of a sweep with the plate parked off, by how far off the plate was (all friction values together)."""
    out = ['  the kits of the 2 mm sweep above, by how far off the plate was (mm) (all friction values together):']
    cells = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = [o for mu, y, o, t, f in TRIALS if t == tag and lo <= f * 10 < hi + (1e-9 if hi == edges[-1] else 0.0)]
        cells.append('%.1f to %.1f: %d of %d' % (lo, hi, sum(1 for o in sel if o == 'exit'), len(sel)))
    out.append('   ' + ';  '.join(cells))
    return out


def offset_table(mus, offsets_mm, n, seed, plate=True, gap=PLATE_GAP):
    """Kits that get out against how far the plate is parked off along the ring (half of the kits each way)."""
    rnd = random.Random(seed)
    sims = {mu: Sim(mu=mu, plate=plate, plate_gap=gap) for mu in mus}
    out = ['  kits that get out (of %d each) against the plate parked this far off along the ring, either way (the pocket turns with the plate: 1 mm is 1.5 degrees):' % n,
           '   off mm      ' + ''.join('  mu %.2f' % m for m in mus)]
    rates = {}
    for off in offsets_mm:
        cells = []
        for mu in mus:
            ok = 0
            for k in range(n):
                yaw, dx, dy = sample_pocket(rnd)
                r = drop_trial(sims[mu], mu, yaw, dx, dy, off_cm=(1 if k % 2 == 0 else -1) * off / 10.0)
                ok += r['outcome'] == 'exit'
            cells.append('%3d/%-3d' % (ok, n))
            rates[(off, mu)] = ok / float(n)
        out.append('   %4.1f         ' % off + '   '.join(cells))
    return out, rates


def gap_table(gaps_mm, mus, n, seed, par=0.2):
    """The same 2 mm parking error with the plate this far above the floor."""
    rnd = random.Random(seed)
    out = ['  kits that get out (of %d each) with the plate parked up to %.0f mm off, against the clearance under the plate:' % (n, par * 10), '   gap mm      ' + ''.join('  mu %.2f' % m for m in mus)]
    for gap in gaps_mm:
        cells = []
        for mu in mus:
            sim = Sim(mu=mu, plate=True, plate_gap=gap / 10.0)
            ok = 0
            for k in range(n):
                yaw, dx, dy = sample_pocket(rnd)
                ok += drop_trial(sim, mu, yaw, dx, dy, off_cm=rnd.uniform(-par, par))['outcome'] == 'exit'
            cells.append('%3d/%-3d' % (ok, n))
        out.append('   %4.1f         ' % gap + '   '.join(cells))
    return out


def carried_table(omegas, mus, n, seed, gap=PLATE_GAP):
    """The plate turning the pocket onto the slot from 20 degrees away (half of the kits from each side) at omega degrees/s, and stopping there at once: the kit is pushed over the slot's edge."""
    rnd = random.Random(seed)
    out = ['  kits that get out (of %d each) when the plate turns the pocket onto the slot from 20 degrees away (13 mm of ring) and stops there; 100 degrees/s is 0.07 m/s at the ring:' % n,
           '   deg/s       ' + ''.join('  mu %.2f' % m for m in mus)]
    rates = {}
    for om in omegas:
        cells = []
        for mu in mus:
            sim = Sim(mu=mu, plate=True, plate_gap=gap)
            ok = 0
            for k in range(n):
                yaw, dx, dy = sample_pocket(rnd)
                ring0 = SLOT_DEG - 20.0 if k % 2 == 0 else SLOT_DEG + 20.0
                ok += drop_trial(sim, mu, yaw, dx, dy, motion=(om, ring0))['outcome'] == 'exit'
            cells.append('%3d/%-3d' % (ok, n))
            rates[(om, mu)] = ok / float(n)
        out.append('   %5.0f        ' % om + '   '.join(cells))
    return out, rates


@contextlib.contextmanager
def bore_width(bore_cm):
    """Rebuild the channel with a bore of this width and the same outside (a printed hole comes out small); restored afterwards."""
    global _geo
    saved = P.CHUTE['in_w']
    P.CHUTE['in_w'] = bore_cm
    _geo = None
    try:
        yield
    finally:
        P.CHUTE['in_w'] = saved
        _geo = None


def corner_rows(mus, n, seed):
    """Tolerance corners on the design as built: a kit at the D7 print tolerance (10.5 mm: space diagonal 18.19 mm, over the 18 mm bore), a bore printed 0.4 mm small, a 6 g kit, with
    nearly inelastic (damping 1) and with bouncy (0.3) impacts, plate parked exactly."""
    out = ['  tolerance corners (kits that get out of %d per friction value, %d in all; plate parked exactly):' % (n, n * len(mus)),
           '   case                                         damping 1         damping 0.3 (bouncy)']
    cases = [('kit 10.3 mm, bore 18.0 mm (as designed)', dict(), None), ('kit 10.5 mm, bore 18.0 mm', dict(size=1.05), None), ('kit 10.3 mm, bore printed 17.6 mm', dict(), 1.76),
             ('kit 10.5 mm, bore printed 17.6 mm', dict(size=1.05), 1.76), ('kit 10.3 mm, 6 g (the mass budget)', dict(mass_g=6.0), None)]
    for name, kw, bore in cases:
        cells = []
        with bore_width(bore) if bore else contextlib.nullcontext():
            for damp in (1.0, 0.3):
                rows = sweep(mus, n, seed, damp=damp, tag='corner', **kw)
                ok = sum(res['exit'] for mu, res, sp, tm, jam in rows)
                cells.append('%3d/%-4d %5.1f %%' % (ok, n * len(mus), 100.0 * ok / (n * len(mus))))
        out.append('   %-44s %s     %s' % (name, cells[0], cells[1]))
    return out


def main(mode='quick', bore=None, plate=True):
    global _geo
    t0 = time.time()
    note = 'the geometry is the Fusion model (chute_geometry_check.py: PASS)'
    if bore:
        P.CHUTE['in_w'] = bore
        P.CHUTE['out_w'] = bore + 2 * P.CHUTE['wall']
        _geo = None
        note = 'WHAT IF: the channel bore set to %.1f mm (outside %.1f mm), everything else as in the Fusion model' % (bore * 10, P.CHUTE['out_w'] * 10)
    lines = ['kit chute dynamics (MuJoCo %s), kit %.1f mm, 1.2 g, slot B (left) to the exit at the body wall; slot %.1f mm, hopper void %.1f mm, bore %.1f mm; %s; %s'
             % (mujoco.__version__, KIT * 10, PL['slot'] * 10, P.CHUTE['hopper_in'] * 10, P.CHUTE['in_w'] * 10, note,
                'plate pocket walls in the model, %.1f mm above the floor' % (PLATE_GAP * 10) if plate else 'NO plate in the model (the first version)')]
    ok = control_report(lines)
    ok = oversize_control(lines) and ok
    full = mode != 'quick'
    n = 100 if full else 30
    mus = (0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60) if full else (0.25, 0.35, 0.45)
    rows = sweep(mus, n, seed=7, tag='nominal', plate=plate)
    lines += format_rows(rows, n, '%d kits per friction value, each with its own random turn (+-29 degrees) and place in the pocket, plate parked exactly, robot level, damping ratio 1 (nearly inelastic):' % n)
    lines += turn_table(mus)
    rows = sweep(mus, n, seed=8, par=0.2, tag='parking', plate=plate)
    lines += format_rows(rows, n, 'the same with the plate parked up to 2 mm off along the ring (+-3 degrees, the limit of plate_tolerance):')
    rows = sweep(mus, n, seed=10, par=0.1, tag='parking1', plate=plate)
    lines += format_rows(rows, n, 'the same with the plate parked up to 1 mm off (1.5 degrees):')
    rows = sweep(mus, n, seed=9, damp=0.3, tag='bouncy', plate=plate)
    lines += format_rows(rows, n, 'the same with bouncier impacts (damping ratio 0.3), plate parked exactly:')
    r_nom, n_nom = rate('nominal')
    r_park, n_park = rate('parking')
    r_park1, n_park1 = rate('parking1')
    r_bounce, n_bounce = rate('bouncy')
    lines.append('  share of kits that get out: %.1f %% plate parked exactly (%d kits), %.1f %% plate up to 1 mm off (%d), %.1f %% up to 2 mm off (%d), %.1f %% bouncy impacts (%d)' % (
        100 * r_nom, n_nom, 100 * r_park1, n_park1, 100 * r_park, n_park, 100 * r_bounce, n_bounce))
    allow = (PL['slot'] - PL['pocket']) / 2 * 10
    limits = ['the plate parked more than %.1f mm off along the ring, which is all the %.0f mm slot leaves over a kit lying corner to corner in the %.0f mm pocket: %.1f %% of the kits get out with the plate up to 2 mm off'
              % (allow, PL['slot'] * 10, PL['pocket'] * 10, 100 * r_park)]
    if full and plate:
        lines += offset_bins()
        out, off_rates = offset_table((0.35, 0.55), (0.0, 0.5, 1.0, 1.5, 2.0), 100, seed=11)
        lines += out
        lines += gap_table((0.1, 0.3, 0.6), (0.35, 0.55), 100, seed=12)
        out, carried = carried_table((60.0, 120.0, 200.0), (0.15, 0.35, 0.55), 30, seed=13)
        lines += out
        limits.append('a plate that turns onto the slot at 60 degrees/s and stops there at once, at friction 0.55: %d of 30 get out (120 degrees/s and faster: %d of 30)' % (round(30 * carried[(60.0, 0.55)]), round(30 * carried[(120.0, 0.55)])))
        lines += corner_rows((0.15, 0.35, 0.55), 60, seed=14)
        limits.append('the bore, 0.16 mm wider than the space diagonal of the cube: with bouncy impacts a 10.5 mm kit or a bore printed 0.4 mm small loses 2 to 5 % (the tolerance corners)')
    lines.append('  limits found (reported, not pass/fail): ' + '; '.join('(%d) %s' % (i + 1, t) for i, t in enumerate(limits)))
    lines.append('time %.0f s' % (time.time() - t0))
    why = []
    if r_nom < 0.99:
        why.append('plate parked exactly: %.1f %% (need 99 %%)' % (100 * r_nom))
    if r_park1 < 0.99:
        why.append('plate up to 1 mm off, the allowance: %.1f %% (need 99 %%)' % (100 * r_park1))
    if r_bounce < 0.99:
        why.append('bouncy impacts: %.1f %% (need 99 %%)' % (100 * r_bounce))
    if not ok:
        why.append('a control failed')
    lines.append('RESULT: ' + ('PASS (for the plate parked within %.1f mm: see the limits above)' % allow if not why else 'FAIL  (' + '; '.join(why) + ')'))
    text = '\n'.join(lines)
    print(text)
    return text


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if a != 'noplate']
    main(args[0] if args else 'quick', float(args[1]) if len(args) > 1 else None, plate='noplate' not in sys.argv[1:])
