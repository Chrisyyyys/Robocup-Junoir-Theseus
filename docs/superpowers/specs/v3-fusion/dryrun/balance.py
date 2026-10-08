"""Weight balance and swept circle of the rev 4 model (plan code on the dry-run emulator; masses are placeholders: printed parts from the model volume, bought parts from the rev 3 budget).
1. every component's mass and centre of mass, in groups;
2. the balance (centre of mass relative to the axle, front load, front-lift and brake-tip limits) and the swept radius for an axle `a` cm behind the body centre (mechanical design section 4).
The visibility of the electronics from above is the `access` report of the plan. Usage: python balance.py   (about 1 minute)"""
import io
import math
import os
import sys
import tempfile
import types

import numpy as np

DRY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DRY)
import plan_code  # noqa: E402

d = tempfile.mkdtemp(prefix='rev4_plan_')
names, problems = plan_code.assemble(d)
assert not problems, problems
sys.path.insert(0, d)
import fake_fusion as F  # noqa: E402

F.install(open(os.path.join(d, 'fusion_lib_patch.py'), encoding='utf-8').read())
import emu_checks as E  # noqa: E402
import v3_model as M3  # noqa: E402
import v4_model as M  # noqa: E402
import v4_checks3d as K  # noqa: E402
import v3_params4 as P  # noqa: E402

ctx = E.make_ctx(M3.tof_positions)
for nm, fn in M.STAGES:
    fn(ctx)
GHOST = K.GHOST

# ------------------------------------------------------------------ 1. masses
rows = []
for o in ctx.root._occ:
    nm = o.component.name
    if nm == GHOST:
        continue
    vol, cx, cy, cz = 0.0, 0.0, 0.0, 0.0
    for b in o.component.bodies:
        v = b.volume
        c = b.physicalProperties.centerOfMass
        vol, cx, cy, cz = vol + v, cx + c.x * v, cy + c.y * v, cz + c.z * v
    if vol <= 0:
        continue
    cx, cy, cz = cx / vol, cy / vol, cz / vol
    if nm in P.PRINTED:
        g, kind = vol * P.PETG_G_CM3 * P.PRINT_FILL, 'printed'
    elif nm in P.BOUGHT_G:
        g, kind = P.BOUGHT_G[nm], 'bought'
    else:
        raise SystemExit('no mass for ' + nm)
    rows.append((nm, kind, g, cx, cy, cz, vol))
for nm, g, x, z in P.UNMODELLED_G:
    rows.append((nm, 'unmodelled', g, x, 0.0, z, 0.0))
M_TOT = sum(r[2] for r in rows)

GROUPS = {
    'drive train (wheels, motors, face plates)': ['Wheel L', 'Wheel R', 'Motor L', 'Motor R', 'Face plate L', 'Face plate R'],
    'omni module': ['Omni wheel', 'Omni arm', 'Omni pins'],
    'battery': ['Battery'],
    'GIGA, shield, floor sensors, posts': ['Arduino GIGA R1', 'Main PCB', 'Floor port FP', 'Silver module SM', 'GIGA posts'],
    'ToF boards (9) and cameras (2)': ['ToF ' + n for n, x, y, a in P.TOF] + ['Camera L', 'Camera R'],
    'tub, bumpers, nub, USB socket': ['Tub', 'Bumper L', 'Bumper R', 'Bumper switch L', 'Bumper switch R', 'Rear nub', 'USB-C service socket'],
    'upper frame, lid, handle bar, control parts': ['Upper frame', 'Lid', 'Handle bar', 'Victim LED'] + [c[0] for c in P.CONTROLS],
    'dropper (floor, plate, kits, N20, hoppers, channels)': ['Dropper floor', 'Dropper plate', 'Kits', 'N20 motor', 'N20 face plate', 'Hopper A right', 'Hopper B left', 'Chute A right', 'Chute B left'],
    'not modelled (wiring, IMU, LCD, fasteners)': [r[0] for r in rows if r[1] == 'unmodelled'],
}
used = set(n for v in GROUPS.values() for n in v)
left = [r[0] for r in rows if r[0] not in used]
assert not left, left
byname = {r[0]: r for r in rows}
print('1. MASS BUDGET (placeholders: printed = volume x 1.27 g/cm3 x 0.45 fill; bought parts from the rev 3 budget)')
print('  %-48s %7s %8s %8s %8s' % ('group', 'g', 'x cm', 'y cm', 'z cm'))
tot = [0.0, 0.0, 0.0, 0.0]
gstats = {}
for gname, members in GROUPS.items():
    g = sum(byname[n][2] for n in members)
    x = sum(byname[n][2] * byname[n][3] for n in members) / g
    y = sum(byname[n][2] * byname[n][4] for n in members) / g
    z = sum(byname[n][2] * byname[n][5] for n in members) / g
    gstats[gname] = (g, x, y, z)
    print('  %-48s %7.1f %8.2f %8.2f %8.2f' % (gname, g, x, y, z))
X = sum(r[2] * r[3] for r in rows) / M_TOT
Y = sum(r[2] * r[4] for r in rows) / M_TOT
Z = sum(r[2] * r[5] for r in rows) / M_TOT
print('  %-48s %7.1f %8.2f %8.2f %8.2f' % ('TOTAL', M_TOT, X, Y, Z))
print('  heaviest single items:', ', '.join('%s %.0f g' % (r[0], r[2]) for r in sorted(rows, key=lambda r: -r[2])[:8]))

# ------------------------------------------------------------------ 2. balance against the axle position
DRIVE = set(GROUPS['drive train (wheels, motors, face plates)']) | {'Silver module SM'}
M_DRIVE = sum(byname[n][2] for n in DRIVE)
print('\n2. BALANCE FOR AN AXLE a cm BEHIND THE BODY CENTRE (drive train %.0f g moves with the axle, everything else stays in the body; omni %.1f cm ahead of the body centre)' % (M_DRIVE, P.OMNI['rest'][0]))
print('  %4s %10s %9s %10s %11s %11s %13s %12s' % ('a', 'COM x', 'COM z', 'front load', 'front load', 'lift limit', 'brake tip', 'swept R'))
print('  %4s %10s %9s %10s %11s %11s %13s %12s' % ('cm', 'cm ahead', 'cm', '%', 'N on omni', 'm/s2', 'limit m/s2', 'cm'))
# swept radius: farthest body point from the axle, every body of every component (sampled)
pts_all = []
for o in ctx.root._occ:
    if o.component.name == GHOST:
        continue
    for b in o.component.bodies:
        w = F._apply(b.comp.matrix, b.sample(40000))
        if len(w):
            pts_all.append(w)
pts_all = np.vstack(pts_all)
for a in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0):
    xa = X + a * (1 - M_DRIVE / M_TOT)                     # COM relative to the new axle
    d_omni = P.OMNI['rest'][0] + a
    f = xa / d_omni
    lift = 9.81 * xa / Z
    brake = 9.81 * (d_omni - xa) / Z
    # swept radius about the axle at x = -a in the body frame (bumper plates included, they are part of the model)
    r_s = float(np.sqrt((pts_all[:, 0] + a) ** 2 + pts_all[:, 1] ** 2).max())
    print('  %4.1f %10.2f %9.2f %9.1f%% %10.2f N %10.2f %13.1f %12.2f' % (a, xa, Z, 100 * f, f * M_TOT * 9.81 / 1000, lift, brake, r_s))
print('  corridor: path 28 cm (half 14.0), worst case with the rules\' 10 percent narrower: 25.2 cm (half 12.6): swept radius must stay below these; a robot centred with 0.3 cm of slack needs R <= 12.3')

print('section 3 (visibility) is now the access report of the plan')
