"""Side-view kinematic check v2. For every drive-wheel position the omni may be compressed by any c in [0, travel]; the body pitch is then fixed by
the omni touching the terrain. A pose is feasible if the omni does not penetrate, the rear support does not penetrate, and nothing else collides.
Reports the PESSIMISTIC value (worst feasible c) and the OPTIMISTIC value (best feasible c) of each clearance, and whether any feasible pose exists.
cm, degrees; body frame origin at the axle centre; z_abs = z_rel + 4.
"""
import math, itertools, sys
from shapely.geometry import Polygon, Point, LineString, MultiPoint
from sidesim import terrain, prof_flat, prof_step_up, prof_step_down, prof_ramp, prof_ramp_down, prof_bump, prof_seam, rot, world, wheel_centre_z, AXLE_Z

def stairs_steps(h, tread, n, top=40.0):
    pts = [(-60, 0), (0, 0)]
    x = 0.0; z = 0.0
    for i in range(n):
        z += h; pts += [(x, z), (x + tread, z)]; x += tread
    pts += [(x + top, z)]
    return pts

def mirror_profile(pts, x0=-60.0, x1=90.0):
    """mirror about x -> descending version"""
    xm = max(p[0] for p in pts)
    q = [(xm - p[0], p[1]) for p in pts][::-1]
    q = [(x0, q[0][1])] + q                      # the high level must run back to x0
    return [(min(x, x1), z) for x, z in q]

class Robot:
    def __init__(self, belly=3.0, bumper_bottom=3.5, bumper_x=(3.4, 10.0), tip_x=10.0, lip=(-6.0, 3.1),
                 skid=(-8.0, 0.5, 0.4), rear_corner=(-9.5, 3.0), rear_chamfer=None, omni_rest=(7.0, 3.0), pivot=(3.0, 2.9), travel=2.5,
                 omni_r=3.0, wheel_r=4.0):
        self.belly, self.bumper_bottom, self.bumper_x, self.tip_x, self.lip = belly, bumper_bottom, bumper_x, tip_x, lip
        self.skid = skid          # (x, z_abs of the lowest point, radius)  -> None for no skid
        self.rear_corner = rear_corner
        self.rear_chamfer = rear_chamfer   # (x_start, z_at_corner): belly line rises from (x_start, belly) to (rear x, z_at_corner)
        self.omni_rest, self.pivot, self.travel, self.omni_r, self.wheel_r = omni_rest, pivot, travel, omni_r, wheel_r
    def omni_centre(self, c):
        px, pz = self.pivot; ox, oz = self.omni_rest
        L = math.hypot(ox - px, oz - pz)
        z = oz + c; dz = z - pz
        dx = math.sqrt(max(L*L - dz*dz, 0.0))
        return (px + dx, z - AXLE_Z)
    def belly_poly_rel(self):
        pts = []
        rx = self.rear_corner[0]
        if self.rear_chamfer:
            xs, zc = self.rear_chamfer
            pts += [(rx, zc - AXLE_Z), (xs, self.belly - AXLE_Z)]
        else:
            pts += [(rx, self.belly - AXLE_Z)]
        pts += [(self.tip_x - 0.5, self.belly - AXLE_Z)]
        return pts
    def rear_points_rel(self):
        """points that must not penetrate: (point, radius)"""
        out = []
        if self.skid: out.append(((self.skid[0], self.skid[1] + self.skid[2] - AXLE_Z), self.skid[2]))
        bp = self.belly_poly_rel()
        out.append((bp[0], 0.05))
        return out

def clear_circle(T, w, rad):
    p = Point(w)
    return (not T.contains(p)) and p.distance(T) >= rad - 1e-9

def clear_point_pitch(T, xd, zd, prel, rad, ph): return clear_circle(T, world(prel, xd, zd, ph), rad)

def pitch_for_omni(T, R, xd, zd, c):
    """smallest pitch at which the omni (compression c) clears the terrain"""
    oc = R.omni_centre(c)
    def clear(ph): return clear_circle(T, world(oc, xd, zd, ph), R.omni_r)
    lo, hi = -math.radians(45), math.radians(55)
    if clear(lo): return lo
    if not clear(hi): return None
    for _ in range(45):
        mid = (lo + hi)/2
        if clear(mid): hi = mid
        else: lo = mid
    return hi

def rear_ok(T, R, xd, zd, ph):
    for prel, rad in R.rear_points_rel():
        if not clear_circle(T, world(prel, xd, zd, ph), rad): return False
    return True

def clearances(T, R, xd, zd, ph):
    out = {}
    bp = [world(p, xd, zd, ph) for p in R.belly_poly_rel()]
    belly = LineString(bp)
    out['belly'] = -T.intersection(belly).length if T.intersects(belly) else belly.distance(T)
    bb = LineString([world((R.bumper_x[0], R.bumper_bottom - AXLE_Z), xd, zd, ph), world((R.bumper_x[1], R.bumper_bottom - AXLE_Z), xd, zd, ph)])
    out['bumper'] = -T.intersection(bb).length if T.intersects(bb) else bb.distance(T)
    lip = Point(world((R.lip[0], R.lip[1] - AXLE_Z), xd, zd, ph))
    out['exit lip'] = -1.0 if T.contains(lip) else lip.distance(T)
    return out

def run(name, profile, R, xs, margin=0.2, verbose=True):
    T = terrain(profile)
    cs = [R.travel*i/10 for i in range(11)]
    res = {'pess': {}, 'opt': {}}; infeasible = []; maxpitch = -99; minpitch = 99; cneed = 0.0
    for xd in xs:
        zd = wheel_centre_z(T, xd, R.wheel_r)
        feas = []
        for c in cs:
            ph = pitch_for_omni(T, R, xd, zd, c)
            if ph is None: continue
            if not rear_ok(T, R, xd, zd, ph): continue
            cl = clearances(T, R, xd, zd, ph)
            feas.append((c, ph, cl))
        if not feas:
            infeasible.append(xd); continue
        # the spring is as stiff as the geometry allows: the body pitches up first, so the real pose is the smallest feasible c
        c0, ph0, cl0 = feas[0]
        cneed = max(cneed, c0)
        for key in cl0:
            if key not in res['pess'] or cl0[key] < res['pess'][key][0]: res['pess'][key] = (cl0[key], xd, math.degrees(ph0), c0)
        best = max(feas, key=lambda f: min(f[2].values()))
        for key in best[2]:
            if key not in res['opt'] or best[2][key] < res['opt'][key][0]: res['opt'][key] = (best[2][key], xd, math.degrees(best[1]), best[0])
        for c, ph, cl in feas:
            maxpitch = max(maxpitch, math.degrees(ph)); minpitch = min(minpitch, math.degrees(ph))
    res['cneed'] = cneed; res['pitch'] = (minpitch, maxpitch)
    ok_pess = (not infeasible) and all(v[0] >= margin for v in res['pess'].values())
    ok_opt = (not infeasible) and all(v[0] >= margin for v in res['opt'].values())
    if verbose:
        print(f'== {name}: {"PASS" if ok_pess else ("pass only if the spring cooperates" if ok_opt else "FAIL")}'
              f'{"  (no feasible pose at xd=" + str([round(x,1) for x in infeasible[:3]]) + ")" if infeasible else ""}  omni compression needed {cneed:.2f}/{R.travel}, pitch {minpitch:.1f}..{maxpitch:.1f}')
        for k in ('belly', 'bumper', 'exit lip'):
            if k in res['pess']:
                a = res['pess'][k]; b = res['opt'][k]
                print(f'     {k:<9} worst-case clearance {a[0]:5.2f} (xd {a[1]:5.1f}, pitch {a[2]:5.1f}, c {a[3]:.2f})   best-case {b[0]:5.2f}')
    return ok_pess, ok_opt, res, infeasible

REQUIRED = [
    ('2 cm single riser up', prof_step_up(2.0)),
    ('2 cm single riser down', prof_step_down(2.0)),
    ('2 x 1 cm steps up (tread 2.5)', stairs_steps(1.0, 2.5, 2)),
    ('2 x 1 cm steps down', mirror_profile(stairs_steps(1.0, 2.5, 2))),
    ('25 deg ramp up', prof_ramp(25.0)),
    ('25 deg ramp down', prof_ramp_down(25.0)),
    ('2 cm bump', prof_bump(2.0, 3.0)),
    ('1 cm bump', prof_bump(1.0, 3.0)),
    ('3 mm seam', prof_seam(0.3)),
]

def evaluate(R, xs=None, verbose=False):
    xs = xs or [x/2 for x in range(-50, 100)]
    allp = True; allo = True; details = {}
    for name, prof in REQUIRED:
        okp, oko, res, inf = run(name, prof, R, xs, verbose=verbose)
        allp &= okp; allo &= oko; details[name] = (okp, oko)
    return allp, allo, details

if __name__ == '__main__':
    print('### V3 as specified before this check: skid 5 mm off the floor 8 cm behind the axle, omni travel 2.5, belly 3.0')
    R0 = Robot()
    evaluate(R0, verbose=True)
