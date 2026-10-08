"""Side-view (x-z) kinematic check of the V3 on stairs, ramps, bumps. cm, degrees.
Body frame: origin at the axle centre; x forward, z up (z_abs = z_rel + 4.0 on flat floor).
Two pitch scenarios per drive-wheel position: STIFF (omni spring does not give until the skid stops the pitch) and SOFT (omni compresses first).
"""
import math, sys
from shapely.geometry import Polygon, Point, LineString

AXLE_Z = 4.0
G = dict(wheel_r=4.0, omni_r=3.0, omni_rest=(7.0, 3.0), pivot=(3.0, 2.9), travel=2.5,
         belly=3.0, bumper_bottom=3.5, bumper_x=(3.4, 10.0), lip=(-6.0, 3.1), rear=(-9.5, 3.0),
         skid=(-8.0, 0.9), skid_r=0.4, tip=(10.0, 3.0))

def rot(p, ph):
    c, s = math.cos(ph), math.sin(ph)
    return (p[0]*c - p[1]*s, p[0]*s + p[1]*c)

def terrain(profile, x0=-60.0, x1=90.0):
    pts = [(x0, -20.0)] + profile + [(x1, -20.0)]
    return Polygon(pts)

def prof_flat(): return [(-60, 0), (90, 0)]
def prof_step_up(h=2.0): return [(-60, 0), (0, 0), (0, h), (90, h)]
def prof_step_down(h=2.0): return [(-60, h), (0, h), (0, 0), (90, 0)]
def prof_stairs(h=2.0, tread=4.3, n=3, top=40.0, down=False):
    pts = [(-60, 0), (0, 0)]
    x = 0.0; z = 0.0
    for i in range(n):
        z += h; pts += [(x, z), (x + tread, z)]; x += tread
    pts += [(x + top, z)]
    if down:
        pts = [(-p[0] + x + top - 60 + 60, p[1]) for p in pts][::-1]
        pts = sorted(pts, key=lambda q: q[0])
    return pts
def prof_ramp(angle=25.0, length=30.0, top=40.0):
    t = math.tan(math.radians(angle)); L = length
    return [(-60, 0), (0, 0), (L*math.cos(math.radians(angle)), L*math.sin(math.radians(angle))), (L*math.cos(math.radians(angle)) + top, L*math.sin(math.radians(angle)))]
def prof_ramp_down(angle=25.0, length=30.0):
    H = length*math.sin(math.radians(angle)); X = length*math.cos(math.radians(angle))
    return [(-60, H), (0, H), (X, 0), (90, 0)]
def prof_bump(h=2.0, w=3.0): return [(-60, 0), (0, 0), (0, h), (w, h), (w, 0), (90, 0)]
def prof_seam(h=0.3): return [(-60, 0), (0, 0), (0, h), (90, h)]

def omni_centre(c):
    """arm: pivot P, length L, horizontal at rest; c = rise of the axle in cm. returns body-frame centre (x, z_rel)"""
    px, pz = G['pivot']; ox, oz = G['omni_rest']
    L = math.hypot(ox - px, oz - pz)
    z = oz + c; dz = z - pz
    dx = math.sqrt(max(L*L - dz*dz, 0.0))
    return (px + dx, z - AXLE_Z)

def wheel_centre_z(T, xd, r):
    lo, hi = r, 40.0
    for _ in range(40):
        mid = (lo + hi)/2
        if Point(xd, mid).distance(T) >= r - 1e-9 and not T.contains(Point(xd, mid)): hi = mid
        else: lo = mid
    return hi

def world(p_rel, xd, zd, ph):
    q = rot(p_rel, ph); return (xd + q[0], zd + q[1])

def solve_pitch_touch(T, xd, zd, point_rel, rad, lo=-math.radians(40), hi=math.radians(50)):
    """smallest pitch at which a circle (centre point_rel in body frame, radius rad) clears the terrain"""
    def clear(ph):
        w = world(point_rel, xd, zd, ph)
        return (not T.contains(Point(w))) and Point(w).distance(T) >= rad - 1e-9
    if clear(lo): return lo
    if not clear(hi): return None
    for _ in range(50):
        mid = (lo + hi)/2
        if clear(mid): hi = mid
        else: lo = mid
    return hi

def skid_limit(T, xd, zd):
    """largest pitch at which the skid circle still clears the terrain (None if it never does for ph>=-40deg)"""
    sk = (G['skid'][0], G['skid'][1] - AXLE_Z)
    def clear(ph):
        w = world(sk, xd, zd, ph); return (not T.contains(Point(w))) and Point(w).distance(T) >= G['skid_r'] - 1e-9
    lo, hi = -math.radians(40), math.radians(50)
    if not clear(lo): return lo
    if clear(hi): return hi
    for _ in range(50):
        mid = (lo + hi)/2
        if clear(mid): lo = mid
        else: hi = mid
    return lo

def parts_clearance(T, xd, zd, ph):
    out = {}
    def L(p0, p1): return LineString([world((p0[0], p0[1] - AXLE_Z), xd, zd, ph), world((p1[0], p1[1] - AXLE_Z), xd, zd, ph)])
    belly = L((G['rear'][0], G['belly']), (G['tip'][0] - 0.5, G['belly']))
    out['belly'] = -T.intersection(belly).length if T.intersects(belly) else belly.distance(T)
    bb = L((G['bumper_x'][0], G['bumper_bottom']), (G['bumper_x'][1], G['bumper_bottom']))
    out['bumper'] = -T.intersection(bb).length if T.intersects(bb) else bb.distance(T)
    lip = Point(world((G['lip'][0], G['lip'][1] - AXLE_Z), xd, zd, ph))
    out['exit lip'] = -1.0 if T.contains(lip) else lip.distance(T)
    rc = Point(world((G['rear'][0], G['belly'] - AXLE_Z), xd, zd, ph))
    out['rear corner'] = -1.0 if T.contains(rc) else rc.distance(T)
    return out

def run(name, profile, xs, verbose=True):
    T = terrain(profile)
    worst = {}; maxpitch = {'stiff': (-99, 0), 'soft': (-99, 0)}; minpitch = {'stiff': (99, 0), 'soft': (99, 0)}
    c_need = 0.0; bottomed = None; wheel_slip_note = None
    for xd in xs:
        zd = wheel_centre_z(T, xd, G['wheel_r'])
        for scen in ('stiff', 'soft'):
            skid_ph = skid_limit(T, xd, zd)
            if scen == 'stiff':
                oc = omni_centre(0.0)
                ph = solve_pitch_touch(T, xd, zd, oc, G['omni_r'])
                c = 0.0
                if ph is None or ph > skid_ph:
                    # skid stops the pitch: compress the omni just enough at the skid limit
                    ph = skid_ph
                    lo, hi = 0.0, G['travel']
                    ok = False
                    for _ in range(40):
                        mid = (lo + hi)/2
                        w = world(omni_centre(mid), xd, zd, ph)
                        if (not T.contains(Point(w))) and Point(w).distance(T) >= G['omni_r'] - 1e-9: hi = mid; ok = True
                        else: lo = mid
                    c = hi
                    w = world(omni_centre(c), xd, zd, ph)
                    if T.contains(Point(w)) or Point(w).distance(T) < G['omni_r'] - 1e-6:
                        if bottomed is None: bottomed = (xd, scen)
            else:
                # soft: omni compresses fully first; pitch = smallest pitch that clears with c = travel, but not above the stiff pitch
                oc = omni_centre(G['travel'])
                ph = solve_pitch_touch(T, xd, zd, oc, G['omni_r'])
                if ph is None: ph = skid_ph
                c = G['travel']
                # if the terrain under the omni is low the omni just hangs extended: same as stiff
                oc0 = omni_centre(0.0); ph0 = solve_pitch_touch(T, xd, zd, oc0, G['omni_r'])
                if ph0 is not None and ph0 < ph: ph = ph0; c = 0.0
                ph = min(ph, skid_ph)
            c_need = max(c_need, c if scen == 'stiff' else 0.0)
            cl = parts_clearance(T, xd, zd, ph)
            for k, v in cl.items():
                key = (scen, k)
                if key not in worst or v < worst[key][0]: worst[key] = (v, xd, math.degrees(ph))
            if math.degrees(ph) > maxpitch[scen][0]: maxpitch[scen] = (math.degrees(ph), xd)
            if math.degrees(ph) < minpitch[scen][0]: minpitch[scen] = (math.degrees(ph), xd)
    if verbose:
        print(f'== {name}')
        print(f'   pitch range stiff {minpitch["stiff"][0]:.1f}..{maxpitch["stiff"][0]:.1f} deg, soft {minpitch["soft"][0]:.1f}..{maxpitch["soft"][0]:.1f} deg; omni compression needed (stiff) up to {c_need:.2f} cm of {G["travel"]}' + (f'; OMNI BOTTOMED at xd={bottomed[0]:.1f}' if bottomed else ''))
        for k in ('belly', 'bumper', 'exit lip', 'rear corner'):
            a = worst[('stiff', k)]; b = worst[('soft', k)]
            fl = ' <-- COLLISION' if min(a[0], b[0]) < 0.05 else ''
            print(f'   {k:<11} min clearance stiff {a[0]:5.2f} cm (xd {a[1]:5.1f}, pitch {a[2]:5.1f})   soft {b[0]:5.2f} cm (xd {b[1]:5.1f}, pitch {b[2]:5.1f}){fl}')
    return worst, c_need, bottomed

if __name__ == '__main__':
    xs = [x/2 for x in range(-50, 100)]
    cases = [
        ('2 cm single riser, climbing', prof_step_up(2.0)),
        ('2 cm single riser, descending', prof_step_down(2.0)),
        ('3 steps of 2 cm, 25 deg incline (tread 4.3)', prof_stairs(2.0, 4.3, 3)),
        ('3 steps of 2 cm, 30 deg incline (tread 3.46)', prof_stairs(2.0, 3.46, 3)),
        ('25 deg ramp up (flat -> ramp -> crest)', prof_ramp(25.0)),
        ('25 deg ramp down (crest -> ramp -> flat)', prof_ramp_down(25.0)),
        ('2 cm speed bump 3 cm wide', prof_bump(2.0, 3.0)),
        ('1 cm speed bump 3 cm wide', prof_bump(1.0, 3.0)),
        ('3 mm tile seam', prof_seam(0.3)),
    ]
    for name, prof in cases:
        run(name, prof, xs)
