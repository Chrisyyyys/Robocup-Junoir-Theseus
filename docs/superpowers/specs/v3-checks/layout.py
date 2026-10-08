"""V3 layout model. Units cm. x forward, y left, z up. Origin: axle midpoint on the floor line (axle at z=4).
Parts are plan polygons (shapely) with a z interval. Round parts are sliced into z slabs.
"""
import math, itertools, random, json
from shapely.geometry import Polygon, Point, LineString, box
from shapely import affinity
from shapely.ops import unary_union

P = dict(
    R_body=9.5, wall=0.2, R_sw=10.0, z_belly=3.5, z_roof=11.5, z_lid=11.9, bump_z0=4.0, bump_z1=6.5, n20_len=4.15,
    wheel_r=4.0, wheel_w=2.0, wheel_yin=7.0, wheel_zc=4.0,
    motor_r=1.0, motor_face_y=6.1, motor_len=4.4, enc_len=0.3,
    omni_r=3.0, omni_w=2.0, omni_x=6.95, omni_z=3.0, omni_travel=2.5, omni_back=0.6,
    arm_pivot=(3.0, 3.6), fork_half=1.5,
    z_pf=9.0, plate_t=1.2, x_p=-2.0, r_ring=3.62, pocket=1.30, slot=1.35, rim=0.3,
    chute_w=1.5, chute_wall=0.15, exit_x=-6.0, exit_z=3.0,
    giga=(10.152, 5.334, 1.5), shield_extra=1.0,
    tof_w=1.8, tof_h=2.1, tof_t=0.45, tof_zc=10.0,
    cam=(4.5, 3.6, 2.9),  # long side, short side, depth along the lens axis
)
P['R_int'] = P['R_body'] - P['wall']
P['wheel_yout'] = P['wheel_yin'] + P['wheel_w']

def half_chord(r, zc, z):
    d = abs(z - zc)
    return math.sqrt(max(r*r - d*d, 0.0))

class Part:
    def __init__(self, name, poly, z0, z1, kind='fixed'):
        self.name, self.poly, self.z0, self.z1, self.kind = name, poly, z0, z1, kind
    def __repr__(self):
        return f'{self.name}[{self.z0:.1f},{self.z1:.1f}] a={self.poly.area:.1f}'

def overlap(a, b, tol=0.0):
    if a.z1 <= b.z0 + tol or b.z1 <= a.z0 + tol: return 0.0
    return a.poly.intersection(b.poly).area

def rect(cx, cy, l, w, ang_deg=0.0):
    r = box(-l/2, -w/2, l/2, w/2)
    r = affinity.rotate(r, ang_deg, origin=(0, 0))
    return affinity.translate(r, cx, cy)

def slab_cyl_y(name, xc, yc0, yc1, r, zc, zlo, zhi, step=0.5, kind='fixed', xshift=0.0):
    """cylinder with axis along y: slabs in z with plan rectangle half-chord wide"""
    parts = []
    z = zlo
    while z < zhi - 1e-9:
        z2 = min(z + step, zhi)
        hc = max(half_chord(r, zc, z), half_chord(r, zc, z2), half_chord(r, zc, (z+z2)/2)) if (zlo <= zc <= zhi and z <= zc <= z2) else max(half_chord(r, zc, z), half_chord(r, zc, z2))
        if hc > 0.0:
            parts.append(Part(name, box(xc - hc + xshift, min(yc0, yc1), xc + hc + xshift, max(yc0, yc1)), z, z2, kind))
        z = z2
    return parts

def fixed_parts(p=P):
    parts = []
    # drive wheels (outside the motors); left y>0, right y<0
    for s, nm in ((1, 'wheelL'), (-1, 'wheelR')):
        parts += slab_cyl_y(nm, 0.0, s*p['wheel_yin'], s*p['wheel_yout'], p['wheel_r'], p['wheel_zc'], 0.0, 2*p['wheel_r'])
    # gearmotors, axis along y at z=4
    for s, nm in ((1, 'motorL'), (-1, 'motorR')):
        y0 = p['motor_face_y']; y1 = y0 - p['motor_len'] - p['enc_len']
        parts += slab_cyl_y(nm, 0.0, s*y0, s*y1, p['motor_r'], p['wheel_zc'], p['wheel_zc']-p['motor_r'], p['wheel_zc']+p['motor_r'], step=0.5)
    # omni wheel envelope over its travel, centre path from the arm geometry (pivot below the axle: the omni moves BACK as it rises)
    px, pz = p['arm_pivot']
    L_arm = math.hypot(p['omni_x'] - px, p['omni_z'] - pz)
    centres = []
    for k in range(0, 26):
        zc = p['omni_z'] + p['omni_travel'] * k / 25
        dz = zc - pz
        dx = math.sqrt(max(L_arm**2 - dz**2, 0.0))
        centres.append((px + dx, zc))
    p['omni_centres'] = centres
    zlo, zhi = p['omni_z'] - p['omni_r'], p['omni_z'] + p['omni_travel'] + p['omni_r']
    z = zlo
    while z < zhi - 1e-9:
        z2 = min(z+0.25, zhi)
        xs0, xs1 = 1e9, -1e9
        for (cx, cz) in centres:
            for zz in (z, (z+z2)/2, z2):
                hc = half_chord(p['omni_r'], cz, zz)
                if hc > 0:
                    xs0 = min(xs0, cx - hc); xs1 = max(xs1, cx + hc)
        if xs1 > xs0:
            parts.append(Part('omni', box(xs0, -p['omni_w']/2, xs1, p['omni_w']/2), z, z2))
        z = z2
    # omni arm envelope (fork around the omni): from pivot to omni axle, z over travel
    parts.append(Part('omniArm', box(px - 0.4, -p['fork_half'], p['omni_x'] - p['omni_back'] + 0.3, p['fork_half']), p['z_belly'], p['omni_z'] + p['omni_travel'] + 0.5))
    return parts

def plate_parts(p=P):
    """dropper plate, its motor column and the two chutes. Plate frame: slot A at plate angle 0, pocket run 30..240, slot B at 270.
    On the robot the blank arc faces the rear; slots at robot angles 225 (right-rear) and 135 (left-rear)."""
    parts = []
    xp = p['x_p']
    R_plate = math.hypot(p['r_ring'] + p['pocket']/2, p['pocket']/2) + p['rim']
    p['R_plate'] = R_plate
    parts.append(Part('plate', Point(xp, 0).buffer(R_plate, 64), p['z_pf'], p['z_pf'] + p['plate_t']))
    parts.append(Part('plateFloor', Point(xp, 0).buffer(R_plate + 0.2, 64), p['z_pf'] - 0.3, p['z_pf']))
    parts.append(Part('n20', rect(xp, 0, 1.0, 1.2), p['z_pf'] - 0.3 - p['n20_len'], p['z_pf'] - 0.3))
    chutes = []
    for s, nm in ((1, 'chuteL'), (-1, 'chuteR')):
        a = math.radians(180 - 45) if s > 0 else math.radians(180 + 45)
        sx = xp + p['r_ring'] * math.cos(a); sy = p['r_ring'] * math.sin(a)
        ex = p['exit_x']; ey = s * math.sqrt(max(p['R_body']**2 - ex**2, 0))
        z_start = p['z_pf'] - 0.3 - p['chute_w']/2
        z_end = p['exit_z'] + p['chute_w']/2
        n = 12
        half = p['chute_w']/2 + p['chute_wall']
        for k in range(n):
            t0, t1 = k/n, (k+1)/n
            x0, y0 = sx + (ex - sx)*t0, sy + (ey - sy)*t0
            x1, y1 = sx + (ex - sx)*t1, sy + (ey - sy)*t1
            zc0 = z_start + (z_end - z_start)*t0; zc1 = z_start + (z_end - z_start)*t1
            seg = LineString([(x0, y0), (x1, y1)]).buffer(half, cap_style=2)
            parts.append(Part(nm, seg, min(zc0, zc1) - half, max(zc0, zc1) + half))
        chutes.append(((sx, sy), (ex, ey)))
    p['chute_ends'] = chutes
    return parts

def tof_positions(p=P):
    """(name, psi_deg position on the wall, aim_deg): position and aim are independent design variables"""
    t = p.get('tof_psi', dict(toed=25, sf=45, sr=135, rear=154))
    ta = t.get('toed_aim', 25)
    return [('tofF', 0, 0), ('tofFL', t['toed'], ta), ('tofFR', -t['toed'], -ta),
            ('tofSL_f', t['sf'], 90), ('tofSR_f', -t['sf'], -90), ('tofSL_r', t['sr'], 90), ('tofSR_r', -t['sr'], -90),
            ('tofRL', t['rear'], 180), ('tofRR', -t['rear'], 180)]

def tof_parts(p=P):
    parts = []
    for nm, psi, aim in tof_positions(p):
        a = math.radians(psi)
        cx, cy = p['R_body'] * math.cos(a) - 0.2*math.cos(a), p['R_body'] * math.sin(a) - 0.2*math.sin(a)
        # board: tof_w tangential, tof_t radial
        poly = rect(cx, cy, p['tof_t'], p['tof_w'], psi)
        z0 = p['tof_zc'] - p['tof_h']/2; z1 = p['tof_zc'] + p['tof_h']/2
        parts.append(Part(nm, poly, z0, z1, 'tof'))
    return parts

def bumper_parts(p=P):
    parts = []
    for s, nm in ((1, 'bumpL'), (-1, 'bumpR')):
        pts = []
        for ang in [12 + (58-12)*k/12 for k in range(13)]:
            a = math.radians(ang*s)
            pts.append((p['R_sw']*math.cos(a), p['R_sw']*math.sin(a)))
        for ang in [58 - (58-12)*k/12 for k in range(13)]:
            a = math.radians(ang*s)
            pts.append(((p['R_sw']-0.25)*math.cos(a), (p['R_sw']-0.25)*math.sin(a)))
        parts.append(Part(nm, Polygon(pts), p['bump_z0'], p['bump_z1'], 'bumper'))
    return parts

def camera_parts(side, psi_deg, zc, orient='h', p=P, name=None, aim_deg=None):
    """OpenMV H7 Plus (45 x 36 mm board, 2.0 x 2.0 cm lens mount, 2.3 cm lens length). Lens tip sits 0.1 cm outside the wall at azimuth psi.
    aim_deg = direction of the optical axis (azimuth from forward, CCW; 90 = straight out to the left). Default: radial.
    orient 'h': board long side horizontal ; 'v': long side vertical. Returns [board, lens]."""
    L, S, D = p['cam']
    tw, th = (L, S) if orient == 'h' else (S, L)
    R = p['R_body']
    aim = psi_deg if aim_deg is None else aim_deg
    a_p = math.radians(psi_deg); a = math.radians(aim)
    tip = ((R + 0.1) * math.cos(a_p), (R + 0.1) * math.sin(a_p))
    u = (math.cos(a), math.sin(a))
    nm = name or ('camL' if side > 0 else 'camR')
    lc = (tip[0] - u[0] * 2.3 / 2, tip[1] - u[1] * 2.3 / 2)
    bc_d = 2.3 + 0.2 + 0.3
    bc = (tip[0] - u[0] * bc_d, tip[1] - u[1] * bc_d)
    lens = Part(nm + '_lens', rect(lc[0], lc[1], 2.3, 2.0, aim), zc - 1.0, zc + 1.0, 'cam')
    board = Part(nm + '_board', rect(bc[0], bc[1], 0.6, tw, aim), zc - th/2, zc + th/2, 'cam')
    return [board, lens]

def interior_ok(part, p=P):
    inner = Point(0, 0).buffer(p['R_int'], 128)
    return part.poly.difference(inner).area < 1e-6

if __name__ == '__main__':
    fp = fixed_parts() + plate_parts() + tof_parts() + bumper_parts()
    print(len(fp), 'parts; R_plate', round(P['R_plate'], 2))
    bad = []
    for a, b in itertools.combinations(fp, 2):
        if a.name == b.name: continue
        if {a.kind, b.kind} == {'tof'}: continue
        o = overlap(a, b)
        if o > 1e-4: bad.append((a.name, b.name, round(o, 3)))
    from collections import Counter
    c = Counter((x, y) for x, y, _ in bad)
    for (x, y), n in sorted(c.items()):
        print('overlap', x, y, n)
