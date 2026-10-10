"""How the front omni wheel is mounted (mechanical design section 4.1, 9 Oct): geometry, clearances, spring and strength of the sprung single arm for the real wheel (Nexus 14145: 60 x 25.6 mm, 12 mm bore,
73 g) against the layout of 8 Oct (a 20 mm wheel and a pivot at z 3.6). cm, N and mm where a label says so.
    python omni_mount.py          prints both layouts, the checks, the spring and the strength numbers (results/omni_mount.txt)
Sources: the wheel from Nexus' datasheet 14145 (vstone.co.jp, read on 9 Oct: diameter 60, width 25.6, axle hole 12, 73 g, 3 kg, 10 rollers); the masses and the centre of mass from balance.py --repo (9 Oct model with the
real wheel, arm and fasteners); the spring from the textbook formulas (Shigley: rate E d^4 / (64.1 D N), index factor Ki, music wire E 207 GPa, tensile strength about 2200 MPa at 1.6 mm); the layout and the mount
from v3_params4 (the one source of the model)."""
import math

import v3_params4 as P

R_WHEEL = P.OMNI_WHEEL['r']
WALL_IN = P.R_INT                                                          # inner face of the tub wall, 10.3
BELLY = P.Z_BELLY
POST_H1 = (P.giga_hole_xy(0)[0], P.giga_hole_xy(0)[1], P.GIGA_POST['r'])   # GIGA hole H1 post: x, y, radius (spec 8.1)
Z_BRIDGE = P.FRAME['z0']                                                   # underside of the front bridge
Z_TRAY = P.TRAY['z'][0]                                                    # underside of the battery tray
TRAVEL = P.OMNI['travel']
W_KG, X_COM, H_COM = 1.286, 1.09, 6.79                                     # balance.py --repo, 9 Oct model with the real omni module (placeholder masses): kg, cm ahead of the axle, cm high

OLD = dict(name='8 Oct', w=2.0, yc=0.0, rest_x=7.0, pz=3.6, arm_len=4.49, arm_t=0.4, gap=0.1)
NEW = dict(name='9 Oct', w=P.OMNI['w'], yc=P.OMNI['yc'], rest_x=P.OMNI['rest'][0], pz=P.OMNI['pz'], arm_len=P.OMNI['arm_len'], arm_t=P.OMNI['arm_t'], gap=P.OMNI['arm_gap'])


def layout(d):
    """Pivot x, the arm angle at rest and fully compressed, and the y ranges of the wheel and the arm."""
    dz = d['pz'] - 3.0
    px = d['rest_x'] - math.sqrt(d['arm_len'] ** 2 - dz ** 2)
    out = dict(d)
    out['px'] = px
    out['phi_rest'] = math.degrees(math.asin(-dz / d['arm_len']))
    out['phi_full'] = math.degrees(math.asin((3.0 + TRAVEL - d['pz']) / d['arm_len']))
    out['y_wheel'] = (d['yc'] - d['w'] / 2, d['yc'] + d['w'] / 2)
    out['y_arm'] = (out['y_wheel'][1] + d['gap'], out['y_wheel'][1] + d['gap'] + d['arm_t'])
    return out


MODEL = (W_KG, X_COM, H_COM, layout(NEW))


def axle_x(L, z):
    return L['px'] + math.sqrt(L['arm_len'] ** 2 - (z - L['pz']) ** 2)


def front_margins(L):
    """Smallest gap from the wheel's front rim, at its outer face, to the wall's inner face over the whole travel, and the largest front edge x."""
    best, xmax = 1e9, 0.0
    for k in range(0, 101):
        z = 3.0 + TRAVEL * k / 100
        x = axle_x(L, z) + R_WHEEL
        xmax = max(xmax, x)
        y = max(abs(L['y_wheel'][0]), abs(L['y_wheel'][1]))
        best = min(best, WALL_IN - math.hypot(x, y))
    return best, xmax


def report_layout(L):
    ok = []
    print('--- layout of %s: wheel %.2f cm wide, y %.2f to %.2f, arm %.2f cm long at y %.2f to %.2f' % (L['name'], L['w'], L['y_wheel'][0], L['y_wheel'][1], L['arm_len'], L['y_arm'][0], L['y_arm'][1]))
    print('  pivot (%.3f, %.2f), rest axle (%.2f, 3.00), arm angle %.1f deg at rest and %+.1f deg fully compressed (%.1f deg of swing); axle x at rest %.2f, at the end %.2f'
          % (L['px'], L['pz'], L['rest_x'], L['phi_rest'], L['phi_full'], L['phi_full'] - L['phi_rest'], L['rest_x'], axle_x(L, 3.0 + TRAVEL)))
    margin, xmax = front_margins(L)
    ok.append(margin >= 0.1)
    print('  front rim: largest x %.2f (front edge at rest %.2f); gap from the rim\'s outer corner to the wall over the travel %.1f mm %s' % (xmax, L['rest_x'] + R_WHEEL, margin * 10, 'ok' if margin >= 0.1 else 'TOO CLOSE (>= 1 mm wanted)'))
    h1 = POST_H1[1] + POST_H1[2] - L['y_wheel'][0]
    ok.append(h1 <= -0.25)
    print('  wheel face to the GIGA post H1: %.1f mm %s' % (-h1 * 10, 'ok' if h1 <= -0.25 else 'CLASH (>= 2.5 mm wanted)'))
    top = 3.0 + TRAVEL + R_WHEEL
    print('  wheel top fully compressed z %.2f, bridge underside %.2f: %.1f mm' % (top, Z_BRIDGE, (Z_BRIDGE - top) * 10))
    return all(ok)


def pivot_checks(L):
    """The pin hole, the spring coil and the arm against the belly line (the arm 12 mm tall)."""
    hole_r = P.OMNI_MOUNT['hole_r']
    coil_od = P.OMNI_SPRING['od']
    arm_half = P.OMNI['arm_half']
    below_hole = (L['pz'] - hole_r) - BELLY
    coil_bottom = L['pz'] - coil_od / 2
    arm_bottom = L['pz'] - arm_half
    print('--- pivot at z %.2f: pin hole bottom %.2f (printed material under it %.1f mm: floor 4 mm + the ear), coil (OD %.1f mm) bottom %.2f (belly %.2f), arm bottom at the pivot %.2f' %
          (L['pz'], L['pz'] - hole_r, below_hole * 10, coil_od * 10, coil_bottom, BELLY, arm_bottom))
    ok = below_hole >= 0.35 and coil_bottom >= BELLY + 0.1 and arm_bottom >= BELLY
    print('  %s (needs 3.5 mm under the hole, 1 mm under the coil, and nothing under the belly line z %.1f)' % ('ok' if ok else 'NOT OK', BELLY))
    return ok


def antenna_to_wheel(L):
    """Nearest distance of the Wi-Fi antenna strip to the aluminium wheel over the travel (the wheel is a cylinder across y)."""
    ax, ay, _ = P.antenna_pose()
    z_lo, z_hi = P.ANTENNA['z']
    a = math.radians(P.ANTENNA['angle'])
    ux, uy = math.cos(a), math.sin(a)
    vx, vy = -uy, ux
    corners = [(ax + sx * P.ANTENNA['t'] / 2 * ux + sy * P.ANTENNA['w'] / 2 * vx, ay + sx * P.ANTENNA['t'] / 2 * uy + sy * P.ANTENNA['w'] / 2 * vy) for sx in (1, -1) for sy in (1, -1)]
    best = 1e9
    for k in range(0, 101):
        zc = 3.0 + TRAVEL * k / 100
        xc = axle_x(L, zc)
        for x, y in corners:
            for z in (z_lo, z_hi):
                rho = math.hypot(x - xc, z - zc)
                best = min(best, math.hypot(max(rho - R_WHEEL, 0.0), max(L['y_wheel'][0] - y, 0.0, y - L['y_wheel'][1])))
    return best


def clearances_new(L):
    """What sits next to the new arm and pivot (plan view and the side view)."""
    M = P.OMNI_MOUNT
    fp = P.FLOOR_FRONT
    hole_y0 = fp['y'] - (fp['w'] + 0.1) / 2
    slot_y1 = P.OMNI_ARM_SLOT_Y[1]
    print('--- neighbours of the new layout')
    print('  arm slot y %.2f to %.2f, floor port (the 20.3 mm board, moved out to y %.1f) hole from y %.2f: floor wall %.1f mm' % (P.OMNI_ARM_SLOT_Y[0], slot_y1, fp['y'], hole_y0, (hole_y0 - slot_y1) * 10))
    bx, by, ba = P.battery_pose()
    ca, sa = math.cos(math.radians(ba)), math.sin(math.radians(ba))
    corners = [(bx + sx * 3.5 * ca - sy * 1.75 * sa, by + sx * 3.5 * sa + sy * 1.75 * ca) for sx in (1, -1) for sy in (1, -1)]
    cmin = min(corners, key=lambda c: c[1])
    near = [(px, by + (px - bx) * math.tan(math.radians(ba)) - 1.75 / ca) for px in (L['px'] - 0.7, L['px'], L['px'] + 0.7)]
    print('  battery (tray underside z %.1f): its near edge is at y %.2f..%.2f over the pivot x range %.2f..%.2f, lowest corner (%.2f, %.2f); the outer ear stands at y %.2f..%.2f, top z %.2f: %.1f mm under the tray' %
          (Z_TRAY, near[0][1], near[2][1], L['px'] - 0.7, L['px'] + 0.7, cmin[0], cmin[1], M['ear']['y'][0], M['ear']['y'][1], M['ear']['z'][1], (Z_TRAY - M['ear']['z'][1]) * 10))
    print('  left drive motor body x up to 1.0 (y from 1.4): the outer ear starts at x %.2f (%.1f mm clear in x); the pillar at y %.2f to %.2f is clear of the motor in y' % (M['ear']['x'][0], (M['ear']['x'][0] - 1.0) * 10, M['pillar']['y'][0], M['pillar']['y'][1]))
    xc, zc = P.omni_at(TRAVEL)
    rear = []
    for z in (4.0, 4.5, 5.0, 5.2):
        for cx, cz in (P.OMNI['rest'], (xc, zc)):
            if abs(z - cz) < R_WHEEL:
                rear.append((cx - math.sqrt(R_WHEEL ** 2 - (z - cz) ** 2)) - M['pillar']['x'][1])
    print('  the wheel\'s rear surface (rest and compressed, z 4.0 to 5.2) against the pillar\'s front face at x %.2f: %.1f mm at the closest' % (M['pillar']['x'][1], min(rear) * 10))
    print('  Wi-Fi antenna on the wall at %.1f degrees: nearest point of the aluminium wheel %.1f mm over the travel (a metal body within a few mm detunes it: the antenna moved from -6 to %.1f degrees; at -6 it was 2.8 mm)' %
          (P.ANTENNA['angle'], antenna_to_wheel(L) * 10, P.ANTENNA['angle']))


def stop_report(L):
    M = P.OMNI_MOUNT
    S = M['stop']
    sx, sz = P.omni_stop_pin()
    slot = P.omni_stop_slot()
    th_rest = slot['a_hi'] - math.degrees((S['play'] - (S['hw'] - S['pin_r'])) / S['r'])
    th_full = slot['a_lo'] + math.degrees((S['play'] - (S['hw'] - S['pin_r'])) / S['r'])
    reach = max(abs(S['r'] * math.sin(math.radians(a))) for a in (slot['a_lo'], slot['a_hi'])) + slot['hw']
    print('--- stop: a fixed M3 x %.0f screw at (%.3f, %.3f), %.0f mm ahead of the pivot (%.1f deg above it), head on the pillar\'s inboard face, through the pillar (hole %.1f mm), across the pocket and the arm\'s slot, '
          'into the ear (%.1f mm pilot hole, %.1f mm deep)' % (S['screw_len'] * 10, sx, sz, S['r'] * 10, P.omni_stop_psi(), 2 * S['hole_r'] * 10, 2 * S['ear_hole_r'] * 10, S['ear_depth'] * 10))
    print('  the arm\'s arc slot: radius %.0f mm about the pivot, %.1f mm wide, from %.1f deg (compression stop) to %.1f deg (rest stop) of the round-end centres, the screw sits at %.1f deg at rest and %.1f deg fully compressed; '
          'material left between the slot and the arm\'s edge %.1f mm, to the pivot bore %.1f mm' %
          (S['r'] * 10, 2 * S['hw'] * 10, slot['a_lo'], slot['a_hi'], th_rest, th_full, (P.OMNI['arm_half'] - reach) * 10, (S['r'] - S['hw'] - P.OMNI['pivot_hole_r']) * 10))
    amp = (L['arm_len'] * math.cos(math.radians(L['phi_rest']))) / S['r']
    print('  a slot end that is 0.2 mm off moves the wheel %.1f mm (the lever is %.1f times the stop radius): file or shim the rest end when the ride height is measured' % (0.2 * amp, amp))
    print('  coil clearance: the screw\'s surface is %.1f mm outside the coil\'s outer surface (radius %.1f mm)' % ((S['r'] - S['pin_r'] - P.OMNI_SPRING['od'] / 2) * 10, P.OMNI_SPRING['od'] / 2 * 10))


def spring(W_kg, x_com, h_com, L, quiet=False):
    """What the spring has to do (the ramp statics of statics.py) and a wound spring for it; returns the numbers."""
    W = W_kg * 9.81
    omni_x = L['rest_x']
    f_level = x_com / omni_x * W
    f_ramp = (x_com + h_com * math.tan(math.radians(25))) / omni_x * W
    f0 = max(1.2 * f_level, f_level + 0.3)                                  # preload at rest: a margin over the level load, so that the arm sits on its stop
    f_full = 1.08 * f_ramp                                                  # 8 % over the ramp load at full travel: the spring must not bottom out on the ramp
    k = (f_full - f0) / TRAVEL
    lever0 = L['arm_len'] * math.cos(math.radians(L['phi_rest']))
    lever1 = L['arm_len'] * math.cos(math.radians(L['phi_full']))
    m0, m1 = f0 * lever0 * 10, f_full * lever1 * 10                         # N.mm
    dtheta = math.radians(L['phi_full'] - L['phi_rest'])
    k_theta = (m1 - m0) / dtheta
    S = P.OMNI_SPRING
    E, uts = S['E'], S['uts']
    d, D = S['wire'] * 10, S['mean_d'] * 10                                 # mm
    c = D / d
    ki = (4 * c * c - c - 1) / (4 * c * (c - 1))
    na = E * d ** 4 / (64.1 * D * k_theta)                                  # active turns the rate needs
    leg1, leg2 = S['legs'][0] * 10, S['legs'][1] * 10
    nb = na - (leg1 + leg2) / (3 * math.pi * D)
    sigma = 32 * m1 * ki / (math.pi * d ** 3)
    sigma_pre = 32 * m0 * ki / (math.pi * d ** 3)
    pre_deg = math.degrees(m0 / k_theta)
    k_chosen = E * d ** 4 / (64.1 * D * S['active_turns'])                  # N.mm/rad of the spring that v3_params4 describes
    pre_deg_chosen = math.degrees(m0 / k_chosen)                            # opening of the coil at rest if the preload torque m0 is to be reached with it
    out = dict(f0=f0, f_full=f_full, k=k, m0=m0, m1=m1, k_theta=k_theta, sigma=sigma, uts=uts, f_ramp=f_ramp, f_level=f_level, na=na, nb=nb, ki=ki, lever0=lever0, pre_deg=pre_deg,
               k_chosen=k_chosen, pre_deg_chosen=pre_deg_chosen)
    if quiet:
        return out
    print('--- spring: mass %.2f kg (%.1f N), COM %.2f cm ahead of the axle and %.2f cm high, omni %.2f cm ahead: front load %.1f %% = %.2f N level; %.1f N descending a 25 degree ramp' %
          (W_kg, W, x_com, h_com, omni_x, 100 * x_com / omni_x, f_level, f_ramp))
    print('  design: preload %.2f N at the axle, %.2f N at full travel (8 %% over the ramp load), rate %.2f N/cm (the 8 Oct spec at 1.07 kg: 1.8)' % (f0, f_full, k))
    print('  lever %.2f cm at rest, %.2f cm at full travel -> torque %.0f N.mm at rest, %.0f N.mm at full travel over %.1f deg: %.0f N.mm/rad (%.2f N.mm per degree)' %
          (lever0, lever1, m0, m1, math.degrees(dtheta), k_theta, k_theta * math.pi / 180))
    print('  spring: music wire %.1f mm, mean coil %.1f mm (OD %.1f, ID %.1f), %.2f active turns = %.2f body turns + legs %.1f and %.1f mm; length close wound %.1f mm; index %.2f, Ki %.3f' %
          (d, D, D + d, D - d, na, nb, leg1, leg2, d * (nb + 1), c, ki))
    print('  chosen in v3_params4: %.2f active turns, %.2f body turns: rate %.0f N.mm/rad (needed %.0f)' % (S['active_turns'], S['body_turns'], E * d ** 4 / (64.1 * D * S['active_turns']), k_theta))
    print('  bending stress %.0f MPa at full travel (%.0f %% of %.0f MPa tensile), %.0f MPa at rest, range %.0f MPa: %s' %
          (sigma, 100 * sigma / uts, uts, sigma_pre, sigma - sigma_pre, 'ok (not over 50 % of the tensile strength)' if sigma <= 0.5 * uts else 'HIGH'))
    print('  preload angle %.1f deg, total working angle %.1f deg; one turn of the M3 adjuster screw (0.5 mm on a %.0f mm leg) = %.2f deg = %.1f N.mm = %.2f N at the axle' %
          (pre_deg, pre_deg + math.degrees(dtheta), leg1, math.degrees(0.5 / leg1), k_theta * 0.5 / leg1, k_theta * 0.5 / leg1 / (lever0 * 10)))
    print('  the rear leg (%.0f mm behind the pivot) rests on the head of an adjuster bolt and is pushed down with %.0f N at rest and %.0f N at full travel; the forward leg (%.0f mm out, on its seat pin) presses the arm with %.0f N at rest and %.0f N at full travel' % (leg1, m0 / leg1, m1 / leg1, leg2, m0 / leg2, m1 / leg2))
    r_c = 2.5
    ratio = lever0 / r_c
    f_c0, f_c1 = f0 * ratio, f_full * ratio
    stroke = TRAVEL * r_c / lever0
    k_c = (f_c1 - f_c0) / stroke
    print('  fallback, a compression spring 25 mm from the pivot pushing down from above: %.1f N preload, %.1f N at full, stroke %.1f mm, rate %.2f N/mm (wire 0.8 mm, OD 9.7, about 9 active coils, free length about 31 mm)' % (f_c0, f_c1, stroke * 10, k_c / 10))
    return out


def strength_numbers(s, L=None):
    """Arm twist and stress at the full-travel load for the two arm materials; returns a dict by name."""
    L = L or MODEL[3]
    f = s['f_full']
    f_imp = 2.5 * f
    off = (L['y_arm'][0] + L['y_arm'][1]) / 2 - L['yc']
    res = {}
    for name, t, h, G, yld in (('aluminium 3 x 12', 0.3, 1.2, 26000.0, 240.0), ('PETG 4 x 12', 0.4, 1.2, 720.0, 40.0)):
        t_mm, h_mm = t * 10, h * 10
        m_bend = f_imp * L['arm_len'] * 10
        sig = 6 * m_bend / (t_mm * h_mm ** 2)
        torque = f * off * 10
        j = 0.281 * h_mm * t_mm ** 3
        res[name] = dict(sigma=sig, yield_=yld, torque=torque, twist_deg=math.degrees(torque * L['arm_len'] * 10 / (G * j)))
    return res


def pivot_loads(L, s):
    """Free-body loads of the arm, the pivot tube and the stop screw. Case A: full travel on the spring. Case B: the wheel hits something at 2.5 times that load with the arm against the compression stop.
    Forces in N, lengths in mm: the forward spring leg presses on the arm at 8 mm, the rear leg on its adjuster 10 mm behind the pivot, the stop screw acts at S['r'] from the pivot."""
    S = P.OMNI_MOUNT['stop']
    f = s['f_full']
    f_imp = 2.5 * f
    lever1 = L['arm_len'] * math.cos(math.radians(L['phi_full'])) * 10
    r_stop = S['r'] * 10
    leg_f, leg_r = P.OMNI_SPRING['legs'][1] * 10, P.OMNI_SPRING['legs'][0] * 10
    F_leg = s['m1'] / leg_f                                  # N: the forward leg on the arm, at full travel
    F_rear = F_leg * leg_f / leg_r                           # N: the rear leg on the adjuster
    R_a = F_leg - f                                          # N: the pivot tube on the arm (up), case A
    F_stop = (f_imp * lever1 - s['m1']) / r_stop             # N: the stop on the arm (down), case B
    R_b = F_leg + F_stop - f_imp                             # N: the pivot tube on the arm, case B
    span_a, span_b = 12.8, 3.7                               # mm: the arm\'s mid-plane from the pillar-side and from the ear-side support of the stop screw (16.5 mm apart)
    m_stop = F_stop * span_a * span_b / (span_a + span_b)    # N.mm: bending of the stop screw at the arm\'s plane, simply supported
    sig_stop = m_stop / (math.pi * 2.39 ** 3 / 32)           # M3 core 2.39 mm
    return dict(f=f, f_imp=f_imp, F_leg=F_leg, F_rear=F_rear, R_a=R_a, F_stop=F_stop, R_b=R_b, m_stop=m_stop, sig_stop=sig_stop, lever1=lever1)


def strength(L, s):
    """Axle, arm, pivot and stop at the full-travel load (a hit at 2.5 times that load as the impact case)."""
    f = s['f_full']
    f_imp = 2.5 * f
    off = (L['y_arm'][0] + L['y_arm'][1]) / 2 - L['yc']
    print('--- strength at %.1f N (impact case %.1f N); the wheel\'s mid-plane is %.1f mm from the arm\'s' % (f, f_imp, off * 10))
    arm_face = 14.0
    sig_axle = 32 * f_imp * arm_face / (math.pi * 4.0 ** 3)
    print('  M4 axle: bending %.0f MPa at the arm face (steel, 8.8 class yield 640 MPa)' % sig_axle)
    for name, r in strength_numbers(s, L).items():
        print('  arm %s: in-plane bending %.0f MPa (yield %.0f), torsion from the wheel offset %.0f N.mm twists the wheel %.2f deg' % (name, r['sigma'], r['yield_'], r['torque'], r['twist_deg']))
    q = pivot_loads(L, s)
    bore_area = 2 * P.OMNI['pivot_hole_r'] * 10 * P.OMNI['arm_t'] * 10                      # mm2: the arm's bore (5 mm) over its 3 mm thickness
    t_od, t_id = 2 * P.OMNI_PIVOT['tube_r'][1] * 10, 2 * P.OMNI_PIVOT['tube_r'][0] * 10
    z_tube = math.pi * (t_od ** 4 - t_id ** 4) / (32 * t_od)                                # mm3
    sig_tube = q['R_b'] * (P.OMNI_PIVOT['tube_len'] * 10) / 4 / z_tube                       # N/mm2: the arm loads the middle of the tube, supported at both ends
    patch = 2 * P.OMNI_MOUNT['hole_r'] * 10 * (P.OMNI_MOUNT['pillar']['y'][1] - P.OMNI_MOUNT['pillar']['y'][0]) * 10
    print('  full travel on the spring: the forward leg presses the arm with %.0f N, the rear leg the adjuster with %.0f N; the arm\'s 5 mm pivot bore carries %.0f N = %.1f MPa on its 5 x 3 mm (aluminium yields at 240)' %
          (q['F_leg'], q['F_rear'], q['R_a'], q['R_a'] / bore_area))
    print('  hit at %.1f N with the arm on its compression stop: the stop screw takes %.0f N (M3 bending %.0f MPa between two supports 16.5 mm apart, 8.8 yields at 640; on the slot end %.1f MPa over 3 x 3 mm), the bore %.0f N = %.1f MPa; '
          'the %.0f x %.1f mm steel tube between pillar and ear bends at %.0f MPa; the pillar and the ear each take about half of that load through the M3 screw: %.1f MPa on a %.1f x 4 mm bearing patch (PETG about 40)' %
          (q['f_imp'], q['F_stop'], q['sig_stop'], q['F_stop'] / 9.0, q['R_b'], q['R_b'] / bore_area, t_od, t_id, sig_tube, q['R_b'] / 2 / patch, 2 * P.OMNI_MOUNT['hole_r'] * 10))


def legs_report(L, s):
    """10 Oct: the forward spring leg, its seat pin on the arm and the arbor under the coil, over the whole travel. Returns the smallest clearances (cm) and the loads in a dict."""
    from shapely.geometry import LineString, Point
    S, K, A, V = P.OMNI_SPRING, P.OMNI_SEAT, P.OMNI_ARBOR, P.OMNI_PIVOT
    r_leg, r_pin, r_stop = S['wire'] / 2, K['r'], P.OMNI_MOUNT['stop']['pin_r']
    sx, sz = P.omni_stop_pin()
    best = dict(over_screw=9.0, pin_screw=9.0, tip_rim=9.0, pin_rim=9.0)
    for k in range(101):
        t = TRAVEL * k / 100
        (x0, z0), (x1, z1) = P.omni_leg(t)
        best['over_screw'] = min(best['over_screw'], Point(sx, sz).distance(LineString([(x0, z0), (x1, z1)])) - r_leg - r_stop)
        cx, cz = P.omni_seat(t)
        best['pin_screw'] = min(best['pin_screw'], math.hypot(cx - sx, cz - sz) - r_pin - r_stop)
        ox, oz = P.omni_at(t)
        best['tip_rim'] = min(best['tip_rim'], math.hypot(x1 - ox, z1 - oz) - r_leg - R_WHEEL)
        best['pin_rim'] = min(best['pin_rim'], math.hypot(cx - ox, cz - oz) - r_pin - R_WHEEL)
    edge = P.OMNI['arm_half'] - (P.omni_seat_n() + r_pin)                                          # bar left above the pin's hole
    lever = K['s'] * 10                                                                            # mm
    force0, force1 = s['m0'] / lever, s['m1'] / lever
    leg_y = S['y'][1] - r_leg                                                                      # the forward leg is the last turn run straight out
    stick = (P.OMNI['arm_y'][0] - leg_y) * 10                                                      # mm from the arm's inboard face to the leg's centre line
    d_pin = 2 * r_pin * 10
    in_bar = (K['y'][1] - P.OMNI['arm_y'][0]) * 10                                                 # mm of the pin inside the bar
    pin_sigma = 32 * force1 * stick / (math.pi * d_pin ** 3)
    work = L['phi_full'] - L['phi_rest']
    n_open = S['body_turns'] - (s['pre_deg_chosen'] + work) / 360.0
    d_open = S['mean_d'] * S['body_turns'] / n_open                                                # cm: the wire keeps its length, the mean diameter grows
    ecc = (d_open - S['wire']) / 2 - A['r']
    top = L['pz'] + ecc + d_open / 2 + S['wire'] / 2
    print("--- forward leg, seat pin and arbor (10 Oct, spec 4.3): the leg's centre line is %.1f mm inboard of the arm's face, %.0f mm above the arm's axis at the coil and %.0f degrees up from it; it rests on a %.0f mm pin %.0f mm out" %
          (stick, S['mean_d'] * 5, S['tilt'], d_pin, lever))
    print("  smallest over the travel: leg over the stop screw %.2f mm, seat pin to the screw %.2f mm, leg tip to the wheel's rim %.2f mm, pin to the rim %.2f mm; %.2f mm of bar over the pin's hole" %
          (best['over_screw'] * 10, best['pin_screw'] * 10, best['tip_rim'] * 10, best['pin_rim'] * 10, edge * 10))
    print("  the pin carries %.0f N at rest and %.0f N at full travel: bending %.0f MPa at its root (stainless yields at about 500), bearing %.1f MPa on %.1f mm in the bar (aluminium yields at 240)" %
          (force0, force1, pin_sigma, force1 / (d_pin * in_bar), in_bar))
    print("  arbor: a printed %.1f mm sleeve on the %.0f mm tube, %.1f mm long (y %.2f to %.2f), under the coil (ID %.1f mm free): %.2f mm radial clearance when free; the load opens the coil to a mean diameter of %.2f mm, "
          "so it can sit up to %.2f mm off-centre (top of the wire at z %.2f, the battery tray's underside %.1f)" %
          (2 * A['r'] * 10, 2 * V['tube_r'][1] * 10, (A['y'][1] - A['y'][0]) * 10, A['y'][0], A['y'][1], S['id'] * 10, (S['id'] / 2 - A['r']) * 10, d_open * 10, ecc * 10, top, P.TRAY['z'][0]))
    best.update(edge=edge, force0=force0, force1=force1, pin_sigma=pin_sigma, d_open=d_open, ecc=ecc)
    return best


def sheet_numbers(L, s):
    """The numbers of the spring order sheet (spec 4.3, Figure 14), quietly. Angles in degrees, seen from the right side of the robot (x forward to the right, z up; counterclockwise = up at the front). The rear
    leg points back (180) and is held; the forward leg lies along the arm and turns with it. `pre` is how far the coil is opened at rest, `work` the further opening over the travel; the free coil's forward leg
    sits `pre` clockwise of where the arm holds it at rest."""
    S = P.OMNI_SPRING
    work = L['phi_full'] - L['phi_rest']
    pre = s['pre_deg_chosen']
    k_deg = s['k_chosen'] * math.pi / 180.0
    fwd_rest = L['phi_rest'] + S['tilt']                                                           # direction of the forward leg at rest (0 = forward, counterclockwise = up)
    fwd_free = fwd_rest - pre                                                                      # in the free coil
    free_angle = (180.0 - fwd_free) % 360.0                                                        # from the free forward leg to the rear leg, counterclockwise
    n_open = S['body_turns'] - (pre + work) / 360.0
    d_open = S['mean_d'] * S['body_turns'] / n_open                                                # cm, mean diameter at full travel
    return dict(free_angle=free_angle, pre=pre, work=work, k_deg=k_deg, fwd_rest=fwd_rest, fwd_free=fwd_free, fwd_full=fwd_rest + work, m_rest=k_deg * pre, m_full=k_deg * (pre + work),
                d_open=d_open, id_open=d_open - S['wire'], od_open=d_open + S['wire'], k=s['k_chosen'], sigma=s['sigma'], sigma_pct=100 * s['sigma'] / S['uts'])


def spring_sheet(L, s):
    """The spring as it would be ordered (spec 4.3, Figure 14): what the maker needs, from v3_params4 and the statics."""
    S = P.OMNI_SPRING
    out = sheet_numbers(L, s)
    phi, work, pre, k_deg, fwd_rest, free_angle = L['phi_rest'], out['work'], out['pre'], out['k_deg'], out['fwd_rest'], out['free_angle']
    print("--- spring order sheet: music wire (ASTM A228 / EN 10270-1 SH) %.2f mm; mean coil diameter %.1f mm (ID %.1f, OD %.1f when free); %.2f body turns, close wound, body length %.1f mm; RIGHT-HAND wound" %
          (S['wire'] * 10, S['mean_d'] * 10, S['id'] * 10, S['od'] * 10, S['body_turns'], S['length'] * 10))
    print("  legs, both straight and tangent to the coil, leaving its top, one at each end of the body: REAR pointing back (horizontal), %.0f mm from the tangent point to where it lies on the adjuster head, %.0f mm in all; "
          "FORWARD %.0f degrees up from the direction of the arm (%.1f degrees below horizontal installed at rest), %.0f mm to its seat pin, %.0f mm in all; bend radius at the roots 1.6 to 2.4 mm, ends rounded" %
          (S['legs'][0] * 10, S['legs'][0] * 10 + 2, S['tilt'], -fwd_rest, S['legs'][1] * 10, (S['legs'][1] + S['tip']) * 10))
    print("  free angle between the legs %.1f degrees (counterclockwise from the forward leg to the rear leg, seen from the right with the forward leg at the far end) = %d whole turns and %.1f degrees; installed at rest the coil is "
          "opened by %.1f degrees and at full travel by %.1f degrees" % (free_angle, int(S['body_turns']), free_angle - 180.0, pre, pre + work))
    print("  rate %.0f N.mm/rad = %.2f N.mm per degree (+-8 %%; %.0f needed); torque %.0f N.mm at %.1f degrees of opening and %.0f N.mm at %.1f degrees (+-10 %%); the load OPENS the coil; stress at full travel %.0f MPa = %.0f %% of the wire's strength" %
          (s['k_chosen'], k_deg, s['k_theta'], k_deg * pre, pre, k_deg * (pre + work), pre + work, s['sigma'], 100 * s['sigma'] / S['uts']))
    return out


def statics_report(W_kg, x_com, h_com, L):
    """The statics.py numbers (the omni pushed over a step by the drive wheels, motor torque, the ramp) for the 9 Oct model: statics.py itself keeps the rev 3 budget of 1.07 kg."""
    W = W_kg * 9.81
    front = x_com / L['rest_x']
    print('--- statics of the 9 Oct model (%.3f kg, COM %.2f cm ahead of the axle, %.2f cm high, omni %.2f cm ahead): front load %.1f %%' % (W_kg, x_com, h_com, L['rest_x'], 100 * front))
    for h in (1.0, 2.0):
        a = math.acos((3.0 - h) / 3.0)
        need, have = front * math.tan(a), (1 - front) * 1.5
        print('  omni over a %.0f cm step, pushed by the drive wheels (traction 1.5 assumed): push needed %.2f W, traction %.2f W, margin x%.1f' % (h, need, have, have / need))
    ww = (1 - front) / 2 * W
    tau_step = ww * 0.04 * math.sin(math.acos((4.0 - 2.0) / 4.0))
    tau_ramp = (W * math.sin(math.radians(25)) / 2) * 0.04
    print('  motor torque per wheel: 2 cm riser %.2f kg.cm, 25 degree ramp %.2f kg.cm (gearbox limit 5.0)' % (tau_step * 10.197, tau_ramp * 10.197))
    need_com = h_com * math.tan(math.radians(25))
    print('  25 degree ramp: no rock-back needs the COM %.1f cm ahead (front load %.0f %%); descending it the front load is %.0f %% = %.1f N' %
          (need_com, 100 * need_com / L['rest_x'], 100 * (x_com + need_com) / L['rest_x'], (x_com + need_com) / L['rest_x'] * W))


if __name__ == '__main__':
    W = P.OMNI_WHEEL
    print('wheel: %s, %.0f mm x %.1f mm, bore %.0f mm, %.0f g (the 8 Oct budget: 30 g)' % (W['name'], W['r'] * 20, W['w'] * 10, W['bore_r'] * 20, W['mass_g']))
    lo, ln = layout(OLD), layout(NEW)
    report_layout(lo)
    report_layout(ln)
    pivot_checks(lo)
    pivot_checks(ln)
    clearances_new(ln)
    stop_report(ln)
    s = spring(*MODEL)
    legs_report(ln, s)
    spring_sheet(ln, s)
    strength(ln, s)
    statics_report(*MODEL)
