"""Mounting check (mechanical design 2026-10-07, sections 5 and 8): where the six GIGA R1 mounting holes land in the robot frame against the parts around them,
how long the ToF cable runs are, and beam numbers for the motor snap cradle. cm unless noted. Needs shapely.
Hole positions: Arduino GIGA R1 datasheet, section 15 (Mounting Holes And Board Outline), board 101.6 x 53.34 mm: six holes of 3.2 mm in the Mega 2560 pattern."""
import math

from shapely.geometry import Point, LineString, box

# datasheet p.18: holes (u from the left/connector edge, v from the top long edge), mm
HOLES = {'H1 (15.24, 2.54)': (15.24, 2.54), 'H2 (90.17, 2.54)': (90.17, 2.54), 'H3 (13.9, 50.7)': (13.9, 50.7), 'H4 (96.7, 50.7)': (96.7, 50.7),
         'H5 (66.1, 17.8)': (66.1, 17.8), 'H6 (66.1, 45.6)': (66.1, 45.6)}
STACK_X0, STACK_X1, STACK_Y0, STACK_Y1 = 2.32 - 5.076, 2.32 + 5.076, -4.03 - 2.667, -4.03 + 2.667     # packing (pack_v3.json): x -2.76..7.40, y -6.70..-1.36
BOSS_R = 0.35                                                                                       # M3 heat-set boss, 7 mm across
OMNI_X = 7.0                                                                                        # omni centre in the mechanical design (rev 3: 7.95)
PIV_X = OMNI_X - math.sqrt(4.4905 ** 2 - 0.36)                                                      # arm length kept at 4.49

def robot_xy(u, v, connector_front=True, top_inboard=True):
    x = (STACK_X1 - u / 10.0) if connector_front else (STACK_X0 + u / 10.0)
    y = (STACK_Y1 - v / 10.0) if top_inboard else (STACK_Y0 + v / 10.0)
    return x, y

KEEP = {
    'right motor body': box(-1.0, -6.1, 1.0, -1.4),
    'right motor prongs': box(-1.30, -6.05, 1.30, -2.85),
    'right motor ledges': box(-1.6, -6.05, 2.75, -5.65),                 # front ledge x 1.06 .. 2.75 carries the plate ear, rear ledge only x -1.6 .. -1.06 (the GIGA post H4 is at x -2.27)
    'right motor web and plate slot': box(-1.6, -6.7, 2.75, -6.05),
    'omni arm swing, two plates': box(PIV_X - 0.5, -1.7, OMNI_X + 0.6, 1.7),
    'omni arm swing, one plate on +y': box(PIV_X - 0.5, 1.0, OMNI_X + 0.6, 1.7),
    'omni wheel': box(OMNI_X - 3.0, -1.05, OMNI_X + 3.0, 1.05),
    'silver module SM': box(-1.0, -1.1, 1.0, 1.1),
    'chute A channel': LineString([(-4.73, -2.73), (-6.0, -8.62)]).buffer(1.10),                      # the 18 mm square channel: outer 21.2 mm plus 0.4 mm
}

if __name__ == '__main__':
    for conn_front in (True, False):
        for top_in in (True, False):
            print(f'\nconnector edge toward the {"front" if conn_front else "rear"}, top (header) edge on the {"inboard" if top_in else "outboard"} side')
            usable, clear = [], []
            for nm, (u, v) in HOLES.items():
                x, y = robot_xy(u, v, conn_front, top_in)
                gap = {kn: (Point(x, y).distance(kp) - BOSS_R) * 10 for kn, kp in KEEP.items()}                  # mm from the boss edge to each keep-out
                hits = [f'{kn} ({g:+.1f} mm)' for kn, g in gap.items() if g < 1.5]
                print(f'  {nm:18s} -> ({x:6.2f}, {y:6.2f})  ' + ('; '.join(hits) if hits else 'clear'))
                single = {kn: g for kn, g in gap.items() if kn != 'omni arm swing, two plates'}                  # with the arm on one side only (+y)
                if min(single.values()) >= 0: usable.append(nm.split()[0])
                if min(single.values()) >= 1.5: clear.append(nm.split()[0])
            xs = [robot_xy(*HOLES[k], conn_front, top_in)[0] for k in HOLES if k.split()[0] in usable]
            print(f'  -> with the omni arm on +y only: usable {" ".join(usable)} (n={len(usable)}), at least 1.5 mm clear: {" ".join(clear)}; usable holes span x {min(xs):.1f} .. {max(xs):.1f} cm' if xs else '  -> no usable hole')
    # USB-C J12 sits about 13.8 mm from the top edge on the connector edge (datasheet p.9, read from the picture)
    for top_in in (True, False):
        y = STACK_Y1 - 1.38 if top_in else STACK_Y0 + 1.38
        print(f'\nUSB-C J12 at x {STACK_X1:.2f}, y {y:.2f} (top edge {"inboard" if top_in else "outboard"}); free space in front of the stack edge: to the tub wall (r 10.3, below z 8.7) {math.sqrt(10.3 ** 2 - y * y) - STACK_X1:.2f} cm, to the ring (r 9.0, above z 8.7) {math.sqrt(81 - y * y) - STACK_X1:.2f} cm')

    # ToF cable runs: module to the nearest edge of the GIGA stack in plan, routed estimate = x1.3 + 3 cm vertical
    fixed = {'F': (9.5, 0.0), 'FL': (8.97, 4.18), 'FR': (8.97, -4.18), 'SFL': (7.18, 6.0), 'SFR': (7.18, -6.0), 'SRL': (-7.18, 6.0), 'SRR': (-7.18, -6.0), 'RL': (-8.4, 4.5), 'RR': (-8.4, -4.5)}
    stack = box(STACK_X0, STACK_Y0, STACK_X1, STACK_Y1)
    print('\nToF module to the nearest edge of the GIGA stack (plan) and a routed estimate (x1.3 + 3 cm):')
    worst, short = 0, 0
    for nm, (x, y) in fixed.items():
        d = Point(x, y).distance(stack); est = d * 1.3 + 3.0
        worst = max(worst, est); short += est <= 10.0
        print(f'  {nm:4s} plan {d:5.1f} cm   routed about {est:5.1f} cm')
    print(f'longest routed run about {worst:.1f} cm; 100 mm cables are long enough for {short} of 9, 200 mm cables for all')

    # motor snap prongs (PETG, E 2.0 GPa in the layer plane): thin vertical walls cantilevered along the motor axis from the face-plate web, so they bend in the layer plane.
    # A hook at height z_lip above the floor sits inside the motor silhouette by the undercut; to pass the equator the prong must deflect from the hook tip to the motor radius.
    E, MU, RM, ZC = 2000.0, 0.35, 10.0, 40.0          # MPa, friction, motor radius mm, motor axis height mm above the floor underside datum (z 4.0 cm)
    def prong(t, b, L, z_lip_mm, undercut=0.75):
        dz = z_lip_mm - ZC; half = math.sqrt(RM ** 2 - dz ** 2)                     # silhouette half-width at the hook height
        tip = half - undercut; d_pass = RM - tip                                    # deflection needed to pass the equator
        I = b * t ** 3 / 12.0
        F = lambda d: 3 * E * I * d / L ** 3                                        # force at deflection d
        eps = 3 * t * d_pass / (2 * L ** 2)
        phi = math.asin(dz / RM); tp = math.tan(phi)                                # contact normal angle above horizontal
        pull = 2 * F(undercut) * (tp + MU) / (1 - MU * tp)                          # force to lift the motor, two prongs, with friction
        fric = 2 * MU * F(undercut) / math.cos(phi) * RM / 98.07                    # kg.cm of friction torque from the two preloaded hooks
        return d_pass, F(d_pass), eps, pull, fric
    print('motor snap prongs (thin walls along the axis, 0.75 mm undercut): thickness t, height b, free length L, hook height above the axis')
    for t, b, L, zl in ((1.6, 11, 32, 44.5), (2.0, 11, 32, 44.5), (2.4, 11, 32, 45.5), (2.4, 11, 40, 47.0), (2.9, 11, 40, 47.0)):
        d, Fp, eps, pull, fr = prong(t, b, L, zl)
        print(f'  t {t} b {b} L {L} hook {zl - ZC:.1f} mm above the axis: deflection to pass the equator {d:.2f} mm, push-in {Fp:.1f} N per prong, strain {eps * 100:.2f} %, pull-out {pull:.1f} N (both prongs), friction torque {fr:.2f} kg.cm')
    print('  (smooth cylinder: the hook can only hold a few N; ears in grooves take the torque. Pull-out is estimated from the wedge angle at the hook, friction 0.35)')
    print('  ear: gearbox limit 5 kg.cm = 490 N.mm over ONE front ear at about 19.5 mm from the axle: %.1f N; bearing area 8 x 3 mm = 24 mm^2: %.2f MPa' % (490 / 19.5, 490 / 19.5 / 24))
    print('torque needed: 1.63 kg.cm (2 cm riser), gearbox limit 5 kg.cm, motor stall 10 kg.cm (Pololu 3493)')
    print('cube 10.3 mm: face diagonal %.2f mm, space diagonal %.2f mm; 16 mm slot and hopper void: %.2f mm of slack over the face diagonal; 18 mm bore: %.2f mm over the space diagonal, '
          '%.2f mm per side for a cube lying square (first design, 13 mm bore: %.2f mm in total, and kits that arrived turned jammed in it)' % (10.3 * math.sqrt(2), 10.3 * math.sqrt(3), 16 - 10.3 * math.sqrt(2), 18 - 10.3 * math.sqrt(3), (18 - 10.3) / 2, 13 - 10.3))
