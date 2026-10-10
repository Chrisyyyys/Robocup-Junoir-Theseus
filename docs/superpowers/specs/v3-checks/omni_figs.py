"""Figure 12 of the mechanical design (2026-10-07-theseus-v3-mechanical-design.md, section 4): how the front omni wheel is mounted.
python omni_figs.py [outdir]   (default ../v3-figures). Needs matplotlib. All numbers come from v3_params4 (the one source of the Fusion model); cm unless a label says mm.
A: side view through the arm (x-z), rest and fully compressed, with the parts list below it. B: front view through the axle (y-z). C: section through the pivot (y-z). D: the arm with its holes and the arc slot (arm frame)."""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MPoly

import v3_params4 as P

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'v3-figures')
os.makedirs(OUT, exist_ok=True)
C = dict(floor='#C8C6BE', print_='#D9D7CE', arm='#0F6E56', wheel='#1D9E75', steel='#5F5E5A', spring='#D85A30', bear='#8A8A84', batt='#EF9F27', fp='#F0997B', sleeve='#B7B6B0')
O, W, B, X, V, M, SP = P.OMNI, P.OMNI_WHEEL, P.OMNI_BEARING, P.OMNI_AXLE, P.OMNI_PIVOT, P.OMNI_MOUNT, P.OMNI_SPRING
K, AB = P.OMNI_SEAT, P.OMNI_ARBOR
ox, oz = O['rest']
px, pz = O['pivot']
w0, w1 = O['y_wheel']
a0, a1 = O['arm_y']
S = M['stop']
stx, stz = P.omni_stop_pin()
slot = P.omni_stop_slot()
xc, zc = P.omni_at(O['travel'])
phi0 = math.atan2(oz - pz, ox - px)
phi1 = math.atan2(zc - pz, xc - px)
zf, zb = P.Z_FLOOR_TOP, P.Z_BELLY
BOX = dict(fc='white', ec='none', pad=0.8, alpha=0.9)
FS = 8.5


def save(fig, name, dpi=130):
    for ext in ('png', 'svg'):
        fig.savefig(os.path.join(OUT, name + '.' + ext), dpi=dpi)
    plt.close(fig)
    print('wrote', os.path.join(OUT, name + '.png'))


def poly(ax, pts, **kw):
    ax.add_patch(MPoly(pts, closed=True, **kw))


def rect(ax, x0, y0, x1, y1, **kw):
    poly(ax, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], **kw)


def note(ax, text, xy, xytext, fs=FS, **kw):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=fs, arrowprops=dict(arrowstyle='-', lw=0.6, color='#333'), bbox=BOX, **kw)


def stadium_pts(p1, p2, r, n=16):
    a = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    pts = [(p2[0] + r * math.cos(a - math.pi / 2 + math.pi * k / n), p2[1] + r * math.sin(a - math.pi / 2 + math.pi * k / n)) for k in range(n + 1)]
    pts += [(p1[0] + r * math.cos(a + math.pi / 2 + math.pi * k / n), p1[1] + r * math.sin(a + math.pi / 2 + math.pi * k / n)) for k in range(n + 1)]
    return pts


def slot_pts(ph, frame_origin=(px, pz)):
    """Polygon of the arc slot with the arm's axis at angle ph (radians) about frame_origin."""
    r, hw = slot['r'], slot['hw']
    lo, hi = math.radians(slot['a_lo']), math.radians(slot['a_hi'])
    ex, ez = frame_origin

    def pt(rad, a):
        return (ex + rad * math.cos(ph + a), ez + rad * math.sin(ph + a))

    n = 14
    pts = [pt(r + hw, lo + (hi - lo) * k / n) for k in range(n + 1)]
    cx, cz = pt(r, hi)
    for k in range(1, 8):
        s = math.pi * k / 8
        pts.append((cx + hw * (math.cos(s) * math.cos(ph + hi) - math.sin(s) * math.sin(ph + hi)), cz + hw * (math.cos(s) * math.sin(ph + hi) + math.sin(s) * math.cos(ph + hi))))
    pts += [pt(r - hw, hi - (hi - lo) * k / n) for k in range(n + 1)]
    cx, cz = pt(r, lo)
    for k in range(1, 8):
        s = math.pi * k / 8
        pts.append((cx - hw * (math.cos(s) * math.cos(ph + lo) - math.sin(s) * math.sin(ph + lo)), cz - hw * (math.cos(s) * math.sin(ph + lo) + math.sin(s) * math.cos(ph + lo))))
    return pts


PARTS = [
    ('Wheel', 'Nexus 14145 omni, 60 mm, 25.6 mm wide, 12 mm hole, 73 g', 1),
    ('Bearings', '604-2RS, 4 x 12 x 4 mm, in the wheel\'s hole: the outboard one flush with the face, the inboard one 5 mm in', 2),
    ('Axle', 'M4 x 25 screw and 0.5 mm washer in the bore, 6/4.1 x 12.6 sleeve, 6/4.1 x 1 spacer; thread-locker', 1),
    ('Arm', '3 x 12 mm 6061 flat bar, 58 mm long, M4 tapped, 5.05 mm bore, arc slot, seat hole: Figure 13 and omni_arm.dxf', 1),
    ('Pivot', 'M3 x 25 socket screw, 5/3.1 x 13.5 mm steel tube, M3 x 5.7 heat-set insert in the ear', 1),
    ('Stop', 'M3 x 25 socket screw (the same as the pivot screw) into a 2.6 mm pilot hole in the ear', 1),
    ('Seat pin', 'stainless dowel pin ISO 2338 2 m6 x 6 in the arm (retaining compound): the forward spring leg rests on it', 1),
    ('Spring', 'torsion, music wire 1.6 mm, coil 10 mm, 4.07 turns, right-hand, legs 10 and 11 mm (custom: Figure 14)', 1),
    ('Arbor', 'printed sleeve 7.4 / 5.2 x 9 mm on the pivot tube, under the coil (in the tub print\'s folder, part \'Spring arbor\')', 1),
    ('Adjuster', 'M3 x 6 hex bolt (head 5.5 across flats) and M3 x 5.7 heat-set insert in the pillar\'s tail', 1),
    ('Printed', 'pillar, tail and outer ear on the tub floor (part of the tub print)', 1),
]


def fig_omni():
    fig = plt.figure(figsize=(21.0, 15.2))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1.0], height_ratios=[1.0, 1.0, 0.92], wspace=0.05, hspace=0.12)

    # ------------------------------------------------------------------ A: side view through the arm
    ax = fig.add_subplot(gs[0:2, 0])
    ax.set_aspect('equal'); ax.set_xlim(-1.3, 11.6); ax.set_ylim(-0.4, 9.5); ax.axis('off')
    ax.set_title('A  Side view through the arm (x forward, z up). Filled: at rest. Dashed: fully compressed (2.5 cm of travel).', fontsize=11, loc='left')
    ax.plot([-1.3, 11.6], [0, 0], color='k', lw=1.2)
    ax.text(11.5, 0.1, 'ground', fontsize=8, ha='right', color='#555')
    rect(ax, -1.3, zb, 1.659, zf, fc=C['floor'], ec='k', lw=0.8)                                    # floor behind the opening
    rect(ax, 9.87, zb, 10.3, zf, fc=C['floor'], ec='k', lw=0.8)
    rect(ax, 10.3, zb, 10.5, 8.7, fc=C['floor'], ec='k', lw=0.8)                                    # wall
    ax.plot([-1.3, 10.5], [zb, zb], color='#888', ls=':', lw=0.9)
    ax.text(-1.25, zb - 0.1, 'belly line z 3.5', fontsize=8, va='top', color='#555')
    ax.plot([0.4, 7.4], [P.TRAY['z'][0]] * 2, color=C['batt'], lw=2.4)
    ax.text(7.45, P.TRAY['z'][0], 'battery tray, underside z 4.9', fontsize=8, va='center', color='#7a4d00')
    ax.plot([5.4, 10.3], [P.FRAME['z0']] * 2, color='#444', lw=2.4)
    ax.text(5.4, P.FRAME['z0'] + 0.1, 'front bridge underside z 8.7', fontsize=8)
    # pillar and tail (behind the plane), ear (in front)
    pl, ea, tl = M['pillar'], M['ear'], M['tail']
    rect(ax, pl['x'][0], pl['z'][0], pl['x'][1], pl['z'][1], fc=C['print_'], ec='#444', lw=0.9, ls='--', alpha=0.7)
    rect(ax, tl['x'][0], tl['z'][0], tl['x'][1], tl['z'][1], fc=C['print_'], ec='#444', lw=0.9, ls='--', alpha=0.7)
    rect(ax, ea['x'][0], ea['z'][0], ea['x'][1], ea['z'][1], fc='none', ec='#444', lw=1.0, ls=':')
    # wheel
    ax.add_patch(Circle((ox, oz), O['r'], fc=C['wheel'], alpha=0.30, ec=C['wheel'], lw=1.4))
    ax.add_patch(Circle((xc, zc), O['r'], fill=False, ec=C['wheel'], ls='--', lw=1.3))
    ax.add_patch(Circle((ox, oz), W['bore_r'], fc='white', ec='k', lw=0.7))
    # arm: rest filled, compressed outline
    poly(ax, stadium_pts((px, pz), (ox, oz), O['arm_half']), fc=C['arm'], alpha=0.8, ec='k', lw=0.9)
    poly(ax, stadium_pts((px, pz), (xc, zc), O['arm_half']), fill=False, ec=C['arm'], ls='--', lw=1.2)
    poly(ax, slot_pts(phi0), fc='white', ec='k', lw=0.7)
    poly(ax, slot_pts(phi1), fill=False, ec='k', ls='--', lw=0.7)
    ax.add_patch(Circle((px, pz), O['pivot_hole_r'], fc='white', ec='k', lw=0.6))
    ax.add_patch(Circle((ox, oz), O['axle_r'], fc=C['steel'], ec='k', lw=0.6))
    # spring: coil, rear leg, adjuster
    ax.add_patch(Circle((px, pz), AB['r'], fc='#E4E2D8', ec='#888', lw=0.8))                      # the printed arbor under the coil
    ax.add_patch(Circle((px, pz), SP['od'] / 2, fill=False, ec=C['spring'], lw=1.8))
    ax.add_patch(Circle((px, pz), SP['id'] / 2, fill=False, ec=C['spring'], lw=1.0))
    ax.add_patch(Circle((px, pz), O['pivot_hole_r'], fc='white', ec='k', lw=0.6))
    ad = M['adjuster']
    zl = pz + SP['mean_d'] / 2
    ax.plot([px - 0.46, ad['x'] - 0.2], [zl, zl], color=C['spring'], lw=2.4)                        # rear leg, straight, 2 mm past the middle of the adjuster head
    rect(ax, ad['x'] - ad['head_r'], ad['head_z'][0], ad['x'] + ad['head_r'], ad['head_z'][1], fc=C['steel'], ec='k', lw=0.6)
    ax.plot([ad['x'], ad['x']], [ad['head_z'][0] - ad['bolt_len'], ad['head_z'][0]], color=C['steel'], lw=2.5)
    ax.add_patch(Circle((stx, stz), S['pin_r'], fc=C['steel'], ec='k', lw=0.7))
    (lx0, lz0), (lx1, lz1) = P.omni_leg(0.0)                                                         # forward leg and its seat pin, rest and compressed
    (cx0, cz0), (cx1, cz1) = P.omni_leg(O['travel'])
    ax.plot([lx0, lx1], [lz0, lz1], color=C['spring'], lw=2.4)
    ax.plot([cx0, cx1], [cz0, cz1], color=C['spring'], lw=1.4, ls='--')
    ax.add_patch(Circle(P.omni_seat(0.0), K['r'], fc=C['steel'], ec='k', lw=0.6))
    ax.add_patch(Circle(P.omni_seat(O['travel']), K['r'], fill=False, ec='k', lw=0.6, ls='--'))
    # labels (left column of free space, top to bottom)
    note(ax, 'torsion spring on a printed 7.4 mm arbor on the pivot tube: music wire\n1.6 mm, coil OD 11.6 mm, 4.07 turns right-hand, 493 N.mm/rad;\n2.4 N preload, 8.5 N at full travel', (px - 0.4, pz + 0.4), (-1.2, 9.2), va='top')
    note(ax, 'arm: 3 x 12 mm aluminium, 46 mm hole to hole;\n-15.1 degrees at rest, +16.4 degrees compressed', (px + 1.6, pz + 0.05 - 0.27 * 1.6 + 0.3), (-1.2, 8.1), va='top')
    note(ax, 'rear leg (10 mm) lies on the head of an M3 adjuster bolt\nin the pillar\'s tail: one turn = 2.9 degrees = 0.55 N at the axle', (ad['x'], zl), (-1.2, 7.0), va='top')
    note(ax, 'forward leg (11 mm) lies along the arm, 4 degrees up from it, on a 2 mm\npin in the arm (dashed: fully compressed): it turns with the arm', (P.omni_seat(0.0)[0], P.omni_seat(0.0)[1] + K['r'] + SP['wire'] / 2), (-1.2, 6.1), va='top')
    note(ax, 'pivot: M3 screw (head on the pillar, from inboard) through\na 5 x 3.1 mm tube into an insert in the ear; z 4.2, %.1f mm of\nprinted material under the hole' % ((pz - M['hole_r'] - zb) * 10), (px - 0.1, pz - 0.15), (-1.2, 2.9), va='top')
    note(ax, 'fixed M3 stop screw, 8 mm ahead of the pivot: it passes through\nthe arc slot in the arm; the two slot ends are the stops', (stx, stz - 0.1), (-1.2, 1.95), va='top')
    ax.text(-1.2, 5.45, 'pillar (y 0.35 to 0.75) and tail: behind the plane, dashed\nouter ear (y 2.10 to 2.98): in front of it, dotted', fontsize=8, va='top', color='#444')
    ax.text(ox, 1.05, 'Nexus 14145\n60 x 25.6 mm, 73 g', fontsize=9, ha='center', va='center', color='#0a4d3a')
    ax.text(ox + 0.12, oz + 0.12, 'axle (6.85, 3.0)', fontsize=8, color='#073', ha='left', va='bottom')
    ax.text(xc, zc + 2.15, 'compressed: wheel top z 8.5\n(bridge 2.0 mm above)', fontsize=8, ha='center', color=C['wheel'])
    ax.annotate('', xy=(10.3, 1.5), xytext=(ox + 3.0, 1.5), arrowprops=dict(arrowstyle='<->', lw=0.7))
    ax.text(10.0, 1.65, 'rim 1.7 mm\nfrom the wall', fontsize=8, ha='center', va='bottom', bbox=BOX)

    # ------------------------------------------------------------------ B: front view through the axle (y-z)
    ax = fig.add_subplot(gs[0, 1])
    ax.set_aspect('equal'); ax.set_xlim(-2.6, 4.9); ax.set_ylim(-0.3, 7.0); ax.axis('off')
    ax.set_title('B  Front view through the axle (y left, z up)', fontsize=11, loc='left')
    ax.plot([-2.6, 4.9], [0, 0], color='k', lw=1.0)
    rect(ax, w0, 0.0, w1, 6.0, fc=C['wheel'], alpha=0.28, ec=C['wheel'], lw=1.2)
    rect(ax, w0, oz - W['bore_r'], w1, oz + W['bore_r'], fc='white', ec='k', lw=0.6)               # the 12 mm hole
    y_b1 = w0 + X['seat']
    for (b0, b1) in ((y_b1, y_b1 + B['w']), (w1 - B['w'], w1)):
        rect(ax, b0, oz - B['od_r'], b1, oz + B['od_r'], fc=C['bear'], ec='k', lw=0.6)
        rect(ax, b0, oz - B['bore_r'], b1, oz + B['bore_r'], fc='white', ec='k', lw=0.5)
    rect(ax, y_b1 + B['w'], oz - X['sleeve_r'][1], w1 - B['w'], oz + X['sleeve_r'][1], fc=C['sleeve'], ec='k', lw=0.5)       # sleeve
    rect(ax, w1, oz - X['sleeve_r'][1], a0, oz + X['sleeve_r'][1], fc=C['sleeve'], ec='k', lw=0.5)                          # 1 mm spacer
    y_under = y_b1 - X['washer_h']
    rect(ax, y_under, oz - X['washer_r'], y_b1, oz + X['washer_r'], fc=C['sleeve'], ec='k', lw=0.5)
    rect(ax, y_under - X['head_h'], oz - X['head_r'], y_under, oz + X['head_r'], fc=C['steel'], ec='k', lw=0.6)             # head
    rect(ax, y_under, oz - O['axle_r'], y_under + X['screw_len'], oz + O['axle_r'], fc=C['steel'], ec='k', lw=0.5)           # shank, tip inside the arm
    rect(ax, a0, oz - O['arm_half'], a1, oz + O['arm_half'], fc=C['arm'], ec='k', lw=0.8, alpha=0.85)                       # arm at the axle
    rect(ax, a0, oz - O['axle_r'], a1, oz + O['axle_r'], fc=C['steel'], ec='k', lw=0.4)
    fpy = (P.FLOOR_FRONT['y'] - P.FLOOR_FRONT['w'] / 2, P.FLOOR_FRONT['y'] + P.FLOOR_FRONT['w'] / 2)
    rect(ax, -2.6, zb, P.OMNI_BAY_Y[0], zf, fc=C['floor'], ec='k', lw=0.7)
    rect(ax, P.OMNI_BAY_Y[1], zb, fpy[0] - 0.05, zf, fc=C['floor'], ec='k', lw=0.7)
    rect(ax, fpy[1] + 0.05, zb, 4.9, zf, fc=C['floor'], ec='k', lw=0.7)
    rect(ax, fpy[0], P.FLOOR_FRONT['z_face'], fpy[1], P.FLOOR_FRONT['z_face'] + 1.4, fc=C['fp'], ec='k', lw=0.7, alpha=0.8)
    ax.text((fpy[0] + fpy[1]) / 2, P.FLOOR_FRONT['z_face'] + 1.5, 'floor port FP\n(20.3 mm board)', fontsize=8, ha='center', va='bottom')
    post = P.giga_hole_xy(0)
    rect(ax, post[1] - P.GIGA_POST['r'], P.GIGA_POST['z'][0], post[1] + P.GIGA_POST['r'], P.GIGA_POST['z'][1], fc=C['print_'], ec='#444', lw=0.9, ls='--', alpha=0.7)
    ax.text(-2.55, 6.95, 'GIGA post H1\n(behind, at x 5.87)', fontsize=8, ha='left', va='top')
    ax.plot([-2.6, 4.9], [zb, zb], color='#888', ls=':', lw=0.8)
    note(ax, 'wheel face to post 2.9 mm', (-1.12, 5.4), (-2.55, 6.0), va='top')
    note(ax, 'wheel to arm 1 mm', (w1 + 0.05, 5.0), (1.1, 6.95), va='top')
    note(ax, 'floor wall 3.4 mm\n(opening to FP hole)', ((P.OMNI_BAY_Y[1] + fpy[0]) / 2, 3.7), (2.1, 5.9), va='top')
    note(ax, 'axle: M4 x 25, its head and washer in the\nwheel\'s bore, tapped into the arm (2.9 mm\nof thread); the tip stops 0.1 mm inside the\narm\'s outer face (a nut out there would sweep\nthrough the floor wall)', (a1 - 0.05, oz - 0.12), (1.7, 2.55), va='top')
    note(ax, 'two 604-2RS bearings in the\n12 mm hole, 12.6 mm sleeve\nbetween the inner rings', (y_b1 + 0.2, oz - 0.5), (-2.55, 1.9), va='top')
    ax.text(w0 + 1.28, 0.55, 'wheel\ny -0.98 to 1.58', fontsize=8, ha='center', color='#0a4d3a')
    ax.text(a0 + 0.15, oz + O['arm_half'] + 0.1, 'arm', fontsize=8.5, color=C['arm'], ha='center')

    # ------------------------------------------------------------------ C: section through the pivot (y-z)
    ax = fig.add_subplot(gs[1, 1])
    ax.set_aspect('equal'); ax.set_xlim(-0.6, 3.6); ax.set_ylim(2.55, 5.45); ax.axis('off')
    ax.set_title('C  Section through the pivot (y left, z up)', fontsize=11, loc='left')
    pl, ea = M['pillar'], M['ear']
    rect(ax, -0.5, zb, M['pocket']['y'][0], zf, fc=C['floor'], ec='k', lw=0.7)
    rect(ax, M['pocket']['y'][1], zb, 3.5, zf, fc=C['floor'], ec='k', lw=0.7)
    rect(ax, pl['y'][0], zf, pl['y'][1], pl['z'][1], fc=C['print_'], ec='k', lw=0.8)
    rect(ax, ea['y'][0], zf, ea['y'][1], ea['z'][1], fc=C['print_'], ec='k', lw=0.8)
    rect(ax, pl['y'][0], pz - M['hole_r'], pl['y'][1], pz + M['hole_r'], fc='white', ec='k', lw=0.5)
    rect(ax, ea['y'][0], pz - V['insert_r'], ea['y'][0] + V['insert_len'], pz + V['insert_r'], fc='#F6E6C8', ec='#BA7517', lw=1.0)
    rect(ax, pl['y'][1], pz - V['tube_r'][1], ea['y'][0], pz + V['tube_r'][1], fc=C['sleeve'], ec='k', lw=0.6)
    rect(ax, pl['y'][0], pz - V['screw_r'], pl['y'][0] + V['screw_len'], pz + V['screw_r'], fc=C['steel'], ec='k', lw=0.6)
    rect(ax, pl['y'][0] - V['head_h'], pz - V['head_r'], pl['y'][0], pz + V['head_r'], fc=C['steel'], ec='k', lw=0.6)
    rect(ax, a0, pz - O['arm_half'], a1, pz + O['arm_half'], fc=C['arm'], ec='k', lw=0.8, alpha=0.85)
    rect(ax, a0, pz - O['pivot_hole_r'], a1, pz + O['pivot_hole_r'], fc=C['sleeve'], ec='k', lw=0.4)
    rect(ax, a0, pz - V['screw_r'], a1, pz + V['screw_r'], fc=C['steel'], ec='k', lw=0.4)
    for sg in (1, -1):                                                                                     # the arbor sleeve (a wall above and below the tube) and the coil on it
        rect(ax, AB['y'][0], pz + sg * AB['r_in'] if sg > 0 else pz - AB['r'], AB['y'][1], pz + AB['r'] if sg > 0 else pz - AB['r_in'], fc='#E4E2D8', ec='#888', lw=0.6)
    for zc_ in (pz + SP['mean_d'] / 2, pz - SP['mean_d'] / 2):
        rect(ax, SP['y'][0], zc_ - SP['wire'] / 2, SP['y'][1], zc_ + SP['wire'] / 2, fc=C['spring'], ec='k', lw=0.5)
    ax.plot([-0.5, 3.6], [zb, zb], color='#888', ls=':', lw=0.8)
    ax.plot([ea['y'][0], 3.6], [P.TRAY['z'][0]] * 2, color=C['batt'], lw=2.4)
    ax.text(3.55, P.TRAY['z'][0] + 0.05, 'battery tray, underside z 4.9', fontsize=8, ha='right', color='#7a4d00')
    ax.text((pl['y'][0] + pl['y'][1]) / 2, pl['z'][1] + 0.05, 'pillar 4 mm', fontsize=8.5, ha='center')
    ax.text((ea['y'][0] + ea['y'][1]) / 2, ea['z'][1] + 0.05, 'ear 8.8 mm', fontsize=8.5, ha='center')
    note(ax, 'coil %.1f mm (%.1f mm off the pillar,\n%.1f mm short of the arm) on the\nprinted arbor (%.1f mm, light grey)' % (SP['length'] * 10, (SP['y'][0] - M['pillar']['y'][1]) * 10, (a0 - SP['y'][1]) * 10, 2 * AB['r'] * 10),
         ((SP['y'][0] + SP['y'][1]) / 2, pz + 0.5), (0.95, 5.4), va='top')
    note(ax, 'M3 x 5.7 insert, blind\nhole 7.6 mm behind it', (ea['y'][0] + 0.3, pz - 0.1), (2.45, 3.3), va='top')
    note(ax, 'tube 5 x 3.1 x 13.5 mm, clamped between pillar\nand ear; the arm (5 mm bore) turns on it', (1.4, pz - 0.22), (-0.5, 3.15), va='top')
    note(ax, 'head on the pillar,\nreached from inboard', (pl['y'][0] - 0.2, pz + 0.15), (-0.55, 5.35), va='top')
    ax.text(1.4, zb - 0.1, 'pocket in the floor, open to the wheel bay', fontsize=8, ha='center', va='top', color='#555')
    ax.text(3.0, 3.7, 'floor 4 mm', fontsize=8, ha='center', va='center')

    # ------------------------------------------------------------------ D: the arm
    ax = fig.add_subplot(gs[2, 1])
    ax.set_aspect('equal'); ax.axis('off')
    L_ = O['arm_len']
    ax.set_xlim(-1.0, L_ + 1.0); ax.set_ylim(-1.9, 1.9)
    ax.set_title('D  The arm in its own frame. 3 x 12 mm 6061 flat bar, ends R6, hole centres 46 mm apart', fontsize=11, loc='left')
    poly(ax, stadium_pts((0, 0), (L_, 0), O['arm_half'], 22), fc=C['arm'], alpha=0.35, ec='k', lw=1.2)
    ax.add_patch(Circle((0, 0), O['pivot_hole_r'], fc='white', ec='k', lw=0.9))
    ax.add_patch(Circle((L_, 0), O['axle_r'], fc='white', ec='k', lw=0.9))
    ax.text(0, -0.78, 'pivot bore 5.05 mm\n(reamed; the arm\nturns on the tube)', fontsize=8.5, ha='center', va='top')
    ax.add_patch(Circle((K['s'], P.omni_seat_n()), K['hole_r'], fc='#B4B2A9', ec='k', lw=0.8))
    note(ax, 'seat pin 2 mm: the forward spring leg\nrests on it (spring seat hole C)', (K['s'] + K['hole_r'], P.omni_seat_n() - 0.05), (1.9, 0.14), va='top')
    ax.text(L_, -0.78, 'M4 tapped: the axle screw\n(thread-locker)', fontsize=8.5, ha='center', va='top')
    poly(ax, slot_pts(0.0, (0.0, 0.0)), fc='white', ec='k', lw=1.0)
    r, hw = slot['r'], slot['hw']
    th_rest = math.radians(slot['a_hi']) - (S['play'] - (hw - S['pin_r'])) / r
    th_full = math.radians(slot['a_lo']) + (S['play'] - (hw - S['pin_r'])) / r
    ax.add_patch(Circle((r * math.cos(th_rest), r * math.sin(th_rest)), S['pin_r'], fc=C['steel'], alpha=0.6, ec='k', lw=0.7))
    ax.add_patch(Circle((r * math.cos(th_full), r * math.sin(th_full)), S['pin_r'], fc=C['steel'], alpha=0.3, ec='k', lw=0.7, ls='--'))
    note(ax, 'screw at rest: the top end\nof the slot is the rest stop', (r * math.cos(th_rest) + 0.1, r * math.sin(th_rest) + 0.1), (0.9, 1.75), va='top')
    note(ax, 'screw fully compressed: the\nbottom end is the compression stop', (r * math.cos(th_full) + 0.1, r * math.sin(th_full) - 0.1), (1.15, -1.0), va='top')
    note(ax, 'arc slot, radius 8 mm about the pivot, 3.3 mm wide;\n0.2 mm of play beyond the screw at each end;\n2.1 mm of bar above and below, 3.4 mm to the bore', (r * math.cos(0) + hw, 0.0), (3.0, 1.75), va='top')
    ax.text(L_ / 2 + 0.6, -1.7, 'a slot end that is 0.2 mm off moves the wheel 1.1 mm: file the rest end when the ride height is measured.  Dimensions: Figure 13', fontsize=8.5, ha='center')

    # ------------------------------------------------------------------ parts list
    ax = fig.add_subplot(gs[2, 0])
    ax.axis('off')
    mod = sum(P.BOUGHT_G[nm] for nm in ('Omni wheel', 'Omni arm', 'Omni pins', 'Omni pivot', 'Omni spring', 'Omni adjuster'))
    ax.set_title('Parts (the module weighs %.0f g in the model; the 8 Oct estimate was 36.5 g)' % mod, fontsize=11, loc='left')
    y = 0.95
    for nm, txt, n in PARTS:
        ax.text(0.0, y, nm, fontsize=9.6, weight='bold', va='top', transform=ax.transAxes)
        ax.text(0.10, y, '%d x  %s' % (n, txt), fontsize=9.6, va='top', transform=ax.transAxes)
        y -= 0.066
    ax.text(0.0, y - 0.01, 'Assembly: arm, arbor and spring on the tube between pillar and ear (forward leg on the pin, rear leg on the adjuster head); pivot screw\n'
            'from inboard into the insert; stop screw from inboard through the slot into the ear; wheel and bearings on the axle screw, tapped into the arm.\n'
            'Wheel service: unscrew the axle screw from below (a ball-end 3 mm hex key at an angle: the silver module is 1.6 cm\n'
            'inboard of the wheel at the same height) until it leaves the arm; the wheel set drops out of the bay with the screw in it.\n'
            'Arm, spring, pivot: lid, dropper unit, frame and GIGA stack out first (section 3.3), then the two screws\n'
            'out from inboard and the module lifts out.',
            fontsize=8.8, va='top', transform=ax.transAxes, color='#333')
    fig.suptitle('Figure 12. How the front omni wheel is mounted (10 Oct): one aluminium arm, a pivot screw on a tube, a torsion spring on an arbor (rear leg on an adjuster, forward leg on a pin in the arm), a fixed stop screw in an arc slot', fontsize=12.5, y=0.985)
    save(fig, 'fig12_omni_mount')


if __name__ == '__main__':
    fig_omni()
