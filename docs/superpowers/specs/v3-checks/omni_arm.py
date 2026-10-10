"""The omni arm as a part to make (mechanical design section 4.2, 10 Oct; backlog O4): the profile from v3_params4 in the arm's own frame, written as a DXF cut file and drawn with its dimensions as Figure 13.
    python omni_arm.py [../v3-figures]      writes omni_arm.dxf, fig13_omni_arm.png and .svg, prints the numbers (results/omni_arm.txt)
Frame (mm): the origin is the centre of the pivot bore, x runs along the arm toward the axle, y is up when the arm is installed (the wheel is on the -z side of its axis in the figure's side view only by gravity).
Nothing here is new geometry: the numbers are the model's (OMNI, OMNI_SEAT, omni_stop_slot), plus the usual shop tolerances."""
import math
import os
import sys

import v3_params4 as P


def features():
    """Every feature of the arm in the arm's frame, mm. The slot is the stop screw's arc about the pivot: radius `r`, half width `hw`, and the angles of the centres of its round ends (rest stop = a_hi, compression
    stop = a_lo)."""
    O, K = P.OMNI, P.OMNI_SEAT
    slot = P.omni_stop_slot()
    r, hw = slot['r'] * 10.0, slot['hw'] * 10.0
    return dict(
        length=O['arm_len'] * 10.0 + 2 * O['arm_half'] * 10.0, width=2 * O['arm_half'] * 10.0, thick=O['arm_t'] * 10.0, centres=O['arm_len'] * 10.0, end_r=O['arm_half'] * 10.0,
        pivot_d=2 * O['pivot_hole_r'] * 10.0, tube_d=2 * P.OMNI_PIVOT['tube_r'][1] * 10.0, axle_x=O['arm_len'] * 10.0, axle_tap=3.3, axle_thread=4.0,
        slot_r=r, slot_hw=hw, slot_a_lo=slot['a_lo'], slot_a_hi=slot['a_hi'], play=P.OMNI_MOUNT['stop']['play'] * 10.0, screw_d=2 * P.OMNI_MOUNT['stop']['pin_r'] * 10.0,
        seat_x=K['s'] * 10.0, seat_y=P.omni_seat_n() * 10.0, seat_d=2 * K['hole_r'] * 10.0, seat_pin_len=(K['y'][1] - K['y'][0]) * 10.0,
        seat_out=(O['arm_y'][0] - K['y'][0]) * 10.0, seat_in=(K['y'][1] - O['arm_y'][0]) * 10.0)


# ---- the cut file: ASCII DXF R12 (LINE, ARC, CIRCLE only), millimetres, one layer
def dxf_entities():
    """The cut profile as a list of ('LINE', x1, y1, x2, y2), ('ARC', cx, cy, r, a0, a1) (degrees, counterclockwise) and ('CIRCLE', cx, cy, r)."""
    f = features()
    e, R = [], f['end_r']
    A, B = (0.0, 0.0), (f['centres'], 0.0)
    e.append(('LINE', A[0], R, B[0], R))
    e.append(('LINE', A[0], -R, B[0], -R))
    e.append(('ARC', A[0], A[1], R, 90.0, 270.0))
    e.append(('ARC', B[0], B[1], R, -90.0, 90.0))
    e.append(('CIRCLE', A[0], A[1], f['pivot_d'] / 2))
    e.append(('CIRCLE', B[0], B[1], f['axle_tap'] / 2))
    e.append(('CIRCLE', f['seat_x'], f['seat_y'], f['seat_d'] / 2))
    r, hw, lo, hi = f['slot_r'], f['slot_hw'], f['slot_a_lo'], f['slot_a_hi']
    e.append(('ARC', 0.0, 0.0, r + hw, lo, hi))
    e.append(('ARC', 0.0, 0.0, r - hw, lo, hi))
    for a, (t0, t1) in ((hi, (hi, hi + 180.0)), (lo, (lo + 180.0, lo + 360.0))):
        cx, cy = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        e.append(('ARC', cx, cy, hw, t0 % 360.0, t1 % 360.0))
    return e


def write_dxf(path):
    lines = ['0', 'SECTION', '2', 'HEADER', '9', '$ACADVER', '1', 'AC1009', '9', '$INSUNITS', '70', '4', '0', 'ENDSEC',
             '0', 'SECTION', '2', 'TABLES',
             '0', 'TABLE', '2', 'LTYPE', '70', '1', '0', 'LTYPE', '2', 'CONTINUOUS', '70', '0', '3', 'Solid line', '72', '65', '73', '0', '40', '0.0', '0', 'ENDTAB',
             '0', 'TABLE', '2', 'LAYER', '70', '1', '0', 'LAYER', '2', 'CUT', '70', '0', '62', '7', '6', 'CONTINUOUS', '0', 'ENDTAB',
             '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES']
    for en in dxf_entities():
        if en[0] == 'LINE':
            lines += ['0', 'LINE', '8', 'CUT', '10', '%.4f' % en[1], '20', '%.4f' % en[2], '30', '0.0', '11', '%.4f' % en[3], '21', '%.4f' % en[4], '31', '0.0']
        elif en[0] == 'ARC':
            lines += ['0', 'ARC', '8', 'CUT', '10', '%.4f' % en[1], '20', '%.4f' % en[2], '30', '0.0', '40', '%.4f' % en[3], '50', '%.4f' % en[4], '51', '%.4f' % en[5]]
        else:
            lines += ['0', 'CIRCLE', '8', 'CUT', '10', '%.4f' % en[1], '20', '%.4f' % en[2], '30', '0.0', '40', '%.4f' % en[3]]
    lines += ['0', 'ENDSEC', '0', 'EOF']
    with open(path, 'w', newline='') as fh:
        fh.write('\r\n'.join(lines) + '\r\n')


def read_dxf(path):
    """The entities of a DXF this module wrote (or any ASCII DXF with LINE, ARC and CIRCLE), as the tuples of dxf_entities(); used by the unit test."""
    with open(path) as fh:
        raw = [ln.strip() for ln in fh.read().splitlines()]
    pairs = list(zip(raw[0::2], raw[1::2]))
    out, cur = [], None
    for code, val in pairs:
        if code == '0':
            if cur:
                out.append(cur)
            cur = {'type': val} if val in ('LINE', 'ARC', 'CIRCLE') else None
        elif cur is not None:
            cur[int(code)] = val if code == '8' else float(val)
    if cur:
        out.append(cur)
    res = []
    for d in out:
        if d['type'] == 'LINE':
            res.append(('LINE', d[10], d[20], d[11], d[21]))
        elif d['type'] == 'ARC':
            res.append(('ARC', d[10], d[20], d[40], d[50], d[51]))
        else:
            res.append(('CIRCLE', d[10], d[20], d[40]))
    return res


def area_mm2():
    """Area of the cut profile, mm2: the stadium less the pivot bore, the tap-drill hole, the seat hole and the slot (an arc strip with two round ends)."""
    f = features()
    stadium = f['centres'] * f['width'] + math.pi * f['end_r'] ** 2
    holes = math.pi * ((f['pivot_d'] / 2) ** 2 + (f['axle_tap'] / 2) ** 2 + (f['seat_d'] / 2) ** 2)
    arc = f['slot_r'] * math.radians(f['slot_a_hi'] - f['slot_a_lo'])
    slot = arc * 2 * f['slot_hw'] + math.pi * f['slot_hw'] ** 2
    return stadium - holes - slot


def mass_g():
    """The arm in 6061 (2.70 g/cm3); the seat pin (stainless, 7.9 g/cm3) is extra."""
    f = features()
    return area_mm2() * f['thick'] * 2.70e-3, math.pi * (f['seat_d'] / 2) ** 2 * f['seat_pin_len'] * 7.9e-3


def ride_gain():
    """Wheel height per unit of slot end: how far the axle rises for a slot end filed back (the arm's horizontal reach over the stop radius)."""
    ox, oz = P.OMNI['rest']
    px, pz = P.OMNI['pivot']
    return P.OMNI['arm_len'] * math.cos(math.atan2(oz - pz, ox - px)) / P.OMNI_MOUNT['stop']['r']


# ---- the drawing
def report():
    f = features()
    m_arm, m_pin = mass_g()
    lines = ['arm: 6061-T6 flat bar %.0f x %.0f mm, %.0f mm long (%.1f mm between the centres, ends R%.0f), profile %.0f mm2, mass %.1f g (+ %.2f g pin; the model budgets 6 g)' %
             (f['width'], f['thick'], f['length'], f['centres'], f['end_r'], area_mm2(), m_arm, m_pin)]
    lines.append('pivot bore %.2f mm (reamed, H8) for the %.2f mm tube; axle hole M%.0f x 0.7 tapped through (drill %.1f), %.1f mm from the pivot' % (f['pivot_d'], f['tube_d'], f['axle_thread'], f['axle_tap'], f['axle_x']))
    lines.append('stop slot: arc of radius %.2f mm about the pivot, %.2f mm wide, round-end centres at %+.2f degrees (rest stop) and %+.2f degrees (compression stop), i.e. at (%.2f, %+.2f) and (%.2f, %+.2f) mm; '
                 'the ends are %.1f mm beyond the %.1f mm screw at each stop; a slot end filed 0.2 mm moves the wheel %.1f mm' %
                 (f['slot_r'], 2 * f['slot_hw'], f['slot_a_hi'], f['slot_a_lo'], f['slot_r'] * math.cos(math.radians(f['slot_a_hi'])), f['slot_r'] * math.sin(math.radians(f['slot_a_hi'])),
                  f['slot_r'] * math.cos(math.radians(f['slot_a_lo'])), f['slot_r'] * math.sin(math.radians(f['slot_a_lo'])), f['play'], f['screw_d'], 0.2 * ride_gain()))
    lines.append('spring seat: %.2f mm hole (H7, through) at (%.2f, %+.2f) mm for the %.0f x %.0f mm pin, %.1f mm standing out of the inboard face, %.1f mm in the bar' %
                 (f['seat_d'], f['seat_x'], f['seat_y'], f['seat_d'], f['seat_pin_len'], f['seat_out'], f['seat_in']))
    top = f['width'] / 2 - (f['seat_y'] + f['seat_d'] / 2)
    s_top = max(f['slot_r'] * math.sin(math.radians(a)) for a in (f['slot_a_lo'], f['slot_a_hi'])) + f['slot_hw']
    lines.append('bar left: %.2f mm above the seat hole, %.2f mm above and below the slot, %.2f mm between the slot and the pivot bore, %.1f mm from the slot to the axle hole' %
                 (top, f['width'] / 2 - s_top, f['slot_r'] - f['slot_hw'] - f['pivot_d'] / 2, f['axle_x'] - (f['slot_r'] + f['slot_hw']) - f['axle_tap'] / 2))
    return lines


def figure(path_png, path_svg=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, Circle, Rectangle, FancyArrowPatch

    f = features()
    fig = plt.figure(figsize=(14.5, 10.2), dpi=130)
    ink, thin, dim = '#1a1a18', '#5F5E5A', '#0F6E56'
    fig.text(0.03, 0.968, 'Figure 13. The omni arm, to make (mm): 6061-T6 bar 12 x 3 mm, 58 mm long, one piece', fontsize=13, fontweight='bold', color=ink, ha='left')
    fig.text(0.03, 0.944, 'Face view as installed (inboard face toward you). Origin: the pivot bore; x along the arm toward the axle, y up. The cut file is omni_arm.dxf (the same numbers).', fontsize=10, color=thin, ha='left')
    ax = fig.add_axes([0.03, 0.50, 0.94, 0.42])
    ax.set_aspect('equal')
    ax.axis('off')
    R, A, Bx = f['end_r'], 0.0, f['centres']
    ax.plot([A, Bx], [R, R], color=ink, lw=2)
    ax.plot([A, Bx], [-R, -R], color=ink, lw=2)
    ax.add_patch(Arc((A, 0), 2 * R, 2 * R, theta1=90, theta2=270, color=ink, lw=2))
    ax.add_patch(Arc((Bx, 0), 2 * R, 2 * R, theta1=-90, theta2=90, color=ink, lw=2))
    for cx, cy, d in ((A, 0.0, f['pivot_d']), (Bx, 0.0, f['axle_tap']), (f['seat_x'], f['seat_y'], f['seat_d'])):
        ax.add_patch(Circle((cx, cy), d / 2, fill=False, color=ink, lw=2))
    r, hw, lo, hi = f['slot_r'], f['slot_hw'], f['slot_a_lo'], f['slot_a_hi']
    ax.add_patch(Arc((0, 0), 2 * (r + hw), 2 * (r + hw), theta1=lo, theta2=hi, color=ink, lw=2))
    ax.add_patch(Arc((0, 0), 2 * (r - hw), 2 * (r - hw), theta1=lo, theta2=hi, color=ink, lw=2))
    ends = {}
    for tag, a, t0, t1 in (('S1', hi, hi, hi + 180.0), ('S2', lo, lo + 180.0, lo + 360.0)):
        cx, cy = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        ends[tag] = (cx, cy)
        ax.add_patch(Arc((cx, cy), 2 * hw, 2 * hw, theta1=t0, theta2=t1, color=ink, lw=2))
    dash = (0, (8, 3, 1, 3))
    ax.plot([-R - 3, Bx + R + 3], [0, 0], color=thin, lw=0.8, ls=dash)
    for cx in (A, Bx):
        ax.plot([cx, cx], [-R - 2, R + 2], color=thin, lw=0.8, ls=dash)
    ax.add_patch(Arc((0, 0), 2 * (r * 1.12), 2 * (r * 1.12), theta1=lo, theta2=hi, color=thin, lw=0.8, ls=dash))
    for a in (hi, lo):
        ax.plot([0, r * 1.12 * math.cos(math.radians(a))], [0, r * 1.12 * math.sin(math.radians(a))], color=thin, lw=0.8, ls=dash)

    def balloon(txt, xy, tip, color=ink):
        ax.annotate(txt, xy=tip, xytext=xy, fontsize=10.5, fontweight='bold', color=color, ha='center', va='center',
                    bbox=dict(boxstyle='circle,pad=0.25', fc='white', ec=color, lw=1.2), arrowprops=dict(arrowstyle='-', color=color, lw=0.9, shrinkA=0, shrinkB=0))

    balloon('A', (-2.0, -10.0), (-1.5, -1.9))
    balloon('B', (Bx + 4.0, -10.0), (Bx + 1.1, -1.2))
    balloon('C', (f['seat_x'] + 4.5, 10.5), (f['seat_x'] + 0.8, f['seat_y'] + 0.8))
    balloon('S1', (ends['S1'][0] - 1.0, 10.5), (ends['S1'][0] - 0.5, ends['S1'][1] + hw - 0.1))
    balloon('S2', (ends['S2'][0] - 1.0, -10.0), (ends['S2'][0] - 0.5, ends['S2'][1] - hw + 0.1))
    ax.text(r * 1.12 * math.cos(math.radians(hi)) + 1.8, r * 1.12 * math.sin(math.radians(hi)) - 2.0, '+%.1f deg' % hi, fontsize=8.5, color=thin, ha='left')
    ax.text(r * 1.12 * math.cos(math.radians(lo)) + 1.8, r * 1.12 * math.sin(math.radians(lo)) + 0.6, '%.1f deg' % lo, fontsize=8.5, color=thin, ha='left')

    def hdim(x0, x1, y, label, below=True):
        ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle='<->', mutation_scale=9, color=dim, lw=1.2))
        ax.text((x0 + x1) / 2, y - 0.8, label, ha='center', va='top', fontsize=10, color=dim)

    hdim(A, Bx, -R - 8.0, '%.1f +-0.1 between the centres' % f['centres'])
    ax.plot([A, A], [-R - 2.5, -R - 9], color=dim, lw=0.6)
    ax.plot([Bx, Bx], [-R - 2.5, -R - 9], color=dim, lw=0.6)
    hdim(-R, Bx + R, -R - 13.5, '58 overall')
    ax.plot([-R, -R], [-1.5, -R - 14.5], color=dim, lw=0.6)
    ax.plot([Bx + R, Bx + R], [-1.5, -R - 14.5], color=dim, lw=0.6)
    ax.add_patch(FancyArrowPatch((Bx + R + 2.4, -R), (Bx + R + 2.4, R), arrowstyle='<->', mutation_scale=9, color=dim, lw=1.2))
    ax.text(Bx + R + 3.2, 0, '12', ha='left', va='center', fontsize=10, color=dim)
    ax.set_xlim(-12.5, Bx + R + 9)
    ax.set_ylim(-R - 19.0, R + 7)
    # edge view
    ax2 = fig.add_axes([0.03, 0.06, 0.26, 0.40])
    ax2.set_aspect('equal')
    ax2.axis('off')
    t = f['thick']
    ax2.add_patch(Rectangle((0, 0), t, 12, fill=False, ec=ink, lw=2))
    kin, kout = f['seat_in'], f['seat_out']
    ax2.add_patch(Rectangle((-kout, f['seat_y'] + 6 - f['seat_d'] / 2), kin + kout, f['seat_d'], fc='#B4B2A9', ec=ink, lw=1.5))
    ax2.text(-kout - 0.5, f['seat_y'] + 6 - 0.2, 'pin %.0f x %.0f\n(C)' % (f['seat_d'], f['seat_pin_len']), ha='right', va='center', fontsize=9, color=ink)
    leg_x = -(P.OMNI['arm_y'][0] - (P.OMNI_SPRING['y'][1] - P.OMNI_SPRING['wire'] / 2)) * 10.0            # the spring's forward leg: the last turn run straight, 1.5 mm inboard of the arm's face
    leg_y = 6 + P.omni_leg_n(f['seat_x'] / 10.0) * 10.0
    ax2.add_patch(Circle((leg_x, leg_y), P.OMNI_SPRING['wire'] * 5, fc='white', ec=thin, lw=1.2, ls=(0, (3, 2))))
    ax2.annotate('spring leg 1.6\n(not part of the arm)', xy=(leg_x - 0.3, leg_y + 0.7), xytext=(-kout - 0.5, leg_y + 3.4), fontsize=8.5, color=thin, ha='right',
                 arrowprops=dict(arrowstyle='-', color=thin, lw=0.8))
    ax2.annotate('', xy=(0, -1.5), xytext=(t, -1.5), arrowprops=dict(arrowstyle='<->', color=dim, lw=1.2))
    ax2.text(t / 2, -2.4, '3.0', ha='center', va='top', fontsize=10, color=dim)
    ax2.annotate('', xy=(0, 6 - f['seat_y'] - 4.5 + 0.0), xytext=(-kout, 6 - f['seat_y'] - 4.5), arrowprops=dict(arrowstyle='<->', color=dim, lw=1.0))
    ax2.text(-kout / 2, 6 - f['seat_y'] - 5.0, '%.1f out' % kout, ha='center', va='top', fontsize=8.5, color=dim)
    ax2.text(-0.8, 1.0, 'inboard\n(spring side)', ha='right', va='bottom', fontsize=9, color=thin)
    ax2.text(t + 0.8, 1.0, 'outboard\n(ear side)', ha='left', va='bottom', fontsize=9, color=thin)
    ax2.set_xlim(-12, t + 9)
    ax2.set_ylim(-6.5, 19)
    ax2.set_title('Edge view of the pivot end (looking along the arm)', fontsize=9.5, loc='left', color=ink)
    # feature table and notes
    ax3 = fig.add_axes([0.33, 0.04, 0.65, 0.46])
    ax3.axis('off')
    rows = [('A', '%.2f' % 0.0, '%.2f' % 0.0, 'pivot bore %.2f H8 (reamed; the %.2f tube turns in it)' % (f['pivot_d'], f['tube_d'])),
            ('B', '%.2f' % f['axle_x'], '%.2f' % 0.0, 'axle hole: drill %.1f, tap M4 x 0.7 through (4 turns in 3 mm)' % f['axle_tap']),
            ('C', '%.2f' % f['seat_x'], '%+.2f' % f['seat_y'], 'spring seat: %.1f H7 through; stainless pin 2 m6 x 6 driven in from the inboard face, %.1f mm out' % (f['seat_d'], f['seat_out'])),
            ('S1', '%.2f' % ends['S1'][0], '%+.2f' % ends['S1'][1], 'slot end, REST stop (%+.2f deg): round end R%.2f; file it 0.1 mm short first' % (hi, hw)),
            ('S2', '%.2f' % ends['S2'][0], '%+.2f' % ends['S2'][1], 'slot end, COMPRESSION stop (%+.2f deg): round end R%.2f' % (lo, hw))]
    ax3.text(0.0, 1.0, 'Features (x, y from the pivot centre, mm; +-0.1 unless noted)', fontsize=10.5, fontweight='bold', color=dim, va='top')
    ax3.text(0.0, 0.935, 'No.', fontsize=9.5, fontweight='bold', color=ink, va='top')
    ax3.text(0.06, 0.935, 'x', fontsize=9.5, fontweight='bold', color=ink, va='top')
    ax3.text(0.13, 0.935, 'y', fontsize=9.5, fontweight='bold', color=ink, va='top')
    ax3.text(0.21, 0.935, 'Feature', fontsize=9.5, fontweight='bold', color=ink, va='top')
    y = 0.885
    for no, x_, y_, txt in rows:
        ax3.text(0.0, y, no, fontsize=9.5, color=ink, va='top')
        ax3.text(0.06, y, x_, fontsize=9.5, color=ink, va='top')
        ax3.text(0.13, y, y_, fontsize=9.5, color=ink, va='top')
        ax3.text(0.21, y, _wrap(txt, 84), fontsize=9.5, color=ink, va='top')
        y -= 0.058 * (1 + _wrap(txt, 84).count('\n'))
    slot_txt = _wrap('The slot is the arc of radius %.2f about A between S1 and S2, %.2f wide (+0.1/0), walls square to the faces and deburred.' % (r, 2 * hw), 84)
    ax3.text(0.21, y, slot_txt, fontsize=9.5, color=ink, va='top')
    y -= 0.058 * (1 + slot_txt.count('\n')) + 0.02
    notes = [
        ('Material', '6061-T6 flat bar 12 x 3 as bought, faces untouched; %.1f g (pin %.2f g). Bare aluminium, or hard-anodised AFTER machining.' % mass_g()),
        ('Cutting', 'Waterjet, laser, or saw and drill; break all edges 0.2; flat to 0.1. A plain printed copy (PETG 4 x 12) does for a first fit only: it twists the wheel 2.2 degrees under load (aluminium 0.14).'),
        ('Ride height', 'The slot ends set it: a slot end filed 0.2 mm moves the wheel %.1f mm. Leave S1 0.1 mm short, assemble, measure the wheel\'s bottom against the floor, file S1 to the measured height (spec 4.4).' % (0.2 * ride_gain())),
        ('Check', 'A to B %.1f +-0.1; a 3.2 mm gauge pin slides the whole slot; the pin\'s top stands %.2f above the axis, where the spring leg lies on it.' % (f['centres'], f['seat_y'] + f['seat_d'] / 2)),
    ]
    for k, v in notes:
        ax3.text(0.0, y, k, fontsize=9.5, fontweight='bold', color=dim, va='top')
        ax3.text(0.13, y, _wrap(v, 104), fontsize=9.3, color=ink, va='top')
        y -= 0.058 * (1 + _wrap(v, 104).count('\n')) + 0.012
    fig.savefig(path_png, dpi=130)
    if path_svg:
        fig.savefig(path_svg)
    plt.close(fig)


def _wrap(text, width):
    words, lines, cur = text.split(), [], ''
    for w in words:
        if len(cur) + len(w) + 1 > width:
            lines.append(cur)
            cur = w
        else:
            cur = (cur + ' ' + w).strip()
    lines.append(cur)
    return '\n'.join(lines)


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'v3-figures')
    os.makedirs(out, exist_ok=True)
    write_dxf(os.path.join(out, 'omni_arm.dxf'))
    figure(os.path.join(out, 'fig13_omni_arm.png'), os.path.join(out, 'fig13_omni_arm.svg'))
    for ln in report():
        print(ln)
    print('wrote omni_arm.dxf and fig13_omni_arm.png/.svg in', os.path.abspath(out))
