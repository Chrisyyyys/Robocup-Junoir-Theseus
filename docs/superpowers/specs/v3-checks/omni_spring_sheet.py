"""The omni's torsion spring as it would be ordered (mechanical design section 4.3, 10 Oct; backlog O5): Figure 14, the numbers from v3_params4 and omni_mount.py.
    python omni_spring_sheet.py [../v3-figures]      writes fig14_omni_spring.png and .svg, prints the sheet (results/omni_spring_sheet.txt)
Views are those of Figure 12's side view (the robot seen from its right: x forward to the right, z up), so a counterclockwise turn lifts the front. The spring's axis is the robot's y axis, pointing away from the viewer:
the end of the coil nearest the viewer (toward the pillar) carries the REAR leg, the far end (toward the arm) the FORWARD leg."""
import math
import os
import sys

import v3_params4 as P
import omni_mount as OM


def numbers():
    L = OM.MODEL[3]
    s = OM.spring(*OM.MODEL, quiet=True)
    return L, s, OM.sheet_numbers(L, s)


def leg_poly(root, direction_deg, length, width):
    """Rectangle of the given width along a straight wire from `root` in `direction_deg`."""
    d = math.radians(direction_deg)
    ux, uy = math.cos(d), math.sin(d)
    nx, ny = -uy, ux
    h = width / 2.0
    x0, y0 = root
    x1, y1 = x0 + length * ux, y0 + length * uy
    return [(x0 + nx * h, y0 + ny * h), (x1 + nx * h, y1 + ny * h), (x1 - nx * h, y1 - ny * h), (x0 - nx * h, y0 - ny * h)]


def rotate(pt, deg):
    a = math.radians(deg)
    return pt[0] * math.cos(a) - pt[1] * math.sin(a), pt[0] * math.sin(a) + pt[1] * math.cos(a)


def report():
    n = numbers()[2]
    S = P.OMNI_SPRING
    lines = ['torsion spring, right-hand wound, the load OPENS the coil (the legs are held from below, so it cannot be wound the other way)',
             'wire %.2f mm music wire (ASTM A228 / EN 10270-1 SH, about %.0f MPa); mean diameter %.1f mm (ID %.1f, OD %.1f free; ID %.1f, OD %.1f at full travel); %.2f body turns close wound = %.1f mm long' %
             (S['wire'] * 10, S['uts'], S['mean_d'] * 10, S['id'] * 10, S['od'] * 10, n['id_open'] * 10, n['od_open'] * 10, S['body_turns'], S['length'] * 10),
             'rear leg: tangent, pointing back (180 degrees), %.0f mm to the adjuster head, %.0f mm in all; forward leg: tangent, %.0f mm to its pin, %.0f mm in all, %.1f degrees below horizontal when free (%.1f at rest, %+.1f at full travel)' %
             (S['legs'][0] * 10, S['legs'][0] * 10 + 2, S['legs'][1] * 10, (S['legs'][1] + S['tip']) * 10, -n['fwd_free'], -n['fwd_rest'], n['fwd_full']),
             'free angle between the legs %.1f degrees (counterclockwise from the forward leg to the rear leg) = %d turns and %.1f degrees' % (n['free_angle'], int(S['body_turns']), n['free_angle'] - 180.0),
             'rate %.0f N.mm/rad = %.2f N.mm/degree; torque %.0f N.mm at %.1f degrees of opening (rest, 2.4 N at the axle) and %.0f N.mm at %.1f degrees (full travel, 8.5 N); stress %.0f MPa = %.0f %% of the wire\'s strength' %
             (n['k'], n['k_deg'], n['m_rest'], n['pre'], n['m_full'], n['pre'] + n['work'], n['sigma'], n['sigma_pct'])]
    return lines


def figure(path_png, path_svg=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, Circle, Polygon as MPoly, Rectangle

    n = numbers()[2]
    S, AB = P.OMNI_SPRING, P.OMNI_ARBOR
    ink, thin, dim, spring = '#1a1a18', '#5F5E5A', '#0F6E56', '#D85A30'
    R, Rm, w = S['od'] * 5, S['mean_d'] * 5, S['wire'] * 10                       # mm
    rear_len, fwd_len = S['legs'][0] * 10 + 2.0, (S['legs'][1] + S['tip']) * 10
    d_fwd = n['fwd_free']
    th_fwd = d_fwd + 90.0                                                       # root of the forward leg on the mean circle (a start leg of the counterclockwise path)
    root_f = (Rm * math.cos(math.radians(th_fwd)), Rm * math.sin(math.radians(th_fwd)))
    root_r = (0.0, Rm)
    fig = plt.figure(figsize=(14.5, 10.4), dpi=130)
    fig.text(0.03, 0.968, 'Figure 14. The omni spring to order (mm): custom torsion spring, music wire 1.6 mm, 4.07 turns, right-hand, 3 pieces', fontsize=13, fontweight='bold', color=ink, ha='left')
    fig.text(0.03, 0.944, 'A: the free spring seen along its axis from the end that carries the REAR leg (the robot seen from its right: front to the right, up is up). B: the same spring from above.',
             fontsize=10, color=thin, ha='left')

    # ---------------------------------------------------------------- A: axial view
    ax = fig.add_axes([0.02, 0.34, 0.60, 0.60])
    ax.set_aspect('equal')
    ax.axis('off')
    ax.add_patch(Circle((0, 0), R, fill=False, ec=ink, lw=2.2))
    ax.add_patch(Circle((0, 0), R - w, fill=False, ec=ink, lw=1.4))
    ax.add_patch(Circle((0, 0), Rm, fill=False, ec=thin, lw=0.8, ls=(0, (6, 3))))
    ax.add_patch(Circle((0, 0), AB['r'] * 10, fc='#E4E2D8', ec='#888', lw=0.9, alpha=0.8))
    ax.add_patch(Circle((0, 0), P.OMNI_PIVOT['tube_r'][1] * 10, fc='#B7B6B0', ec='#555', lw=0.9))
    ax.text(0, 0.0, 'tube 5', fontsize=7.5, ha='center', va='center', color='#222')
    ax.annotate('printed arbor 7.4 mm', xy=(-2.9, -1.9), xytext=(-12.5, -5.2), fontsize=8.8, color=thin, ha='center', arrowprops=dict(arrowstyle='-', color=thin, lw=0.7))
    ax.add_patch(MPoly(leg_poly(root_r, 180.0, rear_len, w), closed=True, fc=spring, ec=ink, lw=1.0, alpha=0.9))
    ax.add_patch(MPoly(leg_poly(root_f, d_fwd, fwd_len, w), closed=True, fc=spring, ec=ink, lw=1.0, alpha=0.9))
    for dd, ls in ((n['pre'], (0, (5, 2))), (n['pre'] + n['work'], (0, (1.5, 2)))):
        ax.add_patch(MPoly(leg_poly(rotate(root_f, dd), d_fwd + dd, fwd_len, w), closed=True, fill=False, ec=ink, lw=1.1, ls=ls))
    lp_r = (root_r[0] - S['legs'][0] * 10, root_r[1])
    lp_f = (root_f[0] + S['legs'][1] * 10 * math.cos(math.radians(d_fwd)), root_f[1] + S['legs'][1] * 10 * math.sin(math.radians(d_fwd)))
    for lp in (lp_r, lp_f):
        ax.plot([lp[0]], [lp[1]], marker='+', ms=11, color=ink, mew=1.4)
    ax.annotate(_wrap('REAR leg: straight, tangent, points back (horizontal), held. %.0f mm to where it lies on the adjuster head (+), %.0f mm in all, end rounded' % (S['legs'][0] * 10, rear_len), 40),
                xy=(lp_r[0] + 2.0, lp_r[1] + w / 2), xytext=(-25.0, 13.8), fontsize=9.3, color=ink, ha='left', va='top', arrowprops=dict(arrowstyle='-', color=ink, lw=0.8))
    ax.annotate(_wrap('FORWARD leg: straight, tangent, %.1f degrees below horizontal when free; %.0f mm to its seat pin (+), %.0f mm in all, end rounded. It follows the arm: solid = free, dashed = installed at rest (+%.1f degrees), dotted = full travel (+%.1f degrees)' %
                      (-d_fwd, S['legs'][1] * 10, fwd_len, n['pre'], n['pre'] + n['work']), 36),
                xy=(lp_f[0] + 2.5, lp_f[1] - 0.3), xytext=(17.6, -3.4), fontsize=9.3, color=ink, ha='left', va='top', arrowprops=dict(arrowstyle='-', color=ink, lw=0.8))
    # free angle: the arc from the forward leg's free direction counterclockwise to the rear leg
    ax.add_patch(Arc((0, 0), 2 * 8.4, 2 * 8.4, theta1=d_fwd, theta2=180.0, color=dim, lw=1.4))
    tip_a = math.radians(180.0)
    ax.annotate('', xy=(8.4 * math.cos(tip_a - 0.01), 8.4 * math.sin(tip_a - 0.01)), xytext=(8.4 * math.cos(tip_a + 0.20), 8.4 * math.sin(tip_a + 0.20)), arrowprops=dict(arrowstyle='->', color=dim, lw=1.4))
    ax.annotate('free angle %.1f degrees\n= %d turns and %.1f degrees\n(counterclockwise from the forward\nleg to the rear leg)' % (n['free_angle'], int(S['body_turns']), n['free_angle'] - 180.0), xy=(-8.4, 0.0), xytext=(-25.0, 2.6),
                fontsize=9.3, color=dim, ha='left', va='top', arrowprops=dict(arrowstyle='-', color=dim, lw=0.8))
    # winding direction: clockwise from the rear-leg end
    ax.add_patch(Arc((0, 0), 2 * 6.9, 2 * 6.9, theta1=200.0, theta2=330.0, color=thin, lw=1.3))
    ax.annotate('', xy=(6.9 * math.cos(math.radians(199.0)), 6.9 * math.sin(math.radians(199.0))), xytext=(6.9 * math.cos(math.radians(212.0)), 6.9 * math.sin(math.radians(212.0))),
                arrowprops=dict(arrowstyle='->', color=thin, lw=1.3))
    ax.text(-25.0, -9.2, 'RIGHT-HAND wound: from the rear-leg end\nthe wire runs away from you, clockwise (grey arc)', fontsize=9.3, color=thin, ha='left', va='top')
    # working direction: the forward leg turns counterclockwise
    wr_ = 15.4
    ax.add_patch(Arc((0, 0), 2 * wr_, 2 * wr_, theta1=d_fwd + 1.0, theta2=n['fwd_full'] - 1.0, color=spring, lw=2.0))
    ea = math.radians(n['fwd_full'] - 1.0)
    ax.annotate('', xy=(wr_ * math.cos(ea + 0.01), wr_ * math.sin(ea + 0.01)), xytext=(wr_ * math.cos(ea - 0.12), wr_ * math.sin(ea - 0.12)), arrowprops=dict(arrowstyle='->', color=spring, lw=2.0))
    ax.text(15.8, 12.5, 'working direction: the forward leg is lifted\n(counterclockwise) by %.1f degrees at rest and\n%.1f degrees at full travel: the coil OPENS' % (n['pre'], n['pre'] + n['work']), fontsize=9.3, color=spring, ha='left', va='top')
    ax.annotate('', xy=(-R, -R - 1.3), xytext=(R, -R - 1.3), arrowprops=dict(arrowstyle='<->', color=dim, lw=1.1))
    ax.text(0, -R - 1.8, 'OD %.1f free (%.1f at full travel);  ID %.1f free;  mean %.1f' % (S['od'] * 10, n['od_open'] * 10, S['id'] * 10, S['mean_d'] * 10), fontsize=9.3, color=dim, ha='center', va='top')
    ax.set_xlim(-26, 31)
    ax.set_ylim(-14.5, 15)

    # ---------------------------------------------------------------- B: top view
    ax2 = fig.add_axes([0.64, 0.34, 0.34, 0.60])
    ax2.set_aspect('equal')
    ax2.axis('off')
    Lb = S['length'] * 10
    ax2.add_patch(Rectangle((0, -R), Lb, 2 * R, fc='#F2EFE6', ec=ink, lw=1.6))
    ax2.add_patch(Rectangle((0, -(R - w)), Lb, 2 * (R - w), fc='white', ec=ink, lw=0.9))
    for k in range(1, int(S['body_turns']) + 1):
        ax2.plot([k * w, k * w], [R - w, R], color=thin, lw=0.7)
        ax2.plot([k * w, k * w], [-R, -(R - w)], color=thin, lw=0.7)
    xa, xb = w / 2.0, Lb - w / 2.0
    ax2.add_patch(Rectangle((xa - w / 2, -rear_len), w, rear_len, fc=spring, ec=ink, lw=1.0, alpha=0.9))
    fy0 = root_f[0]
    fy1 = fy0 + fwd_len * math.cos(math.radians(d_fwd))
    ax2.add_patch(Rectangle((xb - w / 2, fy0), w, fy1 - fy0, fc=spring, ec=ink, lw=1.0, alpha=0.9))
    ax2.annotate('', xy=(0, fy1 + 2.4), xytext=(Lb, fy1 + 2.4), arrowprops=dict(arrowstyle='<->', color=dim, lw=1.1))
    ax2.text(Lb / 2, fy1 + 2.9, 'body %.1f mm' % Lb, fontsize=9.5, color=dim, ha='center', va='bottom')
    ax2.text(Lb / 2, fy1 + 5.2, '(%.2f turns x %.1f mm\n+ one wire)' % (S['body_turns'], w), fontsize=8.8, color=dim, ha='center', va='bottom')
    ax2.text(xa + 0.2, -rear_len - 0.8, 'rear leg: pillar end\n(nearest you in A)', fontsize=9, ha='center', va='top', color=ink)
    ax2.text(xb - 0.2, fy1 + 0.6 - 3.0, '', fontsize=9)
    ax2.annotate('forward leg: arm end\n(farthest from you in A)', xy=(xb + w / 2, (fy0 + fy1) / 2), xytext=(Lb + 2.2, (fy0 + fy1) / 2 + 3.2), fontsize=9, ha='left', va='center', color=ink,
                 arrowprops=dict(arrowstyle='-', color=ink, lw=0.7))
    ax2.plot([-3, Lb + 3], [0, 0], color=thin, lw=0.7, ls=(0, (8, 3, 1, 3)))
    ax2.text(Lb + 3.2, 0, 'axis', fontsize=8.5, ha='left', va='center', color=thin)
    ax2.set_xlim(-4, Lb + 14)
    ax2.set_ylim(-rear_len - 6, fy1 + 11)

    # ---------------------------------------------------------------- table
    ax3 = fig.add_axes([0.03, 0.02, 0.94, 0.31])
    ax3.axis('off')
    rows = [
        ('Wire', 'Music wire (ASTM A228 / EN 10270-1 SH) %.2f +-0.02 mm, tensile strength about %.0f MPa (the calculation uses that). Stress-relieve after coiling (about 230 C, 30 min); shot-peen if offered; oiled, no plating.' % (S['wire'] * 10, S['uts'])),
        ('Coil', 'Mean diameter %.1f mm (ID %.1f, OD %.1f when free), %.2f turns close wound, %.1f mm long, RIGHT-HAND. It runs on a printed arbor of %.1f mm: the coil may not be tighter than ID %.1f at any point of the travel.' % (S['mean_d'] * 10, S['id'] * 10, S['od'] * 10, S['body_turns'], S['length'] * 10, AB['r'] * 20, S['id'] * 10)),
        ('Free angle', '%.1f degrees (+-3) between the leg directions, counterclockwise from the forward leg to the rear leg as in A; equal to %d turns and %.1f degrees. Legs straight and tangent, bend radius at the roots 1.6 to 2.4 mm.' % (n['free_angle'], int(S['body_turns']), n['free_angle'] - 180.0)),
        ('Torque', 'Rate %.2f N.mm per degree (+-8 %%). Opening the coil %.1f degrees from free must need %.0f N.mm and %.1f degrees %.0f N.mm (+-10 %%): the forward leg moves counterclockwise, the rear leg is held. Never loaded the other way.' % (n['k_deg'], n['pre'], n['m_rest'], n['pre'] + n['work'], n['m_full'])),
        ('Stress', 'At full travel %.0f MPa in bending = %.0f %% of the wire\'s strength (the load OPENS the coil, the supports are under the legs). Fatigue is a bench test, not a calculation: 10 000 cycles between the two torques above, torque to stay within 5 %%.' % (n['sigma'], n['sigma_pct'])),
        ('Quantity', '3: one for the 10 000 cycle test, one in the robot, one spare. A catalogue torsion spring of the same wire and coil size, within +-15 % of the rate, with its legs cut and bent to this layout, is worth trying first (the adjuster bolt moves the preload by 0.55 N per turn).'),
        ('Mount', 'Rear leg on the head of the M3 adjuster bolt (it lies over the middle, 2 mm to spare); forward leg on the 2 mm seat pin in the arm. Both legs stay within 1.6 mm of their end turn\'s plane.'),
    ]
    y = 0.98
    for k, v in rows:
        ax3.text(0.0, y, k, fontsize=10, fontweight='bold', color=dim, va='top')
        txt = _wrap(v, 150)
        ax3.text(0.09, y, txt, fontsize=9.6, color=ink, va='top')
        y -= 0.052 * (1 + txt.count('\n')) + 0.026
    fig.savefig(path_png, dpi=130)
    if path_svg:
        fig.savefig(path_svg)
    plt.close(fig)


def _wrap(text, width):
    words, lines, cur = text.split(), [], ''
    for wd in words:
        if len(cur) + len(wd) + 1 > width:
            lines.append(cur)
            cur = wd
        else:
            cur = (cur + ' ' + wd).strip()
    lines.append(cur)
    return '\n'.join(lines)


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'v3-figures')
    os.makedirs(out, exist_ok=True)
    figure(os.path.join(out, 'fig14_omni_spring.png'), os.path.join(out, 'fig14_omni_spring.svg'))
    for ln in report():
        print(ln)
    print('wrote fig14_omni_spring.png/.svg in', os.path.abspath(out))
