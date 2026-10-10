"""Figures 7 to 9 of the mechanical design (2026-10-07-theseus-v3-mechanical-design.md): the proposed structure in side section, the drive-motor snap cradle and the plan of every attachment.
python structure_figs.py [outdir]   (default ../v3-figures).  Concept drawings: parts at their spec positions, new attachments drawn schematically. cm.
9 Oct: the frame heights, the kit pocket and slot, the chute width, the nine ToF pockets (tof_mount.py), the camera boards with their cages and the IMU come from v3_params4 and tof_mount."""
import math, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle, Wedge
from shapely.geometry import LineString

import tof_mount as T
import v3_params4 as P

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'v3-figures')
os.makedirs(OUT, exist_ok=True)
C = dict(body='#5F5E5A', wheel='#2C2C2A', motor='#888780', omni='#1D9E75', bump='#D85A30', tof='#378ADD', cam='#7F77DD', plate='#BA7517', pcb='#0F6E56',
         batt='#EF9F27', chute='#993C1D', frame='#B4B2A9', lid='#E4E2DA', handle='#444441', floor='#D3D1C7', kit='#2C2C2A')

OMNI_X, OMNI_Z = P.OMNI['rest']; PIV_X, PIV_Z = P.OMNI['pivot']; TRAVEL = P.OMNI['travel']      # 9 Oct: omni centre, pivot (arm 4.6 cm) and hard stop come from v3_params4 (8 Oct: 7.0, 2.55 and 3.6)
W0, W1 = P.OMNI['y_wheel']; A0, A1 = P.OMNI['arm_y']; OM = P.OMNI_MOUNT; OP = P.OMNI_OPENING      # the real wheel y -0.98 to 1.58, the arm outboard of it, the pillar and ear, the opening in the floor
HANDLE_X, BAR_X0, BAR_X1, BAR_Z0, BAR_Z1 = 4.0, 3.3, 8.7, 14.0, 15.6    # T-handle: post on the front bridge, 5.4 cm bar along x that starts in front of the dropper unit (so the unit lifts out)
EXIT_Z = P.CHUTE['exit_axis_z']                                           # chute exit axis height at the wall (the lowest point of the 19 mm square channel is z 2.77, chute_exit.py; the terrain tables assume 2.67)

def poly(ax, pts, **kw):
    ax.add_patch(Polygon(pts, closed=True, **kw))

def rect(ax, x0, z0, x1, z1, **kw):
    poly(ax, [(x0, z0), (x1, z0), (x1, z1), (x0, z1)], **kw)

def save(fig, name, dpi=140):
    for ext in ('png', 'svg'):
        fig.savefig(os.path.join(OUT, name + '.' + ext), dpi=dpi)
    plt.close(fig)
    print('wrote', os.path.join(OUT, name + '.png'))

# ------------------------------------------------------------------------------------ figure 7: side section at y = 0
def fig_structure():
    fig, ax = plt.subplots(figsize=(17.5, 9.4))
    ax.set_aspect('equal'); ax.set_xlim(-18.0, 21.0); ax.set_ylim(-3.6, 18.2)
    ax.axhline(0, color='k', lw=2); ax.fill_between([-19, 22], -3.6, 0, color=C['floor'], alpha=0.5)
    # tub: floor with the rear chamfer, front and rear wall up to the frame
    poly(ax, [(-6.5, 3.5), (10.5, 3.5), (10.5, 8.7), (10.3, 8.7), (10.3, 3.9), (-6.5, 3.9)], fc='#EFEDE6', ec='k', lw=1.4, zorder=3)
    poly(ax, [(-6.5, 3.5), (-10.5, 5.3), (-10.5, 8.7), (-10.3, 8.7), (-10.3, 5.35), (-6.5, 3.9)], fc='#EFEDE6', ec='k', lw=1.4, zorder=3)
    # upper frame: ring front and rear, front bridge (3 mm web + rib on TOP), rear spoke with its rib; the dropper floor is a separate lift-out part resting on half-lap seats
    Z1, ZL0, ZL1 = P.FRAME['z1'], P.LID['z0'], P.LID['z1']                                   # ring top and lid (9 Oct: 11.7, and 11.7 to 12.0): the standing ToF boards made the ring 5 mm taller
    for x0, x1 in ((9.0, 10.5), (-10.5, -9.0)):
        rect(ax, x0, 8.7, x1, Z1, fc=C['frame'], ec='k', lw=1.2, zorder=4)
    rect(ax, 3.2, 8.7, 9.0, 9.0, fc=C['frame'], ec='k', lw=1.2, zorder=4)
    rect(ax, 3.25, 9.0, 9.05, 9.8, fc=C['frame'], ec='k', lw=1.2, zorder=4)
    rect(ax, -9.0, 8.7, -7.2, 8.85, fc=C['frame'], ec='k', lw=1.2, zorder=4)                      # spoke web: lower half (the ledge) all along ...
    rect(ax, -9.0, 8.85, -7.55, 9.0, fc=C['frame'], ec='k', lw=1.2, zorder=4)                     # ... upper half only up to the rebate for the floor's rear tab
    rect(ax, -9.0, 9.0, -7.9, 9.5, fc=C['frame'], ec='k', lw=1.2, zorder=4)                       # spoke rib
    rect(ax, -7.11, 8.7, 3.11, 9.0, fc='#F7F5EE', ec='#222', lw=1.5, zorder=5)                    # dropper floor disc: its own part, the base of the lift-out unit
    rect(ax, -7.5, 8.85, -7.1, 9.0, fc='#F7F5EE', ec='#222', lw=1.0, zorder=5)                    # rear tab (top half of the thickness) in the spoke's rebate
    rect(ax, 3.0, 8.85, 3.9, 9.0, fc='none', ec='#222', lw=1.0, ls='--', zorder=5)                # front tabs, beside the plane (y +-0.85 to +-1.25), in rebates of the bridge web
    # dropper plate with a kit in the front pocket (13 mm pocket on the centre line, centre x -2.0 + 3.86)
    Pl = P.PLATE; xp0, xp1 = Pl['cx'] + Pl['r_ring'] - Pl['pocket'] / 2, Pl['cx'] + Pl['r_ring'] + Pl['pocket'] / 2
    poly(ax, [(Pl['cx'] - Pl['R'], 9.0), (xp0, 9.0), (xp0, 10.2), (Pl['cx'] - Pl['R'], 10.2)], fc=C['plate'], ec='k', lw=1.0, zorder=5)
    poly(ax, [(xp1, 9.0), (Pl['cx'] + Pl['R'], 9.0), (Pl['cx'] + Pl['R'], 10.2), (xp1, 10.2)], fc=C['plate'], ec='k', lw=1.0, zorder=5)
    rect(ax, (xp0 + xp1) / 2 - P.KIT / 2, 9.0, (xp0 + xp1) / 2 + P.KIT / 2, 9.0 + P.KIT, fc=C['kit'], ec='k', lw=0.8, zorder=6)
    # N20 cartridge: dropped in from above through the floor, hangs below it, shaft up into the plate
    rect(ax, -2.5, 4.55, -1.5, 8.7, fc=C['motor'], ec='k', lw=1.0, zorder=5); rect(ax, -2.07, 8.7, -1.93, 9.8, fc='#C8C6BE', ec='k', lw=0.6, zorder=6)
    # alternative dropper motor: 28BYJ-48, body centre 0.8 cm to the left of the plate axis, front face on the floor underside; the plane y = 0 cuts its 28 mm body 8 mm off centre
    xs = math.sqrt(1.4 ** 2 - 0.613 ** 2); rect(ax, -1.486 - xs, 6.8, -1.486 + xs, 8.7, fc='none', ec='#7A4B9C', lw=1.2, ls='--', zorder=5)
    # BNO055 IMU on four 5 mm posts in the tub (the board is on the plane, the posts at y +-0.6 are beside it)
    Im = P.IMU; zi = P.Z_FLOOR_TOP + Im['post_h']
    rect(ax, Im['x'] - Im['size'][0] / 2, zi, Im['x'] + Im['size'][0] / 2, zi + Im['size'][2], fc=C['pcb'], ec='k', lw=0.8, zorder=6)
    rect(ax, Im['x'] - Im['chip'][0] / 2, zi + Im['size'][2], Im['x'] + Im['chip'][0] / 2, zi + Im['size'][2] + Im['chip'][2], fc='#222', ec='k', lw=0.6, zorder=6)
    for sxi in (1, -1):
        rect(ax, Im['x'] + sxi * Im['holes'][0] - Im['post'] / 2, P.Z_FLOOR_TOP, Im['x'] + sxi * Im['holes'][0] + Im['post'] / 2, zi, fc='none', ec='#555', lw=0.8, ls='--', zorder=5)
    # chute (beside the plane) and hopper under the slot
    ax.plot([-4.73, -6.0], [7.95, EXIT_Z], color=C['chute'], lw=6, alpha=0.35, solid_capstyle='butt', zorder=2)
    ax.plot([-4.73, -6.0], [7.95, EXIT_Z], color=C['chute'], lw=1.2, ls='--', zorder=2)
    rect(ax, -6.3, 7.2, -3.2, 8.7, fc=C['chute'], alpha=0.25, ec=C['chute'], lw=1.0, ls='--', zorder=2)
    # T-handle: post on the front bridge, bar along x
    rect(ax, HANDLE_X - 0.7, 9.8, HANDLE_X + 0.7, BAR_Z0, fc=C['handle'], ec='k', lw=1.0, zorder=7)
    rect(ax, BAR_X0, BAR_Z0, BAR_X1, BAR_Z1, fc=C['handle'], ec='k', lw=1.0, zorder=7)
    # lid with the slot for the post and the bar (x 2.95 to 9.05); no rear notch any more, the controls are at the front
    rect(ax, -10.5, ZL0, 2.95, ZL1, fc=C['lid'], ec='k', lw=1.2, zorder=6); rect(ax, 9.05, ZL0, 10.5, ZL1, fc=C['lid'], ec='k', lw=1.2, zorder=6)
    # controls: two status LEDs on top of the bridge rib, the victim LED (red) on top of the bar's front end, switch and start button on the deck beside the plane (dashed, y +1.2 to +3.1)
    rect(ax, 7.3, 9.8, 7.7, 10.2, fc='#888780', ec='k', lw=0.8, zorder=7); rect(ax, 6.4, 9.8, 6.8, 10.2, fc='#888780', ec='k', lw=0.8, zorder=7)
    rect(ax, 8.1, BAR_Z1, 8.6, 16.0, fc='#E24B4A', ec='k', lw=0.8, zorder=8)
    rect(ax, 5.35, 9.0, 6.65, 10.3, fc='none', ec='#3B6D11', lw=1.0, ls='--', zorder=7); rect(ax, 6.95, 9.0, 8.15, 9.7, fc='none', ec='#639922', lw=1.0, ls='--', zorder=7)
    rect(ax, 8.36, 7.1, 9.55, 7.9, fc='none', ec='#7A4B9C', lw=1.2, ls='--', zorder=7)                 # USB-C service socket in the front-right wall (y -4.0, beside the plane)
    # ToF F on the plane: a 25.4 mm board standing in its pocket (open to the top, the lid closes it), beam through the 18 mm tunnel; RL and RR beside the plane (dashed pocket)
    Mt, Bt = P.TOF_MOUNT, P.TOF_BOARD
    zb, zt = T.board_z(); xF = T.sensors()['F'][0]; pgF = T.polygons('F')
    px0_, _, px1_, _ = pgF['pocket'].bounds; tx0_, _, _, _ = pgF['tunnel'].bounds
    rect(ax, px0_, Mt['floor_z'], px1_, Z1, fc='white', ec='k', lw=0.8, zorder=5)
    rect(ax, tx0_, P.TOF_Z - Mt['tunnel_w'], 10.6, P.TOF_Z + Mt['tunnel_w'], fc='white', ec='none', zorder=5)
    rect(ax, xF - Bt['t'], zb, xF, zt, fc=C['tof'], ec='k', lw=0.8, zorder=7)
    rect(ax, xF, P.TOF_Z - Bt['chip_w'] / 2, xF + Bt['chip_u'], P.TOF_Z + Bt['chip_w'] / 2, fc='#042C53', ec='k', lw=0.6, zorder=7)
    xR = T.sensors()['RL'][0]
    rect(ax, xR, zb, xR + Bt['t'], zt, fc=C['tof'], ec='k', lw=0.8, zorder=7, alpha=0.6)
    rect(ax, xR - Mt['rec'], Mt['floor_z'], xR - T.u_rear(), Z1, fc='none', ec='#222', lw=0.8, ls='--', zorder=5)               # the rear boards look backwards: u points to -x
    # the left camera board and its lid hump (beside the plane, y 5.5 to 6.8): the board is tilted 20 degrees and stands up into the hump
    Cm, Km = P.CAMERA, P.CAM
    cx0 = P.CAM_X - Cm['lens_v']; tz = Km['zl'] + math.sin(math.radians(Km['tilt'])) * (Cm['tip'] + Cm['board'][2] / 2); hz = math.cos(math.radians(Km['tilt'])) * Cm['board'][0] / 2
    rect(ax, cx0, tz - hz, cx0 + Cm['board'][1], tz + hz, fc=C['cam'], alpha=0.35, ec=C['cam'], lw=1.0, ls='--', zorder=2)
    rect(ax, Cm['pocket_x'][0] - P.LID['hump_skin'], ZL1, Cm['pocket_x'][1] + P.LID['hump_skin'], P.Z_LID + P.LID['hump_skin'], fc=C['cam'], alpha=0.25, ec=C['cam'], ls='--', zorder=2)
    ax.text(3.0, 12.65, 'camera hump (left camera,\nbeside the plane)', fontsize=8, ha='right', va='bottom', color='#3C3489')
    # omni inside the body: rest and compressed, single arm
    ax.add_patch(Circle((OMNI_X, OMNI_Z), 3.0, fc=C['omni'], alpha=0.45, ec=C['omni'], lw=1.6, zorder=4))
    L = math.hypot(OMNI_X - PIV_X, OMNI_Z - PIV_Z); zc = OMNI_Z + TRAVEL; xc = PIV_X + math.sqrt(L * L - (zc - PIV_Z) ** 2)
    ax.add_patch(Circle((xc, zc), 3.0, fill=False, ls='--', ec=C['omni'], lw=1.2, zorder=4))
    ax.plot([PIV_X, OMNI_X], [PIV_Z, OMNI_Z], color='#0F6E56', lw=4, zorder=5); ax.plot([PIV_X], [PIV_Z], marker='o', color='k', ms=6, zorder=6)
    ax.plot([10.5, 10.5], [0, 3.5], color='k', ls=':', lw=0.9)
    # GIGA stack, battery and the printed posts (beside the plane, dashed)
    rect(ax, -2.76, 6.15, 7.4, 8.05, fc=C['pcb'], alpha=0.22, ec=C['pcb'], ls='--', zorder=2)
    for xp in (-2.27, -1.62, 5.87, 6.01):
        rect(ax, xp - 0.35, 3.9, xp + 0.35, 6.15, fc='none', ec='#555', lw=0.9, ls='--', zorder=3)
    rect(ax, 0.15, 5.1, 7.65, 7.6, fc=C['batt'], alpha=0.25, ec=C['batt'], ls='--', zorder=2)
    # drive wheel and motor (beside the plane), rear nub, silver module SM (y -2.7) and front port FP (y +3.5), both 7.5 cm ahead of the axle (beside the plane)
    ax.add_patch(Circle((0, 4.0), 4.0, fc=C['wheel'], alpha=0.16, ec=C['wheel'], ls='--', zorder=1))
    rect(ax, -1.0, 3.0, 1.0, 5.0, fc=C['motor'], alpha=0.55, ec='k', lw=0.8, ls='--', zorder=2)
    rect(ax, -8.4, 2.0, -7.6, 4.3, fc=C['body'], ec='k', lw=0.8, zorder=6)
    rect(ax, 6.5, 2.8, 8.5, 4.2, fc='#F0997B', ec=C['bump'], lw=0.8, ls='--', alpha=0.6, zorder=5); rect(ax, 6.7, 2.8, 8.3, 4.0, fc='none', ec=C['bump'], lw=0.8, ls=':', zorder=5)

    def lab(txt, xy, xyt, ha='left'):
        ax.annotate(txt, xy=xy, xytext=xyt, fontsize=8.6, ha=ha, va='center', color='#222', arrowprops=dict(arrowstyle='-', color='#666', lw=0.7), zorder=10)
    lab('upper frame: ring, front bridge with the control deck and\nrear spoke in ONE print, 6 x M3 down into the tub', (-9.8, 10.0), (-17.6, 16.2))
    lab('dropper unit: floor disc (own part, 3 half-lap seats),\nplate, kits, N20 and hoppers lift out in one piece', (-6.9, 8.9), (-17.6, 13.2))
    lab('rear nub on an M5 thread', (-8.0, 2.4), (-17.6, 1.0))
    lab('BNO055 IMU on four 5 mm posts\nin the tub, header edge forward', (-6.1, 4.55), (-17.6, 2.9))
    lab('hopper under the dropper floor, chute plugs into it\nthrough the wall: no gap, no pillar', (-5.2, 7.0), (-17.6, 7.4))
    lab('N20 cartridge (rev 3 motor): dropped in from above\nthrough the dropper floor, hangs below it, carries nothing', (-2.5, 6.0), (-17.6, 4.6))
    lab('floor sensors SM (y -2.7) and FP (y +3.5), 7.5 cm\nahead of the axle, beside the omni bay (dashed)', (7.5, 3.0), (12.4, 1.3))
    lab('28BYJ-48 bay kept free: body 8 mm off the\nplate axis, front-left (it fits, see section 6.3)', (-2.7, 7.9), (-17.6, 10.2))
    lab('snap-on lid, lifts straight up: 4 hooks,\nno wires in it, slot for the handle', (-3.0, ZL1), (-9.5, 14.9))
    lab('ToF F (one of nine): a 25.4 mm board standing in a pocket,\nbeam through an 18 mm tunnel; the ring is 5 mm taller (top z 11.7)', (9.56, 11.0), (12.4, 11.1))
    lab('T-handle: post on the front bridge, 5.4 cm bar from x 3.3\n(in front of the unit, so it lifts out) through the lid slot', (4.0, 14.8), (-4.5, 17.4))
    lab('front bridge: 3 mm web + rib on top\n(compressed omni tops out at z 8.5)', (8.0, 9.4), (12.4, 12.6))
    lab('controls: status LEDs on the rib; switch and start button on the\ndeck (dashed); USB-C socket in the front-right wall (dashed)', (8.4, 10.2), (9.9, 13.9))
    lab('victim LED on the bar: the highest point,\nclearly visible to the referee (rules 4.2)', (8.35, 16.0), (11.6, 17.2))
    lab('battery in a cradle with a strap', (6.8, 7.3), (12.4, 9.4))
    lab('GIGA stack on 4 printed posts,\nconnector edge toward the front', (7.2, 6.8), (12.4, 7.0))
    lab('omni fully inside the body: front\nedge x 9.85 (wall inner face 10.3)', (OMNI_X + 3.0 - 0.1, 2.3), (12.4, 3.0))
    lab('one aluminium arm on the +y side, pivot z 4.2,\ntorsion spring and stop screw (Figure 12)', (PIV_X, PIV_Z), (4.0, 0.9), ha='center')
    lab('drive motor in a snap cradle\n(Figure 8, plan in Figure 9)', (0.9, 4.6), (-3.0, -1.3), ha='center')
    ax.text(-17.7, -2.3, 'Figure 7. Proposed structure, side section at y = 0. Parts are at their spec positions; the new attachments are drawn schematically. Dashed = beside or behind the plane.\n'
            'Updated 9 Oct: ring top z 11.7 and lid 11.7 to 12.0 (standing ToF boards), 13 mm kit pocket, camera board under its lid hump (height of the board and the hump from v3_params4).', fontsize=8.8, color='#444', va='top')
    ax.set_xlabel('x (cm), forward is right'); ax.set_ylabel('z (cm)')
    fig.tight_layout(); save(fig, 'fig7_structure')

# ------------------------------------------------------------------------------------ figure 8: motor cartridge, looking along the axle
def strip(path, half):
    return list(LineString(path).buffer(half, cap_style=2, join_style=2).exterior.coords)

def callout(ax, n, xy, xyt):
    """Numbered circle at xyt with a thin leader to the point xy."""
    ax.annotate('', xy=xy, xytext=xyt, arrowprops=dict(arrowstyle='-', color='#444', lw=0.8), zorder=15)
    ax.add_patch(Circle(xyt, 0.2, fc='white', ec='#222', lw=1.0, zorder=20)); ax.text(xyt[0], xyt[1], str(n), ha='center', va='center', fontsize=8.4, fontweight='bold', zorder=21)

def fig_motor_clip():
    fig = plt.figure(figsize=(16.5, 6.4)); gs = fig.add_gridspec(1, 3, width_ratios=[1.0, 0.9, 1.15])
    # ---- left: cross-section through the hooks, seen along the axle
    ax = fig.add_subplot(gs[0]); ax.set_aspect('equal'); ax.set_xlim(-3.3, 3.3); ax.set_ylim(2.8, 5.9)
    r_seat = 1.015
    def arc_pts(side, z0, z1, n=10):
        return [(side * math.sqrt(r_seat ** 2 - (z0 + (z1 - z0) * i / n - 4.0) ** 2), z0 + (z1 - z0) * i / n) for i in range(n + 1)]
    poly(ax, [(-3.3, 3.5), (-0.883, 3.5)] + arc_pts(-1, 3.5, 3.9) + [(-3.3, 3.9)], fc='#EFEDE6', ec='k', lw=1.3, zorder=2)
    poly(ax, [(3.3, 3.5), (0.883, 3.5)] + arc_pts(1, 3.5, 3.9) + [(3.3, 3.9)], fc='#EFEDE6', ec='k', lw=1.3, zorder=2)
    poly(ax, [(-1.55, 3.55), (2.35, 3.55), (2.35, 4.6), (1.55, 4.6), (1.55, 5.1), (-1.55, 5.1)], fc='#D3D1C7', ec='#555', lw=1.0, ls='--', alpha=0.6, zorder=3)
    for s_ in (1, -1):        # prong: thin wall 2.4 mm, its inner face follows the motor up to the hook tip at z 4.65 (5.5 mm above the axle)
        arc = [(s_ * math.sqrt(1.02 ** 2 - (z - 4.0) ** 2), z) for z in (4.65, 4.55, 4.45, 4.35, 4.25, 4.15, 4.05, 4.0)]
        poly(ax, [(s_ * 1.30, 3.9), (s_ * 1.30, 4.80), (s_ * 0.80, 4.80)] + arc + [(s_ * 1.06, 3.9)], fc=C['frame'], ec='k', lw=1.1, zorder=4)
    ax.add_patch(Circle((0, 4.0), 1.0, fc=C['motor'], ec='k', lw=1.5, zorder=6)); ax.add_patch(Circle((0, 4.0), 0.2, fc='#C8C6BE', ec='k', lw=0.8, zorder=7))
    ax.set_title('Cross-section through the hooks, seen along the axle', fontsize=9.6)
    callout(ax, 1, (0.8, 4.7), (0.2, 5.5)); callout(ax, 2, (1.25, 4.2), (2.9, 5.2)); callout(ax, 3, (-0.88, 3.65), (-2.4, 3.1)); callout(ax, 4, (2.2, 4.5), (2.9, 3.1))
    ax.set_xlabel('x (cm)'); ax.set_ylabel('z (cm)')
    # ---- middle: plan view of one (left) drive motor in its cradle
    ax = fig.add_subplot(gs[1]); ax.set_aspect('equal'); ax.set_xlim(-3.6, 3.6); ax.set_ylim(0.9, 7.6)
    rect(ax, -1.0, 1.4, 1.0, 6.1, fc=C['motor'], ec='k', lw=1.4, zorder=5, alpha=0.95)                       # motor body, encoder end at y 1.4
    rect(ax, -0.2, 6.1, 0.2, 7.0, fc='#C8C6BE', ec='k', lw=0.8, zorder=5)                                    # shaft toward the wheel
    rect(ax, -1.6, 6.45, 2.75, 6.7, fc=C['frame'], ec='k', lw=1.2, zorder=4)                                   # outer web (shaft slot open at the top)
    for s_ in (1, -1):
        x0, x1 = (1.06, 1.30) if s_ > 0 else (-1.30, -1.06)
        rect(ax, x0, 2.85, x1, 6.05, fc=C['frame'], ec='k', lw=1.1, zorder=6)                                  # prong, free length 32 mm, rooted in the ledge
        poly(ax, [(s_ * 1.06, 2.85), (s_ * 0.76, 2.9), (s_ * 0.76, 3.55), (s_ * 1.06, 3.6)], fc='#8A8880', ec='k', lw=1.1, zorder=6)   # hook at the free end
        rect(ax, 1.06 if s_ > 0 else -1.6, 5.65, 2.75 if s_ > 0 else -1.06, 6.05, fc=C['frame'], ec='k', lw=1.0, zorder=4)   # ledge: front one carries the ear, rear one only roots the prong
    poly(ax, [(-1.55, 6.1), (2.35, 6.1), (2.35, 6.4), (-1.55, 6.4)], fc='none', ec='#333', lw=1.1, ls='--', zorder=7)           # face plate with its one front ear (dashed)
    callout(ax, 1, (0.9, 3.2), (2.4, 2.8)); callout(ax, 2, (-1.2, 4.6), (-2.4, 4.2)); callout(ax, 4, (2.1, 6.25), (3.1, 6.9)); callout(ax, 5, (0, 6.58), (-1.4, 7.25)); callout(ax, 6, (0, 1.4), (1.9, 1.35))
    ax.text(0.45, 7.2, 'to the wheel (y 7 to 9)', fontsize=8.2, va='center')
    ax.set_title('Plan view, left drive motor (right is the mirror image)', fontsize=9.6)
    ax.set_xlabel('x (cm)'); ax.set_ylabel('y (cm)')
    # ---- right: legend and numbers
    ax = fig.add_subplot(gs[2]); ax.axis('off'); ax.set_xlim(0, 10); ax.set_ylim(0, 10)
    legend = ['1  hook 5.5 mm above the axle, 0.75 mm undercut',
              '2  prong: a 2.4 mm wall, 11 mm tall, 32 mm free, that bends sideways',
              '3  seat: the floor cut to the motor radius (20.3 mm)',
              '4  face plate (dashed): its one ear drops into the slot beside the front ledge',
              '5  outer web (3 mm clear of the wheel hub), root of the prongs',
              '6  encoder end of the motor, free in the air']
    for k, t in enumerate(legend):
        ax.text(0.0, 9.85 - 0.5 * k, t, fontsize=8.6, va='top')
    rows = [('A snap cannot hold the torque', 'Friction of the two hooks is only 0.04 to 0.16 kg.cm in the calculation. A 2 cm riser needs 1.63 kg.cm (gearbox limit 5). The torque has to go through a key.'),
            ('Key: face plate with one ear', 'Plate on the motor once (2 x M2.5, 3.5 mm max screw depth). At the gearbox limit the ear carries 25 N on 24 mm^2 (1.0 MPa). One ear because a rear ear would hit the GIGA post. The robot itself has no screw for the motor.'),
            ('Hooks only keep it seated', 'About 1 to 6 N pull-out, 1.3 to 7.6 N push-in per prong, strain 0.4 to 1.0 % (PETG, E 2 GPa). The ground pushes the motor into its seat; test three prints (hook height, undercut).'),
            ('Why thin walls along the axle', 'A printed arm that stands up from the floor bends across the layers and cracks after a few cycles. A wall that bends sideways bends along them.'),
            ('If the hooks prove too weak', 'Add a small detent on the ear, or a finger-tight thumbscrew through the plate ear. Release: spread the prongs with two fingers.')]
    y = 6.5
    for head, txt in rows:
        ax.text(0.0, y, head, fontsize=9.2, fontweight='bold', va='top')
        ax.text(0.0, y - 0.42, txt, fontsize=8.5, va='top', color='#333', wrap=True)
        y -= 1.38
    fig.suptitle('Figure 8. Drive motor in a top-loading snap cradle (concept; exact shapes are finalised in CAD and test prints)', fontsize=10.5)
    fig.tight_layout(); save(fig, 'fig8_motor_clip')

# ------------------------------------------------------------------------------------ figure 9: plan view of every attachment point
def rrect(ax, cx, cy, w, h, ang=0.0, **kw):
    """Rectangle w (along the direction ang) by h, centred on (cx, cy)."""
    a = math.radians(ang); c, s_ = math.cos(a), math.sin(a)
    ax.add_patch(Polygon([(cx + c * dx - s_ * dy, cy + s_ * dx + c * dy) for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))], closed=True, **kw))

def seg(ax, r0, r1, a0, a1, **kw):
    ax.add_patch(Wedge((0, 0), r1, a0, a1, width=r1 - r0, **kw))

def num(ax, n, x, y):
    ax.add_patch(Circle((x, y), 0.5, fc='white', ec='#222', lw=1.0, zorder=20)); ax.text(x, y, str(n), ha='center', va='center', fontsize=8.6, fontweight='bold', zorder=21)

def polar(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))

def cradle(ax, sg):
    """Drive motor in its snap cradle, left motor for sg = +1, right motor for sg = -1 (Figure 8): motor, prongs with hooks, web, ledges, face plate with one front ear."""
    def ys(a, b):
        return (a, b) if sg > 0 else (-b, -a)
    y0, y1 = ys(1.4, 6.1); rect(ax, -1.0, y0, 1.0, y1, fc=C['motor'], ec='k', lw=1.1, zorder=5)
    for s_ in (1, -1):
        x0, x1 = (1.06, 1.30) if s_ > 0 else (-1.30, -1.06)
        y0, y1 = ys(2.85, 6.05); rect(ax, x0, y0, x1, y1, fc=C['frame'], ec='k', lw=0.8, zorder=6)
        poly(ax, [(s_ * 1.06, 2.85 * sg), (s_ * 0.76, 2.9 * sg), (s_ * 0.76, 3.55 * sg), (s_ * 1.06, 3.6 * sg)], fc=C['frame'], ec='k', lw=0.8, zorder=6)
        y0, y1 = ys(5.65, 6.05); rect(ax, 1.06 if s_ > 0 else -1.6, y0, 2.75 if s_ > 0 else -1.06, y1, fc=C['frame'], ec='k', lw=0.8, zorder=6)
    y0, y1 = ys(6.45, 6.7); rect(ax, -1.6, y0, 2.75, y1, fc=C['frame'], ec='k', lw=0.9, zorder=6)
    y0, y1 = ys(6.1, 6.4); rect(ax, -1.55, y0, 2.35, y1, fc='none', ec='#222', lw=0.8, ls='--', zorder=7)


def cam_plan(ax, s):
    """Plan view of the camera on side s (+1 left, -1 right), with the same frame as v4_model.build_cameras4 (camera frame: x along the board, y up in its plane, z along the view axis towards
    the board): the board's footprint, lens holder, cage frame, rails, flange and the two ears, the lid hump's pocket (dashed) and the window in the ring (dashed)."""
    Cm, G, K = P.CAMERA, P.CAGE, P.CAM
    t = math.radians(K['tilt']); c, sn = math.cos(t), math.sin(t)
    ty = K['tip_r'] * math.sin(math.radians(K['psi']))
    tip = Cm['tip']

    def fp(lx0, lx1, ly0, ly1, lz0, lz1):
        xs = [P.CAM_X + s * lx for lx in (lx0, lx1)]
        ys = [s * (ty - c * lz + sn * ly) for ly in (ly0, ly1) for lz in (lz0, lz1)]
        return min(xs), min(ys), max(xs), max(ys)

    def put(box, **kw):
        rect(ax, box[0], box[1], box[2], box[3], **kw)

    bw, bl, bt = Cm['board']
    fz0 = tip - G['frame_t']
    put(fp(-G['frame_x'], G['frame_x'], -G['frame_y'], G['frame_y'], fz0, tip), fc='#B4B2A9', ec='k', lw=0.8, zorder=3)                         # cage frame
    put(fp(-G['open_x'], G['open_x'], -G['open_y'], G['open_y'], fz0, tip), fc='white', ec='k', lw=0.6, zorder=3)                                   # its opening (the lens holder goes through)
    for e in (1, -1):
        put(fp(e * G['rail_x'][0], e * G['rail_x'][1], -G['rail_y'], G['rail_y'], G['flange_z'][0], fz0 + 0.05), fc='#9C9A92', ec='k', lw=0.6, zorder=3)   # rails
    put(fp(-G['flange_x'], G['flange_x'], -G['rail_y'], G['rail_y'], G['flange_z'][0], G['flange_z'][1]), fc='#8F8D86', ec='k', lw=0.8, zorder=3)  # flange with the barrel opening
    for e in (1, -1):                                                                                                                              # the ears: from the flange to the ring's inner face, one M3 screw each
        xe = P.CAM_X + e * G['ear_x']
        y_face = math.sqrt(P.FRAME['r_in'] ** 2 - (abs(xe) + G['ear_w'] / 2) ** 2)
        rect(ax, xe - G['ear_w'] / 2, min(s * 8.0, s * y_face), xe + G['ear_w'] / 2, max(s * 8.0, s * y_face), fc='#8F8D86', ec='k', lw=0.8, zorder=3)
        ax.add_patch(Circle((xe, s * (y_face - 0.45)), 0.2, fc='#444441', ec='none', zorder=4))
    put(fp(-s * Cm['lens_v'], s * (bl - Cm['lens_v']), -bw / 2, bw / 2, tip, tip + bt), fc=C['cam'], alpha=0.6, ec=C['cam'], lw=1.0, zorder=5)         # the OpenMV board: its tail points to +x on both sides
    put(fp(-Cm['holder'][1] / 2, Cm['holder'][1] / 2, -Cm['holder'][0] / 2, Cm['holder'][0] / 2, tip - Cm['holder'][2], tip), fc='#3C3489', alpha=0.8, ec='k', lw=0.6, zorder=6)   # lens holder (estimate)
    ax.add_patch(Circle((P.CAM_X, s * ty), Cm['barrel_d'] / 2, fc='#222', ec='k', lw=0.6, zorder=6))                                                  # barrel tip
    px0, px1 = Cm['pocket_x']; py0, py1 = (Cm['pocket_y'] if s > 0 else (-Cm['pocket_y'][1], -Cm['pocket_y'][0]))
    rect(ax, px0, py0, px1, py1, fc='none', ec=C['cam'], lw=1.0, ls='--', zorder=8)                                                                  # pocket under the lid hump
    wx = Cm['window_w'] / 2
    rect(ax, P.CAM_X - wx, min(s * 8.9, s * 10.5), P.CAM_X + wx, max(s * 8.9, s * 10.5), fc='none', ec=C['cam'], lw=0.8, ls=':', zorder=8)             # the window in the ring

def fig_plan():
    fig, axs = plt.subplots(1, 2, figsize=(21.0, 12.6)); fig.subplots_adjust(left=0.03, right=0.99, top=0.94, bottom=0.30, wspace=0.06)
    for ax in axs:
        ax.set_aspect('equal'); ax.set_xlim(-12.2, 12.2); ax.set_ylim(-12.2, 12.2); ax.set_xlabel('x (cm), forward is right', fontsize=9)
    axs[0].set_ylabel('y (cm), left is up', fontsize=9)
    # ======================================================= left: tub and everything below the frame (z < 8.7)
    ax = axs[0]
    ax.set_title('Tub and everything under the frame (below z 8.7)', fontsize=11)
    ax.add_patch(Circle((0, 0), 10.3, fc='#F6F5F0', ec='none', zorder=0))
    for a0, a1 in ((-65, 65), (115, 120.3), (129.3, 230.7), (239.7, 245)):            # wall with the two wheel arches (65 to 115 and 245 to 295 deg) and the two chute holes
        seg(ax, 10.3, 10.5, a0, a1, fc='#555', ec='#222', lw=0.6, zorder=3)
    for sg in (1, -1):
        arch = [polar(10.4, a) for a in (range(65, 116) if sg > 0 else range(245, 296))]
        ax.plot([p[0] for p in arch], [p[1] for p in arch], color=C['bump'], lw=2.2, ls=':', zorder=3)
        seg(ax, 10.5, 11.0, 12 if sg > 0 else -58, 58 if sg > 0 else -12, fc=C['bump'], ec='k', lw=0.6, alpha=0.75, zorder=2)               # bumper plates
        for a in (14 * sg,):
            x, y = polar(10.25, a); ax.add_patch(Rectangle((x - 0.25, y - 0.3), 0.5, 0.6, fc='k', zorder=4))                              # microswitch at the inner end
        rect(ax, -4.0, 7.0 * sg if sg > 0 else -9.0, 4.0, 9.0 * sg if sg > 0 else -7.0, fc=C['wheel'], alpha=0.35, ec=C['wheel'], lw=0.8, ls='--', zorder=1)   # drive wheel
        cradle(ax, sg)
    rect(ax, 6.5, -3.8, 8.5, -1.6, fc='#F0997B', ec=C['bump'], lw=0.8, zorder=4)                                                   # silver module SM, 7.5 cm ahead of the axle beside the omni bay
    Ff = P.FLOOR_FRONT; rrect(ax, Ff['x'], Ff['y'], Ff['w'], Ff['w'], 0, fc='#F0997B', ec=C['bump'], lw=0.8, zorder=4)           # front floor port FP: the 20.3 mm TCS34725 board, same x as SM on the +y side
    # omni (9 Oct): the real 25.6 mm wheel, one 3 mm aluminium arm outboard of it, the printed pillar (with its tail) and outer ear either side of the arm's corridor, the opening in the floor (dashed)
    rect(ax, OMNI_X - 3.0, W0, OMNI_X + 3.0, W1, fc=C['omni'], alpha=0.45, ec=C['omni'], lw=1.2, zorder=3)
    rect(ax, PIV_X, A0, OMNI_X, A1, fc='#0F6E56', ec='k', lw=0.8, zorder=5)
    for key in ('pillar', 'tail', 'ear'):
        rect(ax, OM[key]['x'][0], OM[key]['y'][0], OM[key]['x'][1], OM[key]['y'][1], fc=C['frame'], ec='k', lw=0.8, zorder=4)
    ax.add_patch(Circle((PIV_X, (A0 + A1) / 2), 0.15, fc='k', zorder=6))
    rect(ax, OP['wheel']['x'][0], OP['wheel']['y'][0], OP['wheel']['x'][1], OP['wheel']['y'][1], fc='none', ec=C['omni'], lw=0.9, ls='--', zorder=4)
    rect(ax, OP['arm']['x'][0], OP['arm']['y'][0], OP['arm']['x'][1], OP['arm']['y'][1], fc='none', ec=C['omni'], lw=0.9, ls='--', zorder=4)
    ax.add_patch(Circle((-8.0, 0), 0.6, fc=C['body'], ec='k', lw=0.8, zorder=4))                                                  # rear nub
    ux_, uy_ = polar(9.8, -24.0); rrect(ax, ux_, uy_, 1.3, 1.4, -24.0, fc='#7A4B9C', ec='k', lw=0.8, zorder=6)                       # USB-C service socket in the front-right wall (flush, 2 cm from J12)
    wx_, wy_, wa_ = P.antenna_pose(); rrect(ax, wx_, wy_, 0.2, 0.64, wa_, fc='#D4537E', ec='k', lw=0.6, zorder=6)                       # Wi-Fi/BLE antenna strip (15.4 mm tall) stuck on the inside of the front wall, drawn thicker than its 0.5 mm
    # GIGA stack on four posts; connector edge at the front
    rect(ax, -2.756, -6.697, 7.396, -1.363, fc=C['pcb'], alpha=0.25, ec=C['pcb'], lw=1.0, ls='--', zorder=2)
    for xp, yp in ((5.87, -1.62), (-1.62, -1.62), (6.01, -6.43), (-2.27, -6.43)):
        ax.add_patch(Circle((xp, yp), 0.35, fc='#222', ec='k', zorder=6))
    for xp, yp in ((0.79, -3.14), (0.79, -5.92)):
        ax.add_patch(Circle((xp, yp), 0.35, fc='none', ec='#222', lw=0.8, ls=':', zorder=6))
    rect(ax, 7.396, -3.0, 7.7, -2.48, fc='#7A4B9C', ec='k', lw=0.8, zorder=6)                                                      # USB-C J12 on the connector edge
    # battery tray, N20 cartridge, 28BYJ-48 bay, hoppers and chutes
    rrect(ax, 3.9, 4.35, 7.0, 3.5, 10, fc=C['batt'], alpha=0.22, ec=C['batt'], lw=1.2, ls='--', zorder=3)
    for dx in (-1.6, 1.6):
        rrect(ax, 3.9 + dx * math.cos(math.radians(10)), 4.35 + dx * math.sin(math.radians(10)), 0.25, 4.0, 10, fc='#555', alpha=0.6, zorder=4)   # strap
    rrect(ax, -2.0, 0, 1.0, 1.2, 0, fc=C['motor'], ec='k', lw=1.0, zorder=5)
    th = [2 * math.pi * k / 90 for k in range(91)]
    ux, uy = math.cos(math.radians(230)), math.sin(math.radians(230)); cx_, cy_ = -2.0 - 0.8 * ux, -0.8 * uy      # 28BYJ-48 bay at 230 degrees: body centre (-1.49, 0.61)
    ax.plot([cx_ + 1.4 * math.cos(t) for t in th], [cy_ + 1.4 * math.sin(t) for t in th], color='#7A4B9C', lw=1.4, ls='--', zorder=6)
    for sgn in (1, -1):
        ax.add_patch(Circle((cx_ + sgn * 1.75 * (-uy), cy_ + sgn * 1.75 * ux), 0.35, fc='none', ec='#7A4B9C', lw=1.0, ls='--', zorder=6))
    rrect(ax, cx_ - 1.45 * ux, cy_ - 1.45 * uy, 0.5, 1.46, 230, fc='none', ec='#7A4B9C', lw=1.0, ls='--', zorder=6)
    rrect(ax, -2.0, 0, 1.0, 1.2, 0, fc=C['motor'], ec='k', lw=1.0, zorder=5)
    for sy in (1, -1):
        sx, sy2 = -4.73, 2.73 * sy; ex, ey = -6.0, 8.62 * sy
        ax.add_patch(Polygon(list(LineString([(sx, sy2), (ex, ey)]).buffer(P.CHUTE['out_w'] / 2, cap_style=2).exterior.coords), closed=True, fc=C['chute'], alpha=0.30, ec=C['chute'], lw=0.9, zorder=2))
        rrect(ax, sx, sy2, P.CHUTE['hopper_out'], P.CHUTE['hopper_out'], 45, fc=C['chute'], alpha=0.45, ec=C['chute'], lw=1.0, zorder=3)
    Im = P.IMU                                                                                                                       # BNO055 IMU board on four 5 mm posts, behind the dropper bay
    rrect(ax, Im['x'], Im['y'], Im['size'][0], Im['size'][1], 0, fc=C['pcb'], alpha=0.35, ec=C['pcb'], lw=1.0, zorder=4)
    for sxi in (1, -1):
        for syi in (1, -1):
            ax.add_patch(Circle((Im['x'] + sxi * Im['holes'][0], Im['y'] + syi * Im['holes'][1]), Im['post'] / 2, fc='#222', ec='k', lw=0.6, zorder=6))
    ax.add_patch(Circle((-2.0, 0), 5.11, fill=False, ec='#999', lw=0.8, ls=':', zorder=1))
    for a in (12, -12, 60, -60, 118, -118):                                                                                          # six M3 insert bosses (with a rib to the wall) for the upper frame
        x, y = polar(9.75, a); ax.add_patch(Circle((x, y), 0.4, fc='#DDD', ec='k', lw=1.0, zorder=6)); ax.add_patch(Circle((x, y), 0.17, fc='k', zorder=7))
    # callouts (numbers, legend below)
    for n, x, y in ((1, 0.0, 10.9), (2, 1.9, 4.0), (3, 5.2, 1.3), (4, -8.0, 1.6), (5, 4.9, -2.7), (6, 9.1, -1.7), (7, 4.4, 5.6), (8, -2.0, -1.4), (9, -2.0, 3.1), (10, -4.73, 4.3),
                    (11, *polar(8.5, 60)), (12, 9.4, 3.4), (12, 7.5, -4.6), (13, *polar(11.8, 35)), (14, 7.6, -6.3), (15, 11.55, -0.6), (16, -7.3, -1.6)):
        num(ax, n, x, y)
    # ======================================================= right: upper frame and lid
    ax = axs[1]
    ax.set_title('Upper frame (z 8.7 to 11.7) and lid (11.7 to 12.0)', fontsize=11)
    seg(ax, 9.0, 10.5, 0, 360, fc='#B4B2A9', ec='k', lw=1.0, zorder=2)
    ax.add_patch(Circle((-2.0, 0), 5.11, fc='#D9D7CD', ec='k', lw=0.9, zorder=1))                                                    # dropper floor disc
    rect(ax, 3.2, -1.25, 9.1, 1.25, fc='#B4B2A9', ec='k', lw=0.9, zorder=2); rect(ax, -9.1, -1.0, -7.2, 1.0, fc='#B4B2A9', ec='k', lw=0.9, zorder=2)
    rect(ax, 5.2, 1.25, 9.15, 3.6, fc='#B4B2A9', ec='k', lw=0.9, zorder=2)                                                           # control deck: the bridge web widened to the +y side
    for x0_, x1_, y0_, y1_ in ((3.0, 3.9, 0.85, 1.25), (3.0, 3.9, -1.25, -0.85), (-7.5, -6.9, -0.5, 0.5)):
        rect(ax, x0_, y0_, x1_, y1_, fc='#D9D7CD', ec='k', lw=0.8, zorder=3)                                                         # the dropper floor's three seat tabs
    for sy in (1, -1):
        rrect(ax, -4.73, 2.73 * sy, P.PLATE['slot'], P.PLATE['slot'], 45, fc='white', ec='k', lw=0.8, zorder=3)                    # slots A and B (17 mm)
    rrect(ax, -2.0, 0, 1.04, 1.24, 0, fc='white', ec='k', lw=0.8, zorder=3)                                                         # N20 pocket in the floor
    ax.add_patch(Circle((-2.0, 0), 4.91, fill=False, ec=C['plate'], lw=1.6, zorder=4))                                               # kit plate (above the floor)
    for i in range(8):
        a = math.radians(225 + 30 + 30 * i); rrect(ax, P.PLATE['cx'] + P.PLATE['r_ring'] * math.cos(a), P.PLATE['r_ring'] * math.sin(a), P.PLATE['pocket'], P.PLATE['pocket'], math.degrees(a), fc=C['kit'], alpha=0.5, ec='none', zorder=4)
    for nm in T.sensors():                                                                                                          # the nine ToF pockets from tof_mount.py: the printed block, the pocket, the standing board
        pg = T.polygons(nm); x, y, a = T.sensors()[nm]
        poly(ax, list(pg['block'].exterior.coords), fc='#9C9A92', ec='k', lw=0.6, zorder=3)
        poly(ax, list(pg['pocket'].exterior.coords), fc='white', ec='k', lw=0.8, zorder=4)
        poly(ax, list(pg['pcb'].exterior.coords), fc=C['tof'], ec='k', lw=0.6, zorder=5)
        k = 11.4 / math.hypot(x, y)                                                                                                 # the label sits outside the ring
        ax.text(x * k, y * k, nm, fontsize=7.4, ha='center', va='center', color='#0C447C', zorder=6)
    for sy in (1, -1):
        cam_plan(ax, sy)                                                                                                            # camera boards on their cages, lid hump pockets and ring windows
    rect(ax, 5.35, 1.2, 6.65, 3.1, fc='#3B6D11', ec='k', lw=0.8, zorder=6)                                                          # power switch on the deck
    for (px, py, r, col) in ((7.55, 2.15, 0.6, '#639922'), (7.5, 0.0, 0.2, '#888'), (6.6, 0.0, 0.2, '#888')):   # start button, two status LEDs (on the rib)
        ax.add_patch(Circle((px, py), r, fc=col, ec='k', lw=0.6, zorder=6))
    ux_, uy_ = polar(9.8, -24.0); rrect(ax, ux_, uy_, 1.3, 1.4, -24.0, fc='none', ec='#7A4B9C', lw=1.3, ls='--', zorder=7)                # USB-C service socket in the front-right wall (z 7.1 to 7.9, under the ring and the FR pocket)
    for a in (12, -12, 60, -60, 118, -118):                                                                                          # frame screws M3, heads recessed under the lid
        x, y = polar(9.75, a); ax.add_patch(Circle((x, y), 0.33, fc='#DDD', ec='k', lw=1.0, zorder=6)); ax.add_patch(Circle((x, y), 0.15, fc='k', zorder=7))
    ax.add_patch(Circle((0, 0), 10.6, fill=False, ec='#444', lw=1.2, ls='--', zorder=7))                                             # lid outline
    for a in (71, -71, 114, -114):                                                                                                   # four lid hooks (tongues in the skirt)
        x, y = polar(10.6, a); rrect(ax, x, y, 0.5, 1.8, a + 90, fc=C['bump'], ec='k', lw=0.8, zorder=8)
    for a in (12, -12):
        x, y = polar(10.6, a); rrect(ax, x, y, 0.3, 1.0, a + 90, fc='white', ec='#444', lw=0.8, zorder=8)                              # finger notches
    rect(ax, 2.95, -0.9, 9.05, 0.9, fc='none', ec='#444', lw=1.0, ls='--', zorder=8); rect(ax, 4.9, 0.7, 9.2, 3.5, fc='none', ec='#444', lw=1.0, ls='--', zorder=8)         # lid slot and front notch
    ax.add_patch(Circle((HANDLE_X, 0), 0.7, fc=C['handle'], ec='k', lw=0.8, zorder=9)); rect(ax, BAR_X0, -0.8, BAR_X1, 0.8, fc=C['handle'], alpha=0.55, ec='k', lw=0.8, zorder=8)
    ax.add_patch(Circle((8.35, 0.0), 0.25, fc='#E24B4A', ec='k', lw=0.8, zorder=10))                                                 # victim LED on top of the bar's front end
    for n, x, y in ((1, 11.5, -1.5), (2, -2.4, 11.2), (3, 5.4, -0.8), (4, 4.0, 0.0), (5, -2.0, -3.5), (6, -8.0, 1.5), (7, 6.6, 3.6), (8, *polar(11.8, 55)), (9, *polar(11.8, 71)), (10, *polar(11.8, 12)),
                    (11, 7.0, -1.8), (12, -2.0, 1.9)):
        num(ax, n, x, y)
    # legends
    LA = ['1  wheel arch in the lower wall: the wheel slides 1 cm out along its shaft and drops out downwards (dotted)',
          '2  drive motor cartridge in its snap cradle: prongs, outer web, face plate with one front ear (Figure 8); two M2.5 once, none on the robot',
          '3  omni: Nexus 14145, one aluminium arm on the +y side, pivot tube between a printed pillar and ear (Figure 12); dashed = floor opening',
          '4  rear nub on an M5 thread, jam nut',
          '5  GIGA on 4 printed posts (M3 inserts); hollow dotted = H5 and H6 hole positions, unused (over the motor)',
          '6  USB-C J12 and the reset button on the front edge (boot button under the ring): right-angle plug, 2 cm to the front-right socket',
          '7  battery tray with a notch over the left motor, velcro strap, XT30 at the front end; the battery leaves 4.5 cm back and 1.5 cm or more inboard',
          '8  N20 cartridge in the dropper floor pocket (rev 3 motor)',
          '9  28BYJ-48 bay kept free (dashed purple): body 8 mm off the plate axis, toward the front-left (offset direction 230 degrees)',
          '10 hopper under each 17 mm slot (sealed to the dropper floor, lifts out with it); 19 mm square channel plugs through the wall hole',
          '11 six M3 insert bosses that carry the upper frame',
          '12 floor sensors 7.5 cm ahead of the axle: SM silver module (y -2.7, placeholder) and FP (y +3.5, 20.3 mm TCS34725 board); holders not designed yet',
          '13 bumper plates 12 to 58 degrees, hinge at the outer end, microswitch (black) at the inner end',
          '14 USB-C service socket in the front-right wall at -24 degrees (panel mount, flush, 2 cm cable to J12 on the GIGA)',
          '15 Wi-Fi/BLE antenna (the flat strip from the GIGA box, 15 x 6 mm) stuck on the recess wall behind the right bumper at -22 degrees, 100 mm cable to the u.FL socket J14; test mode only',
          '16 BNO055 IMU breakout (27 x 20 mm) on four 5 mm posts, header edge forward, four M2.5 self-tapping screws (v3_params4.IMU)']
    LB = ['1  nine ToF boards (Adafruit 3317, 25.4 x 17.8 mm) stand in pockets on two M2.5 screws; the lower connector plugs down through the floor',
          '2  OpenMV H7 Plus tilted 20 degrees on a printed cage: 4 x M2.5 for the board, 2 x M3 through the ears into the ring; dashed: hump pocket, window',
          '3  front bridge: 3 mm web (underside z 8.7) with a rib on top; two of the dropper floor\'s three half-lap seats are beside the rib',
          '4  T-handle: post on the bridge rib at x 4.0, 5.4 cm bar (x 3.3 to 8.7, z 14.0 to 15.6) through the lid slot; it stops in front of the dropper unit',
          '5  dropper floor disc R 5.11 (own part) with slots A and B, the N20 pocket and three seat tabs; plate R 4.91 (orange ring, 8 kits) rides on it; the unit lifts out',
          '6  rear spoke (3 mm web with a rib): carries the rear tab of the floor',
          '7  control deck on the +y side of the bridge: power switch and start button; two status LEDs on the rib; the victim LED (red) is on top of the bar; USB-C socket in the wall',
          '8  six M3 screws, heads recessed under the lid, at +-12, +-60 and +-118 degrees (ring_gaps.py)',
          '9  four lid hooks (tongues cut into the skirt, they flex in the layer plane) at +-71 and +-114 degrees',
          '10 finger notches at +-12 degrees',
          '11 lid: slot for the post and bar (x 2.95 to 9.05) joined to a front notch over the deck (both dashed); nothing electrical in the lid',
          '12 kit plate pockets (8 kits in 13 mm pockets, loaded with the lid off)']
    fig.text(0.03, 0.27, 'Left panel', fontsize=9.6, fontweight='bold', va='top'); fig.text(0.03, 0.248, '\n'.join(LA), fontsize=8.2, va='top', linespacing=1.55)
    fig.text(0.515, 0.27, 'Right panel', fontsize=9.6, fontweight='bold', va='top'); fig.text(0.515, 0.248, '\n'.join(LB), fontsize=8.2, va='top', linespacing=1.55)
    fig.suptitle('Figure 9. Where everything attaches (plan, x forward, y left). Concept: positions from the packing and the checks, shapes schematic; ToF pockets, camera cages and IMU as modelled on 9 Oct.', fontsize=11.5)
    save(fig, 'fig9_mounting', dpi=105)


if __name__ == '__main__':
    fig_structure()
    fig_motor_clip()
    fig_plan()
