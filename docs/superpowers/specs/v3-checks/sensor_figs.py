"""Figures 10 and 11 of the mechanical design (2026-10-07-theseus-v3-mechanical-design.md): the ToF boards and their mount, and the OpenMV camera with what is known and what must be measured.
python sensor_figs.py [outdir]   (default ../v3-figures). Needs matplotlib and shapely. All numbers come from v3_params4 and tof_mount; cm unless a label says mm."""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MPoly, Rectangle, Wedge

import tof_mount as T
import v3_params4 as P

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'v3-figures')
os.makedirs(OUT, exist_ok=True)
C = dict(body='#5F5E5A', frame='#B4B2A9', tof='#378ADD', cam='#7F77DD', pcb='#0F6E56', est='#D85A30', known='#185FA5', lid='#E4E2DA', wheel='#2C2C2A', soft='#EFEDE6', ok='#1D9E75')


def save(fig, name, dpi=140):
    for ext in ('png', 'svg'):
        fig.savefig(os.path.join(OUT, name + '.' + ext), dpi=dpi)
    plt.close(fig)
    print('wrote', os.path.join(OUT, name + '.png'))


def poly(ax, pts, **kw):
    ax.add_patch(MPoly(pts, closed=True, **kw))


def rect(ax, x0, y0, x1, y1, **kw):
    poly(ax, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], **kw)


def dim(ax, p, q, text, off=0.0, fs=7, color='#222'):
    ax.annotate('', xy=q, xytext=p, arrowprops=dict(arrowstyle='<->', color=color, lw=0.7))
    mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
    if abs(p[0] - q[0]) < 1e-9:                                                      # a vertical dimension: the text turned and left of the line
        ax.text(mx - 0.08, my, text, fontsize=fs, ha='right', va='center', rotation=90, color=color, bbox=dict(fc='white', ec='none', pad=0.5, alpha=0.85))
    else:
        ax.text(mx, my + off, text, fontsize=fs, ha='center', va='bottom', color=color, bbox=dict(fc='white', ec='none', pad=0.5, alpha=0.85))


# ---------------------------------------------------------------------------------------------------- figure 10: ToF boards and their mount
def fig_tof():
    B, M = P.TOF_BOARD, P.TOF_MOUNT
    fig = plt.figure(figsize=(19.0, 9.4))
    gs = fig.add_gridspec(1, 3, width_ratios=[0.85, 1.35, 1.7], wspace=0.06)
    box = dict(fc='white', ec='none', pad=1.0, alpha=0.9)
    # A: front view of the board (the chip side), standing: v across, w up
    ax = fig.add_subplot(gs[0])
    ax.set_aspect('equal'); ax.set_xlim(-2.6, 3.2); ax.set_ylim(-2.7, 2.6); ax.axis('off')
    S, Lh = B['short'] / 2, B['long'] / 2
    rect(ax, -S, -Lh, S, Lh, fc='#CDE3F7', ec='k', lw=1.3)
    for sv in (1, -1):
        for sw in (1, -1):
            ax.add_patch(Circle((sv * B['hole_short'], sw * B['hole_long']), B['hole_d'] / 2, fc='white', ec='k', lw=0.9))
            ax.add_patch(Circle((sv * B['hole_short'], sw * B['hole_long']), B['pad_d'] / 2, fc='none', ec='#BA7517', lw=0.7))
    conns = []
    for sw in (1, -1):
        e0, e1 = Lh - B['conn_end'] - B['conn_len'], Lh - B['conn_end']
        lo, hi = (e0, e1) if sw > 0 else (-e1, -e0)
        rect(ax, -B['conn_w'] / 2, lo, B['conn_w'] / 2, hi, fc='#EEEEEE', ec='k', lw=0.9)
        conns.append(((B['conn_w'] / 2, (lo + hi) / 2), sw))
    rect(ax, -B['chip_v'] / 2, -B['chip_w'] / 2, B['chip_v'] / 2, B['chip_w'] / 2, fc='#042C53', ec='k', lw=0.8)
    for sv in (1, -1):                                                              # the two posts at the upper holes
        rect(ax, sv * B['hole_short'] - M['post_r'], B['hole_long'] - M['post_r'], sv * B['hole_short'] + M['post_r'], B['hole_long'] + M['post_r'], fc='none', ec=C['est'], lw=1.1, ls='--')
    ax.annotate('VL53L0X chip', xy=(B['chip_v'] / 2, 0), xytext=(1.25, 0.0), fontsize=7, va='center', arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.annotate('top connector (unused)', xy=conns[0][0], xytext=(1.25, 1.15), fontsize=7, va='center', arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.annotate('bottom connector:\nthe plug goes up\nthrough the floor', xy=conns[1][0], xytext=(1.25, -1.15), fontsize=7, va='center', arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.annotate('the two posts of the mount', xy=(B['hole_short'] + M['post_r'], B['hole_long']), xytext=(1.25, 2.05), fontsize=7, va='center', color=C['est'], arrowprops=dict(arrowstyle='->', lw=0.7, color=C['est']))
    dim(ax, (-S - 0.35, -Lh), (-S - 0.35, Lh), '25.4 mm', off=-0.1)
    dim(ax, (-S, -Lh - 0.3), (S, -Lh - 0.3), '17.78 mm', off=-0.42)
    dim(ax, (-S - 0.85, -B['hole_long']), (-S - 0.85, B['hole_long']), '20.32', off=-0.05)
    dim(ax, (-B['hole_short'], Lh + 0.22), (B['hole_short'], Lh + 0.22), '12.7', off=0.0)
    ax.set_title('A  The board: Adafruit 3317, July 2020 version, chip side', fontsize=10, weight='bold')
    ax.text(0.3, -2.62, 'from the EAGLE file: R 2.54 mm corners, 2.5 mm holes in 3.2 mm pads,\nconnectors 6.1 x 4.25 x about 3 mm, 0.58 mm in from each end', ha='center', fontsize=6.8, color='#333')
    # B: section along the beam through the board's centre (u horizontal outwards, z up)
    ax = fig.add_subplot(gs[1])
    ax.set_aspect('equal'); ax.set_xlim(-2.1, 2.7); ax.set_ylim(7.9, 13.2); ax.axis('off')
    z0, z1, zf = P.FRAME['z0'], P.FRAME['z1'], M['floor_z']
    zb, zt = T.board_z()
    ur, rw = T.u_rear(), M['rear_wall']
    rect(ax, ur - rw, z0, ur, z1, fc=C['frame'], ec='k', lw=1.1)                                 # rear wall
    rect(ax, ur - rw, z0, 2.7, zf, fc=C['frame'], ec='k', lw=1.1)                                  # floor
    rect(ax, M['rec'], zf, 2.7, P.TOF_Z - M['tunnel_w'], fc=C['frame'], ec='k', lw=1.1)            # wall below the tunnel
    rect(ax, M['rec'], P.TOF_Z + M['tunnel_w'], 2.7, z1, fc=C['frame'], ec='k', lw=1.1)            # wall above the tunnel
    rect(ax, -B['t'], zb, 0.0, zt, fc='#CDE3F7', ec='k', lw=1.1)                                     # PCB
    rect(ax, 0.0, P.TOF_Z - B['chip_w'] / 2, B['chip_u'], P.TOF_Z + B['chip_w'] / 2, fc='#042C53', ec='k', lw=0.8)
    for sw in (1, -1):
        e0, e1 = Lh - B['conn_end'] - B['conn_len'], Lh - B['conn_end']
        lo, hi = (e0, e1) if sw > 0 else (-e1, -e0)
        rect(ax, 0.0, P.TOF_Z + lo, B['conn_h'], P.TOF_Z + hi, fc='#EEEEEE', ec='k', lw=0.8)
    zh = T.hole_z()
    rect(ax, 0.0, zh - M['post_r'], M['rec'] + 0.06, zh + M['post_r'], fc=C['frame'], ec='k', lw=1.0)                    # post
    rect(ax, -0.05, zh - M['pilot_r'], 0.45, zh + M['pilot_r'], fc='white', ec='k', lw=0.6)                               # pilot
    hd, hh = B['screw_head']
    rect(ax, -B['t'] - hh, zh - hd / 2, -B['t'], zh + hd / 2, fc='#5F5E5A', ec='k', lw=0.8)                            # screw head
    rect(ax, -B['t'], zh - 0.1, -B['t'] + B['screw_len'], zh + 0.1, fc='#5F5E5A', ec='k', lw=0.8)                       # screw shaft
    rect(ax, ur - rw - 0.05, zh - M['access_r'], ur + 0.02, zh + M['access_r'], fc='white', ec='k', lw=0.6)             # access hole
    rect(ax, -0.05, z0 - 0.1, M['rec'] + 0.04, zf + 0.05, fc='white', ec='none', lw=0)                                    # plug shaft
    ax.plot([-0.05, -0.05], [z0, zf], color='k', lw=1.0); ax.plot([M['rec'] + 0.04, M['rec'] + 0.04], [z0, zf], color='k', lw=1.0)
    rect(ax, ur - rw - 0.1, z1, 2.7, z1 + 0.3, fc=C['lid'], ec='k', lw=1.0)                                                # lid
    half = math.radians(12.5)
    ax.fill([B['chip_u'], 2.7, 2.7], [P.TOF_Z, P.TOF_Z + (2.7 - B['chip_u']) * math.tan(half), P.TOF_Z - (2.7 - B['chip_u']) * math.tan(half)], color=C['tof'], alpha=0.18, lw=0)
    ax.annotate('the screwdriver reaches the screws\nthrough two holes in the rear wall', xy=(ur - rw / 2, zh), xytext=(-2.05, 12.75), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7), bbox=box)
    ax.annotate('two M2.5 x 6 screws pull the board\nonto the two posts (self-tapping)', xy=(-0.1, zh + 0.1), xytext=(0.25, 12.4), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7), bbox=box)
    ax.annotate('the plug of the lower connector and its cable\ngo down through the floor into the tub', xy=(0.15, (z0 + zf) / 2), xytext=(0.5, 8.05), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7), bbox=box)
    ax.annotate('beam tunnel 18 x 18 mm;\nthe cone (+-12.5 degrees) fits', xy=(2.0, P.TOF_Z + 0.2), xytext=(1.0, 11.0), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7), bbox=box)
    ax.text(ur - rw / 2, 8.15, 'rear\nwall', fontsize=7, ha='center', va='bottom')
    ax.text(-B['t'] / 2 - 0.02, zb - 0.05, 'PCB\n1.6', fontsize=6.5, ha='center', va='top')
    ax.text(0.9, zf + 0.05, 'pocket floor z %.1f' % zf, fontsize=7, bbox=box)
    ax.text(ur - rw - 0.05, z1 + 0.34, 'lid z %.1f' % z1, fontsize=7, va='bottom')
    dim(ax, (-1.15, zb), (-1.15, zt), '25.4 mm,\nthe board stands\non its short end', off=-1.0)
    ax.set_title('B  Section along the beam (cm, z up)', fontsize=10, weight='bold')
    # C: plan of the ring with the nine pockets
    ax = fig.add_subplot(gs[2])
    ax.set_aspect('equal'); ax.set_xlim(-11.4, 11.6); ax.set_ylim(-11.6, 11.4); ax.axis('off')
    ax.add_patch(Circle((0, 0), P.R_BODY, fc=C['soft'], ec='k', lw=1.4))
    ax.add_patch(Circle((0, 0), P.FRAME['r_in'], fc='white', ec='k', lw=0.9))
    for nm in T.sensors():
        pg = T.polygons(nm)
        poly(ax, list(pg['block'].exterior.coords), fc=C['frame'], ec='none', alpha=0.9, zorder=2)
        poly(ax, list(pg['tunnel'].exterior.coords), fc=C['tof'], ec='none', alpha=0.15, zorder=3)
        poly(ax, list(pg['pocket'].exterior.coords), fc='white', ec='k', lw=0.9, zorder=4)
        poly(ax, list(pg['pcb'].exterior.coords), fc='#CDE3F7', ec='k', lw=0.8, zorder=5)
        for post in pg['posts']:
            poly(ax, list(post.exterior.coords), fc=C['est'], ec='none', alpha=0.8, zorder=6)
        x, y, aim = T.sensors()[nm]
        ax.text(x * 0.68, y * 0.68, nm, fontsize=7.5, ha='center', va='center', color='#0C447C', zorder=8)
        a = math.radians(aim)
        ax.annotate('', xy=(x + 3.0 * math.cos(a), y + 3.0 * math.sin(a)), xytext=(x, y), arrowprops=dict(arrowstyle='->', color=C['tof'], lw=0.9), zorder=7)
    cam_x = P.CAM_X
    for s in (1, -1):
        rect(ax, cam_x - 1.4, s * 8.4 if s > 0 else -10.8, cam_x + 1.4, 10.8 if s > 0 else -8.4, fc='white', ec='#3C3489', lw=1.0, ls='--', zorder=3)
    for a in P.screw_angles():
        x, y = P.polar(P.SCREW['r'], a)
        ax.add_patch(Circle((x, y), P.SCREW['cbore_r'], fc='#444441', ec='none', zorder=6))
    ax.set_title('C  The nine pockets in the ring, plan (front to the right). Grey: the printed block; blue: beam tunnel;\nred: posts; dots: frame screws; dashed: camera windows', fontsize=9.5, weight='bold')
    ax.text(0, -11.45, 'moved on 9 Oct: FL, FR 1 mm and 1 degree; the side sensors x +-7.18 -> +-6.85; the rear ones y +-4.5 -> +-3.6; F unchanged', ha='center', fontsize=7.2, color='#333')
    save(fig, 'fig10_tof_mount')


# ---------------------------------------------------------------------------------------------------- figure 11: the camera, what is known and what to measure
def fig_camera():
    C_, G = P.CAMERA, P.CAGE
    bw, bl, bt = C_['board']
    fig = plt.figure(figsize=(19.0, 10.0))
    gs = fig.add_gridspec(1, 2, width_ratios=[0.9, 1.3], wspace=0.05)
    ax = fig.add_subplot(gs[0])
    ax.set_aspect('equal'); ax.set_xlim(-1.5, 5.6); ax.set_ylim(-0.4, 5.6); ax.axis('off')

    def Y(v):
        return 5.0 - v                                                              # v down from the camera end at the top
    hu, hv = C_['holder'][0], C_['holder'][1]
    lu, lv = C_['lens_u'], C_['lens_v']
    rect(ax, 0, Y(bl), bw, Y(0), fc='#F2B6A0', ec='k', lw=1.3)
    for u, v, d in C_['holes']:
        ax.add_patch(Circle((u, Y(v)), d / 2, fc='white', ec='k', lw=0.9))
    rect(ax, lu - hu / 2, Y(lv + hv / 2), lu + hu / 2, Y(lv - hv / 2), fc='#3C3489', ec=C['est'], lw=1.4, ls='--', alpha=0.55)
    ax.add_patch(Circle((lu, Y(lv)), C_['barrel_d'] / 2, fc='#222', ec=C['est'], lw=1.4, ls='--', alpha=0.85))
    ax.plot([lu], [Y(lv)], 'w+', ms=8)
    # known dimensions, blue
    dim(ax, (-0.35, Y(bl)), (-0.35, Y(0)), '44.45', color=C['known'])
    dim(ax, (0, Y(bl) - 0.3), (bw, Y(bl) - 0.3), '35.56', color=C['known'], off=-0.42)
    dim(ax, (C_['holes'][0][0], Y(2.2)), (C_['holes'][1][0], Y(2.2)), '29.5 between the upper holes', color=C['known'], off=-0.02)
    dim(ax, (bw + 0.3, Y(0)), (bw + 0.3, Y(C_['holes'][0][1])), '3.1', color=C['known'])
    dim(ax, (bw + 0.75, Y(0)), (bw + 0.75, Y(C_['holes'][2][1])), '8.2', color=C['known'])
    ax.text(bw + 1.0, Y(0.2), 'holes: 3.0 and 2.4 mm from the left edge,\n32.5 and 33.3 from it on the right;\ndrilled 2.8 and 3.05 mm', fontsize=7, color=C['known'], va='top')
    # dimensions to measure, orange
    dim(ax, (lu + hu / 2 + 0.25, Y(0)), (lu + hu / 2 + 0.25, Y(lv)), 'M1', color=C['est'])
    dim(ax, (0, Y(2.9)), (lu, Y(2.9)), 'M2', color=C['est'], off=-0.02)
    dim(ax, (lu - hu / 2, Y(lv + hv / 2) - 0.25), (lu + hu / 2, Y(lv + hv / 2) - 0.25), 'M3', color=C['est'], off=-0.42)
    dim(ax, (lu + hu / 2 + 0.6, Y(lv - hv / 2)), (lu + hu / 2 + 0.6, Y(lv + hv / 2)), 'M4', color=C['est'])
    ax.text(lu, Y(lv) - 0.05, 'M5', fontsize=7.5, color='white', ha='center', va='top')
    ax.text(0.2, Y(3.6), 'micro-SD socket on the BACK, about 2 mm tall (M6)', fontsize=7, color='#333')
    ax.set_title('A  OpenMV Cam H7 Plus seen from the front (lens side), camera end at the top', fontsize=10, weight='bold')
    # B: side section through the camera in the robot (y horizontal outwards to the right, z up), left side camera
    ax = fig.add_subplot(gs[1])
    ax.set_aspect('equal'); ax.set_xlim(4.2, 11.2); ax.set_ylim(7.4, 13.0); ax.axis('off')
    t = math.radians(P.CAM['tilt'])
    tip = (P.CAM['tip_r'] * math.sin(math.radians(P.CAM['psi'])), P.CAM['zl'])
    ez = (-math.cos(t), math.sin(t))                                                  # towards the board: inwards and up
    ey = (math.sin(t), math.cos(t))                                                   # up in the board's plane

    def W(ly, lz):
        return (tip[0] + ly * ey[0] + lz * ez[0], tip[1] + ly * ey[1] + lz * ez[1])

    rect(ax, 9.0, P.FRAME['z0'], 10.5, P.FRAME['z1'], fc=C['frame'], ec='k', lw=1.1)               # ring
    rect(ax, 9.0, 8.7, 10.5, 10.6, fc='white', ec='none', zorder=2)                               # window
    ax.plot([9.0, 10.5], [10.6, 10.6], color='k', lw=1.0, zorder=3)
    rect(ax, 10.3, 7.4, 10.5, 8.3, fc=C['soft'], ec='k', lw=0.9)                                  # the wall strip above the arch (notched at the window)
    rect(ax, 4.2, P.LID['z0'], C_['pocket_y'][0], P.LID['z1'], fc=C['lid'], ec='k', lw=1.0)       # lid plate, in three parts round the pocket
    rect(ax, C_['pocket_y'][1], P.LID['z0'], 10.5, P.LID['z1'], fc=C['lid'], ec='k', lw=1.0)
    rect(ax, C_['hump_y'][0], P.LID['z1'], C_['hump_y'][1], P.Z_LID + 0.2, fc=C['lid'], ec='k', lw=1.0)
    rect(ax, C_['pocket_y'][0], P.Z_LID, C_['pocket_y'][1], P.Z_LID + 0.2, fc=C['lid'], ec='k', lw=1.0)
    ax.add_patch(Wedge((7.0, 4.0), 4.0, 0, 180, fc=C['wheel'], ec='none', alpha=0.9))
    poly(ax, [W(-bw / 2, C_['tip']), W(bw / 2, C_['tip']), W(bw / 2, C_['tip'] + bt), W(-bw / 2, C_['tip'] + bt)], fc='#F2B6A0', ec='k', lw=1.1, zorder=4)
    poly(ax, [W(-C_['holder'][0] / 2, C_['tip']), W(C_['holder'][0] / 2, C_['tip']), W(C_['holder'][0] / 2, C_['tip'] - C_['holder'][2]), W(-C_['holder'][0] / 2, C_['tip'] - C_['holder'][2])], fc='#3C3489', ec=C['est'], lw=1.2, ls='--', alpha=0.7, zorder=4)
    poly(ax, [W(-C_['barrel_d'] / 2, C_['tip'] - C_['holder'][2]), W(C_['barrel_d'] / 2, C_['tip'] - C_['holder'][2]), W(C_['barrel_d'] / 2, 0.0), W(-C_['barrel_d'] / 2, 0.0)], fc='#222', ec=C['est'], lw=1.2, ls='--', alpha=0.85, zorder=5)
    fz0 = C_['tip'] - G['frame_t']
    for sgn in (1, -1):                                                                # the frame's upper and lower bar
        poly(ax, [W(sgn * G['open_y'], fz0), W(sgn * G['frame_y'], fz0), W(sgn * G['frame_y'], C_['tip']), W(sgn * G['open_y'], C_['tip'])], fc='#B4B2A9', ec='k', lw=1.0, zorder=5)
    poly(ax, [W(-G['rail_y'], G['flange_z'][1]), W(G['rail_y'], G['flange_z'][1]), W(G['rail_y'], fz0 + 0.05), W(-G['rail_y'], fz0 + 0.05)], fc='#B4B2A9', ec='k', lw=1.0, zorder=5, alpha=0.6)      # the rails
    poly(ax, [W(-G['rail_y'], G['flange_z'][0]), W(G['rail_y'], G['flange_z'][0]), W(G['rail_y'], G['flange_z'][1]), W(-G['rail_y'], G['flange_z'][1])], fc='#8F8D86', ec='k', lw=1.0, zorder=6)    # the flange
    px, py = W(G['frame_y'] - 0.2, C_['tip'] + bt + 0.08)
    ax.add_patch(Circle((px, py), 0.1, fc='#5F5E5A', ec='none', zorder=6))
    for ang in (-P.CAM['vfov'] / 2, P.CAM['vfov'] / 2):                               # the view cone
        a = math.radians(-P.CAM['tilt'] + ang)
        ax.plot([tip[0], tip[0] + 2.4 * math.cos(a)], [tip[1], tip[1] + 2.4 * math.sin(a)], color=C['cam'], lw=1.0, ls=':')
    ax.annotate('lid hump: the board and its screw heads stand up into it', xy=(6.6, 12.1), xytext=(4.3, 12.7), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.annotate('cage frame (bars above and below the lens holder)', xy=W(-G['frame_y'] + 0.1, fz0 + 0.2), xytext=(4.3, 8.0), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.annotate('flange and rails of the cage', xy=W(0.0, G['flange_z'][0] + 0.15), xytext=(8.2, 7.75), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.annotate('view cone 65.9 x 51.8 degrees,\nthrough a 28 x 23 mm window', xy=(9.8, 9.55), xytext=(7.9, 12.3), fontsize=7.5, arrowprops=dict(arrowstyle='->', lw=0.7))
    ax.text(9.15, 11.35, 'ring', fontsize=7.5)
    ax.text(7.0, 7.55, 'tyre (top z 8.0)', fontsize=7, color='white', ha='center')
    ax.set_title('B  Section through the left camera: y across, z up (cm); the board is tilted 20 degrees', fontsize=10, weight='bold')
    fig.text(0.5, 0.045, 'Blue: read from the OpenMV drawing (+-0.5 mm).  Orange, dashed: ESTIMATED from the product photo, not published. To measure with a caliper on your board: '
             'M1 lens axis from the camera end, M2 lens axis from the left edge,\nM3 holder width, M4 holder length and how far it overhangs the camera end, M5 barrel diameter, M6 height of the lens tip over the BACK of the board and over its FRONT, '
             'board thickness, and whether header pins are soldered (they stand 3 mm out of the back).\nIn the model the board is 2.9 mm under the lid hump ceiling, the cage 2.1 mm, and the cage clears the wheel by 10 mm.', ha='center', fontsize=8, color='#222')
    save(fig, 'fig11_camera_measure')


if __name__ == '__main__':
    fig_tof()
    fig_camera()
