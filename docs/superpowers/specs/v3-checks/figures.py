"""Draw the V3 baseline from the model: python figures.py [outdir] [which...]   which = top side front plate terrain sensors (default all)
All geometry comes from v3_params.py; the terrain poses come from sidesim2.py / nubtable.py (the same code that produced the clearance numbers)."""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly, Circle, Rectangle, Wedge
from matplotlib.lines import Line2D
from shapely.geometry import Polygon as SPoly, Point as SPoint
from shapely.ops import nearest_points
from v3_params import *

C = dict(body='#5F5E5A', wheel='#2C2C2A', motor='#888780', omni='#1D9E75', bump='#D85A30', tof='#378ADD', cam='#7F77DD',
         plate='#BA7517', elec='#639922', batt='#EF9F27', chute='#993C1D', floor='#B4B2A9', kit='#444441', warn='#E24B4A', pcb='#0F6E56', sw='#222222', fs='#F0997B')
PCB_NAME, BATT_NAME = 'GIGA R1 + main PCB stack', 'battery (placeholder 7.0x3.5x2.5)'

def rect_poly(cx, cy, l, w, ang_deg=0.0):
    a = math.radians(ang_deg); c, s = math.cos(a), math.sin(a)
    pts = [(-l / 2, -w / 2), (l / 2, -w / 2), (l / 2, w / 2), (-l / 2, w / 2)]
    return np.array([(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts])

def strip(p0, p1, half):
    d = np.array(p1) - np.array(p0); L = np.hypot(*d); n = np.array([-d[1], d[0]]) / L * half
    return np.array([np.array(p0) + n, np.array(p1) + n, np.array(p1) - n, np.array(p0) - n])

def poly(ax, pts, **kw):
    p = MPoly(np.asarray(pts), closed=True, **kw); ax.add_patch(p); return p

def dim(ax, p0, p1, text, off=(0, 0), fs=8, color='#2C2C2A'):
    ax.annotate('', xy=p1, xytext=p0, arrowprops=dict(arrowstyle='<->', lw=0.8, color=color), zorder=9)
    if text:
        ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, fontsize=fs, ha='center', va='center', color=color, zorder=10, bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.8))

def legend(fig, items, ncol=4, y=0.005):
    hs = [Line2D([0], [0], marker='s', color='none', markerfacecolor=c, markeredgecolor=c, alpha=0.75, markersize=9, label=l) for l, c in items]
    fig.legend(handles=hs, loc='lower center', ncol=ncol, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, y))

def pack_color(name): return C['batt'] if name.startswith('battery') else C['pcb']
def short_name(name): return 'battery\n(placeholder)' if name.startswith('battery') else 'GIGA R1 + main PCB\n(shield stack)'

def cam_side():
    t = math.radians(CAM['tilt']); tip = np.array([CAM['tip_r'] * math.sin(math.radians(CAM['psi'])), CAM['zl']])
    u = np.array([math.cos(t), -math.sin(t)]); h = np.array([math.sin(t), math.cos(t)]); bw, bh = CAM['board']; ld, lw, lh = CAM['lens']
    quad = lambda a0, a1, hh: np.array([tip - u * a0 + h * hh, tip - u * a1 + h * hh, tip - u * a1 - h * hh, tip - u * a0 - h * hh])
    return tip, quad(2.5, 3.1, bh / 2), quad(0.0, ld, lh / 2)

def view_z(d_h, zl=None, tilt=None):
    zl = CAM['zl'] if zl is None else zl; tilt = CAM['tilt'] if tilt is None else tilt
    return (zl - d_h * math.tan(math.radians(tilt + CAM['vfov'] / 2)), zl - d_h * math.tan(math.radians(tilt - CAM['vfov'] / 2)))

def save(fig, out, name):
    os.makedirs(out, exist_ok=True)
    fig.savefig(os.path.join(out, name + '.png'), dpi=140); fig.savefig(os.path.join(out, name + '.svg')); plt.close(fig); print('wrote', os.path.join(out, name + '.png'), flush=True)

def tof_rect(x, y, aim): return rect_poly(x, y, TOF_T, TOF_W, aim)

def draw_plan_core(ax, labels=True, cables=True, zlow=False):
    """body, wheels, motors, omni, bumpers + switches, ToF + cones, cameras + views, plate + chutes, PCB, battery, floor sensors. Returns nothing."""
    for y, ls in ((WALL_NOM, '-'), (WALL_MIN, ':')):
        for s in (1, -1): ax.axhline(s * y, color='#888780', ls=ls, lw=1.3, zorder=1)
    ax.add_patch(Circle((0, 0), R_BODY, fc='#F1EFE8', ec='k', lw=2.2, zorder=2)); ax.add_patch(Circle((0, 0), R_INT, fill=False, ec='#888780', lw=0.7, zorder=2))
    ax.add_patch(Circle((0, 0), R_SWEPT, fill=False, ec='k', ls='--', lw=1.0, zorder=2))
    ax.plot([-6.5, -6.5], [-8.25, 8.25], ls='--', color='#888780', lw=0.9, zorder=3)
    ax.add_patch(Circle((NUB['x'], 0), NUB['r'], fc=C['body'], zorder=6))
    # PCB + battery first (under everything that is above them), cables
    parts, _ = load_pack(); pcb_poly = None
    for name, d in parts.items():
        l, w = PART_SIZES[name]; pts = rect_poly(d['x'], d['y'], l, w, d['angle'])
        if name == PCB_NAME:
            pcb_poly = SPoly(pts); poly(ax, pts, fc=C['pcb'], alpha=0.30, ec=C['pcb'], lw=1.6, zorder=3)
            inner = rect_poly(d['x'], d['y'], l - 0.8, w - 0.8, d['angle']); poly(ax, inner, fill=False, ec=C['pcb'], lw=0.6, ls=':', zorder=3)
            if labels: ax.text(d['x'] + 3.6, d['y'] - 0.2, 'GIGA R1 + main PCB\n(2-layer shield)\nz 6.2-8.1', fontsize=6.8, ha='center', va='center', color='#04342C', zorder=8)
        else:
            poly(ax, pts, fc=C['batt'], alpha=0.35, ec=C['batt'], lw=1.0, ls='--', zorder=3)
            if labels: ax.text(d['x'] + 0.9, d['y'] + 1.0, 'battery (placeholder)\nz 5.1-7.6', fontsize=6.5, ha='center', va='center', zorder=8)
    # wheels, motors
    for s in (1, -1):
        ax.add_patch(Rectangle((-WHEEL['r'], min(s * WHEEL['y0'], s * WHEEL['y1'])), 2 * WHEEL['r'], WHEEL['y1'] - WHEEL['y0'], fc=C['wheel'], ec='k', zorder=4))
        y0 = MOTOR['y_face'] - MOTOR['length'] - MOTOR['enc']
        ax.add_patch(Rectangle((-MOTOR['r'], min(s * y0, s * MOTOR['y_face'])), 2 * MOTOR['r'], MOTOR['y_face'] - y0, fc=C['motor'], ec='k', zorder=4))
    # omni and arm
    ox, oz = OMNI['rest']; px, pz = OMNI['pivot']; L = math.hypot(ox - px, oz - pz); zc = oz + OMNI['travel']; xc = px + math.sqrt(L * L - (zc - pz) ** 2)
    ax.add_patch(Rectangle((ox - OMNI['r'], -OMNI['w'] / 2), 2 * OMNI['r'], OMNI['w'], fc=C['omni'], alpha=0.55, ec=C['omni'], zorder=4))
    ax.add_patch(Rectangle((xc - OMNI['r'], -OMNI['w'] / 2), 2 * OMNI['r'], OMNI['w'], fill=False, ls='--', ec=C['omni'], zorder=4))
    ax.add_patch(Rectangle((px - 0.4, -OMNI['fork_half']), ox - 0.3 - (px - 0.4), 2 * OMNI['fork_half'], fill=False, ec=C['omni'], lw=0.8, zorder=4))
    # bumpers and switches
    for s in (1, -1):
        a0, a1 = (BUMPER['a0'], BUMPER['a1']) if s > 0 else (-BUMPER['a1'], -BUMPER['a0'])
        ax.add_patch(Wedge((0, 0), BUMPER['r_out'], a0, a1, width=BUMPER['t'], fc=C['bump'], ec=C['bump'], zorder=5))
        a = math.radians(s * BUMPER_SW_DEG); r = BUMPER['r_out'] - 0.55
        poly(ax, rect_poly(r * math.cos(a), r * math.sin(a), 0.7, 0.45, s * BUMPER_SW_DEG), fc=C['sw'], zorder=8)
    # ToF + cones
    for nm, x, y, aim in TOF:
        ax.add_patch(Wedge((x, y), 7.0, aim - TOF_CONE / 2, aim + TOF_CONE / 2, fc=C['tof'], alpha=0.13, ec=C['tof'], lw=0.4, zorder=3))
        poly(ax, tof_rect(x, y, aim), fc=C['tof'], ec='k', lw=0.6, zorder=8)
        if cables and pcb_poly is not None:
            p0, p1 = nearest_points(SPoint(x, y), pcb_poly); ax.plot([p0.x, p1.x], [p0.y, p1.y], color='#5F5E5A', lw=0.6, ls=':', zorder=3)
        if labels:
            lx, ly = (x * 1.14, y * 1.14) if nm != 'F' else (x, y + 1.7); ax.text(lx, ly, nm, fontsize=7.5, ha='center', va='center', color='#0C447C', zorder=9, fontweight='bold')
    # cameras
    t = math.radians(CAM['tilt']); cx = CAM['tip_r'] * math.cos(math.radians(CAM['psi'])); cy = CAM['tip_r'] * math.sin(math.radians(CAM['psi']))
    for s in (1, -1):
        ld, lw, _ = CAM['lens']; bw, bh = CAM['board']; ylen = ld * math.cos(t)
        ax.add_patch(Rectangle((cx - lw / 2, min(s * (cy - ylen), s * cy)), lw, ylen, fc=C['cam'], alpha=0.65, ec=C['cam'], zorder=6))
        yb0, yb1 = cy - 3.1 * math.cos(t), cy - 2.5 * math.cos(t)
        ax.add_patch(Rectangle((cx - bw / 2, min(s * yb0, s * yb1)), bw, abs(yb1 - yb0), fc=C['cam'], alpha=0.35, ec=C['cam'], zorder=6))
        half = (WALL_NOM - cy) * math.tan(math.radians(CAM['hfov'] / 2))
        poly(ax, [(cx, s * cy), (cx - half, s * WALL_NOM), (cx + half, s * WALL_NOM)], fc=C['cam'], alpha=0.11, ec=C['cam'], lw=0.5, zorder=3)
        ax.plot([cx - 0.9, cx + 0.9], [s * R_BODY, s * R_BODY], color='white', lw=3.5, zorder=3)
    # plate, chutes
    ax.add_patch(Circle((PLATE['cx'], PLATE['cy']), PLATE['R'], fc='#FAEEDA', ec=C['plate'], lw=1.3, alpha=0.85, zorder=5))
    for i in range(1, PLATE['n'] + 1):
        x, y, a = pocket_xy(i)
        poly(ax, rect_poly(x, y, PLATE['pocket'], PLATE['pocket'], a), fc='white', ec=C['plate'], lw=0.8, zorder=6)
        poly(ax, rect_poly(x, y, 1.03, 1.03, a), fc=C['kit'], alpha=0.8, ec='none', zorder=6)
        ax.text(x, y, str(i), fontsize=5.5, color='white', ha='center', va='center', zorder=7)
    for nm in ('A', 'B'):
        sx, sy = slot_xy(nm); ex, ey = EXIT['x'], (EXIT['y'] if sy > 0 else -EXIT['y'])
        poly(ax, rect_poly(sx, sy, PLATE['slot'], PLATE['slot'], 45), fc='black', zorder=7)
        poly(ax, strip((sx, sy), (ex, ey), (EXIT['bore'] + 2 * EXIT['wall']) / 2), fc=C['chute'], alpha=0.28, ec=C['chute'], lw=0.6, zorder=4)
        ax.plot([ex], [ey], marker='v' if ey > 0 else '^', color=C['chute'], ms=6, zorder=8)
        lx, ly = KIT_LAND[0], (KIT_LAND[1] if ey > 0 else -KIT_LAND[1])
        ax.annotate('', xy=(lx, ly), xytext=(ex, ey), arrowprops=dict(arrowstyle='->', color=C['chute'], lw=1.0, ls=':'), zorder=8)
        ax.plot([lx], [ly], marker='X', color=C['chute'], ms=7, zorder=8)
    # floor sensors
    poly(ax, rect_poly(FLOOR_FRONT['x'], FLOOR_FRONT['y'], FLOOR_FRONT['w'], FLOOR_FRONT['w'], 0), fc=C['fs'], ec=C['bump'], lw=0.9, zorder=9)
    poly(ax, rect_poly(SILVER['x'], SILVER['y'], SILVER['l'], SILVER['w'], 0), fc=C['fs'], ec=C['bump'], lw=0.9, zorder=9)
    if labels:
        ax.text(FLOOR_FRONT['x'], FLOOR_FRONT['y'], 'FP', fontsize=6, ha='center', va='center', zorder=10); ax.text(SILVER['x'], SILVER['y'], 'SM', fontsize=6, ha='center', va='center', zorder=10)
    return pcb_poly

# ------------------------------------------------------------------------------------------------ figure 1: top view
def fig_top(out):
    fig, ax = plt.subplots(figsize=(12.5, 10.4))
    ax.set_aspect('equal'); ax.set_xlim(-15.5, 17.5); ax.set_ylim(-16.5, 16.5)
    draw_plan_core(ax)
    ax.text(-15.3, WALL_NOM + 0.25, 'wall, 28 cm path', fontsize=8, color='#5F5E5A'); ax.text(-15.3, WALL_MIN + 0.25, 'wall, 25.2 cm path (-10 %)', fontsize=8, color='#5F5E5A')
    ax.text(0.2, -R_SWEPT - 0.8, 'swept circle R 11.0 (bumper tips, omni front)  |  body R 10.5', fontsize=8.5, ha='center')
    ax.text(-7.6, 0.9, 'nub', fontsize=7, ha='right'); ax.text(-0.15, 5.2, 'motor', color='white', fontsize=7, ha='center', va='center', zorder=9); ax.text(-2.6, 8.0, 'drive wheel\n80 mm', color='white', fontsize=6.3, ha='center', va='center', zorder=9)
    ax.text(6.4, 0.0, 'omni 60 mm', fontsize=6.3, ha='center', va='center', zorder=9, color='#04342C')
    ax.text(9.0, 10.2, 'bumper plates\n12-58 deg', fontsize=7.2, color='#993C1D'); ax.text(9.0, -11.3, 'bumper plates', fontsize=7.2, color='#993C1D')
    ax.text(3.4, 14.5, 'OpenMV camera above each wheel, lens recessed 1.5 cm behind a 2 cm window;\nview 65.9 deg wide = 7.2 cm of wall on a 28 cm path', fontsize=7.2, color='#3C3489', va='bottom')
    ax.text(-8.6, 12.0, 'kit lands here (2.7 cm from a 28 cm wall)', fontsize=7, color='#712B13', ha='center'); ax.text(-6.2, 9.7, 'exit', fontsize=6.8, color='#712B13', ha='right')
    ax.text(-4.9, 4.6, 'slot B (left)', fontsize=6.5, ha='right', color='#633806', zorder=9); ax.text(-4.9, -4.6, 'slot A (right)', fontsize=6.5, ha='right', va='top', color='#633806', zorder=9)
    ax.plot([0.95], [0.0], marker='P', color='#A32D2D', ms=9, zorder=11); ax.text(1.3, -0.6, 'COM +0.95', fontsize=6.5, color='#A32D2D', va='top', zorder=11)
    ax.annotate('', xy=(16.5, -13.8), xytext=(12.5, -13.8), arrowprops=dict(arrowstyle='->', lw=1.4)); ax.text(14.5, -14.7, 'forward (x)', ha='center', fontsize=8)
    ax.set_xlabel('x (cm)'); ax.set_ylabel('y (cm), left is up')
    ax.set_title('Figure 1. Top view (plan), all parts projected. Body R 10.5, swept R 11.0, belly z 3.5, ToF z 10.0, plate z 9.0-10.2', fontsize=11)
    fig.text(0.5, 0.058, 'Dotted grey lines: the nine ToF cables to the main PCB.  Dashed omni rectangle: omni fully compressed.  FP = front floor port (black / blue / red, x 6.5).  SM = silver module between the motors at the axle line.\nBlack squares on the bumper plates: microswitches (inner ends; the outer ends have two spare inputs).  F, FL, FR, SFL, SFR, SRL, SRR, RL, RR = the nine VL53L0X.\nDashed vertical line at x -6.5: where the rear underside chamfer starts.  SM sits on the axle line between the motors.', ha='center', fontsize=7.8, color='#444441')
    legend(fig, [('drive wheel / motor', C['wheel']), ('omni + arm', C['omni']), ('bumper plate', C['bump']), ('ToF + 25 deg cone', C['tof']), ('camera + view', C['cam']), ('dropper plate + chutes', C['plate']),
                 ('battery (placeholder)', C['batt']), ('GIGA R1 + main PCB', C['pcb']), ('floor sensors', C['fs'])], ncol=5, y=0.0)
    fig.tight_layout(rect=(0, 0.095, 1, 1)); save(fig, out, 'fig1_top')

# ------------------------------------------------------------------------------------------------ figure 2: side section
def fig_side(out):
    fig, ax = plt.subplots(figsize=(13.5, 9.0))
    ax.set_aspect('equal'); ax.set_xlim(-13.5, 20.5); ax.set_ylim(-2.5, 14.6)
    ax.axhline(0, color='k', lw=2.2, zorder=1); ax.fill_between([-14, 21], -2.5, 0, color=C['floor'], alpha=0.5, zorder=0)
    (cx0, cz0), (cx1, cz1) = CHAMFER
    poly(ax, [(-R_BODY, cz1), (cx0, cz0), (R_BODY, Z_BELLY), (R_BODY, Z_LID), (-R_BODY, Z_LID)], fc='#F1EFE8', ec='k', lw=2.0, zorder=2)
    ax.plot([-R_BODY, R_BODY], [Z_ROOF, Z_ROOF], color='#888780', ls='--', lw=0.8, zorder=3); ax.text(-10.3, Z_ROOF - 0.05, 'roof 11.5', fontsize=7, va='top', color='#5F5E5A')
    ax.add_patch(Circle((0, AXLE_Z), WHEEL['r'], fc=C['wheel'], alpha=0.30, ec=C['wheel'], ls='--', zorder=3)); ax.add_patch(Circle((0, AXLE_Z), MOTOR['r'], fc=C['motor'], ec='k', zorder=4))
    ax.text(0, 1.1, 'drive wheel 80 mm\n(behind the plane)', fontsize=7, ha='center', va='center', zorder=6)
    bx0, bx1 = BUMPER['r_out'] * math.cos(math.radians(BUMPER['a1'])), BUMPER['r_out'] * math.cos(math.radians(BUMPER['a0']))
    poly(ax, [(bx0, BUMPER['z0']), (bx1, BUMPER['z0']), (bx1, BUMPER['z1']), (bx0, BUMPER['z1'])], fc=C['bump'], alpha=0.30, ec=C['bump'], ls='--', zorder=3)
    ax.text((bx0 + bx1) / 2 + 0.6, BUMPER['z1'] + 0.12, 'bumper band z 4.0-6.5', fontsize=7, ha='center', color='#993C1D')
    ox, oz = OMNI['rest']; px, pz = OMNI['pivot']; L = math.hypot(ox - px, oz - pz); zc = oz + OMNI['travel']; xc = px + math.sqrt(L * L - (zc - pz) ** 2)
    ax.add_patch(Circle((ox, oz), OMNI['r'], fc=C['omni'], alpha=0.35, ec=C['omni'], lw=1.6, zorder=4)); ax.add_patch(Circle((xc, zc), OMNI['r'], fill=False, ls='--', ec=C['omni'], lw=1.2, zorder=4))
    ax.plot([px, ox], [pz, oz], color=C['omni'], lw=2.2, zorder=5); ax.plot([px, xc], [pz, zc], color=C['omni'], lw=1.2, ls='--', zorder=5); ax.plot([px], [pz], marker='o', color='k', ms=5, zorder=6)
    ax.text(px - 0.2, pz - 0.35, 'arm pivot\n(3.5, 3.6)', fontsize=6.5, ha='right', va='top'); ax.text(ox + 0.5, oz - 1.1, 'omni 60 mm', fontsize=7, ha='center', va='center', color='#085041', zorder=6)
    ax.text(R_SWEPT + 0.3, 2.0, 'dashed omni:\nfully compressed\n(2.5 cm up)', fontsize=6.8, ha='left', va='center', color='#085041')
    zn = cz0 + (cx0 - NUB['x']) * (cz1 - cz0) / (cx0 - cx1)
    ax.plot([NUB['x'], NUB['x']], [NUB['z_low'] + 2 * NUB['r'], zn], color=C['body'], lw=2, zorder=3); ax.add_patch(Circle((NUB['x'], NUB['z_low'] + NUB['r']), NUB['r'], fc=C['body'], zorder=4))
    ax.text(NUB['x'] + 0.7, 1.0, 'rear nub\n(ball / PTFE)', fontsize=7, ha='left', va='center'); ax.text(-8.9, 5.2, 'chamfer 24 deg', fontsize=7, ha='center', color='#5F5E5A')
    xp = PLATE['cx']; Rp = PLATE['R']
    poly(ax, [(xp - Rp, PLATE['z0']), (xp + Rp, PLATE['z0']), (xp + Rp, PLATE['z0'] + PLATE['t']), (xp - Rp, PLATE['z0'] + PLATE['t'])], fc=C['plate'], alpha=0.65, ec=C['plate'], zorder=4)
    poly(ax, [(xp - Rp - 0.2, PLATE['z0'] - PLATE['floor_t']), (xp + Rp + 0.2, PLATE['z0'] - PLATE['floor_t']), (xp + Rp + 0.2, PLATE['z0']), (xp - Rp - 0.2, PLATE['z0'])], fc='#854F0B', alpha=0.5, ec='none', zorder=4)
    n20 = (PLATE['z0'] - PLATE['floor_t'] - PLATE['n20_len'], PLATE['z0'] - PLATE['floor_t'])
    poly(ax, [(xp - 0.5, n20[0]), (xp + 0.5, n20[0]), (xp + 0.5, n20[1]), (xp - 0.5, n20[1])], fc=C['motor'], ec='k', zorder=4); ax.text(xp, n20[0] - 0.15, 'N20 + encoder', fontsize=6.5, ha='center', va='top')
    ax.text(xp, PLATE['z0'] + PLATE['t'] + 0.15, 'dropper plate, R 4.9, z 9.0-10.2', fontsize=7, ha='center', color='#633806')
    sx, sy = slot_xy('B'); ex = EXIT['x']; zs0 = PLATE['z0'] - PLATE['floor_t'] - EXIT['bore'] / 2; ze = EXIT['z_nose'] + EXIT['bore'] / 2
    poly(ax, strip((sx, zs0), (ex, ze), EXIT['bore'] / 2 + EXIT['wall']), fc=C['chute'], alpha=0.35, ec=C['chute'], lw=0.8, zorder=5)
    ax.plot([ex], [EXIT['z_nose']], marker='v', color=C['chute'], ms=6, zorder=8); ax.text(ex + 0.35, EXIT['z_nose'] - 0.55, 'chute exit nose z 3.0 (both sides)', fontsize=7, ha='left', va='top', color='#712B13', bbox=dict(fc='white', ec='none', alpha=0.7, pad=0.6), zorder=9)
    ax.text(-6.6, 6.4, 'chute\n35 deg', fontsize=7, ha='right', color='#712B13')
    for nm, x, y, aim in TOF:
        poly(ax, [(x - TOF_T / 2, TOF_Z - TOF_H / 2), (x + TOF_T / 2, TOF_Z - TOF_H / 2), (x + TOF_T / 2, TOF_Z + TOF_H / 2), (x - TOF_T / 2, TOF_Z + TOF_H / 2)], fc=C['tof'], ec='k', lw=0.6, zorder=7, alpha=1.0 if nm == 'F' else 0.55)
    tx = TOF[0][1]; Lb = 4.5; tg = math.tan(math.radians(TOF_CONE / 2))
    poly(ax, [(tx, TOF_Z), (tx + Lb, TOF_Z + Lb * tg), (tx + Lb, TOF_Z - Lb * tg)], fc=C['tof'], alpha=0.15, ec=C['tof'], lw=0.5, zorder=3)
    rx = TOF[7][1]; poly(ax, [(rx, TOF_Z), (rx - Lb, TOF_Z + Lb * tg), (rx - Lb, TOF_Z - Lb * tg)], fc=C['tof'], alpha=0.12, ec=C['tof'], lw=0.5, zorder=3)
    for xx, xt, txt in ((-8.3, -8.3, 'rear and side-rear ToF\nRL, RR (x -9.3), SRL, SRR (x -7.3)'), (7.28, 3.0, 'side-front ToF\nSFL, SFR (x 7.3)'), (9.5, 7.9, 'front ToF: F (x 9.5, recessed 1 cm)\nFL, FR (x 9.3, toed out)')):
        ax.annotate(txt, xy=(xx, TOF_Z + TOF_H / 2), xytext=(xt, 13.7), fontsize=7, color='#0C447C', ha='center', va='center', arrowprops=dict(arrowstyle='-', color='#0C447C', lw=0.6))
    t = math.radians(CAM['tilt']); cxc = CAM['tip_r'] * math.cos(math.radians(CAM['psi'])); bw, bh = CAM['board']
    top = CAM['zl'] + 3.1 * math.sin(t) + (bh / 2) * math.cos(t); bot = CAM['zl'] + 2.5 * math.sin(t) - (bh / 2) * math.cos(t)
    poly(ax, [(cxc - bw / 2, bot), (cxc + bw / 2, bot), (cxc + bw / 2, top), (cxc - bw / 2, top)], fc=C['cam'], alpha=0.22, ec=C['cam'], ls='--', zorder=3)
    ax.text(cxc, top + 0.15, f'camera boards (both sides, behind the plane)\nlens z {CAM["zl"]}, top {top:.2f}', fontsize=7, ha='center', va='bottom', color='#3C3489')
    parts, _ = load_pack()
    for name, d in parts.items():
        l, w = PART_SIZES[name]; ang = math.radians(d['angle']); half_x = abs(l * math.cos(ang)) / 2 + abs(w * math.sin(ang)) / 2
        poly(ax, [(d['x'] - half_x, d['z0']), (d['x'] + half_x, d['z0']), (d['x'] + half_x, d['z1']), (d['x'] - half_x, d['z1'])], fc=pack_color(name), alpha=0.28, ec=pack_color(name), ls='--', zorder=3)
        ax.text(d['x'] + (0.0 if name == PCB_NAME else 1.5), (d['z0'] + d['z1']) / 2 + (0.5 if name == PCB_NAME else -0.5), short_name(name).replace('\n', ' '), fontsize=6.4, ha='center', va='center', zorder=8)
    fp = FLOOR_FRONT; sm = SILVER
    poly(ax, [(fp['x'] - fp['w'] / 2, fp['z_face']), (fp['x'] + fp['w'] / 2, fp['z_face']), (fp['x'] + fp['w'] / 2, Z_BELLY), (fp['x'] - fp['w'] / 2, Z_BELLY)], fc=C['fs'], ec=C['bump'], zorder=6)
    poly(ax, [(sm['x'] - sm['l'] / 2, sm['z_face']), (sm['x'] + sm['l'] / 2, sm['z_face']), (sm['x'] + sm['l'] / 2, Z_BELLY), (sm['x'] - sm['l'] / 2, Z_BELLY)], fc=C['fs'], ec=C['bump'], zorder=6)
    ax.annotate('front floor port FP (x 6.5, face z 2.8)', xy=(fp['x'], fp['z_face']), xytext=(8.0, -0.95), fontsize=7, color='#712B13', ha='center', va='center', arrowprops=dict(arrowstyle='-', color='#712B13', lw=0.6))
    ax.annotate('silver module SM (axle line, face z 2.8)', xy=(sm['x'], sm['z_face']), xytext=(-2.6, -0.95), fontsize=7, color='#712B13', ha='center', va='center', arrowprops=dict(arrowstyle='-', color='#712B13', lw=0.6))
    ax.plot([0.95], [6.4], marker='P', color='#A32D2D', ms=9, zorder=9); ax.text(1.25, 6.05, 'COM +0.95, z 6.4', fontsize=6.5, color='#A32D2D', va='top'); ax.plot([0], [AXLE_Z], marker='+', color='k', ms=9, zorder=9)
    ax.axvline(R_SWEPT, color='k', ls='--', lw=0.9); ax.text(R_SWEPT + 0.1, 14.0, 'swept R 11.0', fontsize=7.5)
    xd = 14.6
    dim(ax, (xd, 0), (xd, Z_BELLY), 'belly\n3.5', fs=7); dim(ax, (xd + 1.3, 0), (xd + 1.3, 8.0), 'wheel top 8.0', fs=7); dim(ax, (xd + 2.6, 0), (xd + 2.6, TOF_Z), 'ToF 10.0', fs=7); dim(ax, (xd + 3.9, 0), (xd + 3.9, Z_LID), 'lid 12.1', fs=7)
    dim(ax, (-R_BODY, -1.9), (R_BODY, -1.9), 'body diameter 21.0', fs=8); dim(ax, (NUB['x'] - 1.3, 0), (NUB['x'] - 1.3, NUB['z_low']), '2.0', fs=7, off=(-0.5, 0))
    ax.set_xlabel('x (cm), forward is right'); ax.set_ylabel('z (cm)')
    ax.set_title('Figure 2. Side section at y = 0 (parts off the plane are projected, dashed). Axle at z 4.0; omni and nub in the flat-floor pose', fontsize=11)
    legend(fig, [('drive wheel / motor', C['wheel']), ('omni + arm', C['omni']), ('bumper (projected)', C['bump']), ('ToF + cone', C['tof']), ('camera (projected)', C['cam']), ('dropper + chute', C['plate']), ('battery', C['batt']), ('GIGA R1 + main PCB', C['pcb']), ('floor sensors', C['fs'])], ncol=5)
    fig.tight_layout(rect=(0, 0.06, 1, 1)); save(fig, out, 'fig2_side')

# ------------------------------------------------------------------------------------------------ figure 3: front section
def fig_front(out):
    fig, ax = plt.subplots(figsize=(13, 8.8))
    ax.set_aspect('equal'); ax.set_xlim(17, -17); ax.set_ylim(-2.2, 14.8)      # y runs to the left: view from behind, looking forward
    ax.axhline(0, color='k', lw=2.2); ax.fill_between([-18, 18], -2.2, 0, color=C['floor'], alpha=0.5)
    for y, ls in ((WALL_NOM, '-'), (WALL_MIN, ':')):
        for s in (1, -1): ax.plot([s * y, s * y], [0, 13.4], color='#5F5E5A', ls=ls, lw=1.6 if ls == '-' else 1.2)
    for s in (1, -1):
        ax.text(s * WALL_NOM, 14.25, 'wall, 28 cm path', fontsize=7.5, ha='center', va='bottom', color='#5F5E5A'); ax.text(s * WALL_MIN, 13.6, 'wall, 25.2 cm', fontsize=7.5, ha='center', va='bottom', color='#5F5E5A')
    poly(ax, [(-R_BODY, Z_BELLY), (-WHEEL['y1'], Z_BELLY), (-WHEEL['y1'], 8.15), (-WHEEL['y0'], 8.15), (-WHEEL['y0'], Z_BELLY), (WHEEL['y0'], Z_BELLY), (WHEEL['y0'], 8.15), (WHEEL['y1'], 8.15), (WHEEL['y1'], Z_BELLY), (R_BODY, Z_BELLY), (R_BODY, Z_LID), (-R_BODY, Z_LID)], fc='#F1EFE8', ec='k', lw=1.8, zorder=2)
    ax.plot([-R_BODY, R_BODY], [Z_ROOF, Z_ROOF], color='#888780', ls='--', lw=0.8, zorder=3); ax.text(-1.2, Z_ROOF - 0.12, 'roof 11.5', fontsize=7, va='top')
    for s in (1, -1):
        poly(ax, [(s * WHEEL['y0'], 0), (s * WHEEL['y1'], 0), (s * WHEEL['y1'], 8.0), (s * WHEEL['y0'], 8.0)], fc=C['wheel'], ec='k', zorder=4)
        y0 = MOTOR['y_face'] - MOTOR['length'] - MOTOR['enc']
        poly(ax, [(s * y0, 3.0), (s * MOTOR['y_face'], 3.0), (s * MOTOR['y_face'], 5.0), (s * y0, 5.0)], fc=C['motor'], ec='k', zorder=4)
        ax.text(s * 8.0, -0.9, 'drive wheel 80 x 20 mm', fontsize=7, ha='center', va='center')
    poly(ax, [(-SILVER['w'] / 2, SILVER['z_face']), (SILVER['w'] / 2, SILVER['z_face']), (SILVER['w'] / 2, Z_BELLY), (-SILVER['w'] / 2, Z_BELLY)], fc=C['fs'], ec=C['bump'], zorder=5)
    ax.text(0, 4.6, 'motors (gap 2.8)', fontsize=7, ha='center', va='center'); ax.annotate('silver module SM (axle line)', xy=(0, SILVER['z_face']), xytext=(0, -0.9), fontsize=7, ha='center', va='center', color='#712B13', arrowprops=dict(arrowstyle='-', color='#712B13', lw=0.6))
    hc = math.sqrt(PLATE['R'] ** 2 - (0 - PLATE['cx']) ** 2)
    poly(ax, [(-hc, PLATE['z0']), (hc, PLATE['z0']), (hc, PLATE['z0'] + PLATE['t']), (-hc, PLATE['z0'] + PLATE['t'])], fc=C['plate'], alpha=0.65, ec=C['plate'], zorder=4)
    ax.text(0, PLATE['z0'] + PLATE['t'] + 0.12, 'dropper plate (section)', fontsize=7, ha='center', color='#633806')
    parts, _ = load_pack()
    for name, d in parts.items():
        l, w = PART_SIZES[name]; ang = math.radians(d['angle']); half_y = abs(l * math.sin(ang)) / 2 + abs(w * math.cos(ang)) / 2
        poly(ax, [(d['y'] - half_y, d['z0']), (d['y'] + half_y, d['z0']), (d['y'] + half_y, d['z1']), (d['y'] - half_y, d['z1'])], fc=pack_color(name), alpha=0.28, ec=pack_color(name), ls='--', zorder=3)
        ax.text(d['y'], (d['z0'] + d['z1']) / 2, short_name(name).replace('\n', ' '), fontsize=6.2, ha='center', va='center', zorder=8)
    groups = {}
    for nm, x, y, aim in TOF: groups.setdefault(round(y * 2) / 2, []).append(nm)
    for y, names in groups.items():
        y = {7.5: 7.28, -7.5: -7.28, 4.5: 4.43, -4.5: -4.43}.get(y, y)
        poly(ax, [(y - TOF_W / 2, TOF_Z - TOF_H / 2), (y + TOF_W / 2, TOF_Z - TOF_H / 2), (y + TOF_W / 2, TOF_Z + TOF_H / 2), (y - TOF_W / 2, TOF_Z + TOF_H / 2)], fc=C['tof'], alpha=0.35, ec=C['tof'], lw=0.9, zorder=7)
        ax.text(y, TOF_Z, '\n'.join(names), fontsize=5.8, ha='center', va='center', color='#0C447C', zorder=8)
    for s in (1, -1): ax.plot([s * 7.28, s * WALL_NOM], [TOF_Z, TOF_Z], color=C['tof'], lw=0.8, zorder=3)
    tip, board, lens = cam_side()
    for s in (1, -1):
        mir = lambda p: np.array([s * p[0], p[1]])
        poly(ax, [mir(p) for p in board], fc=C['cam'], alpha=0.5, ec=C['cam'], zorder=6); poly(ax, [mir(p) for p in lens], fc=C['cam'], alpha=0.85, ec='k', lw=0.6, zorder=6)
        zb, zt = view_z(WALL_NOM - tip[0])
        poly(ax, [mir(tip), mir((WALL_NOM, zb)), mir((WALL_NOM, zt))], fc=C['cam'], alpha=0.12, ec=C['cam'], lw=0.8, zorder=2)
        poly(ax, [mir((WALL_NOM, 5.0)), mir((WALL_NOM + 0.15, 5.0)), mir((WALL_NOM + 0.15, 9.0)), mir((WALL_NOM, 9.0))], fc=C['warn'], ec=C['warn'], zorder=5)
        ax.plot([s * R_BODY] * 2, [7.4, 9.5], color='white', lw=4, zorder=3)
    zb, zt = view_z(WALL_NOM - tip[0]); zb2, zt2 = view_z(WALL_MIN - tip[0])
    ax.annotate('camera: lens z 9.3, tilt 20 deg down,\nwindow about 2 x 2 cm', xy=(tip[0], tip[1]), xytext=(5.5, 13.1), fontsize=7.2, color='#3C3489', ha='center', va='center', arrowprops=dict(arrowstyle='->', color='#3C3489', lw=0.8))
    dim(ax, (tip[0], 10.9), (WALL_NOM, 10.9), '5.2 cm (28 cm path)', fs=7); dim(ax, (tip[0], 11.6), (WALL_MIN, 11.6), '3.8 (25.2)', fs=7)
    ax.text(WALL_NOM - 0.5, 7.0, 'victim band z 5-9', fontsize=7, color=C['warn'], ha='left', va='center', rotation=90)
    for s in (1, -1):
        poly(ax, [(s * EXIT['y'] - 0.35, EXIT['z_nose']), (s * EXIT['y'] + 0.35, EXIT['z_nose']), (s * EXIT['y'] + 0.35, EXIT['z_nose'] + 0.75), (s * EXIT['y'] - 0.35, EXIT['z_nose'] + 0.75)], fc=C['chute'], alpha=0.5, ec=C['chute'], zorder=5)
        tt = np.linspace(0, 1, 12); yy = s * (EXIT['y'] + (KIT_LAND[1] - EXIT['y']) * tt); zz = 3.4 - 2.9 * tt * tt
        ax.plot(yy, zz, color=C['chute'], ls=':', lw=1.2, zorder=6); ax.plot([s * KIT_LAND[1]], [0.5], marker='X', color=C['chute'], ms=6, zorder=7)
    ax.text(12.9, 1.7, 'chute exit and kit flight\n(x -6, behind the wheel)', fontsize=7, color='#712B13', ha='center')
    ax.axvline(R_SWEPT, color='k', ls='--', lw=0.8); ax.axvline(-R_SWEPT, color='k', ls='--', lw=0.8); ax.text(R_SWEPT - 0.15, 13.0, 'swept 11.0', fontsize=7, rotation=90, va='center', ha='right')
    ax.set_xlabel("y (cm): view from behind, looking forward; the robot's left is the left of the picture"); ax.set_ylabel('z (cm)')
    ax.set_title('Figure 3. Front section at the axle (x = 0). Cameras sit above the wheels, lens 1.5 cm inside the shell, tilted 20 deg down', fontsize=11)
    fig.text(0.5, 0.062, f'Camera view of the wall (each side): 28 cm path, lens 5.2 cm from the wall: z {zb:.1f} to {zt:.1f} cm.   25.2 cm path, lens 3.8 cm from the wall: z {zb2:.1f} to {zt2:.1f} cm.   Victim letter band z 5-9 (red).\nAll nine ToF are drawn as blue boxes at z 10.0, grouped by their y position (FL and RL share y 4.4, SFL and SRL share y 7.3); only the two SF boards sit at the axle line, the others are projected.', ha='center', fontsize=7.8, color='#444441')
    legend(fig, [('wheel / motor', C['wheel']), ('ToF (all nine, projected)', C['tof']), ('camera + view', C['cam']), ('victim band', C['warn']), ('dropper plate, chute, kit', C['plate']), ('battery', C['batt']), ('GIGA R1 + main PCB', C['pcb']), ('silver module', C['fs'])], ncol=4)
    fig.tight_layout(rect=(0, 0.11, 1, 1)); save(fig, out, 'fig3_front')

# ------------------------------------------------------------------------------------------------ figure 4: dropper plate
def fig_plate(out):
    fig, axes = plt.subplots(2, 2, figsize=(12, 11.3))
    states = [('Parked: both slots sit over the blank arc, no kit over a slot', 0.0, set(), set()),
              ('Right drop, one kit: plate turns clockwise 30 deg, pocket 1 over slot A', -30.0, {1}, set()),
              ('Left drop, one kit: plate turns counter-clockwise 30 deg, pocket 8 over slot B', 30.0, {8}, set()),
              ('Right drop, two kits: pocket 2 now over slot A, pocket 1 already empty', -60.0, {2}, {1})]
    for ax, (title, rot, falling, empty) in zip(axes.ravel(), states):
        ax.set_aspect('equal'); ax.set_xlim(-10.6, 5.9); ax.set_ylim(-9.4, 9.4); ax.axis('off'); ax.set_title(title, fontsize=9.5)
        ax.add_patch(Circle((PLATE['cx'], PLATE['cy']), PLATE['R'], fc='#FAEEDA', ec=C['plate'], lw=1.5))
        ax.add_patch(Wedge((PLATE['cx'], PLATE['cy']), PLATE['R'] + 0.3, PLATE['slotB_robot_deg'], PLATE['slotA_robot_deg'], width=0.3, fc='#B4B2A9', ec='none'))
        ax.text(PLATE['cx'] - PLATE['R'] - 0.6, 0, 'rear', fontsize=8, ha='right', va='center', color='#5F5E5A')
        for i in range(1, PLATE['n'] + 1):
            x, y, a = pocket_xy(i, rot)
            poly(ax, rect_poly(x, y, PLATE['pocket'], PLATE['pocket'], a), fc='white', ec=C['plate'], lw=0.9, zorder=5)
            if i in falling: col, txt = C['warn'], 'white'
            elif i in empty: col, txt = None, '#5F5E5A'
            else: col, txt = C['kit'], 'white'
            if col: poly(ax, rect_poly(x, y, 1.03, 1.03, a), fc=col, alpha=0.9, ec='none', zorder=5)
            ax.text(x, y, str(i), fontsize=6.5, color=txt, ha='center', va='center', zorder=6)
        for nm in ('A', 'B'):
            sx, sy = slot_xy(nm); ex, ey = EXIT['x'], (EXIT['y'] if sy > 0 else -EXIT['y'])
            poly(ax, rect_poly(sx, sy, PLATE['slot'], PLATE['slot'], 45), fc='black', zorder=4)
            ax.text(sx - 1.3, sy + (0.9 if sy > 0 else -0.9), 'slot ' + nm + (' (left)' if nm == 'B' else ' (right)'), fontsize=7.5, ha='right', va='center')
            poly(ax, strip((sx, sy), (ex, ey), (EXIT['bore'] + 2 * EXIT['wall']) / 2), fc=C['chute'], alpha=0.28, ec=C['chute'], lw=0.5, zorder=3)
            ax.plot([ex], [ey], marker='v' if ey > 0 else '^', color=C['chute'], ms=6)
        ax.text(PLATE['cx'], PLATE['cy'], f'turn\n{rot:+.0f} deg', fontsize=7.5, ha='center', va='center', color='#633806')
    fig.suptitle('Figure 4. Dropper plate, top view, forward is right. Black squares are the two fixed slots, brown strips the chutes to the side exits', fontsize=11)
    fig.text(0.5, 0.012, 'Pocket 14.0 mm, slot 14.5 mm, ring radius 38.6 mm, plate radius 49.1 mm (dia 9.8 cm), pitch 30 deg, kit 10.3 mm.\n8 pockets at 30 to 240 deg, slots at 0 and 270 deg of the plate frame; the nearest pocket is 30 deg from each slot.\n2.5 mm of plate between a parked pocket and a slot, 2.8 mm between pockets.  Red = kit falling now, dark = kit in its pocket, hollow = emptied; grey arc = blank arc (90 deg).', ha='center', fontsize=8, color='#444441')
    fig.tight_layout(rect=(0, 0.06, 1, 0.97)); save(fig, out, 'fig4_plate')

# ------------------------------------------------------------------------------------------------ figure 5: terrain poses
def fig_terrain(out):
    from nubtable import mk, CASES, ramp_with_bump, XS, stats
    from sidesim2 import pitch_for_omni, rear_ok, clearances, wheel_centre_z, world, terrain
    R = mk()
    def search(profile, mode):
        T = terrain(profile); best = None
        for xd in XS:
            zd = wheel_centre_z(T, xd, R.wheel_r); feas = None
            for i in range(11):
                c = R.travel * i / 10; ph = pitch_for_omni(T, R, xd, zd, c)
                if ph is None or not rear_ok(T, R, xd, zd, ph): continue
                feas = (c, ph); break
            if feas is None: continue
            c, ph = feas; cl = clearances(T, R, xd, zd, ph); key = min(cl.values()) if mode == 'min' else ph
            if best is None or (key < best[0] if mode == 'min' else key > best[0]): best = (key, xd, zd, ph, c, cl)
        return T, best
    def draw(ax, T, pose, title, note, worst=None):
        key, xd, zd, ph, c, cl = pose; ax.set_aspect('equal')
        ax.set_xlim(xd - 17, xd + 17); ax.set_ylim(-1.0, 20.5 if 'ramp' in title.lower() else 17.5)
        tx = [p[0] for p in T.exterior.coords]; tz = [p[1] for p in T.exterior.coords]; ax.fill(tx, tz, color=C['floor'], alpha=0.9, zorder=1); ax.plot(tx, tz, color='#444441', lw=1.2, zorder=2)
        W = lambda p: world(p, xd, zd, ph)
        body = [(-R_BODY, CHAMFER[1][1] - AXLE_Z), (CHAMFER[0][0], Z_BELLY - AXLE_Z), (R_BODY, Z_BELLY - AXLE_Z), (R_BODY, Z_LID - AXLE_Z), (-R_BODY, Z_LID - AXLE_Z)]
        poly(ax, [W(p) for p in body], fc='#F1EFE8', ec='k', lw=1.6, zorder=3, alpha=0.95)
        bx0, bx1 = BUMPER['r_out'] * math.cos(math.radians(BUMPER['a1'])), BUMPER['r_out'] * math.cos(math.radians(BUMPER['a0']))
        poly(ax, [W((bx0, BUMPER['z0'] - AXLE_Z)), W((bx1, BUMPER['z0'] - AXLE_Z)), W((bx1, BUMPER['z1'] - AXLE_Z)), W((bx0, BUMPER['z1'] - AXLE_Z))], fc=C['bump'], alpha=0.45, ec=C['bump'], zorder=4)
        ax.add_patch(Circle((xd, zd), WHEEL['r'], fc=C['wheel'], alpha=0.85, ec='k', zorder=5))
        ow = W(R.omni_centre(c)); ax.add_patch(Circle(ow, OMNI['r'], fc=C['omni'], alpha=0.6, ec=C['omni'], zorder=5))
        nub = W((NUB['x'], NUB['z_low'] + NUB['r'] - AXLE_Z)); ax.add_patch(Circle(nub, NUB['r'], fc=C['body'], zorder=5))
        lip = W((EXIT['x'], EXIT['z_nose'] - AXLE_Z)); ax.plot([lip[0]], [lip[1]], marker='v', color=C['chute'], ms=6, zorder=6)
        ax.set_title(title, fontsize=10)
        extra = f'\nworst over the whole crossing: belly {worst["belly"]:.2f}, bumper {worst["bumper"]:.2f}, chute nose {worst["lip"]:.2f} cm; max pitch {worst["pitch_max"]:+.1f} deg' if worst else ''
        ax.text(0.02, 0.97, note + f'\nthis pose: pitch {math.degrees(ph):+.1f} deg, omni compressed {c:.2f} of 2.5 cm, clearance belly {cl["belly"]:.2f}, bumper {cl["bumper"]:.2f}, chute nose {cl["exit lip"]:.2f} cm' + extra, transform=ax.transAxes, fontsize=7.6, va='top', zorder=10, bbox=dict(fc='white', ec='none', alpha=0.85))
        ax.set_xticks([]); ax.set_yticks([])
    fig, axes = plt.subplots(2, 2, figsize=(14, 10.2))
    T, p = search(CASES['riser_up'], 'max'); draw(axes[0, 0], T, p, '2 cm riser, going up: the pose with the most tilt (omni already on the step)', 'passes (omni on the step, drive wheel at the riser)', stats(CASES['riser_up'], R, XS))
    T, p = search(CASES['riser_down'], 'min'); draw(axes[0, 1], T, p, '2 cm riser, going down: the pose with the least clearance', 'passes', stats(CASES['riser_down'], R, XS))
    T, p = search(CASES['ramp_up'], 'min'); draw(axes[1, 0], T, p, '25 deg ramp, going up: the foot of the ramp, least clearance', 'passes (rear nub + chamfer let the body pitch up to 25 deg)', stats(CASES['ramp_up'], R, XS))
    T, p = search(ramp_with_bump(25, 12, 3, 2.0), 'min'); draw(axes[1, 1], T, p, 'Dangerous Zone only: 2 cm bump on a 25 deg ramp (rules 3.5.4, 3.5.7)', 'FAILS: belly and bumper clear by under 1 mm; known limitation', stats(ramp_with_bump(25, 12, 3, 2.0), R, XS))
    fig.suptitle('Figure 5. Side view poses on the terrain (stiff-spring pose, belly 3.5, nub 2.0, rear chamfer). Dark circle = drive wheel, green = omni, triangle = chute nose', fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); save(fig, out, 'fig5_terrain')

# ------------------------------------------------------------------------------------------------ figure 6: sensor map
def ray_len(x, y, aim_deg, half=WALL_NOM):
    a = math.radians(aim_deg); c, s = math.cos(a), math.sin(a); ts = []
    if abs(c) > 1e-9: ts.append(((half if c > 0 else -half) - x) / c)
    if abs(s) > 1e-9: ts.append(((half if s > 0 else -half) - y) / s)
    return min(t for t in ts if t > 0)

def fig_sensors(out):
    fig = plt.figure(figsize=(15.5, 8.8)); gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.0], wspace=0.0)
    ax = fig.add_subplot(gs[0]); ax2 = fig.add_subplot(gs[1]); ax2.axis('off')
    fig.subplots_adjust(left=0.05, right=0.995, top=0.93, bottom=0.07)
    ax.set_aspect('equal'); ax.set_xlim(-15.5, 15.5); ax.set_ylim(-15.2, 15.2)
    ax.add_patch(Circle((0, 0), R_BODY, fc='#F1EFE8', ec='k', lw=2.0, zorder=2)); ax.add_patch(Circle((0, 0), R_SWEPT, fill=False, ec='k', ls='--', lw=0.9, zorder=2))
    for y in (WALL_NOM, -WALL_NOM): ax.axhline(y, color='#888780', lw=1.4, zorder=1)
    ax.axvline(WALL_NOM, color='#888780', lw=1.4, zorder=1); ax.axvline(-WALL_NOM, color='#888780', lw=1.4, zorder=1)
    for s in (1, -1):
        ax.add_patch(Rectangle((-WHEEL['r'], min(s * WHEEL['y0'], s * WHEEL['y1'])), 2 * WHEEL['r'], WHEEL['y1'] - WHEEL['y0'], fc=C['wheel'], alpha=0.5, ec='k', zorder=3))
        a0, a1 = (BUMPER['a0'], BUMPER['a1']) if s > 0 else (-BUMPER['a1'], -BUMPER['a0'])
        ax.add_patch(Wedge((0, 0), BUMPER['r_out'], a0, a1, width=BUMPER['t'], fc=C['bump'], ec=C['bump'], zorder=5))
        a = math.radians(s * BUMPER_SW_DEG); r = BUMPER['r_out'] - 0.55; poly(ax, rect_poly(r * math.cos(a), r * math.sin(a), 0.7, 0.45, s * BUMPER_SW_DEG), fc=C['sw'], zorder=8)
        ld, lw, _ = CAM['lens']; t = math.radians(CAM['tilt']); cx = CAM['tip_r'] * math.cos(math.radians(CAM['psi'])); cy = CAM['tip_r'] * math.sin(math.radians(CAM['psi']))
        half = (WALL_NOM - cy) * math.tan(math.radians(CAM['hfov'] / 2))
        poly(ax, [(cx, s * cy), (cx - half, s * WALL_NOM), (cx + half, s * WALL_NOM)], fc=C['cam'], alpha=0.12, ec=C['cam'], lw=0.5, zorder=3)
        ax.add_patch(Rectangle((cx - lw / 2, min(s * (cy - ld * math.cos(t)), s * cy)), lw, ld * math.cos(t), fc=C['cam'], alpha=0.7, zorder=6))
    ax.add_patch(Rectangle((OMNI['rest'][0] - 3, -1), 6, 2, fc=C['omni'], alpha=0.45, zorder=4))
    parts, _ = load_pack()
    for name, d in parts.items():
        l, w = PART_SIZES[name]; pts = rect_poly(d['x'], d['y'], l, w, d['angle'])
        poly(ax, pts, fc=pack_color(name), alpha=0.22, ec=pack_color(name), ls='--', zorder=3)
        if name == PCB_NAME: pcb = SPoly(pts); ax.text(d['x'] + 3.4, d['y'], 'main PCB\n(GIGA shield)', fontsize=7.5, ha='center', va='center', color='#04342C', zorder=8)
        else: ax.text(d['x'], d['y'], 'battery', fontsize=7, ha='center', va='center', zorder=8)
    reads = {'F': 45, 'FL': 51.5, 'FR': 51.5, 'SFL': 67, 'SFR': 67, 'SRL': 67, 'SRR': 67, 'RL': 47, 'RR': 47}
    mux = {'F': 'A0', 'FL': 'A1', 'FR': 'A2', 'SFL': 'A3', 'SFR': 'A4', 'SRL': 'B0', 'SRR': 'B1', 'RL': 'B2', 'RR': 'B3'}
    for i, (nm, x, y, aim) in enumerate(TOF, 1):
        rl = ray_len(x, y, aim)
        ax.add_patch(Wedge((x, y), rl, aim - TOF_CONE / 2, aim + TOF_CONE / 2, fc=C['tof'], alpha=0.18, ec=C['tof'], lw=0.5, zorder=3))
        poly(ax, tof_rect(x, y, aim), fc=C['tof'], ec='k', lw=0.6, zorder=8)
        p0, p1 = nearest_points(SPoint(x, y), pcb); ax.plot([p0.x, p1.x], [p0.y, p1.y], color='#5F5E5A', lw=0.6, ls=':', zorder=3)
        lx, ly = (x * 1.2, y * 1.2) if nm != 'F' else (x - 1.5, y + 0.0); ax.text(lx, ly, str(i), fontsize=10, ha='center', va='center', color='white', zorder=9, bbox=dict(boxstyle='circle,pad=0.2', fc='#185FA5', ec='none'))
    for (px_, py_, lab) in ((FLOOR_FRONT['x'], FLOOR_FRONT['y'], '10'), (SILVER['x'], SILVER['y'], '11')):
        poly(ax, rect_poly(px_, py_, 1.6 if lab == '10' else SILVER['l'], 1.6 if lab == '10' else SILVER['w'], 0), fc=C['fs'], ec=C['bump'], zorder=9); ax.text(px_, py_, lab, fontsize=7.5, ha='center', va='center', zorder=10, fontweight='bold')
    ax.annotate('', xy=(14.8, -13.8), xytext=(11.3, -13.8), arrowprops=dict(arrowstyle='->', lw=1.4)); ax.text(13.0, -14.6, 'forward', ha='center', fontsize=8)
    ax.set_xlabel('x (cm)'); ax.set_ylabel('y (cm)'); ax.set_title('Robot centred in a tile, square to the walls (grey walls at 14 cm: a 28 cm path)', fontsize=10)
    rows = [('#', 'sensor', 'x, y (cm)', 'aim', 'reads', 'mux port')]
    desc = {'F': 'front centre (recessed 1 cm)', 'FL': 'front, toed out left', 'FR': 'front, toed out right', 'SFL': 'side, front left', 'SFR': 'side, front right', 'SRL': 'side, rear left', 'SRR': 'side, rear right', 'RL': 'rear left', 'RR': 'rear right'}
    for i, (nm, x, y, aim) in enumerate(TOF, 1): rows.append((str(i), f'{nm}: {desc[nm]}', f'{x:5.2f}, {y:+5.2f}', f'{aim:+d} deg' if aim not in (0, 180) else ('0 deg' if aim == 0 else '180 deg'), f'{reads[nm]:.0f} mm', mux[nm]))
    rows.append(('10', 'FP: front floor port (black, blue, red)', f'{FLOOR_FRONT["x"]:.2f}, {FLOOR_FRONT["y"]:+.2f}', 'down', 'floor', 'A5'))
    rows.append(('11', 'SM: silver pair (straight + tilted 20 deg)', f'{SILVER["x"]:.2f}, {SILVER["y"]:+.2f}', 'down', 'floor', 'A6'))
    y0 = 0.95; dy = 0.058
    ax2.text(0.0, 1.0, 'Sensor list (positions at body R 10.5; ToF modules at z 10.0)', fontsize=10, fontweight='bold', transform=ax2.transAxes)
    colx = [0.0, 0.05, 0.50, 0.66, 0.79, 0.91]
    for r_i, row in enumerate(rows):
        yy = y0 - r_i * dy
        for c_i, cell in enumerate(row):
            ax2.text(colx[c_i], yy, cell, fontsize=7.8 if r_i else 8.2, fontweight='bold' if r_i == 0 else 'normal', transform=ax2.transAxes, va='center', color='#0C447C' if (r_i and r_i <= 9 and c_i == 0) else '#222222')
        ax2.plot([0, 1.0], [yy - dy / 2, yy - dy / 2], color='#D3D1C7', lw=0.5, transform=ax2.transAxes, clip_on=False)
    yy = y0 - len(rows) * dy - 0.015
    notes = ['"reads" = distance from the sensor to the wall it faces, robot centred and square in a 28 cm path. VL53L0X minimum range is about 30 mm.',
             'In a 25.2 cm path, 2 cm off centre, a side sensor reads 33 mm, close to that minimum (fallback part: VL53L4CD, 1 mm minimum).',
             'Mux A = TCA9548A at 0x70, mux B = 0x71: 16 ports for 9 ToF + 2 floor ports + 5 spare. The port numbers are a proposal, not decided.',
             'Main PCB inputs and outputs: 4 bumper switch inputs (2 populated), start button, power switch, victim LED + 2 status LEDs, microSD (SPI),',
             '   BNO055 (I2C 0x28, main bus; position not modelled), 2 camera UARTs, dropper N20 + encoder + home switch, 2 drive channels with encoders.']
    for k, n in enumerate(notes): ax2.text(0.0, yy - k * 0.038, n, fontsize=7.4, transform=ax2.transAxes, color='#444441')
    fig.suptitle('Figure 6. Sensor map: nine ToF (blue, numbered 1-9), two floor ports (10, 11), cameras (purple), bumper switches (black)', fontsize=11)
    save(fig, out, 'fig6_sensors')

if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'v3-figures'
    which = sys.argv[2:] or ['top', 'side', 'front', 'plate', 'terrain', 'sensors']
    for w in which: dict(top=fig_top, side=fig_side, front=fig_front, plate=fig_plate, terrain=fig_terrain, sensors=fig_sensors)[w](out)
