"""Handle numbers of the rev 4 design (mechanical design sections 3.2 and 7.2): the front bridge as a cantilever carrying the T-handle post, and how far the robot hangs nose-up when carried by the bar.
Inputs (cm): bridge web 2.5 wide x 0.3 thick, rib 1.0 wide x 0.8 tall on top (z 9.0 to 9.8), PETG E = 2 GPa, post at x 4.0, front ring at x 9.0 (lever 5.0 cm), loads 15 N (design) and 45 N (x3),
centre of mass (1.09, 6.79) from the dry-run balance report of 9 Oct with the real omni module (0.86, 6.96 before it; 0.89, 6.82 on 8 Oct), grip height 14.8 (middle of the bar). [calc]  Usage: python handle_calc.py"""
import math

E = 2.0e9 / 1.0e4                                     # 2 GPa in N/cm^2
parts = [(2.5, 0.3, 0.15), (1.0, 0.8, 0.7)]           # width, height, centre height above the underside of the web
A = sum(b * h for b, h, c in parts)
yb = sum(b * h * c for b, h, c in parts) / A
I = sum(b * h ** 3 / 12 + b * h * (c - yb) ** 2 for b, h, c in parts)
print('bridge section: area %.2f cm2, centroid %.3f cm above the underside, I = %.4f cm4' % (A, yb, I))
L = 5.0
for F in (15.0, 45.0):
    M = F * L
    print('  %2.0f N at the post, lever %.1f cm: moment %.0f N.cm, stress at the web underside %.1f MPa, tip deflection %.2f mm' % (F, L, M, M * yb / I / 100.0, F * L ** 3 / (3 * E * I) * 10.0))
com_x, com_z, grip_z = 1.09, 6.79, 14.8
print('carried by the bar, nose-up angle (COM x %.2f, z %.2f, grip z %.1f):' % (com_x, com_z, grip_z))
for gx, what in ((0.9, 'rear end of the first 8 cm bar'), (3.3, 'rear end of the 5.4 cm bar'), (6.0, 'middle of the 5.4 cm bar'), (8.7, 'front end')):
    print('  grip at x %.1f (%s): %.1f degrees' % (gx, what, math.degrees(math.atan2(gx - com_x, grip_z - com_z))))
