"""Terrain cases with the omni mount of 9 Oct (spec 4.1): the real 60 mm wheel, rest axle at x 6.85, the pivot raised to z 4.2 and the arm 4.6 cm long (8 Oct: rest 7.0, pivot z 3.6, arm 4.49).
Same side-view simulator and cases as omni_inside.py (sidesim2, nubtable); only the Robot's pivot, rest position and arm change. The chute lip is the 3D model's (-6.2, 2.67). About 4 minutes. cm.
    python omni_terrain9.py            both layouts (8 Oct, then 9 Oct) on the nine required cases and the Dangerous Zone extras"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sidesim2 import Robot  # noqa: E402
import nubtable as N  # noqa: E402

import omni_mount as M  # noqa: E402


def mk(layout, lip=(-6.2, 2.67), travel=2.5, nub=2.0, belly=3.5, bumper_bottom=4.0):
    L = M.layout(layout)
    return Robot(belly=belly, bumper_bottom=bumper_bottom, bumper_x=(5.8, 10.8), tip_x=11.0, lip=lip,
                 skid=(-8.0, nub, 0.4), rear_corner=(-10.5, 5.3), rear_chamfer=(-6.5, 5.3), pivot=(L['px'], L['pz']),
                 omni_rest=(L['rest_x'], 3.0), travel=travel)


if __name__ == '__main__':
    cases = list(N.CASES)
    for lay in (M.OLD, M.NEW):
        L = M.layout(lay)
        print('--- omni layout of %s: rest axle (%.2f, 3.00), pivot (%.3f, %.2f), arm %.2f cm, chute lip z 2.67' % (lay['name'], L['rest_x'], L['px'], L['pz'], L['arm_len']))
        N.show('omni ' + lay['name'], mk(lay), cases, N.CASES)
        print('--- Dangerous Zone extras, same layout')
        N.show('omni ' + lay['name'], mk(lay), list(N.EXTRA), N.EXTRA)
