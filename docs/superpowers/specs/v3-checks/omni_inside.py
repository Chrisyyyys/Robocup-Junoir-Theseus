"""Terrain cases with the front omni moved back inside the body outline (mechanical design 2026-10-07, section 4).
Uses sidesim2 / nubtable unchanged; only the Robot parameters differ: omni centre x (arm length kept at 4.49, so the pivot moves with it) and the chute lip.
The chute lip is (-6.2, 2.67) from the 3D model (lowest point of the round tube where it leaves the wall); the spec's 2D value is (-7.0, 3.0). About 8 minutes. cm."""
import math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sidesim2 import Robot
import nubtable as N

ARM_L = math.hypot(7.95 - 3.5, 3.0 - 3.6)          # 4.49: the arm length checked in the spec

def mk2(omni_x=7.95, lip=(-7.0, 3.0), travel=2.5, nub=2.0, belly=3.5, bumper_bottom=4.0):
    px = omni_x - math.sqrt(ARM_L ** 2 - 0.6 ** 2)
    return Robot(belly=belly, bumper_bottom=bumper_bottom, bumper_x=(5.8, 10.8), tip_x=11.0, lip=lip,
                 skid=(-8.0, nub, 0.4), rear_corner=(-10.5, 5.3), rear_chamfer=(-6.5, 5.3), pivot=(px, 3.6),
                 omni_rest=(omni_x, 3.0), travel=travel)

if __name__ == '__main__':
    cases = list(N.CASES)
    print('--- rev 3 as checked in the spec (omni centre 7.95, front edge 10.95, lip x -7.0 z 3.0)')
    N.show('omni 7.95', mk2(7.95), cases, N.CASES)
    print('--- rev 3 omni with the chute lowest point from the 3D model (lip z 2.67)')
    N.show('omni 7.95, lip 2.67', mk2(7.95, lip=(-6.2, 2.67)), ['riser_down', 'ramp_down', 'bump2', 'riser_up'], N.CASES)
    for ox in (7.3, 7.0, 6.5):
        print(f'--- omni centre {ox} (front edge {ox + 3.0:.2f}), arm length kept, pivot x {ox - math.sqrt(ARM_L ** 2 - 0.36):.2f}, lip z 2.67')
        N.show(f'omni {ox}', mk2(ox, lip=(-6.2, 2.67)), cases, N.CASES)
    print('--- Dangerous Zone extras, lip z 2.67')
    N.show('omni 7.0', mk2(7.0, lip=(-6.2, 2.67)), list(N.EXTRA), N.EXTRA)
    N.show('omni 7.95', mk2(7.95, lip=(-6.2, 2.67)), list(N.EXTRA), N.EXTRA)
