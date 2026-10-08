"""Terrain cases of the rev 4 body with the drive axle moved back by `a` cm relative to the body centre (mechanical design section 4, 'Axle position').
Everything except the wheels moves forward by `a` in the axle frame: the omni (7.0 + a), its pivot, the bumpers, the front tip (11.0 + a), the nub and the rear corner. The chute lip stays
where the wheel needs it (the channel is re-aimed), so its axle-frame x is given directly. Same simulator and cases as omni_inside.py (sidesim2 / nubtable); only the Robot changes.
Usage: python axle_shift.py [a ...]   (default 0 1 2 3; about 4 minutes)"""
import math
import sys

from sidesim2 import Robot          # noqa: E402
import nubtable as N                # noqa: E402

ARM_L = math.hypot(7.95 - 3.5, 3.0 - 3.6)           # the rev 3 arm length, kept


def mk(a, omni_x=7.0, lip_axle=(-6.2, 2.67), travel=2.5, nub=2.0, belly=3.5, bumper_bottom=4.0):
    ox = omni_x + a
    px = ox - math.sqrt(ARM_L ** 2 - 0.36)
    return Robot(belly=belly, bumper_bottom=bumper_bottom, bumper_x=(5.8 + a, 10.8 + a), tip_x=11.0 + a, lip=lip_axle,
                 skid=(-8.0 + a, nub, 0.4), rear_corner=(-10.5 + a, 5.3), rear_chamfer=(-6.5 + a, 5.3), pivot=(px, 3.6), omni_rest=(ox, 3.0), travel=travel)


if __name__ == '__main__':
    cases = list(N.CASES)
    for a in [float(v) for v in sys.argv[1:]] or [0.0, 1.0, 2.0, 3.0]:
        print('=' * 12, 'axle %.1f cm behind the body centre: omni at %.2f from the axle, nub at %.2f, front tip %.2f' % (a, 7.0 + a, -8.0 + a, 11.0 + a), flush=True)
        N.show('axle back %.1f' % a, mk(a), cases, N.CASES)
        N.show('axle back %.1f (DZ)' % a, mk(a), list(N.EXTRA), N.EXTRA)
        sys.stdout.flush()
