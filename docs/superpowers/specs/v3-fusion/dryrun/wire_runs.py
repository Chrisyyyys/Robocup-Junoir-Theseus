"""Wire runs from the controls to the connector area of the main PCB (spec 7.3): the first rev 4 draft's rear panel against the front deck of the plan.
Manhattan length along the structure (|dx| + |dy| + |dz|, cm) from each part to J12 on the connector edge of the GIGA (7.40, -2.74) at the height of the shield (z 7.3). [calc]
The rear panel of the first draft stood on the ring top (z 11.2) at x -9.4, between the RL and RR windows (165 to 195 degrees, about 5 cm of arc): positions y -2 to +2 are used for its parts.
Usage: python wire_runs.py"""
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'v3-checks'))
import plan_code  # noqa: E402

build = tempfile.mkdtemp(prefix='rev4_plan_')
names, problems = plan_code.assemble(build)
assert not problems, problems
sys.path.insert(0, build)
import v3_params4 as P  # noqa: E402

J12 = P.giga_j12()
Z_PCB = 7.3


def run(x, y, z):
    return abs(x - J12[0]) + abs(y - J12[1]) + abs(z - Z_PCB)


rear = [run(-9.4, y, 11.2) for y in (-2.0, -1.0, 0.0, 1.0, 2.0)]
print('first draft, rear panel on the ring (x -9.4, y -2 to +2, z 11.2): %.1f to %.1f cm' % (min(rear), max(rear)))
print('front deck of the plan:')
for nm, x, y, shape, z0, z1, colour in P.CONTROLS:
    print('  %-14s at (%.2f, %.2f, z %.1f): %.1f cm' % (nm, x, y, z0, run(x, y, z0)))
led = P.HANDLE['led']
print('  %-14s at (%.2f, %.2f, z %.1f), on the handle bar, wire down the post: %.1f cm' % ('Victim LED', led['x'], led['y'], led['z'][0], run(led['x'], led['y'], led['z'][0])))
cx, cy, ang = P.usb_pose()
print('  USB-C socket (front-right wall) to J12 in a straight line: %.1f cm' % math.hypot(cx - J12[0], cy - J12[1]))
