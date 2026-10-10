"""ToF board mount, plan view and 3D positions (mechanical design section 8.3, rev 4 of 9 Oct). cm, degrees; x forward, y left, z up. Plain Python (the Fusion builders import it); shapely is imported only inside the
functions that build polygons for the checks.

The board and the mount are described in v3_params4 (TOF_BOARD, TOF_MOUNT, TOF). The board stands with its 25.4 mm side vertical: lying down, a 25.4 mm wide board needs 3.2 to 3.7 cm of ring (the pockets of the front-left
and side sensors then overlap, 9 Oct), standing up it needs 2.4 cm and the frame grows by 5 mm (ring top 11.7). Local frame of one sensor: origin at the centre of the PCB's front (chip) face at height TOF_Z, u along the
beam (outwards), v to the left of the beam, w up. The PCB lies at u -0.16 .. 0 (v +-0.889, w +-1.27), the chip and the two connectors stand out of its front face (to u +0.10 and +0.305: the chip at the centre, a connector
at the top and one at the bottom end), the two posts stand in front of it (u 0 .. +0.36, the recess floor) at the upper holes, the rear wall is behind it (u -0.39 and further back). The board is dropped into the pocket from
above and rests on the pocket floor; two M2.5 x 6 screws from behind (through the rear wall's access holes) pull it onto the posts; the plug of the lower connector comes up through a shaft in the floor."""
import math

import v3_params4 as P

B, M = P.TOF_BOARD, P.TOF_MOUNT
Z0 = P.TOF_Z
R_BLOCK = 10.47            # the printed block stays inside the body radius 10.5


def frame(x, y, aim):
    a = math.radians(aim)
    return (x, y, math.cos(a), math.sin(a), -math.sin(a), math.cos(a))


def uv(fr, u, v):
    """World (x, y) of the local point (u, v)."""
    x, y, ax, ay, nx, ny = fr
    return x + u * ax + v * nx, y + u * ay + v * ny


def rect_pts(fr, u0, u1, v0, v1):
    """The corners of the rectangle u0..u1 by v0..v1, as world (x, y) points, counter-clockwise for u1 > u0 and v1 > v0 (seen from above, u pointing outwards)."""
    return [uv(fr, u0, v0), uv(fr, u1, v0), uv(fr, u1, v1), uv(fr, u0, v1)]


def sensors():
    """name -> (x, y, aim)."""
    return {nm: (x, y, aim) for nm, x, y, aim in P.TOF}


def u_rear():
    """Front face of the rear wall (the pocket's back): behind the PCB by the PCB, the screw heads and a little play."""
    return -(B['t'] + M['rear_gap'])


def v_half():
    """The pocket's half width along the ring: half the board's short side and a little play."""
    return B['short'] / 2 + M['side_gap']


def pocket_plan(name):
    x, y, aim = sensors()[name]
    return rect_pts(frame(x, y, aim), u_rear(), M['rec'], -v_half(), v_half())


def block_plan(name):
    """What is added to the ring so that a pocket that hangs in the bore has walls (a box round the pocket: rear wall, side walls, front wall; the pocket is cut out of it afterwards). Where a corner
    would stand out of the body radius it is pulled in to R_BLOCK."""
    x, y, aim = sensors()[name]
    pts = rect_pts(frame(x, y, aim), u_rear() - M['rear_wall'], M['rec'] + M['front_wall'], -v_half() - M['side_wall'], v_half() + M['side_wall'])
    out = []
    for px, py in pts:
        r = math.hypot(px, py)
        out.append((px, py) if r <= R_BLOCK else (px * R_BLOCK / r, py * R_BLOCK / r))
    return out


def shaft_plan(name):
    """The shaft down through the pocket floor for the plug of the lower connector (the connector's mouth faces down; 0.7 cm wide, a little wider than the plug)."""
    x, y, aim = sensors()[name]
    return rect_pts(frame(x, y, aim), -0.05, M['rec'] + 0.04, -M['shaft_v'], M['shaft_v'])


def tunnel_plan(name, reach=4.0):
    """The beam tunnel from the recess outwards, a straight prism 2 x tunnel_v wide (z: Z0 +- tunnel_w)."""
    x, y, aim = sensors()[name]
    return rect_pts(frame(x, y, aim), M['rec'] - 0.05, reach, -M['tunnel_v'], M['tunnel_v'])


def _two(name, u0, u1, half_v):
    """Plan rectangles of the two features at the upper holes (v = +-hole_short), u0..u1 long, +-half_v wide."""
    x, y, aim = sensors()[name]
    fr = frame(x, y, aim)
    return [rect_pts(fr, u0, u1, s * B['hole_short'] - half_v, s * B['hole_short'] + half_v) for s in (1, -1)]


def post_plans(name):
    """The two posts (square in the model, round with an M2.5 pilot in the print): from the board's front face to a little into the front wall behind the recess floor."""
    return _two(name, 0.0, M['rec'] + 0.06, M['post_r'])


def pilot_plans(name):
    """The pilot holes in the posts (square 2 x 2 mm in the model): from the board's front face 4.5 mm into the post and the wall behind it."""
    return _two(name, -0.05, 0.45, M['pilot_r'])


def access_plans(name):
    """The screwdriver's way in: square holes through the rear wall, in line with the screws."""
    return _two(name, u_rear() - M['rear_wall'] - 0.05, u_rear() + 0.02, M['access_r'])


def screw_plans(name):
    """(heads, shafts) of the two M2.5 screws as plan rectangles: the head against the PCB's rear face, the shaft through the PCB into the post."""
    hd, hh = B['screw_head']
    heads = _two(name, -B['t'] - hh, -B['t'], hd / 2)
    shafts = _two(name, -B['t'], -B['t'] + B['screw_len'], 0.10)
    return heads, shafts


def hole_z():
    """Height of the two upper holes (the posts, pilots, screws and access holes are all at this height)."""
    return Z0 + B['hole_long']


def board_z():
    """(bottom, top) of the PCB."""
    return Z0 - B['long'] / 2, Z0 + B['long'] / 2


def beam_origin(name):
    """Where the cone's rays start: the middle of the chip's window, 0.1 cm in front of the PCB's front face."""
    x, y, aim = sensors()[name]
    ox, oy = uv(frame(x, y, aim), B['chip_u'] + 0.01, 0.0)
    return ox, oy, Z0


# ---------------------------------------------------------------------------------------------------- shapely views for the checks
def polygons(name):
    """Plan polygons (shapely) of one sensor: pcb, connectors, pocket, block, shaft, tunnel, posts, access, and cut = pocket + tunnel + shaft + access (everything removed from the ring)."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    x, y, aim = sensors()[name]
    fr = frame(x, y, aim)
    S = B['short'] / 2
    out = dict(pcb=Polygon(rect_pts(fr, -B['t'], 0.0, -S, S)), connectors=Polygon(rect_pts(fr, 0.0, B['conn_h'], -B['conn_w'] / 2, B['conn_w'] / 2)),
               pocket=Polygon(pocket_plan(name)), block=Polygon(block_plan(name)), shaft=Polygon(shaft_plan(name)), tunnel=Polygon(tunnel_plan(name)),
               posts=[Polygon(p) for p in post_plans(name)], access=[Polygon(p) for p in access_plans(name)])
    out['cut'] = unary_union([out['pocket'], out['tunnel'], out['shaft']] + out['access'])
    return out


def ring_corner_radius(name):
    """Largest distance from the robot's axis of the pocket's corners (the recess corners are the farthest out): at least wall_min must be left of the ring in front of them."""
    return max(math.hypot(px, py) for px, py in pocket_plan(name))


if __name__ == '__main__':
    r_lim = P.FRAME['r_out'] - M['wall_min']
    for nm, (x, y, aim) in sensors().items():
        print('%-4s (%6.2f, %6.2f) aim %4d  pocket corner radius %.2f (limit %.2f)' % (nm, x, y, aim, ring_corner_radius(nm), r_lim))
    print('board z %.2f to %.2f, pocket floor %.2f, upper holes at z %.3f, ring top %.2f' % (board_z() + (M['floor_z'], hole_z(), P.FRAME['z1'])))
