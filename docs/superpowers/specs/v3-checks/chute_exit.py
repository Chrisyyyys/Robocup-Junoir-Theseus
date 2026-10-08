"""Lowest point of a chute where it leaves the R 10.5 body, for the round 15 mm bore (outer r 0.9) and the rounded-square 13 mm channel (outer 16.2 mm),
at several exit heights. The 2D side view ends the tube with a flat cap at the exit point and so reads z 3.0; a straight tube meets the cylindrical wall
at about 22 degrees, so its lower edge runs further down the slope (mechanical design 2026-10-07, section 6; 3D model: z 2.67 for the round tube). cm."""
import math

R_BODY = 10.5
SLOT_B = (-4.73, 2.73, 7.95)          # slot B centre, axis start z 7.95 (spec section 7); slot A is the mirror image in y
EXIT_XY = (-6.0, 8.62)                # exit on the wall at x -6.0 (spec)

def tube_min_z(p0, e, shape, half, r_body=R_BODY, n=181):
    d = [e[i] - p0[i] for i in range(3)]
    L = math.sqrt(sum(c * c for c in d)); d = [c / L for c in d]
    h = math.hypot(d[0], d[1]); hx, hy = d[0] / h, d[1] / h
    lat = (-hy, hx, 0.0)                                       # horizontal, perpendicular to the axis
    down = tuple(-c for c in (-d[2] * hx, -d[2] * hy, h))      # perpendicular to the axis in the vertical plane, pointing down
    pts = []
    if shape == 'round':
        pts = [(half * math.cos(2 * math.pi * k / n), half * math.sin(2 * math.pi * k / n)) for k in range(n)]
    else:
        for k in range(-10, 11):
            pts += [(k / 10 * half, -half), (k / 10 * half, half), (-half, k / 10 * half), (half, k / 10 * half)]
    best = 1e9
    for (a, b) in pts:                                         # a along lat, b along 'down'
        def pos(s):
            return [p0[i] + d[i] * s + lat[i] * a + down[i] * b for i in range(3)]
        lo, hi = 0.0, 3.0 * L
        for _ in range(60):                                    # march along the axis until the point leaves the body radius
            mid = (lo + hi) / 2
            x, y, z = pos(mid)
            if math.hypot(x, y) <= r_body: lo = mid
            else: hi = mid
        best = min(best, pos(lo)[2])
    return best

if __name__ == '__main__':
    p0 = SLOT_B
    for ez in (3.75, 4.0, 4.1, 4.15, 4.2):
        e = (EXIT_XY[0], EXIT_XY[1], ez)
        d = [e[i] - p0[i] for i in range(3)]
        slope = math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))
        print(f'exit axis z {ez:.2f}  slope {slope:4.1f} deg   round r0.9 lowest z {tube_min_z(p0, e, "round", 0.9):5.2f}   square outer 16.2 lowest z {tube_min_z(p0, e, "square", 0.81):5.2f}')
    lo, hi = 3.75, 4.6                                          # exit axis height that puts the square channel's lowest point at z 3.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if tube_min_z(p0, (EXIT_XY[0], EXIT_XY[1], mid), 'square', 0.81) < 3.0: lo = mid
        else: hi = mid
    print(f'square channel: exit axis z {hi:.2f} puts the lowest point at z 3.00 (slope {math.degrees(math.atan2(p0[2] - hi, math.hypot(EXIT_XY[0] - p0[0], EXIT_XY[1] - p0[1]))):.1f} deg)')
