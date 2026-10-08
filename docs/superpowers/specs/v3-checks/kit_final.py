import math, sys
g = 9.81
R = 10.5; ring = 3.86; xp = -2.0; ex = -6.0
sx, sy = xp + ring * math.cos(math.radians(135)), ring * math.sin(math.radians(135)); ey = math.sqrt(R * R - ex * ex)
z0 = 9.0 - 0.3 - 0.75; z1 = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0 + 0.75   # exit axis height: 3.75 (rev 3) or 4.15 (13 mm square channel, mechanical design)
plan = math.hypot(ex - sx, ey - sy); drop = z0 - z1; L = math.hypot(plan, drop); sl = math.atan2(drop, plan)
print(f'chute: slot ({sx:.2f},{sy:.2f}) -> exit ({ex:.2f},{ey:.2f}), plan {plan:.2f} cm, drop {drop:.2f} cm, slope {math.degrees(sl):.1f} deg, path {L:.2f} cm')
cam_x = 8.8 * math.cos(math.radians(88))
for mu in (0.25, 0.35, 0.5):
    a = g * (math.sin(sl) - mu * math.cos(sl)); v = math.sqrt(2 * a * L / 100); vh = v * math.cos(sl); vz = v * math.sin(sl)
    h = (z1 - 0.5) / 100; t = (-vz + math.sqrt(vz * vz + 2 * g * h)) / g; dist = vh * t * 100
    ux, uy = (ex - sx) / plan, (ey - sy) / plan; lx, ly = ex + ux * dist, ey + uy * dist
    vimp = math.sqrt(vh ** 2 + (vz + g * t) ** 2)
    print(f'  mu {mu}: exit speed {v:.2f} m/s, flight {t*1000:.0f} ms, carries {dist:.1f} cm, lands ({lx:.1f}, {ly:.1f}), {14.0-ly:.1f} cm from a 28 cm wall; impact speed {vimp:.2f} m/s (Sprint 3 test from 3 cm: {math.sqrt(2*g*0.03):.2f})')
    if mu == 0.35:
        for u in (-3.0, 0.0, 2.5, 3.6):
            along = (cam_x + u) - lx; across = 14.0 - ly
            print(f'     victim {u:+.1f} cm ahead of the camera at the stop: along-wall {along:5.1f}, to the wall {across:4.1f}, plan distance {math.hypot(along, across):5.1f} cm (rule: kit entirely within 15 cm; slack along the wall {math.sqrt(15**2-across**2):.1f} cm)')
