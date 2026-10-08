import json, math
# placeholder masses in grams (weigh the real parts!), position (x forward from the axle, z height above the floor) in cm
fixed = [
 ('drive wheels cast silicone 2x55', 110, 0.0, 4.0), ('gearmotors 20D+encoder 2x95', 190, 0.0, 4.0),
 ('omni 60 mm + arm', 50, 7.0, 4.0), ('printed chassis', 150, 0.0, 6.5), ('lid+handle', 45, 0.0, 12.0),
 ('dropper plate+floor', 30, -2.0, 9.3), ('chutes', 15, -5.5, 6.0), ('N20+encoder', 12, -2.0, 7.0), ('bumpers+switches', 25, 8.5, 5.3),
 ('mux x2, regulator, wiring', 55, 0.0, 6.0), ('9 ToF boards + cables', 30, 1.0, 10.0), ('2 cameras + LED strips', 55, 0.0, 10.0),
 ('8 kits', 48, -0.5, 9.5), ('BNO055, TCS34725, LCD, hardware', 70, 0.0, 7.0)]
def com(giga, batt):
    items = fixed + [('GIGA + shield', 80, giga[0], giga[1]), ('battery', 110, batt[0], batt[1])]
    M = sum(m for _, m, _, _ in items)
    x = sum(m*x for _, m, x, _ in items)/M; z = sum(m*z for _, m, _, z in items)/M
    return M, x, z
L = 7.95
if __name__ == '__main__':
    for label, giga, batt in (('battery at the rear (what an unconstrained packing does)', (1.61, 7.4), (-6.30, 4.85)),
                              ('battery in the front bay (x=+4, z 4.8)', (1.61, 7.4), (4.0, 4.85)),
                              ('battery forward and GIGA centred', (0.0, 7.4), (4.0, 4.85)),
                              ('battery at x=+2.5', (1.61, 7.4), (2.5, 4.85))):
        M, x, z = com(giga, batt)
        print(f'{label:<56} mass {M/1000:.2f} kg  COM x={x:+.2f} cm (front load {100*x/L:4.1f}%)  COM height {z:.1f} cm')
    print()
    print('sensitivity: every 100 g moved 5 cm shifts the COM by', round(100*5/1100, 2), 'cm; the real battery mass and position decide the front load')
