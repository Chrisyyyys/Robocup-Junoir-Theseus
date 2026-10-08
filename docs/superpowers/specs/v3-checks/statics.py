import math, sys
W_kg = 1.07; g = 9.81; W = W_kg*g
r_w = 4.0; L = float(sys.argv[1]) if len(sys.argv) > 1 else 7.95; x_com = 0.95; h_com = 6.4   # packing result (pack_v3.json) with placeholder masses; edit to try other centres of mass
front = x_com / L
print(f'placeholder mass {W_kg} kg, COM +{x_com} cm ahead of the axle, height {h_com} cm, omni {L} cm ahead -> front load {100*front:.1f}%')
print()
print('1) Drive wheel climbing a step with no help from other wheels (2WD, rigid wheel): needs friction coefficient mu >= tan(alpha), cos(alpha) = (r-h)/r')
for d in (80, 90, 100):
    r = d/20
    row = []
    for h in (1.0, 1.5, 2.0):
        a = math.acos((r - h)/r); row.append(f'h={h}: mu >= {math.tan(a):.2f}')
    print(f'   wheel dia {d} mm: ' + '   '.join(row))
print('   (rigid-wheel statics; soft cast silicone, speed and the real stair surface decide. Measure mu: tilt-plane test with a tyre sample on the stair material)')
print()
print('2) Passive 60 mm omni climbing a step, pushed by the drive wheels (traction mu=1.5 assumed)')
for h in (1.0, 2.0):
    r = 3.0; a = math.acos((r - h)/r); need = front*math.tan(a); have = (1 - front)*1.5
    print(f'   step {h} cm: push needed {need:.2f} W, traction available {have:.2f} W  -> margin x{have/need:.1f}')
for xc in (1.5, 2.0):
    f2 = xc/L; a = math.acos((3.0 - 2.0)/3.0); print(f'   if the COM is +{xc} cm (front load {100*f2:.0f}%): push needed {f2*math.tan(a):.2f} W vs traction {(1-f2)*1.5:.2f} W')
print()
print('3) Motor torque (Pololu 20D 195:1, 12 V: gearbox limit 5 kg-cm, stall 10 kg-cm at 1.6 A, 72 rpm no-load)')
Ww = (1 - front)/2*W
tau_step = Ww*(r_w/100)*math.sin(math.acos((r_w-2.0)/r_w))
tau_ramp = (W*math.sin(math.radians(25))/2)*(r_w/100)
print(f'   per wheel, 2 cm riser (rigid): {tau_step:.3f} N-m = {tau_step*10.197:.2f} kg-cm; 25 deg ramp: {tau_ramp:.3f} N-m = {tau_ramp*10.197:.2f} kg-cm (limit 5.0)')
print(f'   no-load speed {72*math.pi*0.08/60:.2f} m/s at 12 V; a 300 mm tile takes at least {0.3/(72*math.pi*0.08/60):.1f} s')
print()
print('4) Ramp 25 deg: weight line vs the drive contact')
behind = h_com*math.tan(math.radians(25)) - x_com
print(f'   COM is {behind:.2f} cm BEHIND the drive wheel contact -> the robot rests on the rear skid (8 cm behind, 2 cm off the floor): tilts back atan(2/8)={math.degrees(math.atan(2/8)):.1f} deg, skid carries ~{100*behind/8:.0f}% of the normal load')
need_com = h_com*math.tan(math.radians(25))
print(f'   no rock-back needs the COM at least {need_com:.1f} cm ahead (front load {100*need_com/L:.0f}%)')
print('   descending a 25 deg ramp: front load =', f'{100*(x_com + h_com*math.tan(math.radians(25)))/L:.0f}%', '-> spring must carry', f'{(x_com + h_com*math.tan(math.radians(25)))/L*W:.1f} N', 'within 2.5 cm travel (k >=', f'{((x_com + h_com*math.tan(math.radians(25)))/L*W - front*W)/2.5:.1f} N/cm)')
