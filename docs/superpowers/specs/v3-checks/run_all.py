"""Run every V3 check and write the raw output of each to results/ (about 10-15 minutes). Usage: python run_all.py [name ...]   (the slow controller_fit checks run only when named)
Needs: pip install shapely numpy matplotlib. Run it from this folder. final_pack2.py rewrites pack_v3.json (random search with a fixed seed: the result repeats)."""
import os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE); os.makedirs('results', exist_ok=True)
RUNS = [
    ('statics', ['statics.py'], 'friction, torque, omni push, ramp statics (spec section 4)'),
    ('terrain', ['nubtable.py', 'all'], 'side-view terrain results and the nub-height table (section 4)'),
    ('omni_travel', ['more_checks.py', 'travel'], 'omni travel 2.5 versus 3.0 cm (section 4)'),
    ('silver_module', ['more_checks.py', 'axle'], 'silver module near the axle: clearance (section 5)'),
    ('front_floor_port', ['fsens.py'], 'front floor port: clearance (section 5)'),
    ('ramp_foot', ['tofcone.py'], 'ToF reading at the foot of a 25 degree ramp (section 5)'),
    ('plate_variants', ['variants.py'], 'ring radius needed for 13 and 14 mm pockets (section 7)'),
    ('plate_tolerance', ['platetol.py', '38.6', '14.0', '16.0'], 'plate positioning tolerance, 14 mm pockets and the 16 mm slot of rev 4 (section 7; rev 3 had 14.5)'),
    ('kit_landing', ['kit_final.py'], 'chute exit speed, landing point, distance to the victim (section 7)'),
    ('bumper_and_chute_exit', ['geom_checks.py'], 'bumper end angle and chute exit x at R 10.5 (sections 6, 7)'),
    ('packing', ['final_pack2.py'], 'controller stack and battery packing, centre of mass (section 9)'),
    ('tof_cones', ['tof_check2.py'], 'every ToF cone traced against the robot (section 5)'),
    ('camera_beside_wheel', ['beside.py'], 'camera beside the wheel at victim height: placements found (section 8)'),
    ('camera_recess', ['cam_recess.py'], 'camera lens recessed: feasibility and packing (section 8)'),
    ('camera_view', ['camview.py'], 'camera view of the wall, kit landing (section 8)'),
    ('figures', ['figures.py', '../v3-figures'], 'the six figures'),
    # mechanical design 2026-10-07
    ('chute_exit', ['chute_exit.py'], 'lowest point of the chute where it leaves the body, round and square (mechanical design section 6)'),
    ('mount_check', ['mount_check.py'], 'GIGA mounting holes against the parts around them, ToF cable runs, clip numbers (mechanical design sections 5, 8)'),
    ('stepper_bay', ['stepper_bay.py'], '28BYJ-48 under the dropper floor: which orientations clear the stack, battery and hoppers (mechanical design section 6.3)'),
    ('ring_gaps', ['ring_gaps.py'], 'room on the ring for the six frame screws and four lid hooks between the ToF and camera windows (mechanical design sections 3.1, 7.1)'),
    ('statics_omni_7', ['statics.py', '7.0'], 'statics with the omni 7.0 cm ahead of the axle: front load, push, spring (mechanical design section 4)'),
    ('kit_landing_square', ['kit_final.py', '4.15'], 'chute exit speed and landing with the exit axis at z 4.15 (mechanical design section 6.2)'),
    ('structure_figs', ['structure_figs.py', '../v3-figures'], 'figures 7 to 9'),
    ('cube_slot', ['cube_slot.py'], 'which cube attitudes fit a square slot: 16 mm leaves out only the corner stand (mechanical design section 6.2)'),
    ('fsens_front', ['fsens_front.py'], 'floor-sensor window clearance on the terrain cases with the sensors 6.5 to 8.5 cm ahead of the axle (mechanical design section 5.3)'),
    ('handle_calc', ['handle_calc.py'], 'front bridge as a cantilever under the handle post, and the nose-up angle when carried by the bar (mechanical design sections 3.2, 7.2)'),
    ('axle_shift', ['axle_shift.py'], 'terrain cases with the drive axle 0, 1, 2, 3 cm behind the body centre (mechanical design section 4, axle position)'),
    # slow (about 5-15 minutes each): only run when named
    ('controller_fit', ['fit_stack_only.py'], 'does the GIGA stack (1.5 / 1.9 / 2.5 cm) or a smaller main board fit at R 9.5, 10.0, 10.5 (sections 0.2, 9)', True),
    ('controller_fit_with_battery', ['fit_R.py'], 'same with the battery placed too, and the small-board case at R 9.5 (sections 0.5, 9)', True),
    ('omni_inside', ['omni_inside.py'], 'terrain cases with the omni moved inside the body, 7.95 / 7.3 / 7.0 / 6.5 (mechanical design section 4)', True),
]

if __name__ == '__main__':
    want = set(sys.argv[1:])
    for entry in RUNS:
        name, cmd, what = entry[:3]; slow = len(entry) > 3 and entry[3]
        if (want and name not in want) or (not want and slow): continue
        t0 = time.time(); print(f'{name:<24} {what} ...', end=' ', flush=True)
        r = subprocess.run([sys.executable] + cmd, capture_output=True, text=True)
        open(os.path.join('results', name + '.txt'), 'w', encoding='utf-8').write(r.stdout + (('\n[stderr]\n' + r.stderr) if r.stderr.strip() else ''))
        print(f'{"ok" if r.returncode == 0 else "FAILED"} ({time.time() - t0:.0f} s)', flush=True)
