"""Dry run of the rev 4 Fusion rebuild without Fusion: the plan's own code (or, with --repo, the modules in v3-fusion/) is built and checked on a geometry emulator.
Usage (from this folder; shapely, numpy and matplotlib must be importable, see v3-checks/README.md):
    python dryrun.py                      everything except the slow clearance report: emulator self-test, plan code, unit tests, 19 build stages with their probes and local
                                          interference, then the whole-model reports; about 4 minutes
    python dryrun.py stages               only the build stages
    python dryrun.py envelope removal     only these reports (envelope belly omni stepper mass interference removal access tof camera clearances)
    python dryrun.py --clearances         everything including the clearance report (about 10 minutes)
    python dryrun.py --repo               use v3-fusion/v4_*.py and v3-checks/v3_params4.py as they are in the repo instead of the code in the plan
Exit status 1 if anything failed. A mass report with RESULT: WARN does not fail the run (the front-lift limit is below the firmware assumption: a finding, not an error)."""
import argparse
import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FUSION_DIR = os.path.dirname(HERE)
CHECKS = os.path.join(os.path.dirname(FUSION_DIR), 'v3-checks')
REPORTS = ['envelope', 'belly', 'omni', 'stepper', 'mass', 'interference', 'removal', 'access', 'tof', 'camera', 'clearances']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', nargs='*', help='stages, tests, or report names (default: tests, stages and every report but clearances)')
    ap.add_argument('--repo', action='store_true', help='use the modules in the repo instead of the code in the plan')
    ap.add_argument('--clearances', action='store_true', help='include the (slow) clearance report')
    args = ap.parse_args()
    want = args.what or (['tests', 'stages'] + [r for r in REPORTS if r != 'clearances'])
    if args.clearances and 'clearances' not in want:
        want.append('clearances')
    results = []                                                   # (name, ok, note)

    # ---- the code under test
    build_dir = None
    sys.path.insert(0, HERE)
    if args.repo:
        test_cwd = CHECKS
    else:
        import plan_code
        build_dir = tempfile.mkdtemp(prefix='rev4_plan_')
        names, problems = plan_code.assemble(build_dir)
        print('code from the plan: %s -> %s' % (', '.join(names), build_dir))
        for p in problems:
            print('  PROBLEM', p)
        results.append(('plan code extracted, patches applied', not problems, '%d problems' % len(problems)))
        sys.path.insert(0, build_dir)
        test_cwd = build_dir

    # ---- the emulator (installed with the plan's axis_prism if fusion_lib does not have one yet), then its self-test
    import fake_fusion as F
    fl_patch = os.path.join(build_dir, 'fusion_lib_patch.py') if build_dir else None
    F.install(axis_prism_source=open(fl_patch, encoding='utf-8').read() if fl_patch and os.path.exists(fl_patch) else None)
    import selftest
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        good = selftest.run()
    print('emulator self-test:', 'ok' if good else 'FAIL')
    if not good:
        print(buf.getvalue())
    results.append(('emulator self-test', good, ''))

    # ---- unit tests of the parameters
    if 'tests' in want:
        env = dict(os.environ)
        env['PYTHONPATH'] = os.pathsep.join([CHECKS] + ([env['PYTHONPATH']] if env.get('PYTHONPATH') else []))
        t0 = time.time()
        proc = subprocess.run([sys.executable, '-W', 'ignore', '-m', 'unittest', 'test_v3_params4'], cwd=test_cwd, env=env, capture_output=True, text=True)
        tail = [ln for ln in (proc.stdout + proc.stderr).strip().split('\n') if ln.strip()][-3:]
        print('unit tests: %s (%.1f s)  %s' % ('ok' if proc.returncode == 0 else 'FAIL', time.time() - t0, ' | '.join(tail)))
        if proc.returncode != 0:
            print(proc.stdout + proc.stderr)
        results.append(('unit tests test_v3_params4', proc.returncode == 0, tail[-1] if tail else ''))

    need_model = any(w == 'stages' or w in REPORTS for w in want)
    if not need_model:
        return summary(results)

    # ---- the model
    import emu_checks as E
    import v3_model as M3
    import v4_model as M
    import v4_checks3d as K
    import v3_checks3d as K3
    E.install_report_patches(K, K3)
    ctx = E.make_ctx(M3.tof_positions)
    import v3_params4 as P
    for nm, x, y, aim in P.TOF:
        got = ctx.tof[nm]
        assert abs(got[0] - x) < 1e-6 and abs(got[1] - y) < 1e-6 and got[2] == aim, 'ToF %s differs between v3_model and v3_params4' % nm

    print('\nself-test of the probe API in the emulator:')
    st = K.selftest(ctx)
    print(st)
    results.append(('probe API self-test (v4_checks3d.selftest)', 'RESULT: PASS' in st, ''))
    t_all = time.time()
    for name, fn in M.STAGES:
        t0 = time.time()
        fn(ctx)
        if 'stages' in want:
            ok, lines = K.check_stage(ctx, name)
            bad = [ln for ln in lines if ln.strip().startswith('FAIL')]
            n_ok = sum(1 for ln in lines if ln.strip().startswith('ok'))
            print('stage %-12s %s  (%d ok, %d FAIL, %.1f s)' % (name, 'PASS' if ok else 'FAIL', n_ok, len(bad), time.time() - t0))
            for ln in bad:
                print(ln)
            results.append(('stage ' + name, ok, '%d probes and overlaps ok' % n_ok))
    if F.WARN:
        print('\nemulator warnings (a cut that removes nothing or a join that touches nothing makes Fusion raise):')
        for cat, msg in F.WARN:
            print('  %-30s %s' % (cat, msg))
    results.append(('no cut that removes nothing, no join that touches nothing', not F.WARN, '%d warnings' % len(F.WARN)))
    print('model built in %.0f s: %d components' % (time.time() - t_all, len(ctx.root._occ)))

    # ---- whole-model reports
    for kind in REPORTS:
        if kind not in want:
            continue
        t0 = time.time()
        E.VERBOSE = (kind == 'clearances')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ret = getattr(K, 'report_' + kind)(ctx)
        text = buf.getvalue() + (ret if isinstance(ret, str) else '')
        print('\n' + '=' * 12, 'report', kind, '(%.0f s)' % (time.time() - t0))
        print(text)
        lines = text.split('\n')
        res = [ln for ln in lines if ln.startswith('RESULT:')]
        fail_lines = [ln for ln in lines if ln.strip().startswith('FAIL')]
        blocked = [ln for ln in lines if re.search(r'blocked\s+([1-9]\d*) / 17', ln) or re.search(r': ([1-9]\d*) / 25 view rays', ln)]
        omni_bad = [ln for ln in lines if ' x ' in ln and 'cm3 at travel' in ln]
        passed = not (fail_lines or blocked or omni_bad) and not any('FAIL' in r for r in res)
        results.append(('report ' + kind, passed, (res[0] if res else '') or ('no blocked ray, no interference' if passed else '')))
    return summary(results)


def summary(results):
    print('\n' + '=' * 12, 'summary')
    bad = 0
    for name, ok, note in results:
        print('  %s %-58s %s' % ('ok  ' if ok else 'FAIL', name, note))
        bad += 0 if ok else 1
    print('RESULT:', 'PASS' if not bad else 'FAIL (%d)' % bad)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
