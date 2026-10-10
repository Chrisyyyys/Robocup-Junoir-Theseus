"""Entry points for the Fusion MCP script runner (see README.md). Every call reloads the repo modules, so edits take effect without restarting Fusion.
Inside a Fusion script:
    import sys; sys.path.insert(0, r"<repo>\\docs\\superpowers\\specs\\v3-fusion"); import v4_run
    def run(context): print(v4_run.stage(['tub'], reset=True))"""
import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CHECKS = os.path.join(os.path.dirname(HERE), 'v3-checks')
for _p in (HERE, CHECKS):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load():
    mods = {}
    for n in ('fusion_lib', 'v3_params', 'v3_params4', 'v3_model', 'v3_checks3d', 'v4_model', 'v4_checks3d'):
        mods[n] = importlib.reload(importlib.import_module(n))
    return mods['v4_model'], mods['v4_checks3d']


def stage(names, reset=False, build=True, check=True):
    """Build the named stages ('all' for every stage), then run each stage's probes and the local interference of its components. Returns a report string."""
    M, K = _load()
    ctx = M.build(names, reset=reset) if build else M.make_ctx()
    wanted = [n for n, _ in M.STAGES] if names in (None, 'all') else (list(names) if isinstance(names, (list, tuple)) else [names])
    out, ok = [], True
    if check:
        for nm in wanted:
            good, lines = K.check_stage(ctx, nm)
            ok = ok and good
            out += lines
    out.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(out)


def report(kind, **kw):
    """Run one whole-model report from v4_checks3d: interference, envelope, belly, removal, access, mass, clearances, tof, camera, omni, legs, stepper."""
    M, K = _load()
    return getattr(K, 'report_' + kind)(M.make_ctx(), **kw)


def selftest():
    M, K = _load()
    return K.selftest(M.make_ctx())


def show(hide=(), ghost=False):
    """Show every component except the named ones (and except the hidden 28BYJ-48 bay unless ghost=True); used before screenshots."""
    M, K = _load()
    hidden = tuple(hide) + (() if ghost else ('Stepper bay 28BYJ-48',))
    return sys.modules['v3_model'].show_only(M.make_ctx(), hide=hidden)
