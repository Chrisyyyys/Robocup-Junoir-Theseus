"""Pull the code out of the rev 4 plan (docs/superpowers/plans/2026-10-07-theseus-v3-rev4-fusion-model.md) so it can be dry-run before, or after, it exists in the repo.
Every python block under a `Create` or `Append to` step is written to the module it names; the plan's two textual patches (axis_prism for fusion_lib, the omni_sweep header of
v3_checks3d) are applied to copies. Nothing in the repo is changed."""
import ast
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
FUSION_DIR = os.path.dirname(HERE)
SPECS = os.path.dirname(FUSION_DIR)
PLAN = os.path.join(os.path.dirname(SPECS), 'plans', '2026-10-07-theseus-v3-rev4-fusion-model.md')


def assemble(out_dir, plan_path=PLAN):
    """Write the plan's modules into out_dir. Returns (names of the files written, list of problems); a problem is a syntax error or a patch anchor that was not found."""
    os.makedirs(out_dir, exist_ok=True)
    lines = open(plan_path, encoding='utf-8').read().split('\n')
    target, mode = None, None
    files, problems = {}, []
    i = 0
    while i < len(lines):
        ln = lines[i]
        if re.match(r'\s*- \[[ x]\] \*\*Step', ln) or ln.startswith('### '):
            target, mode = None, None
        m = re.search(r'\b(Create|Append to)\b[^`\n]*`([^`]+\.py)`', ln)
        if m and not ln.startswith('    ') and not ln.startswith('|'):
            target, mode = os.path.basename(m.group(2)), ('w' if m.group(1) == 'Create' else 'a')
        elif 'Replace the existing `axis_tool`' in ln:
            target, mode = 'fusion_lib_patch.py', 'w'
        if ln.startswith('```python'):
            j = i + 1
            while j < len(lines) and not lines[j].startswith('```'):
                j += 1
            code = '\n'.join(lines[i + 1:j])
            try:
                ast.parse(code)
            except SyntaxError as e:
                problems.append('plan line %d (%s): syntax error %s at block line %s' % (i + 1, target, e.msg, e.lineno))
            if target:
                files[target] = (code + '\n') if (mode == 'w' or target not in files) else (files[target] + '\n\n' + code + '\n')
                mode = 'a'
            i = j
        i += 1
    for name, code in files.items():
        open(os.path.join(out_dir, name), 'w', encoding='utf-8').write(code)

    # the plan's patch of v3_checks3d.omni_sweep (Task 2, Step 3), applied to a copy
    src = open(os.path.join(FUSION_DIR, 'v3_checks3d.py'), encoding='utf-8').read()
    plan = '\n'.join(lines)
    blk = re.search(r"```python\n(def omni_sweep\(ctx, steps=13, omni=None, omni_at=None, travel=None\):.*?)\n```", plan, re.S)
    if 'omni=None' in src:                                    # already patched in the repo
        out = src
    elif not blk:
        problems.append('the plan block for omni_sweep was not found')
        out = src
    else:
        tail = '    a_rest = math.atan2(oz - pz, ox - px)'
        try:
            a = src.index('def omni_sweep(ctx, steps=13):')
            b = src.index(tail, a) + len(tail)
            out = src[:a] + blk.group(1) + src[b:]
            for old, new in (('trav = M.OMNI_TRAVEL_MECH * k / (steps - 1)', 'trav = travel * k / (steps - 1)'), ('xc, zc = M.omni_at(trav)', 'xc, zc = omni_at(trav)'),
                             ('% (steps, M.OMNI_TRAVEL_MECH))', '% (steps, travel))')):
                if out.count(old) != 1:
                    problems.append('omni_sweep patch anchor not found exactly once: ' + old)
                out = out.replace(old, new)
            compile(out, 'v3_checks3d.py', 'exec')
        except (ValueError, SyntaxError) as e:
            problems.append('omni_sweep patch failed: %s' % e)
            out = src
    open(os.path.join(out_dir, 'v3_checks3d.py'), 'w', encoding='utf-8').write(out)
    files['v3_checks3d.py'] = out
    return sorted(files), problems


if __name__ == '__main__':
    import sys
    import tempfile
    d = sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp(prefix='rev4_plan_')
    names, problems = assemble(d)
    print('wrote', ', '.join(names), 'to', d)
    for p in problems:
        print('PROBLEM', p)
    sys.exit(1 if problems else 0)
