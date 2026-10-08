"""Two extra checks of the finished rev 4 model with Fusion's own analysis (design.analyzeInterference), made after the plan's reports. Not part of the rebuild plan's code:
the dry-run emulator has no analyzeInterference. Each takes about 5 s for the 51 components.

report()        every BODY against every other body, so it also sees overlaps inside one component, which v4_checks3d.report_interference (component against component,
                overlaps below 1e-3 cm3 ignored) does not. PASS when every overlap is one of the expected ones in EXPECTED, under its limit.
report_loose()  with coincident faces counted: which components touch or overlap no other component. A part that rests on its support touches it; a part that touches nothing floats
                in the model (it sits in a pocket or hole with clearance, or its holder is a detail the model leaves out). PASS when every such part is one of the known ones in LOOSE.

Inside a Fusion script (see README.md, Rebuild):
    import sys; sys.path.insert(0, r"<repo>\\docs\\superpowers\\specs\\v3-fusion"); import native_interference
    def run(context): print(native_interference.report()); print(native_interference.report_loose())
"""
import adsk.core
import adsk.fusion

GHOST = 'Stepper bay 28BYJ-48'          # the hidden keep-out for the 28BYJ-48 is left out, as in the other reports

# (name prefix of one body, name prefix of the other, largest overlap accepted in cm3, why it is deliberate)
EXPECTED = [
    ('Tub shell', 'Cradle prong', 0.13, "the prong's root is sunk 0.5 mm into the floor and its outer end sits inside the cradle ledge (a separate body only because it flexes; one printed part with the tub)"),
    ('Lid plate', 'Hook bump', 0.011, "the bump's outer face is sunk 0.1 to 0.2 mm into the skirt's inner face (a separate body only because it flexes; one printed part with the lid)"),
    ('Bumper plate', 'Microswitch', 0.0004, 'the bumper plate rests on its microswitch (touching, as in the rev 3 model)'),
]

# (component name prefix, why it touches nothing in the model)
LOOSE = [
    ('ToF ', 'the board sits in its ring pocket with clearance; the snap tab is not modelled'),
    ('Camera ', 'the board floats under its lid hump; its cradle with the snap tabs is not modelled'),
    ('Floor port FP', 'in its floor hole with clearance; the snap pocket is not modelled'),
    ('Silver module SM', 'in its floor hole with clearance; the snap pocket is not modelled'),
    ('Rear nub', 'the nub boss and its M5 thread are not modelled, so the nub stops short of the tub'),
    ('USB-C service socket', 'in its wall hole with clearance; the fixing is not modelled'),
    ('Wi-Fi antenna', 'stuck on the wall with adhesive; modelled 0.05 to 0.1 mm off the wall face on purpose'),
]


def _why(a, b, vol):
    for pa, pb, limit, why in EXPECTED:
        if (a.startswith(pa) and b.startswith(pb)) or (a.startswith(pb) and b.startswith(pa)):
            return (vol <= limit), why, limit
    return False, 'not expected', 0.0


def _analyse(coincident):
    """(results, number of occurrences, number of bodies, occurrence names) of the analysis over every occurrence except the hidden stepper bay."""
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    ents = adsk.core.ObjectCollection.create()
    n_bodies, names = 0, []
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name != GHOST:
            ents.add(o)
            names.append(o.component.name)
            n_bodies += o.component.bRepBodies.count
    inp = design.createInterferenceInput(ents)
    inp.areCoincidentFacesIncluded = coincident
    return design.analyzeInterference(inp), ents.count, n_bodies, names


def report():
    """One line per overlapping pair of bodies, the expected ones marked with their reason; RESULT: PASS when every overlap is expected and within its limit."""
    res, n_occ, n_bodies, _ = _analyse(False)
    rows = []
    for i in range(res.count):
        r = res.item(i)
        rows.append((r.interferenceBody.volume, r.entityOne.name, r.entityTwo.name))
    lines = ['Fusion interference analysis, %d occurrences (%d bodies), coincident faces not counted: %d overlapping body pairs' % (n_occ, n_bodies, len(rows))]
    ok = True
    for vol, a, b in sorted(rows, reverse=True):
        good, why, limit = _why(a, b, vol)
        ok = ok and good
        lines.append('  %s %9.5f cm3  %s  x  %s   %s' % ('ok  ' if good else 'FAIL', vol, a, b, ('(at most %g) ' % limit + why) if good else why))
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)


def _nearest(name):
    """Smallest distance in mm from the named component to any other (the hidden stepper bay left out), and the body it is measured to."""
    app = adsk.core.Application.get()
    root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
    occ = {root.occurrences.item(i).component.name: root.occurrences.item(i) for i in range(root.occurrences.count)}
    best = (1e9, '?')
    for j in range(occ[name].bRepBodies.count):
        mine = occ[name].bRepBodies.item(j)
        for other, o in occ.items():
            if other in (name, GHOST):
                continue
            for k in range(o.bRepBodies.count):
                d = app.measureManager.measureMinimumDistance(mine, o.bRepBodies.item(k)).value * 10.0
                if d < best[0]:
                    best = (d, '%s / %s' % (other, o.bRepBodies.item(k).name))
    return best


def report_loose():
    """The components that touch or overlap no other component, with the distance to the nearest part and the reason it is known; RESULT: PASS when no unknown part floats."""
    res, n_occ, _, names = _analyse(True)
    touching = set()
    for i in range(res.count):
        r = res.item(i)
        a, b = r.entityOne.parentComponent.name, r.entityTwo.parentComponent.name
        if a != b:
            touching.update((a, b))
    loose = sorted(n for n in names if n not in touching)
    lines = ['Fusion analysis with coincident faces counted, %d occurrences: %d touch or overlap no other component' % (n_occ, len(loose))]
    ok = True
    for n in loose:
        why = next((w for p, w in LOOSE if n.startswith(p)), None)
        ok = ok and why is not None
        d, where = _nearest(n)
        lines.append('  %s %-22s nearest %5.2f mm (%s): %s' % ('known' if why else 'FAIL ', n, d, where, why or 'floats: nothing touches it'))
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    return '\n'.join(lines)
