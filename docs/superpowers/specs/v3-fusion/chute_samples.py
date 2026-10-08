"""Fusion side of v3-checks/chute_geometry_check.py: for every point of a JSON list, which of the hopper, the channel and the dropper floor contain it (pointContainment, strictly inside).
Inside a Fusion script:
    import sys; sys.path.insert(0, r"<repo>\\docs\\superpowers\\specs\\v3-fusion"); import chute_samples
    def run(context): print(chute_samples.evaluate(r"POINTS.json", r"SAMPLES.json"))
"""
import json

import adsk.core
import adsk.fusion

COMPONENTS = ('Hopper B left', 'Chute B left', 'Dropper floor')


def evaluate(points_path, out_path):
    root = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
    inside = adsk.fusion.PointContainment.PointInsidePointContainment
    bodies = []
    for name in COMPONENTS:
        found = []
        for i in range(root.occurrences.count):
            o = root.occurrences.item(i)
            if o.component.name == name:
                found = [o.bRepBodies.item(j) for j in range(o.bRepBodies.count)]
        bodies.append(found)
    pts = json.load(open(points_path))['points']
    res = []
    for p in pts:
        q = adsk.core.Point3D.create(p[0], p[1], p[2])
        res.append([any(b.pointContainment(q) == inside for b in group) for group in bodies])
    json.dump({'components': list(COMPONENTS), 'inside': res}, open(out_path, 'w'))
    return 'evaluated %d points, %d inside some body' % (len(pts), sum(1 for r in res if any(r)))
