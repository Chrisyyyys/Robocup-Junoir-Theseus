"""Does the convex-piece model of the chute (chute_geometry.py) match the Fusion model? Three steps, because Fusion's Python has no numpy:

  1. python chute_geometry_check.py make  POINTS.json            random points in the chute region and points 0.3 mm either side of the faces of the pieces
  2. in Fusion:  import chute_samples; chute_samples.evaluate(r"POINTS.json", r"SAMPLES.json")        (v3-fusion/chute_samples.py: pointContainment on 'Hopper B left', 'Chute B left', 'Dropper floor')
  3. python chute_geometry_check.py compare POINTS.json SAMPLES.json     counts the points where the two disagree; points within 0.2 mm of a surface of the model are not counted

PASS needs no disagreement away from the surfaces and no more than 0.5 % of the points either side of a face on the wrong side (the exit trim is a plane here and a cylinder in Fusion, 0.3 mm
apart at the tube's edge). The result is in results/chute_geometry.txt.
"""
import json
import random
import sys

import numpy as np

import chute_geometry as G

CLASSES = ('hopper', 'channel', 'floor')
REGION = ((-7.2, -2.6), (0.8, 9.2), (6.6, 9.3))           # x, y, z of the region sampled (cm)
NEAR = 0.02                                                  # cm: a point this close to a surface of the model is ignored
STEP = 0.03                                                  # cm: distance of the face points from the face


def predict(geo, p):
    return [G.contains(geo[k], p) for k in CLASSES]


def near_surface(geo, p):
    """True when moving the point by NEAR along an axis changes any prediction: it is within about 0.2 mm of a surface."""
    base = predict(geo, p)
    for ax in range(3):
        for s in (-NEAR, NEAR):
            q = np.array(p, float)
            q[ax] += s
            if predict(geo, q) != base:
                return True
    return False


def make(path, n_uniform=7000, n_faces=4000, seed=1):
    geo = G.build(1)
    rnd = random.Random(seed)
    pts = []
    for _ in range(n_uniform):
        pts.append([rnd.uniform(*REGION[0]), rnd.uniform(*REGION[1]), rnd.uniform(*REGION[2])])
    faces = []
    for k in CLASSES:
        for v, planes in geo[k]:
            for h in G._dedupe(planes):
                on = v[np.abs(v @ h.n - h.b) < 1e-6]
                if len(on) >= 3:
                    faces.append((on, h))
    for _ in range(n_faces):
        on, h = rnd.choice(faces)
        w = np.array([rnd.random() for _ in range(len(on))])
        c = (on * (w / w.sum())[:, None]).sum(axis=0)          # a random point of the face (convex combination of its corners)
        for s in (-STEP, STEP):
            pts.append([float(x) for x in c + h.n * s])
    json.dump({'uniform': n_uniform, 'step': STEP, 'points': pts}, open(path, 'w'))
    print('wrote %d points to %s' % (len(pts), path))


def compare(points_path, samples_path, out_path=None):
    geo = G.build(1)
    P_ = json.load(open(points_path))
    S_ = json.load(open(samples_path))
    pts, res = P_['points'], S_['inside']
    n_uni = P_['uniform']
    lines = ['chute model against the Fusion bodies: %d random points in x %s, y %s, z %s and %d points %.1f mm either side of the faces of the pieces' % (
        n_uni, REGION[0], REGION[1], REGION[2], len(pts) - n_uni, STEP * 10)]
    bad_uni, near_uni, n_in, bad_faces = [], 0, 0, []
    patch = lambda p: abs(p[0] - (-4.729)) <= 1.9 and abs(p[1] - 2.729) <= 1.9          # the floor model is a 3.8 x 3.8 cm patch round the slot
    for i in range(n_uni):
        p, r = pts[i], res[i]
        want = predict(geo, p)
        got = [bool(r[0]), bool(r[1]), bool(r[2])]
        if not patch(p):
            want[2] = got[2] = False
        n_in += any(got)
        if want != got:
            if near_surface(geo, p):
                near_uni += 1
            else:
                bad_uni.append((p, want, got))
    lines.append('  random points: %d inside some body, %d disagree with the model, %d more disagree within 0.2 mm of a surface (ignored)' % (n_in, len(bad_uni), near_uni))
    for p, want, got in bad_uni[:8]:
        lines.append('    at %s model %s Fusion %s (hopper, channel, floor)' % ([round(x, 3) for x in p], want, got))
    n_face = len(pts) - n_uni
    wrong = 0
    for i in range(n_uni, len(pts)):
        p, r = pts[i], res[i]
        want = predict(geo, p)
        got = [bool(r[0]), bool(r[1]), bool(r[2])]
        if not patch(p):
            want[2] = got[2] = False
        if any(want) != any(got):
            wrong += 1
            bad_faces.append((p, want, got))
    lines.append('  face points: %d, %d on the wrong side (%.2f %%)' % (n_face, wrong, 100.0 * wrong / max(1, n_face)))
    for p, want, got in bad_faces[:8]:
        lines.append('    at %s model %s Fusion %s' % ([round(x, 3) for x in p], want, got))
    ok = not bad_uni and wrong <= 0.005 * n_face
    lines.append('RESULT: ' + ('PASS' if ok else 'FAIL'))
    text = '\n'.join(lines)
    print(text)
    if out_path:
        open(out_path, 'w').write(text + '\n')
    return ok


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'make':
        make(sys.argv[2])
    elif len(sys.argv) >= 4 and sys.argv[1] == 'compare':
        compare(sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv) > 4 else None)
    else:
        print(__doc__)
