"""Scratch: parametric hopper + channel head for design experiments (convex pieces for MuJoCo), built on chute_geometry's CSG helpers."""
import math

import numpy as np

import chute_geometry as G
import v3_params4 as P
from chute_geometry import Hs, box, rect_prism, difference, cut, clip, solid

DEFAULT = dict(W=1.3, H=1.3, wall=0.16, back=0.0, trough='z77', slot_ang=45.0, void=1.45, hopper_out=1.95, flange=2.35, keep_hopper_floor=True,
               lead_in=0.0, ceiling_lead=0.0)


def pieces(**kw):
    q = dict(DEFAULT)
    q.update(kw)
    Ch, F, pl = P.CHUTE, P.FRAME, P.PLATE
    p0, d, ex, ey, ez, e = G.channel_frame(1)
    sx, sy = p0[0], p0[1]
    ang = q['slot_ang']
    out = []
    # dropper floor patch with the slot (14.5 square turned like the slot)
    patch = box((sx, sy, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), -1.9, 1.9, -1.9, 1.9, F['z0'], F['z0'] + P.DROPPER_FLOOR['t'])
    rim = [Hs((math.cos(a), math.sin(a), 0.0), P.DROPPER_FLOOR['r'] + pl['cx'] * math.cos(a) + pl['cy'] * math.sin(a)) for a in (2 * math.pi * k / 48 for k in range(48))]
    for v, pl_ in solid(clip(difference(patch, rect_prism(sx, sy, pl['slot'], pl['slot'], ang, F['z0'] - 0.2, F['z0'] + 0.5)), rim)):
        out.append(('floor', v))
    wo = q['W'] / 2 + q['wall']          # outer half width
    ho = q['H'] / 2 + q['wall']          # outer half height
    wi, hi = q['W'] / 2, q['H'] / 2
    # hopper: shell around the void, minus the socket the channel head sits in
    hop = [rect_prism(sx, sy, q['hopper_out'], q['hopper_out'], ang, Ch['hopper_z0'], F['z0']),
           rect_prism(sx, sy, q['flange'], q['flange'], ang, F['z0'] - Ch['flange_t'], F['z0'])]
    void = rect_prism(sx, sy, q['void'], q['void'], ang, Ch['hopper_z0'] + 0.16, F['z0'] + 0.1)
    socket = box(p0, ex, ey, ez, -wo, wo, -ho, ho, -0.5 - q['back'], 3.0)
    pcs = []
    for h in hop:
        pcs += difference(h, void)
    pcs = cut(pcs, socket)
    for v, pl_ in solid(pcs):
        out.append(('hopper', v))
    # channel head: tube, minus the bore, minus the trough
    far = np.linalg.norm(e - p0) + 0.8
    tube = box(p0, ex, ey, ez, -wo, wo, -ho, ho, -q['back'], far)
    bore = box(p0, ex, ey, ez, -wi, wi, -hi, hi, -0.2 - q['back'], far + 0.2)
    pcs = difference(tube, bore)
    foot = rect_prism(sx, sy, q['void'], q['void'], ang, 5.0, F['z0'] + 0.3)
    if q['trough'] == 'z77':
        trough = rect_prism(sx, sy, q['void'], q['void'], ang, 7.7, F['z0'] + 0.3)
        pcs = cut(pcs, trough)
    elif q['trough'] == 'floor':
        above = Hs(-ey, hi - ey @ p0)                                        # above the bore floor surface
        pcs = cut(pcs, foot + [above])
    r = np.array([e[0], e[1], 0.0])
    r = r / np.linalg.norm(r)
    pcs = clip(pcs, Hs(r, r @ np.array([e[0], e[1], 0.0])))
    for v, pl_ in solid(pcs):
        out.append(('channel', v))
    return out
