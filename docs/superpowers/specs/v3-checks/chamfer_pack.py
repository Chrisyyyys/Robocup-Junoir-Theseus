"""Cross-check of the packing (plan/z-band model) against the rear underside chamfer of the side-view model.
The chamfer rises from (x=-6.5, z=3.5) to (x=-10.5, z=5.3); inside the body a part must sit above the underside plus a floor thickness.
Library: chamfer_gap() is used by final_pack2.py and cam_recess.py. (The first packing audit that found mux A and mux B under the chamfer is not shipped.)"""
import json, math, random, sys
from final_check import *

R = 10.5
CH_X0, CH_Z0, CH_X1, CH_Z1 = -6.5, 3.5, -10.5, 5.3
SLOPE = (CH_Z1 - CH_Z0) / (CH_X0 - CH_X1)          # 0.45 per cm
FLOOR_T = 0.2                                        # printed floor / chamfer wall, cm (vertical)

def underside(x):
    return CH_Z0 if x >= CH_X0 else CH_Z0 + (CH_X0 - x) * SLOPE

def chamfer_gap(poly, z0):
    """vertical gap between the part bottom and the inner chamfer surface (negative = collides)"""
    xs = [p[0] for p in poly.exterior.coords]
    return z0 - (underside(min(xs)) + FLOOR_T)
