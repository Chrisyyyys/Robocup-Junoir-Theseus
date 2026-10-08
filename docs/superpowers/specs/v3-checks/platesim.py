"""Dropper plate: polygon simulation of every possible drop sequence.
Plate frame (degrees, CCW positive, mm): slot A at 0, pockets k=0..7 at phi1+30k, slot B at 270.  Plate rotation theta moves pocket k to phi_k+theta.
Old firmware semantics (dispenser.cpp): dispenseLeft(N) turns CW to bring the next N pockets of the A end over slot A one after another,
dispenseRight(N) turns CCW for the B end; each drop returns to home (theta=0).  Pockets are used nearest-first from their own end.
"""
import math, itertools, sys
from shapely.geometry import Polygon
from shapely import affinity

def sq(cx, cy, ang_deg, rad_len, tan_len):
    a = math.radians(ang_deg); ur = (math.cos(a), math.sin(a)); ut = (-math.sin(a), math.cos(a))
    pts = [(cx + sr*rad_len/2*ur[0] + st*tan_len/2*ut[0], cy + sr*rad_len/2*ur[1] + st*tan_len/2*ut[1]) for sr, st in ((-1,-1),(1,-1),(1,1),(-1,1))]
    return Polygon(pts)

class Plate:
    def __init__(self, r=36.2, pocket=13.0, slot=13.5, pitch=30.0, phi1=30.0, n=8, slotA=0.0, slotB=270.0, kit=10.3):
        self.r, self.pocket, self.slot, self.pitch, self.phi1, self.n, self.kit = r, pocket, slot, pitch, phi1, n, kit
        self.phi = [phi1 + pitch*k for k in range(n)]
        self.slotA, self.slotB = slotA, slotB
        self.SA = self.slot_poly(slotA); self.SB = self.slot_poly(slotB)
    def slot_poly(self, ang):
        a = math.radians(ang); return sq(self.r*math.cos(a), self.r*math.sin(a), ang, self.slot, self.slot)
    def pocket_poly(self, k, theta):
        ang = self.phi[k] + theta; a = math.radians(ang)
        return sq(self.r*math.cos(a), self.r*math.sin(a), ang, self.pocket, self.pocket)
    def kit_poly(self, k, theta):
        """the kit sits somewhere inside its pocket: the worst case is pushed towards the slot side; use the pocket shrunk to the kit size, centred"""
        ang = self.phi[k] + theta; a = math.radians(ang)
        return sq(self.r*math.cos(a), self.r*math.sin(a), ang, self.kit, self.kit)

def exposure(pl, k, theta, slot_poly):
    """fraction of the kit footprint (centred) that is over the slot opening"""
    kp = pl.kit_poly(k, theta)
    return kp.intersection(slot_poly).area / kp.area

def simulate(pl, steps_per_deg=4, expose_limit=0.0):
    """returns (ok, messages).  Checks all sequences of L/R drops of 1 or 2 kits (<= 8 kits in total)."""
    msgs = []
    # parked closedness: no pocket polygon touches a slot, margin in mm
    park = min(min(pl.pocket_poly(k, 0).distance(pl.SA), pl.pocket_poly(k, 0).distance(pl.SB)) for k in range(pl.n))
    # inter-pocket walls
    wall = min(pl.pocket_poly(i, 0).distance(pl.pocket_poly(i+1, 0)) for i in range(pl.n-1))
    ok = True
    bad = 0; checked = 0
    # sequences: each element (side, N)
    def seqs(left):
        if left == 0: yield (); return
        for side in 'LR':
            for N in (1, 2):
                if N <= left:
                    for rest in seqs(left - N): yield ((side, N),) + rest
        yield ()
    seen = set()
    for seq in seqs(pl.n):
        if seq in seen: continue
        seen.add(seq)
        full = [True]*pl.n; usedA = 0; usedB = 0
        for side, N in seq:
            if side == 'L':
                kidx = [usedA + j for j in range(N)]      # pockets from the A end, nearest first
                target_theta = -(pl.phi[kidx[-1]] - pl.slotA)
                direction = -1
                for kk in kidx: pass
                usedA += N
            else:
                kidx = [pl.n - 1 - (usedB + j) for j in range(N)]
                target_theta = (pl.slotB - pl.phi[kidx[-1]])
                direction = +1
                usedB += N
            # sweep from 0 to target and back to 0
            path = []
            tmax = abs(target_theta); nsteps = int(tmax*steps_per_deg)+1
            for s in range(nsteps+1): path.append(direction*tmax*s/nsteps)
            drops_expected = set(kidx)
            dropped_here = set()
            for th in path + path[::-1]:
                for k in range(pl.n):
                    if not full[k]: continue
                    ea = exposure(pl, k, th, pl.SA); eb = exposure(pl, k, th, pl.SB)
                    right_slot_exp, wrong_slot_exp = (ea, eb) if side == 'L' else (eb, ea)
                    if wrong_slot_exp > expose_limit:
                        bad += 1
                        if len(msgs) < 5: msgs.append(f'seq {seq}: full pocket {k} over the WRONG slot at theta={th:.1f} (exposure {wrong_slot_exp:.2f})')
                    if right_slot_exp > expose_limit and k not in drops_expected:
                        bad += 1
                        if len(msgs) < 5: msgs.append(f'seq {seq}: full pocket {k} over the right slot but not meant to drop (theta={th:.1f}, exposure {right_slot_exp:.2f})')
                    if right_slot_exp > 0.5 and k in drops_expected: dropped_here.add(k)
            # the target pockets must have reached the slot with real overlap at the end of the outward sweep
            for k in kidx:
                e = exposure(pl, k, target_theta, pl.SA if side == 'L' else pl.SB)
                if e < 0.9:
                    bad += 1
                    if len(msgs) < 5: msgs.append(f'seq {seq}: target pocket {k} only {e:.2f} over the slot at the end of the sweep')
            for k in kidx: full[k] = False
            # parked again: no full pocket over a slot
            for k in range(pl.n):
                if full[k] and (exposure(pl, k, 0, pl.SA) > 0 or exposure(pl, k, 0, pl.SB) > 0):
                    bad += 1
                    if len(msgs) < 5: msgs.append(f'seq {seq}: parked with full pocket {k} over a slot')
            checked += 1
    return dict(sequences=len(seen), drops_checked=checked, violations=bad, park_margin_mm=round(park, 2), wall_mm=round(wall, 2), msgs=msgs)

if __name__ == '__main__':
    print('previous spec idea (slots 180 deg apart, pockets every 45 deg, half-pitch offset) for comparison:')
    old = Plate(r=33.0, pocket=14.0, slot=14.0, pitch=45.0, phi1=22.5, n=8, slotA=0.0, slotB=180.0)
    # slots 180 apart: both slots see pockets at once; show the exposure at parked/aligned positions
    print('  pockets', [round(p, 1) for p in old.phi])
    for th in (0, -22.5, -45.0, 22.5, 45.0):
        eA = [round(exposure(old, k, th, old.SA), 2) for k in range(8)]; eB = [round(exposure(old, k, th, old.SB), 2) for k in range(8)]
        print(f'  theta={th:>6}: exposure over slot A {max(eA)}, over slot B {max(eB)}')
    print()
    for label, kw in (('V3 plate: pocket 13.0, slot 13.5, r 36.2, pitch 30', dict()),
                      ('tighter: pocket 13.0, slot 13.5, r 34.0', dict(r=34.0)),
                      ('pocket 14.0, slot 14.5, r 38.6', dict(r=38.6, pocket=14.0, slot=14.5)),
                      ('pocket 12.5, slot 13.0, r 35.0', dict(r=35.0, pocket=12.5, slot=13.0))):
        pl = Plate(**kw)
        res = simulate(pl)
        print(label); print('  ', {k: v for k, v in res.items() if k != 'msgs'})
        for m in res['msgs']: print('     ', m)
