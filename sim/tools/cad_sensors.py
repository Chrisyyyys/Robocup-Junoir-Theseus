#!/usr/bin/env python3
"""Reads the robot's geometry out of a Fusion 360 archive (.f3z) so that sim/config/robot.cfg can follow the CAD.

    pip install zstandard
    python sim/tools/cad_sensors.py path/to/V2.f3z

What it prints (all in mm, in the frame of each part of the CAD, which is NOT shared between parts):
  - the largest flat plates (chassis length and width),
  - cylinders of 40 mm radius (the wheels: centre, width),
  - every group of four 2.7 mm holes in a 12.7 x 20.3 mm rectangle: the mounting holes of an Adafruit VL53L0X breakout. For each
    one: where its centre is, which way the board faces, and the heights found in that part.
The part frames are not tied together in the archive, so heights above the floor and which end is the front have to be checked
by hand (the V2 numbers in robot.cfg were worked out that way, see sim/FINDINGS.md, "The V2 robot").

How it works: a .f3z is a zip of .f3d files, each a zip of Fusion's binary streams plus ASM binary (SAB) body files, all compressed with
zstd. This reads the SAB body files only (vertices, planes and cylinders), which is enough for plates, wheels and screw holes.
"""
import io
import math
import re
import struct
import sys
import zipfile

try:
    import zstandard
except ImportError:
    sys.exit("pip install zstandard")


def read_entry(zf, name):
    info = zf.getinfo(name)
    if info.compress_type != 93:  # 93 = zstd
        return zf.read(name)
    fp = zf.fp
    fp.seek(info.header_offset)
    sig, ver, flag, method, mtime, mdate, crc, csize, usize, nlen, elen = struct.unpack("<IHHHHHIIIHH", fp.read(30))
    fp.seek(info.header_offset + 30 + nlen + elen)
    return zstandard.ZstdDecompressor().decompress(fp.read(info.compress_size), max_output_size=max(info.file_size, 1))


def tokens(b):
    i = b.find(b"\x0d\x09asmheader")
    n = len(b)
    while i < n:
        t = b[i]
        i += 1
        if t in (0x03, 0x04, 0x15, 0x0C):
            yield t, struct.unpack_from("<i", b, i)[0]
            i += 4
        elif t == 0x05:
            yield t, struct.unpack_from("<f", b, i)[0]
            i += 4
        elif t == 0x06:
            yield t, struct.unpack_from("<d", b, i)[0]
            i += 8
        elif t in (0x07, 0x0D, 0x0E):
            ln = b[i]
            i += 1
            yield t, b[i:i + ln].decode("latin1")
            i += ln
        elif t == 0x08:
            ln = struct.unpack_from("<H", b, i)[0]
            i += 2
            yield t, b[i:i + ln].decode("latin1")
            i += ln
        elif t == 0x09:
            ln = struct.unpack_from("<I", b, i)[0]
            i += 4
            yield t, b[i:i + ln].decode("latin1")
            i += ln
        elif t in (0x0A, 0x0B, 0x0F, 0x10, 0x11):
            yield t, None
        elif t in (0x13, 0x14):
            yield t, struct.unpack_from("<3d", b, i)
            i += 24
        elif t == 0x01:
            yield t, b[i]
            i += 1
        elif t == 0x02:
            yield t, struct.unpack_from("<h", b, i)[0]
            i += 2
        else:
            raise ValueError("unknown token 0x%02x" % t)


def parse(b):
    recs, cur = [], []
    for tag, val in tokens(b):
        if tag == 0x11:
            recs.append(cur)
            cur = []
        else:
            cur.append((tag, val))
    out = []
    for r in recs:
        j, names = 0, []
        while j < len(r) and r[j][0] in (0x0D, 0x0E):
            names.append(r[j][1])
            j += 1
        out.append((names[-1] if names else "?", names, r[j:]))
    return out


def ptrs(fields):
    return [v for t, v in fields if t == 0x0C]


def face_points(recs):
    """For every face: (surface normal, origin, its vertex positions in mm)."""
    def rec(i):
        return recs[i] if 0 <= i < len(recs) else ("?", [], [])
    faces = []
    for cls, names, f in recs:
        if cls != "face":
            continue
        loop = surf = None
        for q in ptrs(f):
            c = rec(q)[0]
            if c == "loop" and loop is None:
                loop = q
            elif c == "surface" and surf is None:
                surf = q
        pts, seen_loops = [], set()
        while loop is not None and loop >= 0 and loop not in seen_loops and rec(loop)[0] == "loop":
            seen_loops.add(loop)
            lp = ptrs(rec(loop)[2])
            ce = next((q for q in lp if rec(q)[0] == "coedge"), None)
            nxt = next((q for q in lp if rec(q)[0] == "loop" and q != loop), None)
            seen = set()
            while ce is not None and ce >= 0 and ce not in seen and rec(ce)[0] == "coedge":
                seen.add(ce)
                cp = ptrs(rec(ce)[2])
                edge = next((q for q in cp if rec(q)[0] == "edge"), None)
                if edge is not None:
                    for v in ptrs(rec(edge)[2]):
                        if rec(v)[0] == "vertex":
                            for pp in ptrs(rec(v)[2]):
                                if rec(pp)[0] == "point":
                                    for t, val in rec(pp)[2]:
                                        if t == 0x13:
                                            pts.append(tuple(c * 10 for c in val))
                ce = next((q for q in cp if rec(q)[0] == "coedge"), None)
            loop = nxt
        normal = origin = None
        if surf is not None:
            vecs = [val for t, val in rec(surf)[2] if t in (0x13, 0x14)]
            if len(vecs) >= 2:
                origin, normal = tuple(c * 10 for c in vecs[0]), vecs[1]
        faces.append((normal, origin, pts))
    return faces


def analyse(path):
    outer = zipfile.ZipFile(path)
    names = {}
    try:
        import json
        dd = json.loads(outer.read("DesignDescription.json").decode("utf-8", "replace"))
        for o in dd["designDescription"]["designGraphs"][0]["designObjects"]:
            names[o["relativePath"]] = o["friendlyName"]
    except Exception:
        pass
    holes_all = []
    for rel in outer.namelist():
        if not rel.endswith(".f3d"):
            continue
        inner = zipfile.ZipFile(io.BytesIO(outer.read(rel)))
        for n in inner.namelist():
            if not n.endswith(".smb"):
                continue
            recs = parse(read_entry(inner, n))
            part = names.get(rel, rel[:8])
            # plates: biggest horizontal faces
            plates = []
            for normal, origin, pts in face_points(recs):
                if not pts or normal is None:
                    continue
                if abs(abs(normal[2]) - 1) < 1e-6:
                    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
                    plates.append(((max(xs) - min(xs)) * (max(ys) - min(ys)), part, round(min(xs), 1), round(max(xs), 1), round(min(ys), 1), round(max(ys), 1), round(pts[0][2], 1)))
            for pl in sorted(set(plates), reverse=True)[:2]:
                if pl[0] > 8000:
                    print("plate  %-18s x %7.1f .. %7.1f   y %7.1f .. %7.1f   z %6.1f   (%.0f x %.0f mm)" % (pl[1], pl[2], pl[3], pl[4], pl[5], pl[6], pl[3] - pl[2], pl[5] - pl[4]))
            # cylinders: wheels and holes
            seen = set()
            for cls, nm, f in recs:
                if "cone" in nm and "surface" in nm:
                    vecs = [v for t, v in f if t in (0x13, 0x14)]
                    if len(vecs) < 3:
                        continue
                    o = tuple(c * 10 for c in vecs[0])
                    ax = vecs[1]
                    r = math.dist((0, 0, 0), vecs[2]) * 10
                    a = max(range(3), key=lambda i: abs(ax[i]))
                    if abs(abs(ax[a]) - 1) > 1e-6:
                        continue
                    key = (a, tuple(round(c, 1) for c in o), round(r, 1))
                    if key in seen:
                        continue
                    seen.add(key)
                    if abs(r - 40) < 0.3:
                        print("wheel  %-18s axis %s  centre (%.1f, %.1f, %.1f)  radius %.1f" % (part, "xyz"[a], o[0], o[1], o[2], r))
                    if 1.2 <= r <= 1.5:
                        holes_all.append((part, a, o))
    # 4-hole patterns of an Adafruit VL53L0X breakout: 12.7 x 20.3 mm
    print()
    groups = {}
    for part, a, o in holes_all:
        groups.setdefault((part, a, round(abs(o[a]) if a == 0 else o[a], 0), math.copysign(1, o[a]) if a == 0 else 1), []).append(o)
    for (part, a, _, sign), lst in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][3])):
        perp = [i for i in range(3) if i != a]
        pts = {(round(o[perp[0]], 1), round(o[perp[1]], 1)): o for o in lst}
        done = set()
        for (u, v) in pts:
            for du, dv in ((12.7, 20.3), (20.3, 12.7)):
                if all(any(abs(u + i * du - q[0]) < 0.25 and abs(v + j * dv - q[1]) < 0.25 for q in pts) for i in (0, 1) for j in (0, 1)):
                    cu, cv = u + du / 2, v + dv / 2
                    key = (round(cu, 1), round(cv, 1))
                    if key in done:
                        continue
                    done.add(key)
                    print("VL53L0X mount in %-14s board faces along %s: centre %s = %6.1f, %s = %6.1f, hole axis at %s = %6.1f" % (
                        part, "xyz"[a], "xyz"[perp[0]], cu, "xyz"[perp[1]], cv, "xyz"[a], pts[(u, v)][a]))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    analyse(sys.argv[1])
