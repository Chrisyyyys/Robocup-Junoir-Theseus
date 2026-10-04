"""Random RoboCupJunior Rescue Maze style fields for the simulator.

Follows the 2026 field rules (30 cm tiles, walls on tile edges, black holes, blue tiles, silver checkpoints, red tile and
dangerous zone, victims on walls, linear and floating tiles, ramps of at most 25 degrees joining two areas, speed bumps,
stairs, debris and obstacles that leave at least 20 cm of free path).

Profiles:
  walls  - walls only
  basic  - walls, black / blue / silver tiles and victims (no ramps or obstacles)
  full   - everything: also speed bumps, stairs, debris, obstacles, a ramp to a second area and a dangerous zone

Tile codes in the maze file: . floor, S start, X black, B blue, C silver (checkpoint), R red (entrance of the dangerous zone),
# void, ^ > v < ramp, T stairs, b speed bump, p speed bump inside the dangerous zone (up to 2 cm), d debris.
"""

import random

N, E, S, W = 0, 1, 2, 3
DX = {N: 0, E: 1, S: 0, W: -1}
DY = {N: 1, E: 0, S: -1, W: 0}
SIDE_CH = "NESW"


class Maze:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.hwall = [[True] * w for _ in range(h + 1)]   # hwall[j][x]: wall on line y=j
        self.vwall = [[True] * (w + 1) for _ in range(h)]  # vwall[y][i]: wall on line x=i
        self.code = [["." for _ in range(w)] for _ in range(h)]  # code[y][x], y=0 south
        self.victims = []   # (x, y, side, type)
        self.obstacles = []
        self.start = (0, 0)
        self.start_dir = N
        self.ramp_angle = 20
        self.name = ""

    def inside(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def wall(self, x, y, d):
        if d == N:
            return self.hwall[y + 1][x]
        if d == S:
            return self.hwall[y][x]
        if d == E:
            return self.vwall[y][x + 1]
        return self.vwall[y][x]

    def set_wall(self, x, y, d, v):
        if d == N:
            self.hwall[y + 1][x] = v
        elif d == S:
            self.hwall[y][x] = v
        elif d == E:
            self.vwall[y][x + 1] = v
        else:
            self.vwall[y][x] = v

    def neighbors(self, x, y, ignore_walls=False):
        for d in (N, E, S, W):
            nx, ny = x + DX[d], y + DY[d]
            if self.inside(nx, ny) and (ignore_walls or not self.wall(x, y, d)):
                yield d, nx, ny

    def reachable(self, start=None, avoid=()):
        start = start or self.start
        seen = {start}
        stack = [start]
        while stack:
            x, y = stack.pop()
            for d, nx, ny in self.neighbors(x, y):
                if (nx, ny) in seen or (nx, ny) in avoid or self.code[ny][nx] in "#X":
                    continue
                seen.add((nx, ny))
                stack.append((nx, ny))
        return seen

    def to_text(self, header=""):
        lines = []
        if header:
            lines += ["# " + l for l in header.splitlines()]
        if self.name:
            lines.append("name " + self.name)
        inline = {}
        extra = []
        for (x, y, side, t) in self.victims:
            if side in (W, E) and (x, y, side) not in inline:
                inline[(x, y, side)] = t
            else:
                extra.append((x, y, side, t))
        for r in range(self.h + 1):
            j = self.h - r
            lines.append("+" + "".join(("---" if self.hwall[j][x] else "   ") + "+" for x in range(self.w)))
            if r == self.h:
                break
            y = self.h - 1 - r
            row = ""
            for x in range(self.w):
                row += "|" if self.vwall[y][x] else " "
                row += inline.get((x, y, W), " ") + self.code[y][x] + inline.get((x, y, E), " ")
            row += "|" if self.vwall[y][self.w] else " "
            lines.append(row)
        for (x, y, side, t) in extra:
            lines.append("victim %d %d %s %s" % (x, self.h - 1 - y, SIDE_CH[side], t))
        for (x, y, dx, dy, r) in self.obstacles:
            lines.append("obstacle %d %d %d %d %d" % (x, self.h - 1 - y, dx, dy, r))
        lines.append("start_dir " + SIDE_CH[self.start_dir])
        lines.append("ramp_angle %d" % self.ramp_angle)
        return "\n".join(lines) + "\n"


def carve_area(m, rng, x0, y0, w, h, loop_frac, rooms):
    """Spanning tree maze in the rectangle, then extra openings for loops and open areas."""
    cells = [(x, y) for x in range(x0, x0 + w) for y in range(y0, y0 + h)]
    start = rng.choice(cells)
    seen = {start}
    stack = [start]
    while stack:
        x, y = stack[-1]
        opts = [(d, x + DX[d], y + DY[d]) for d in (N, E, S, W)
                if x0 <= x + DX[d] < x0 + w and y0 <= y + DY[d] < y0 + h and (x + DX[d], y + DY[d]) not in seen]
        if not opts:
            stack.pop()
            continue
        d, nx, ny = rng.choice(opts)
        m.set_wall(x, y, d, False)
        seen.add((nx, ny))
        stack.append((nx, ny))
    inner = []
    for x in range(x0, x0 + w):
        for y in range(y0, y0 + h):
            if x + 1 < x0 + w and m.wall(x, y, E):
                inner.append((x, y, E))
            if y + 1 < y0 + h and m.wall(x, y, N):
                inner.append((x, y, N))
    rng.shuffle(inner)
    for (x, y, d) in inner[: int(len(inner) * loop_frac)]:
        m.set_wall(x, y, d, False)
    for _ in range(rooms):  # open 2x2 areas, like the open rooms in real fields
        rx = rng.randrange(x0, x0 + w - 1)
        ry = rng.randrange(y0, y0 + h - 1)
        m.set_wall(rx, ry, E, False)
        m.set_wall(rx, ry, N, False)
        m.set_wall(rx + 1, ry, N, False)
        m.set_wall(rx, ry + 1, E, False)


def pick_tiles(m, rng, count, ok, avoid):
    cand = [(x, y) for x in range(m.w) for y in range(m.h) if (x, y) not in avoid and ok(x, y)]
    rng.shuffle(cand)
    return cand[:count]


def place_floor(m, rng, profile, area_cells, used):
    """Black, blue and silver tiles inside one area."""
    n = len(area_cells)
    sx, sy = m.start
    near_start = {(sx + DX[d], sy + DY[d]) for d in (N, E, S, W)} | {m.start}
    blacks = 0
    want_black = max(1, n // 14)
    cells = list(area_cells)
    rng.shuffle(cells)
    for (x, y) in cells:
        if blacks >= want_black:
            break
        if (x, y) in used or (x, y) in near_start or m.code[y][x] != ".":
            continue
        m.code[y][x] = "X"
        everyone = {c for c in area_cells if m.code[c[1]][c[0]] not in "X#"}
        if not everyone <= m.reachable():
            m.code[y][x] = "."  # would cut off part of the maze
            continue
        used.add((x, y))
        blacks += 1
    for code, count in (("C", max(1, n // 16)), ("B", max(1, n // 20))):
        for (x, y) in cells:
            if count <= 0:
                break
            if (x, y) in used or (x, y) == m.start or m.code[y][x] != ".":
                continue
            m.code[y][x] = code
            used.add((x, y))
            count -= 1


def linear_tiles(m):
    """Tiles a left-hand or right-hand wall follower passes when it starts on the start tile (rules 3.3: 'linear tiles'; the
    others are 'floating tiles'). Black tiles and void count as walls."""
    def blocked(x, y, d):
        if m.wall(x, y, d):
            return True
        nx, ny = x + DX[d], y + DY[d]
        return not m.inside(nx, ny) or m.code[ny][nx] in "X#"
    lin = {m.start}
    for hand in (-1, 1):
        for d0 in (N, E, S, W):
            x, y = m.start
            d = d0
            seen = set()
            for _ in range(m.w * m.h * 8):
                if (x, y, d) in seen:
                    break
                seen.add((x, y, d))
                nd = None
                for cand in ((d + hand) % 4, d, (d - hand) % 4, (d + 2) % 4):
                    if not blocked(x, y, cand):
                        nd = cand
                        break
                if nd is None:
                    break
                d = nd
                x, y = x + DX[d], y + DY[d]
                lin.add((x, y))
    return lin


def floating_walls(m):
    """Set of wall segments (x, y, side) not connected to the outer wall."""
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def unite(a, b):
        parent[find(a)] = find(b)

    segs = []
    for j in range(m.h + 1):
        for x in range(m.w):
            if m.hwall[j][x]:
                unite((x, j), (x + 1, j))
                segs.append(((x, j), (x + 1, j)))
    for y in range(m.h):
        for i in range(m.w + 1):
            if m.vwall[y][i]:
                unite((i, y), (i, y + 1))
                segs.append(((i, y), (i, y + 1)))
    anchored = set()
    for i in range(m.w + 1):
        for j in range(m.h + 1):
            border = i in (0, m.w) or j in (0, m.h)
            near_void = any(m.inside(i + dx, j + dy) and m.code[j + dy][i + dx] == "#"
                            for dx in (-1, 0) for dy in (-1, 0))
            if border or near_void:
                anchored.add(find((i, j)))
    return lambda a: find(a) not in anchored


def place_victims(m, rng, profile, cells, count):
    lin = linear_tiles(m)
    obstacle_tiles = {(o[0], o[1]) for o in m.obstacles}
    faces = []
    for (x, y) in cells:
        # rules 3.6.3: never on walls facing black, silver, blue or red tiles, tiles with obstacles, speed bumps or stairs, or ramps
        if m.code[y][x] in "XCBR#<>^vbpT" or (x, y) in obstacle_tiles:
            continue
        for d in (N, E, S, W):
            if m.wall(x, y, d):
                faces.append((x, y, d, (x, y) not in lin))
    rng.shuffle(faces)
    faces.sort(key=lambda f: not f[3])  # put a few victims on floating walls first
    floating_quota = max(1, count // 4)
    chosen, used_tiles = [], set()
    for f in faces:
        if len(chosen) >= count:
            break
        if (f[0], f[1]) in used_tiles:
            continue
        if f[3] and floating_quota <= 0:
            continue
        if f[3]:
            floating_quota -= 1
        chosen.append(f)
        used_tiles.add((f[0], f[1]))
    letters = "HSU"
    colours = "RYG"   # cognitive targets (harmed, stable, unharmed)
    for (x, y, d, fl) in chosen:
        pool = letters if (profile == "basic" and rng.random() < 0.6) or rng.random() < 0.55 else colours
        m.victims.append((x, y, d, rng.choice(pool)))


def place_dangerous_zone(m, rng, area_cells, used):
    """Rules 3.5: a pocket completely surrounded by walls, entered through one red tile, that the rest of the field does not
    depend on. Here a dead-end corridor of 2-3 tiles: the tile next to the junction is the red entrance, behind it come speed
    bumps up to 2 cm (code p), debris (d) or stairs (T)."""
    if rng.random() < 0.3:
        return
    cellset = set(area_cells)

    def open_nb(x, y):
        return [(nx, ny) for d, nx, ny in m.neighbors(x, y) if m.code[ny][nx] != "#"]
    leaves = [c for c in area_cells if c not in used and m.code[c[1]][c[0]] == "." and len(open_nb(*c)) == 1]
    rng.shuffle(leaves)
    for leaf in leaves:
        chain = [leaf]
        prev, cur = None, leaf
        while len(chain) < 3:
            nbs = [c for c in open_nb(*cur) if c != prev]
            if not nbs:
                break
            nxt = nbs[0]
            if nxt in used or nxt not in cellset or m.code[nxt[1]][nxt[0]] != "." or len(open_nb(*nxt)) != 2:
                break
            chain.append(nxt)
            prev, cur = cur, nxt
        # the entrance is the last tile of the chain (the one next to the junction); keep one tile of the pocket behind it
        if len(chain) < 2:
            continue
        entrance, inside = chain[-1], chain[:-1]
        m.code[entrance[1]][entrance[0]] = "R"
        used.add(entrance)
        for (x, y) in inside:
            used.add((x, y))
        straight = lambda x, y: (m.wall(x, y, E) and m.wall(x, y, W)) or (m.wall(x, y, N) and m.wall(x, y, S))
        for (x, y) in inside:
            kind = rng.choice(["p", "p", "d", "T", "."])
            if kind in "pT" and not straight(x, y):
                kind = "d"
            m.code[y][x] = kind
        return


def generate(seed, profile="basic", width=None, height=None):
    rng = random.Random(seed)
    if profile not in ("walls", "basic", "full"):
        raise ValueError("profile must be walls, basic or full")
    if profile == "full":
        w1 = width or rng.randint(5, 7)
        h1 = height or rng.randint(4, 6)
        w2, h2 = rng.randint(3, 4), rng.randint(3, h1)
        ramp_len = rng.randint(1, 2)
        gw, gh = w1 + ramp_len + w2, h1
        m = Maze(gw, gh)
        for y in range(gh):
            for x in range(gw):
                m.code[y][x] = "#"
        for y in range(h1):
            for x in range(w1):
                m.code[y][x] = "."
        oy = rng.randint(0, h1 - h2)
        for y in range(oy, oy + h2):
            for x in range(w1 + ramp_len, gw):
                m.code[y][x] = "."
        carve_area(m, rng, 0, 0, w1, h1, 0.25, 1)
        carve_area(m, rng, w1 + ramp_len, oy, w2, h2, 0.2, 0)
        ry = rng.randint(oy, oy + h2 - 1)
        for k in range(ramp_len):
            m.code[ry][w1 + k] = ">"
            m.set_wall(w1 + k, ry, W, False)
        m.set_wall(w1 + ramp_len - 1, ry, E, False)
        m.ramp_angle = rng.choice([15, 18, 20, 22, 25])
        area1 = [(x, y) for x in range(w1) for y in range(h1)]
        area2 = [(x, y) for x in range(w1 + ramp_len, gw) for y in range(oy, oy + h2)]
        ramp_cells = {(w1 + k, ry) for k in range(ramp_len)}
    else:
        gw = width or rng.randint(5, 8)
        gh = height or rng.randint(4, 7)
        m = Maze(gw, gh)
        carve_area(m, rng, 0, 0, gw, gh, 0.22 if profile != "walls" else 0.15, rng.randint(0, 2))
        area1 = [(x, y) for x in range(gw) for y in range(gh)]
        area2 = []
        ramp_cells = set()
    # walls around void tiles; none between two void tiles
    for y in range(m.h):
        for x in range(m.w):
            for d in (N, E, S, W):
                nx, ny = x + DX[d], y + DY[d]
                if not m.inside(nx, ny):
                    m.set_wall(x, y, d, True)
                    continue
                a, b = m.code[y][x] == "#", m.code[ny][nx] == "#"
                if (x, y) in ramp_cells and d in (N, S):
                    m.set_wall(x, y, d, True)
                    continue
                if a and b:
                    m.set_wall(x, y, d, False)
                elif a != b:
                    m.set_wall(x, y, d, True)
    # start on the outer edge of the first area
    ax = max(c[0] for c in area1)
    ay = max(c[1] for c in area1)
    edge = [(x, y) for (x, y) in area1 if x in (0, ax) or y in (0, ay)]
    m.start = rng.choice(edge)
    sx, sy = m.start
    m.code[sy][sx] = "S"
    opens = [d for d in (N, E, S, W) if not m.wall(sx, sy, d)]
    m.start_dir = rng.choice(opens) if opens else N
    used = {m.start}
    if profile != "walls":
        place_floor(m, rng, profile, area1, used)
        if area2:
            place_floor(m, rng, profile, area2, used)
    if profile == "full":
        place_dangerous_zone(m, rng, area1 + area2, used)
        flat = lambda x, y: m.code[y][x] == "."
        for (x, y) in pick_tiles(m, rng, rng.randint(1, 2), flat, used):
            m.code[y][x] = "b"
            used.add((x, y))
        for (x, y) in pick_tiles(m, rng, rng.randint(1, 2), flat, used):
            m.code[y][x] = "d"
            used.add((x, y))
        corridor = lambda x, y: flat(x, y) and ((m.wall(x, y, E) and m.wall(x, y, W) and not m.wall(x, y, N) and not m.wall(x, y, S))
                                                or (m.wall(x, y, N) and m.wall(x, y, S) and not m.wall(x, y, E) and not m.wall(x, y, W)))
        for (x, y) in pick_tiles(m, rng, 1, corridor, used):
            m.code[y][x] = "T"
            used.add((x, y))
        open_tile = lambda x, y: flat(x, y) and sum(m.wall(x, y, d) for d in (N, E, S, W)) <= 1
        for (x, y) in pick_tiles(m, rng, rng.randint(1, 2), open_tile, used):
            side = rng.choice([d for d in (N, E, S, W) if m.wall(x, y, d)] or [N])
            # rules 3.4.4: touching a wall, so that at least 20 cm of the tile stays free; 100 = 140 (half the 28 cm path) - 40 (radius)
            m.obstacles.append((x, y, DX[side] * 100, DY[side] * 100, 40))
            used.add((x, y))
    if profile != "walls":
        cells = area1 + area2
        n_victims = max(2, len(cells) // 6)
        place_victims(m, rng, profile, cells, n_victims)
    m.name = "%s maze, seed %d" % (profile, seed)
    return m


if __name__ == "__main__":
    import sys
    prof = sys.argv[1] if len(sys.argv) > 1 else "basic"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    print(generate(seed, prof).to_text())
