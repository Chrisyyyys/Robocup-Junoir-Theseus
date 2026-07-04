# Right-Wall Follower — `center()` Rework

**Date:** 2026-07-04
**Scope:** Change what `center()` returns as the PID error, converting it from opposite-sensor
two-wall centering to a single-side (right wall) follower based on Hanafi et al., *Wall Follower
Autonomous Robot Development Applying Fuzzy Incremental Controller* (ICA 2013).

## Goal

`center()` returns an integer error that `movement.ino` feeds into `center_PID`
(`movement.ino:150-151`). Rework the returned value so the robot follows the **right-hand wall**
at a fixed clearance, using the two right-side sensors (front + back) to recover both distance and
yaw, per the paper's combined error `E_Tot = E_d + E_theta`.

## Geometry

- Tile width: `TILE_MM = 300`
- Robot width: `ROBOT_WIDTH_MM = 140`
- Ideal per-side clearance when tracking a wall: `TARGET_SIDE_GAP_MM = (300 - 140) / 2 = 80` (already defined, `Main.ino:34`)

## Sensor layout (confirmed by user)

- Sensors **2 (front)** and **3 (back)** — **right** side of the robot.
- Sensors **6 (front)** and **5 (back)** — **left** side.
- `measure(n)` returns mm, or the sentinels `-1` / `8191` for no valid reading.

This rework uses the **right** pair: `front = measure(2)`, `back = measure(3)`.

## Formula (paper mapping)

- `D = (front + back) / 2` — average distance to the right wall ("average of two")
- angle term `= back - front` — proportional to yaw relative to the wall ("one minus the other")
- `E = (TARGET_SIDE_GAP_MM - D) + (back - front)`

Sign convention (preserves the old code's direction): **positive error steers the robot away from
the right wall.**
- Too close (`D < 80`) → `TARGET - D > 0` → steer away. ✓
- Nosed in toward the wall (`front < back`) → `back - front > 0` → steer away. ✓

## Wall-presence gate

A side reading is a valid right-wall reference only when it is a real, in-tile reading. Max legit
in-tile side gap is `TILE - ROBOT = 160`; a sensor seeing through a gap into the next tile reads
`>= 380`. Gate at a threshold between the two:

- New define: `#define SIDE_WALL_MAX_MM 200` in `Main.ino` (next to the other geometry defines).
- Both `front` and `back` must be valid (`!= -1`, `!= 8191`) **and** `<= SIDE_WALL_MAX_MM`.
- If either fails, the right wall is not reliably present → `center()` returns `0` (no correction),
  which is how `movement.ino` already interprets "no reference."

This replaces the old `% 300` normalization trick.

## Implementation

`Main/Distance.ino:372-377` becomes:

```cpp
// Right-wall follower error, fed to center_PID in movement.ino.
// Uses the two right-side sensors (front = 2, back = 3) per Hanafi et al. (2013):
//   E_Tot = (ideal - D) + angle,  where D = avg gap, angle = back - front.
// Positive error steers away from the right wall (matches the old sign convention).
// Returns 0 when the right wall isn't present on BOTH sensors (no reliable reference).
int center(){
  int front = measure(2);   // right-front gap (mm)
  int back  = measure(3);   // right-back gap  (mm)
  bool wallPresent = front != -1 && front != 8191 && front <= SIDE_WALL_MAX_MM
                  && back  != -1 && back  != 8191 && back  <= SIDE_WALL_MAX_MM;
  if(!wallPresent) return 0;
  double D = (front + back) / 2.0;                       // distance term
  double e = (TARGET_SIDE_GAP_MM - D) + (back - front);  // (ideal - D) + angle
  return (int)e;
}
```

`Main.ino` (~line 34, with the other geometry defines):

```cpp
#define SIDE_WALL_MAX_MM 200  // mm; a side reading beyond this is the next tile through a gap, not this tile's wall
```

`movement.ino` — **unchanged**. Same `int` signature; it keeps calling `center()` and feeding the
result to `center_PID`.

## Behavior change (intentional)

- The robot no longer centers between two walls; it holds `TARGET_SIDE_GAP_MM` from the **right**
  wall only. When there is no right wall, `center()` returns 0 and the PID makes no lateral
  correction (heading hold / other logic governs).

## Verification / open risk

- **Angle-term sign** (`back - front`) is derived, not bench-tested. If the robot oscillates or
  diverges when a right wall is present, flip it to `front - back`. This is the one value to confirm
  on hardware.
- Bench check: place the robot parallel to a right wall closer than 80 mm → expect a positive error;
  farther than 80 mm → negative error; angled nose-in → positive error.
