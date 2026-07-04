# LCD Navigation Status + Tile Re-Sense Confidence Check

Date: 2026-07-04

## Problem

The robot has a 16x2 LCD (`LiquidCrystal lcd`, already initialized in `Main.ino`) that is
currently only used for transient one-off popups via `lcdPrint()` (e.g. `"victim: H"`,
`"starting bfs"`), which show for 1 second and then clear to blank. There is no persistent
display of where the robot currently thinks it is (coordinates, facing direction, tile
color/type).

Separately, the robot's position (`x_pos`, `y_pos`) is tracked purely by dead reckoning
(encoder counts advancing `x_pos`/`y_pos` by one tile per completed `fwd()` call). There is
no check that a re-visited tile's sensed walls actually match what the map already recorded
for that cell, so drift (wheel slip, a botched turn that wasn't fully corrected, etc.) can
silently desync the robot's belief about its position from reality, and the existing
`writeWallsToCurrentTile()` call would blindly overwrite previously-trusted wall data with a
possibly-wrong new reading, compounding the confusion on every future visit.

## Goals

1. Show persistent navigation status on the LCD: current X/Y coordinate, facing direction,
   current tile's color/type, and whether the robot's position is currently trusted.
2. When arriving at a tile that has been visited before, re-sense its walls and compare them
   against the stored map data for that cell. If they disagree enough to suggest the robot
   isn't where it thinks it is, flag it (visible on the LCD) and avoid overwriting the
   already-trusted map data for that tile with the new, possibly-drift-corrupted reading.

## Non-goals

- Actively correcting/relocalizing `x_pos`/`y_pos` (e.g. snapping to a best-matching
  neighboring cell). Explicitly deferred — guessing a "corrected" position risks making
  things worse than just flagging and continuing with dead reckoning.
- Re-checking tile color/type as part of the mismatch determination. Only wall pattern is
  compared; color display on the LCD is informational only (rendered from the tile's
  already-stored `TileTypes`, which is set during `fwd()`'s approach into the tile).
- Any change to `MazeTile.h`'s bitset layout. The mismatch flag is a per-cycle global, not
  per-tile stored state.

## Design

### 1. LCD persistent status display

New function `updateStatusDisplay()`, added to `uart_camera_comms.ino` next to `lcdPrint()`.
Locks `lcdMutex`, then writes:

- Line 1: `X%02d Y%02d %c` built from `x_pos`, `y_pos`, and a facing-direction character
  looked up from `currentDir` (`{'N','E','S','W'}` indexed by the `Direction` enum). Example:
  `X20 Y20 N`.
- Line 2: `COL:%s %s` built from a new helper `colorReadingStr(int)` (returns `"WHT"`,
  `"BLU"`, `"RED"`, `"BLK"`, or `"UNK"` for `read_color()`'s 0/1/2/-1/other return values)
  applied to a **fresh** `read_color()` call, plus `tilecheck ? "LOST" : "OK"`. Example:
  `COL:WHT OK` or `COL:BLK LOST`.

  **Revised during implementation:** the design originally planned to source the color
  field from `mapGrid[x_pos][y_pos].getType()` (`TileTypes`). A teammate's commit
  (`ebc9af4`, landed mid-session) reworked color handling so `setType(BLUE)` is no longer
  called anywhere and `setType(BLACK)` is only ever applied to a tile the robot never
  actually enters (it backs off instead) — so `getType()` would show `BLANK`/"WHT" for
  almost every tile the robot is actually standing on. A fresh `read_color()` call is used
  instead. The same commit also removed the checkpoint/silver-marking side effect from
  `read_color()`, so calling it again here (in addition to the call `EXECUTE_MOVE` already
  makes right after arriving at a tile) is side-effect-free.

Both lines are cleared (overwritten with spaces) before the new text is written, following
the existing pattern in `lcdPrint()`.

`updateStatusDisplay()` is called once per tile, from the top of the `SENSE_TILE` case in
`Main.ino`'s `loop()`, immediately after the mismatch check (below) sets `tilecheck`.

**Interaction with existing popups:** `lcdPrint()` is changed so that after it unlocks
`lcdMutex` (not before — locking `lcdMutex` again inside `updateStatusDisplay()` while
`lcdPrint()` still holds it would deadlock), it calls `updateStatusDisplay()`. This means
popups like `"victim: H"` still show for their existing 1-second window, and the persistent
status line reappears automatically right after, instead of leaving the screen blank until
the next tile.

### 2. Tile re-sense confidence check

New `#define WALL_MISMATCH_THRESHOLD 2` in `Main.ino`, alongside the other movement/sensing
tunables (`MIN_DIST`, `BLACK_THRESHOLD`, etc.).

New function `checkTileMismatch(bool wallF, bool wallR, bool wallB, bool wallL)` in
`navigation.ino`:

```
bool checkTileMismatch(bool wallF, bool wallR, bool wallB, bool wallL){
  Tile &t = mapGrid[x_pos][y_pos];
  if(!t.getVisited()) return false; // no trustworthy prior data for this tile yet

  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);

  int mismatches = 0;
  if(t.getWall(absF) != wallF) mismatches++;
  if(t.getWall(absR) != wallR) mismatches++;
  if(t.getWall(absB) != wallB) mismatches++;
  if(t.getWall(absL) != wallL) mismatches++;

  return mismatches >= WALL_MISMATCH_THRESHOLD;
}
```

The `getVisited()` gate (rather than `getDiscovered()`) is important: the home tile is
marked `discovered` in `setup()` before any real walls have ever been sensed there, so
gating on `getDiscovered()` would false-positive a "LOST" flag on the very first tile at
power-on. `getVisited()` is only set true by `updateFullyExploredAt()`, after a tile's walls
have actually been written at least once, so the first visit to any tile is correctly never
flagged (nothing to compare against yet).

The `>= 2` threshold tolerates a single noisy/misread distance sensor without falsely
flagging "lost," while still catching the case where the robot is clearly on a
different tile than it thinks (which will usually disagree on more than one side).

**Wiring into the state machine (`Main.ino`):**

- `SENSE_TILE` case: right after the existing `readWallsRel(wallF, wallR, wallB, wallL);`
  call, add `tilecheck = checkTileMismatch(wallF, wallR, wallB, wallL);` followed by
  `updateStatusDisplay();`.
- `UPDATE_MAP` case: change the unconditional `writeWallsToCurrentTile(wallF, wallR, wallB,
  wallL);` to only run `if(!tilecheck)`. When `tilecheck` is true, that call is skipped
  (log a `Serial.println` noting the mismatch) so the previously-trusted wall data for that
  cell is preserved instead of being clobbered by a reading taken while the robot's position
  belief may be wrong. `updateFullyExploredAt(x_pos, y_pos)` still runs unconditionally in
  both branches, since it just consumes whichever wall data is currently authoritative for
  that cell.

No changes to `pickNextDirection()`, `BFS()`, or any other map consumer — they keep reading
`mapGrid` exactly as before; the only change is which writes are allowed to land when a
mismatch is detected.

## Testing / verification approach

This is Arduino firmware with no unit test harness and dependencies on physical sensors
(distance sensors, gyro, color sensor), so verification is manual/bench-based rather than
automated:

- Compile-check the sketch (Arduino CLI or IDE) after each change to catch syntax/type
  errors before it ever reaches hardware.
- Bench test 1 (status display): power on, confirm line 1/2 show sane values at the home
  tile (`X20 Y20 N`, `COL:WHT OK` for a 40x40 grid with `MAP_SIZE/2` = 20), drive it through
  a few tiles manually or via the maze and confirm the coordinates/direction/color update
  once per tile and popups (e.g. forcing a victim detection) restore the status display
  afterward instead of leaving the screen blank.
- Bench test 2 (mismatch detection): after the robot has mapped a few tiles, physically
  place it on a previously-visited tile that does NOT match (e.g. manually pick it up and
  set it down one tile off, or block/unblock a wall on a tile it already visited) and
  confirm: (a) the LCD shows `LOST` on the next `SENSE_TILE`, (b) `Serial.println` logs the
  mismatch and confirms the write was skipped, (c) the tile's map data (checked via serial
  log or later BFS behavior) is unchanged from before the mismatched visit.
- Bench test 3 (no false positives): confirm a normal, correct revisit of an already-mapped
  tile (e.g. during backtracking after a pause/resume) does NOT flag `LOST`.
