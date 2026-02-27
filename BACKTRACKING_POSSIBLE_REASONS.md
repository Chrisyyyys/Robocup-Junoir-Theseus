# Possible reasons the robot chooses a traveled edge before an unexplored edge

## 1) Turn command mismatch (relative vs absolute)
`turnNeededDeg()` is used as if it computes a relative turn from `currentDir` to `plannedMoveDir`, but the active implementation returns `0/90/180/270` based only on the target direction enum value. Then `Main.ino` passes that value to `absoluteturn()`.

`absoluteturn()` appears to expect an **absolute gyro heading angle**, not a relative turn command. If map heading and gyro heading are not perfectly aligned, robot motion can diverge from map updates (`currentDir`, `markEdgeBothWays`, `stepForward`), causing the planner to think it explored one edge while physically traveling another.

## 2) Unknown walls are initialized as open
`initializeMap()` sets all walls in all tiles to `false` (open). Until a tile is sensed, planner logic may treat unknown space as traversable and make decisions from incomplete/optimistic data.

This can create apparent "preference" for previously traveled routes when map consistency drifts.

## 3) Fallback planner explicitly allows traveled edges
`pickNextDirection()` first tries open+untraveled, but then immediately falls back to any open direction with front/left/right/back priority.

If any untraveled edge is incorrectly marked traveled (or inferred blocked due to sensor noise), the fallback may repeatedly pick already-traveled directions.

## 4) Edge marking depends on assumed successful move and assumed heading
After movement, code updates map state with:
- `currentDir = plannedMoveDir;`
- `markEdgeBothWays(x_pos, y_pos, currentDir);`
- `stepForward(currentDir, x_pos, y_pos);`

If the robot did not actually rotate/move exactly as planned (wheel slip, turn timeout, gyro drift), edge/position bookkeeping becomes inconsistent and future choices can favor traveled edges unexpectedly.

## 5) BACKPEDAL state re-plans from same tile without explicit reverse action
When `blacktoggle == true`, the code enters `BACKPEDAL` and immediately re-runs planning, but does not explicitly execute a guaranteed reverse-to-previous-tile operation first. This can leave navigation in a state where direction priority, stale edge flags, or stale heading produce non-intuitive "go back" behavior.

## 6) Sensor interpretation likely inverted/noisy risk
`readWallsRel()` sets `wallX = (detectWall(...) == 0)`. If `detectWall()` semantics differ (e.g., nonzero means wall), then open/blocked status may be inverted intermittently, causing wrong edge prioritization.

## 7) No global frontier/path planner yet
Code comment says "figure out BFS later." Without BFS/frontier targeting, local greedy direction priority can oscillate or re-enter known corridors even when unexplored branches exist elsewhere.
