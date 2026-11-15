# PathPlanningSim

Grid sim for multi-drone search path planning, with a pygame visualizer.

```
python3 -m venv .venv && .venv/bin/python3 -m pip install -r requirements.txt
.venv/bin/python3 main.py   # visualizer: Space ticks (hold to repeat), buttons play / reset / randomize
.venv/bin/python3 Sim.py    # headless run
```

## How the path planning works

The field is split into one vertical column per drone, and each drone sweeps its column with a boustrophedon ("bous") path: across a row, up one cell, back across the next row. Drones cannot see obstacles until they are within one cell of them. A drone follows its bous path until a cell ahead turns out to be an obstacle, then drops that cell and uses A* (a shortest-path search) to reach the next free bous cell, routing around every obstacle any drone has seen so far. When that detour passes through another drone's column, the two drones settle each shared cell by who would get there first.

- **Blocked cell:** obstacle cells are removed from the bous path, and A* connects the last free cell to the next free one (several blocked cells in a row are skipped together).
- **Shared knowledge:** obstacles seen by one drone are known to all of them straight away.
- **Detour through a neighbour's cell, neighbour arrives first:** the cell is already covered, so the detouring drone just flies through.
- **Both arrive on the same tick:** the neighbour flies normally; the detouring drone waits one tick, then follows it through.
- **Neighbour would arrive later:** the cell is reassigned to the detouring drone and removed from the neighbour's path.
- **Reassigned cells are kept:** a drone must still fly over every cell it took, even if a later re-plan would bypass it.
- **Unreachable cell:** if A* finds no route to a cell, it is skipped for good and the drone moves on to the next one.
- **Collisions:** a drone never enters a cell another drone is staying in or entering, and two drones never swap places.
- **Right of way:** a drone on its bous path goes before a detouring one; ties go to the lower drone index.
- **Stuck behind a drone:** the drone giving way waits one tick, then runs A* around the other drone, or skips the cell if that cell was its target.
- **Start:** drones spawn side by side at the bottom centre; the walk to their own column never reassigns cells.
- **Finish:** a drone that completes its path waits in its last cell until the others are done.
