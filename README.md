# PathPlanningSim

A grid sim for testing how several drones search a field together. Four drones sweep a 20 x 40 grid that has obstacles they can't see until they are one cell away.

```
python3 -m venv .venv && .venv/bin/python3 -m pip install -r requirements.txt
.venv/bin/python3 main.py   # visualizer
.venv/bin/python3 Sim.py    # headless run, prints ticks and coverage
```

In the visualizer, Space or the right arrow steps one tick and the left arrow steps back. Hold either to repeat. The buttons along the bottom play, pause, reset and randomize the obstacles, and the slider sets how much of the field is obstacles (0 to 20%). Esc quits.

## How the path planning works

The field is cut into one vertical column per drone. Each drone sweeps its column with a boustrophedon ("bous") path: across a row, up one cell, back across the next row. It follows that path until a cell ahead turns out to be an obstacle. Then it drops the blocked cell and runs A*, a shortest-path search, to the next free bous cell, routing around every obstacle any drone has seen so far. A detour often cuts through a neighbour's column, and the two drones settle each shared cell by who would reach it first.

- Obstacle cells are removed from the bous path, and A* joins the last free cell to the next free one. Several blocked cells in a row are skipped in one go.
- An obstacle seen by one drone is known to all four on the same tick.
- If the neighbour reaches a shared cell first, the detouring drone just flies through it.
- If both would reach it on the same tick, the neighbour flies as normal and the detouring drone waits one tick, then follows.
- If the neighbour would reach it later, the cell is reassigned to the detouring drone and comes off the neighbour's path.
- A drone has to fly over every cell it took, even when a later re-plan would bypass one.
- A cell that A* can't reach is skipped for good.
- A drone never enters a cell that another drone is staying in or entering, and two drones never swap places.
- A drone on its bous path goes before a detouring one. Ties go to the lower drone index.
- The drone that gives way waits one tick, then runs A* around the other drone. If the blocked cell was its target, it skips that cell.
- Drones start side by side at the bottom centre. The walk to their own column never reassigns cells.
- A drone that finishes waits in its last cell until the others are done.
