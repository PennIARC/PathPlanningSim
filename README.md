# PathPlanningSim

A grid sim for testing how several drones search a field together. Four drones sweep a 30 x 60 grid that has obstacles they can't see until they are one cell away.

```
python3 -m venv .venv && .venv/bin/python3 -m pip install -r requirements.txt
.venv/bin/python3 main.py         # visualizer
.venv/bin/python3 Sim.py          # headless run, prints ticks and coverage
.venv/bin/python3 edge_cases.py   # regenerates the screenshots below
```

In the visualizer, Space plays and pauses. The right arrow steps one tick and the left arrow steps back; hold either to repeat. The buttons along the bottom do the same, plus reset and randomize the obstacles. Esc quits. Three sliders control the obstacles:

- **obstacles** is the share of the field that is obstacles, 0 to 20%.
- **clustering** is how many times as likely a cell beside an existing obstacle is to become one. 1x is an even scatter, 3x is the default.
- **rounds** is how many rounds the obstacles are placed in. Clustering only applies from the second round on, so more rounds let clusters grow larger.

## How the path planning works

The field is cut into one vertical column per drone. Each drone sweeps its column with a boustrophedon ("bous") path: across a row, up one cell, across the next row. When a cell ahead turns out to be an obstacle, the drone drops it and runs A*, a shortest-path search, to the next cell it needs, routing around every obstacle any drone has seen so far. A detour often cuts through a neighbour's column, and the two drones settle each shared cell by who would reach it first.

- To pick where to go next, a drone looks at both ends of every unbroken run of unvisited cells in its column. Each end costs its A* steps plus 3 for every row it lies above the drone, and the cheapest wins. In open field that gives the usual sweep, started from whichever end of the row is nearer.
- Obstacle cells are removed from the sweep as they are seen. An obstacle seen by one drone is known to all four on the same tick.
- If the neighbour reaches a shared cell first, the detouring drone just flies through it.
- If both would reach it on the same tick, the neighbour flies as normal and the detouring drone waits one tick, then follows.
- If the neighbour would reach it later, the cell is reassigned to the detouring drone and comes off the neighbour's sweep.
- A drone has to fly over every cell it took, even when a later re-plan would bypass one. It visits them before going back to its own rows.
- A cell that A* can't reach is skipped for good.
- A drone never enters a cell that another drone is staying in or entering, and two drones never swap places.
- A drone making a single step along its sweep goes before a detouring one. Ties go to the lower drone index.
- Drones start side by side at the bottom centre. The walk to their own column never reassigns cells.
- A drone that finishes waits in its last cell until the others are done.

## Parameters

Every number the sim depends on, and where it came from. "Given" means it was a design decision, "tuned" means it was chosen by measurement, and "picked" means a reasonable value that has not been tuned.

### Field and drones

| Parameter | Value | What it does | Where the value comes from |
|---|---|---|---|
| `GRID_WIDTH` x `GRID_HEIGHT` (`Pathing.py`) | 30 x 60 | Size of the field in cells. | Given. |
| `NUM_DRONES` (`Drones.py`) | 4 | Number of drones, and so of columns. | Given. |
| Column width | 7 or 8 cells | Width of each drone's column. | Derived: `GRID_WIDTH / NUM_DRONES`, split as evenly as whole cells allow (8, 7, 7, 8). |
| `SIGHT_RANGE` (`Drones.py`) | 1 | How far a drone sees: its own cell plus the 8 around it. | Given. |
| `NEIGHBOURS` (`Pathing.py`) | 4 directions | Drones move one cell left, right, up or down per tick, never diagonally. | Given. |

### Planner (`Planner` in `Pathing.py`)

| Parameter | Value | What it does | Where the value comes from |
|---|---|---|---|
| `CLIMB_COST` | 3 | Extra cost, in steps, for each row a candidate cell lies above the drone. It makes a drone finish cells at or below its row before moving up, unless they are more than about 3 steps further away than the next row. | Tuned on the 30 x 60 field with 4 drones. Values 1 to 4 finished fastest and were within noise of each other; 6 and above were 3 to 5% slower at 8 to 20% obstacles. It is not scaled with column width, so re-measure if the grid or drone count changes much. |
| `REROUTE_AFTER` | 2 ticks | How long a drone that must give way stays blocked behind another drone before it looks for a way round. | Picked. It is the "wait one tick, then go round" rule: one tick waiting, then act on the second. |
| `GIVE_UP_AFTER` | 8 ticks | How long a drone stays blocked with no way round before it skips the cell it was heading for. Also how long the drone with right of way holds its line in a head-on meeting. | Picked, not tuned. Long enough for another drone to clear a short corridor. |
| `STALL_FACTOR` | 3 | A drone that goes `STALL_FACTOR x (width + height)` ticks without ticking off a cell drops the one it is heading for. | Picked as a safety net. On the 30 x 60 field the limit is 270 ticks, far longer than any real detour. It has never triggered in testing. |
| Prediction horizon | length of the detour | How far ahead a neighbour's movements are played forward when deciding who reaches a shared cell first. | Derived. Anything the neighbour would reach later than the detouring drone counts as "later" anyway. |
| Tie-breaks | left, lower index | Two equally cheap cells: the left one. Two drones with equal right of way: the lower index. | Picked, so that runs are repeatable. |

### Obstacle generation (`Sim.py`)

| Parameter | Value | What it does | Where the value comes from |
|---|---|---|---|
| `OBSTACLE_RATE` | 0.08 | Share of cells that are obstacles. Slider: 0 to 20% in 0.5% steps. | Picked as a default that gives frequent detours without walling areas off. |
| `OBSTACLE_ROUNDS` | 3 | Rounds the obstacles are placed in. Slider: 1 to 8. | Given. |
| `CLUSTER_WEIGHT` | 3.0 | From round 2 on, how many times as likely a cell beside an obstacle is to be picked. Slider: 1x to 10x in 0.5 steps. | Picked for "slightly more likely". At 10% obstacles it raises the share of obstacles touching another from 33% to about 53%. |

### Visualizer (`main.py`, `Drones.py`, `palette.py`)

| Parameter | Value | What it does | Where the value comes from |
|---|---|---|---|
| `STEPS_PER_SECOND` | 40 | Ticks per second while playing or holding a step key. | Given. |
| `HOLD_DELAY` | 0.2 s | How long an arrow key or the Step / Back button is held before it repeats. | Given. |
| `SCALE_DOWN_FACTOR` | 2 | The scene is drawn at half resolution and scaled up. | Picked, to match the look of the earlier GridTesting viewer. |
| Cell size on screen | 16 px on an 1800 x 1169 display | Size of one grid cell. | Derived: the largest whole size that fits three grids across and 60 rows down. |
| `TRAIL_SUPERSAMPLE` | 2 | Trails are drawn this many times larger, then shrunk, to smooth them. | Tuned for speed on the 30 x 60 field: at 3 a redraw took about 25 ms, at 2 about 17 ms. |
| `TRAIL_LANES` | 3 | Most trail lines drawn side by side between two cells; a fourth pass onwards overlaps these. | Given. |
| `TRAIL_SPACING`, `CORNER_RADIUS` | 0.22, 0.3 of a cell | Gap between side-by-side trail lines, and how rounded a trail's corners are. | Picked by eye. |
| `TRAIL_WIDTH`, `TRAIL_OPACITY` | 1.5 px, 0.55 | Thickness and strength of trails. | Picked by eye. |
| `ALLOCATION_OPACITY`, `VISIT_OUTLINE` | 0.6, 2 px | Allocation grid: strength of a drone's tint, and thickness of the outline on a visited cell. | Picked by eye. |
| `MUTE` | 0.4 | How far drone and obstacle colours are pulled towards grey. | Picked by eye. |

## Edge cases

Each picture is the drone view of a small map built to make one situation happen, after the run has finished. A ring marks where a drone started and a dot where it ended. Purple cells are obstacles. Where a drone flew between the same two cells more than once, the lines are drawn side by side. Drones are red, yellow, green and blue, from left to right.

To add a case, put its map in `CASES` in `edge_cases.py`, run the script, and describe it here. The maps are tuned to the planner as it stands, so after changing a planner rule or parameter, re-run the script and check each picture still shows what its entry says.

### A cell in the sweep is an obstacle

![blocked cell](docs/edge_cases/blocked_cell.png)

The drone is sweeping a two-wide column and the next cell up, on the left, turns out to be an obstacle.

It drops that cell and runs A* to the next one it needs, on the right of the same row. The shortest way there is back through the cell it just left, so the trail doubles up.

### A detour through a neighbour's column

![neighbour cells](docs/edge_cases/neighbour_cells.png)

An obstacle blocks yellow at the left edge of its column, and its detour crosses three cells of red's column. Red has already swept two of them and would not reach the third for a while.

Yellow flies straight through the two swept cells and nothing changes hands. The third is reassigned to yellow, which covers it on the way; red takes it off its own sweep and never visits it.

### Both drones due at a shared cell on the same tick

![same tick](docs/edge_cases/same_tick.png)

Green is crossing yellow's column to get round a cluster of obstacles, and would arrive at one of yellow's cells on the tick yellow is due there.

The cell stays yellow's and yellow flies as planned. Green waits one tick in the cell before it, then follows yellow through.

### A cell that cannot be reached

![unreachable cell](docs/edge_cases/unreachable_cell.png)

A free cell on the top row of yellow's column is shut in by three obstacles and the edge of the field.

A* finds no route to it, so yellow drops it for good and finishes without it. The cell is seen from next door but never flown over.

### Two drones meet head-on

![head on](docs/edge_cases/head_on.png)

An obstacle on the bottom row blocks blue's walk to its column, so it cuts through its neighbours' columns instead. Twice it comes face to face with another drone, each wanting the cell the other is in.

Drones never swap places. The planner spots each meeting before either drone moves, and the one without right of way, blue both times, re-plans around the other, so neither loses a tick.

### A gap one cell wide

![corridor](docs/edge_cases/corridor.png)

The top row of yellow's column can only be reached through a single cell at the edge of red's column. Yellow goes in as red is coming out.

There is no way round. The drone without right of way, yellow, backs out one cell, red comes through, and yellow goes in after it.

### Boxed in at the start

![boxed in start](docs/edge_cases/boxed_in_start.png)

Red's walk along the bottom row to its own column runs into a dead end, with obstacles ahead and above. The only way out is back through the cell yellow is about to move into.

Red has nowhere to step aside to, so yellow moves out of the way even though it has right of way. Red comes out and goes round, and yellow carries on.

### A finished drone parked in the way

![parked drone](docs/edge_cases/parked_drone.png)

Red finishes first and parks in its last cell, at the top of its column. Yellow's last cell is next door and can only be reached through the cell red is sitting in.

A finished drone normally stays put. One that blocks another drone steps aside to the nearest cell off that drone's route, here one cell to the left, and yellow finishes.

### The walk to the starting column

![start walk](docs/edge_cases/start_walk.png)

Drones start side by side in the middle of the bottom row, so the two outer ones cross a neighbour's column to reach their own.

That walk is transit only. The cells crossed stay with their owner, who still sweeps them, and the usual collision rules apply.

### Arriving at the far end of a row

![nearer end](docs/edge_cases/nearer_end.png)

A row of obstacles leaves one gap, at the left edge, so the drone comes up into the next row at its left end. A fixed left-right zigzag would have it start that row from the right.

The drone picks whichever end of a row is fewer A* steps away. It sweeps from the left and saves a crossing.

### A wall up the middle of a column

![wall](docs/edge_cases/wall.png)

A wall of obstacles splits six rows of the drone's column into a left half and a right half. Taking rows strictly in order would mean flying round the end of the wall twice for every row.

Moving up a row costs 3 extra steps in the drone's scoring, far less than a trip round the wall. So it sweeps the left half to the top of the wall, crosses once, sweeps the right half downwards, and then carries on above.

### Cases without a picture yet

- **A taken cell that a re-plan would bypass.** A drone that took a neighbour's cell for a detour keeps it as a required stop: if it has to re-plan, the new route still goes through that cell first. No small map shows this cleanly yet.
- **A drone that stops making progress.** A drone that goes 270 ticks (on this field) without ticking off a cell drops the one it is heading for and picks another, so one cell can never stall a run. It has not happened in testing.

## Obstacles

Obstacles are placed in rounds, three by default. The first round is uniformly random. In each later round, a free cell directly left, right, above or below an existing obstacle is `clustering` times as likely to be picked as any other cell, so obstacles form clusters.
