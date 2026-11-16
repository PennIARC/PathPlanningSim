# PathPlanningSim

A grid sim for testing how several drones search a field together. Four drones sweep a 30 x 60 grid that has obstacles they can't see until they are one cell away.

```
python3 -m venv .venv && .venv/bin/python3 -m pip install -r requirements.txt
.venv/bin/python3 main.py         # visualizer
.venv/bin/python3 Sim.py          # headless run, prints ticks and coverage
.venv/bin/python3 edge_cases.py   # regenerates the screenshots below
.venv/bin/python3 stress_test.py  # checks the planner on many maps, from open field to very dense
```

In the visualizer, Space plays and pauses. The right arrow steps one tick and the left arrow steps back; hold either to repeat. The buttons along the bottom do the same, plus reset and randomize the obstacles. Esc quits. Three sliders control the obstacles:

- **obstacles** is the share of the field that is obstacles, 0 to 30%.
- **clustering** is how many times as likely a cell beside an existing obstacle is to become one. 1x is an even scatter, 3x is the default, and the slider goes to 20x.
- **rounds** is how many rounds the obstacles are placed in. Clustering only applies from the second round on, so more rounds let clusters grow larger. 1 to 12.

`stress_test.py` runs the planner on seven kinds of map, including ones far denser and more clustered than a real field, and checks every tick that no two drones share a cell or swap places and that none flies onto an obstacle. The harsh maps are there to shake out edge cases, not to tune against. It names the worst map of each kind, which can be replayed with `main.py --seed N --obstacles 0.3 --clustering 20 --rounds 12`.

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
- Before moving up a row, a drone checks for cells it skipped lower down. If the shortest route to one runs only through cells already seen, and the trip is no longer than the rows it still has to climb, it goes back for it then instead of at the end.
- Before moving up a row, a drone also checks whether another drone is closer to the cells it is leaving behind. Each connected group of them goes to whichever drone has the shortest trip to it. Unseen cells count as free in that trip, so the owner keeps the cells while a way in may still exist.
- Every tick the work is shared out. If two neighbours' remaining work differs by more than one cell, the lighter one takes half the difference from the edge of the other's column. Cells come first from the rows where the boundary has moved least from the original split, and among those from the rows furthest up the field, so the boundary stays straight.
- Among equally short routes, a drone takes the one over the most cells that another drone has flown and it has not, so a second pair of eyes sees them.
- When a detour runs sideways over cells already flown, and the same stretch one row up or down is 3 or more of the drone's unvisited cells, the drone flies that row instead and steps back at the end. That costs two extra steps. Where the route was about to turn onto that row anyway it costs nothing, so one new cell is enough.
- A drone that finishes takes over the cells the busiest drone would reach last, as many as it can finish by the time that drone is done. If nobody needs help it waits in its last cell.

## Parameters

Every number the planner depends on, all in `Planner` in `Pathing.py`, and where it came from. "Tuned" means chosen by measurement, "picked" means a reasonable value that was not tuned, and "derived" means it follows from something else. Measurements are on the 30 x 60 field with 4 drones, so re-measure if the grid or drone count changes much.

| Parameter | Value | What it does | Where the value comes from |
|---|---|---|---|
| `CLIMB_COST` | 3 | Extra cost, in steps, for each row a candidate cell lies above the drone. It makes a drone finish cells at or below its row before moving up, unless they are more than about 3 steps further away than the next row. | Tuned. Values 1 to 4 finished fastest and were within noise of each other; 6 and above were 3 to 5% slower at 8 to 20% obstacles. It is not scaled with column width. |
| `REROUTE_AFTER` | 2 ticks | How long a drone that must give way stays blocked behind another drone before it looks for a way round. | Picked. It is the "wait one tick, then go round" rule: one tick waiting, then act on the second. |
| `GIVE_UP_AFTER` | 8 ticks | How long a drone stays blocked with no way round before it skips the cell it was heading for. Also how long the drone with right of way holds its line in a head-on meeting. | Picked, not tuned. Long enough for another drone to clear a short corridor. |
| `STALL_FACTOR` | 3 | A drone that goes `STALL_FACTOR x (width + height)` ticks without ticking off a cell drops the one it is heading for. | Picked as a safety net. On the 30 x 60 field the limit is 270 ticks, far longer than any real detour. It has never triggered in testing. |
| Prediction horizon | length of the detour | How far ahead a neighbour's movements are played forward when deciding who reaches a shared cell first. | Derived. Anything the neighbour would reach later than the detouring drone counts as "later" anyway. |
| `BACKTRACK_WEIGHT` | 1.0 | A drone about to move up goes back for a skipped cell if the trip is at most this many times the rows it still has to climb. 0 switches the behaviour off. | Derived, then checked. Going back now costs about two trips; leaving the cell to the end costs about one trip plus the rows left to climb, so the break-even is 1. Measured: 0 was 4% slower on random maps and up to 10% on maps with a walled-off area; 0.5 to 2.5 were within noise of each other. |
| `REBALANCE_EVERY` | 1 tick | How often work is shared out between drones. 0 switches sharing off. | Chosen for consistency with the rest of the planner, which re-plans every tick. It does not matter for speed: intervals of 1, 2, 5, 10, 20 and 50 ticks finished within noise of each other on five kinds of map (30 maps each), and switching sharing off was 3 to 5% slower. |
| `REBALANCE_GAP` | 1 cell | Two neighbours' remaining work must differ by more than this before cells move between them. Half the difference moves. | Chosen to keep the loads as even as possible, against the measurement. Thresholds of 1, 4 and 10 finished default maps in 488, 484 and 483 ticks on average (30 maps), and dense maps in 527, 523 and 516. At 1, cells change hands about 400 times a run on a default map; at 10, about 17. |
| `HELP_MIN` | 6 cells | A finished drone only helps a drone with at least this many cells left. | Picked. 2, 6 and 20 finished within noise of each other. |
| `HANDOVER_WEIGHT` | 1.0 | Cells a drone is leaving behind go to another drone if that drone's trip to them, times this, is shorter than the owner's. 0 switches handing over off. | Picked as the plain comparison, then checked. 0.75, 1, 1.5 and 2 finished default maps within a tick of each other. Switching it off makes no difference on default maps and is 1.6% slower on dense, clustered maps and 3% slower on very dense ones. |
| `SIDE_ROW_MIN` | 3 cells | A sideways stretch of a detour over already-flown cells moves to the row above or below, for two extra steps, if that covers at least this many of the drone's unvisited cells. 0 switches it off. | Tuned. Off, 2, 3, 4 and 6 finished default maps in 488, 480, 476, 477 and 487 ticks on average (30 maps) and sparse maps in 478, 474, 467, 469 and 476. The open field is 2 ticks slower with it on (455 against 453). Where the route turns onto that row anyway the swap is free and one new cell is enough; asking for 3 there as well was 2 ticks slower on default maps. |
| Boundary shift order | least shifted row, then furthest up | Which cells move when neighbours even out their work: one per row, from the rows where the boundary is closest to the original split. | Measured against taking the top row first, which hollowed out the top rows into long sideways slivers. Default maps went from 476 to 468 ticks on average (30 maps), sparse from 467 to 461, dense from 520 to 508. |
| Tie-breaks | new eyes, left, lower index | Two equally short routes: the one over more cells that only other drones have flown. Two equally cheap cells: the left one. Two drones with equal right of way: the lower index. | Picked. The first gives a cell a second look for free; the others make runs repeatable. |

## Edge cases

<br>

### One drone sweeping its column

#### A cell in the sweep is an obstacle

![blocked cell](docs/edge_cases/blocked_cell.png)

The drone is sweeping a two-wide column and the next cell up, on the left, turns out to be an obstacle.

It drops that cell and runs A* to the next one it needs, on the right of the same row. The shortest way there is back through the cell it just left, so the trail doubles up.

<br>

#### Obstacles on the first row

![blocked start](docs/edge_cases/blocked_start.png)

Three obstacles fill the bottom row of red's column, so the first cells of its sweep do not exist.

The first row gets no special treatment. Red heads for the cheapest end of the first row it can reach, one row up, and starts sweeping there without waiting.

<br>

#### A cell that cannot be reached

![unreachable cell](docs/edge_cases/unreachable_cell.png)

A free cell on the top row of yellow's column is shut in by three obstacles and the edge of the field.

A* finds no route to it, so yellow drops it for good and finishes without it. The cell is seen from next door but never flown over.

<br>

#### Arriving at the far end of a row

![nearer end](docs/edge_cases/nearer_end.png)

A row of obstacles leaves one gap, at the left edge, so the drone comes up into the next row at its left end. A fixed left-right zigzag would have it start that row from the right.

The drone picks whichever end of a row is fewer A* steps away. It sweeps from the left and saves a crossing.

<br>

#### A row cut in two by one obstacle

![split row](docs/edge_cases/split_row.png)

An obstacle splits a row into a single cell on the left and four cells on the right. The drone arrives on the left.

It flies the single cell, goes over the obstacle through the row above and drops back down to finish the right-hand part before carrying on upwards. A cell below the drone costs only its steps, while every row up costs 3 more, so a row is not left half done.

<br>

#### A wall up the middle of a column

![wall](docs/edge_cases/wall.png)

A wall of obstacles splits six rows of the drone's column into a left half and a right half. Taking rows strictly in order would mean flying round the end of the wall twice for every row.

Moving up a row costs 3 extra steps in the drone's scoring, far less than a trip round the wall. The drone starts up the right half, goes back for the left half after two rows while that is still a short trip, sweeps it to the top of the wall, crosses once, and finishes the right half. The arrows mark where it turns back.

<br>

#### A detour over cells already flown

![side row](docs/edge_cases/side_row.png)

A two-cell wall splits two rows of the column. The drone has flown the three right-hand cells of the row below the wall and now has to go back left for the rest of that row, over the cells it just flew.

It steps up instead, flies the four cells to the right of the wall, and steps back down beside the wall: two extra steps for four new cells, which it then never has to come round the wall for. This map takes 69 ticks with the swap and 77 without.

<br>

#### A walled-off area that gets skipped

![skipped area](docs/edge_cases/skipped_area.png)

An area on the left is walled off on its right and on top, so it can only be entered from below. The drone finishes a row on the far side of the wall and starts climbing there. Left alone, it would sweep to the top of the field and then fly all the way back down for the area.

Before each move up it checks the skipped cells. Here the known way back, round the foot of the wall, is 5 steps and nothing unseen could be shorter, with 12 rows still to climb, so it goes back after two rows. That saves 10 ticks on this map. Had an unseen cell offered a possible shortcut, it would have kept sweeping along the wall until that cell was seen.

<br>

#### A cell that takes too long to reach

![stalled](docs/edge_cases/stalled.png)

For this picture the stall limit was cut from 51 ticks to 3. The drone sets off round the foot of the wall for a cell on the other side, and the trip is longer than that.

After 4 ticks without flying over a new cell it drops the cell it was heading for and picks the next one, which leaves one cell unflown. At the real limit, 270 ticks on the full field, this has never happened in testing. It is there so that one cell can never hold up a run.

<br>

### Crossing a neighbour's column

#### The walk to the starting column

![start walk](docs/edge_cases/start_walk.png)

Drones start side by side in the middle of the bottom row, so the two outer ones cross a neighbour's column to reach their own.

That walk is transit only. The cells crossed stay with their owner, who still sweeps them, and the usual collision rules apply.

<br>

#### A detour through a neighbour's column

![neighbour cells](docs/edge_cases/neighbour_cells.png)

An obstacle blocks yellow at the left edge of its column, and its detour crosses three cells of red's column. Red has already swept two of them and would not reach the third for a while.

Yellow flies straight through the two swept cells and nothing changes hands. The third is reassigned to yellow, which covers it on the way; red takes it off its own sweep and never visits it.

<br>

#### Both drones due at a shared cell on the same tick

![same tick](docs/edge_cases/same_tick.png)

Yellow is crossing green's column to get round obstacles, and would arrive at one of green's cells on the tick green is due there.

The cell stays green's and green flies as planned. Yellow waits one tick in the cell before it, then follows green through. The dot marks where it waits.

<br>

#### A taken cell that a re-plan would bypass

![kept stop](docs/edge_cases/kept_stop.png)

Green's way up is walled off, so it plans a detour through yellow's column and takes two of yellow's top-row cells that the route crosses. One of them turns out to be an obstacle, which kills the detour, and green has to go the long way round its own side.

The other taken cell is no longer on the way to anything, but yellow has dropped it. Green keeps it as a required stop: it comes round, steps one cell out along the top row to fly over it, and comes back.

<br>

#### New eyes over the same eyes

![new eyes](docs/edge_cases/new_eyes.png)

Red has flown into a one-cell pocket and heads for its next row, three rows up. Two routes are equally short. One runs over a cell only red has flown; the other over a cell on the edge of yellow's column that only yellow has flown.

Red takes the one through yellow's cell, which gives that cell a second look, where re-flying its own cell would add nothing. This only settles ties between equally short routes and never makes a route longer.

<br>

### Drones in each other's way

#### Two drones meet head-on

![head on](docs/edge_cases/head_on.png)

Red's last cell is cut off from the rest of its column by obstacles, so red comes down the edge of yellow's column to reach it, just as yellow is coming up that edge. They are face to face, each wanting the cell the other is in.

Drones never swap places. The planner spots the meeting before either drone moves. Both are on detours, so the lower index, red, has right of way, and yellow re-plans through the next cell over. Neither loses a tick.

<br>

#### A gap one cell wide

![corridor](docs/edge_cases/corridor.png)

The top row of yellow's column can only be reached through a single cell at the edge of red's column. Yellow goes in as red is coming out.

There is no way round. The drone without right of way, yellow, backs out one cell, red comes through, and yellow goes in after it.

<br>

#### Boxed in at the start

![boxed in start](docs/edge_cases/boxed_in_start.png)

Red's walk along the bottom row to its own column runs into a dead end, with obstacles ahead and above. The only way out is back through the cell yellow is about to move into.

Red has nowhere to step aside to, so yellow moves out of the way even though it has right of way. Red comes out and goes round, and yellow carries on.

<br>

#### Stuck behind a drone that is not moving

![wait then go round](docs/edge_cases/wait_then_go_round.png)

Blue is cutting through the middle of the field to get round obstacles on its walk out. The cell ahead holds green, which is itself held up by yellow.

Blue waits one tick in case the way clears. Green is still there, so on the next tick blue plans a route around the cell green is in and carries on.

<br>

#### The next cell has another drone in it

![occupied target](docs/edge_cases/occupied_target.png)

Green's next cell is one step away, but yellow and then red pass through it on their way round an obstacle.

Green waits a tick, then drops the cell instead of queueing for it, since a drone is flying over it anyway. Green carries on with its sweep and never goes back for that cell.

<br>

#### A finished drone parked in the way

![parked drone](docs/edge_cases/parked_drone.png)

Red finishes first and parks in its last cell, at the top of its column. Yellow's last cell is behind two obstacles, and the only way to it starts with the cell red is sitting in.

A finished drone with nobody to help stays put. Once yellow has waited two ticks behind it, red steps aside to the nearest cell off yellow's route, one cell to the left, and yellow goes through and finishes.

<br>

### Sharing the work

#### One drone has more to do than its neighbour

![boundary shift](docs/edge_cases/boundary_shift.png)

Yellow's column is one cell wider than red's, which leaves it 14 more cells to cover.

Neighbours compare their remaining work every tick. Red takes half the difference, seven cells: the nearest cell of yellow's column in each of the top seven rows. Its rows are one cell wider from there up, and the two finish on the same tick.

<br>

#### A drone finishes early

![finished drone helps](docs/edge_cases/finished_drone_helps.png)

Red's column is one cell narrower than yellow's, so red finishes while yellow still has most of its top row to do. Boundary shifting is switched off for this picture.

A finished drone takes over the cells the busiest drone would reach last, as many as it can finish by the time that drone is done. Red flies on into yellow's top row and takes four cells, and the two finish within two ticks of each other.

<br>

#### Cells walled off from their own drone

![pocket handover](docs/edge_cases/pocket_handover.png)

A pocket in yellow's column is closed on the top, right and bottom and opens only towards red's column, and a short wall runs up the boundary above it. Yellow climbs past on the far side, and left alone would fly all the way back round for it at the end.

Each time yellow is about to move up, any drone closer to the cells it is leaving behind takes them. Red is beside the opening, so the pocket's rows join red's sweep. Red then has more to do, so the boundary shift gives yellow cells from the edge of red's column in the rows above the wall. The run takes 125 ticks instead of 138.
