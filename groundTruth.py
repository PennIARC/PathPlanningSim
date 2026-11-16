"""A near-best plan for the four drones when every obstacle is known in advance, to compare the planner against.

    .venv/bin/python3 groundTruth.py --open                   # the default map, seed 0, and open the picture
    .venv/bin/python3 groundTruth.py --seed 1 --obstacles 0.3 --clustering 20 --rounds 12
    .venv/bin/python3 groundTruth.py --demo                   # the three-map picture used in the README

It prints how many ticks the plan takes next to the planner's own result on
the same map, and saves a picture of the routes (ground_truth.png by default).

How it works:
  1. LKH finds one short round trip over every free cell. LKH is a solver for
     the travelling-salesman problem (the shortest route through a set of
     points); it comes with the `elkai` package. Distances are real step
     counts around the obstacles.
  2. The round trip is cut into four consecutive pieces, one per drone, so
     that the longest piece (with the flight out to it) is as short as
     possible. The pieces follow the shape of the trip, which already bends
     round walls and pockets.
  3. LKH re-solves each piece on its own from that drone's start cell.
  4. Cells on the edge of the longest route are moved one at a time into a
     neighbouring drone's route wherever that shortens the longest route.
     Steps 3 and 4 repeat a few times.

What it is not:
  - Not a proven optimum. Finding one is not feasible at this size.
  - Drones are planned without regard to each other's position in time, so
    two routes can cross the same cell on the same tick. The number of such
    clashes is printed. Fixing them would cost a few ticks, so the result is
    slightly optimistic.

The printed lower bound is exact: a drone flies over at most one new cell per
tick, so no plan can finish in fewer than (cells to cover) / (drones) ticks.

Google OR-Tools' routing solver was also tried on the same problem. Given
120 seconds a map against LKH's 20 to 40, its plans were 20 to 45% longer, so
it was dropped.

LKH is free for academic and non-commercial use only.
"""
import argparse
import collections
import math
import os
import subprocess
import time

import elkai

from Pathing import NEIGHBOURS
from Sim import CLUSTER_WEIGHT, OBSTACLE_RATE, OBSTACLE_ROUNDS, Simulation

FAR = 1_000_000      # distance used to pin the end marker to the start cell; larger than any real route
SPLIT_ROTATIONS = 300  # how many places to try starting the first piece when cutting the round trip
SWAP_ROUNDS = 3        # times the routes are re-solved and edge cells moved again



def steps_from(source, free):
    """Steps from `source` to every reachable cell in `free`, moving one cell left, right, up or down at a time."""
    steps = {source: 0}
    queue = collections.deque([source])
    while queue:
        cell = queue.popleft()
        for dx, dy in NEIGHBOURS:
            nxt = (cell[0] + dx, cell[1] + dy)
            if nxt in free and nxt not in steps:
                steps[nxt] = steps[cell] + 1
                queue.append(nxt)
    return steps


class GroundTruth:
    """Holds one map and solves it."""

    def __init__(self, sim, log=print):
        self.sim = sim
        self.log = log
        world = sim.world
        self.starts = [drone.position for drone in sim.drones]
        free = {(x, y) for y in range(world.height) for x in range(world.width) if not world.cells[y][x].isObstacle}
        self.free = set(steps_from(self.starts[0], free))  # only cells that can actually be reached
        self.cells = sorted(self.free - set(self.starts))  # cells to share out
        self.stepsCache = {}  # cell -> steps from it to every other cell
        self.lkhCalls = 0

    def steps(self, cell):
        if cell not in self.stepsCache:
            self.stepsCache[cell] = steps_from(cell, self.free)
        return self.stepsCache[cell]

    def matrix(self, nodes):
        return [[fromA[b] for b in nodes] for fromA in (self.steps(a) for a in nodes)]

    def lower_bound(self):
        """No plan can beat this: each drone flies over at most one new cell per tick."""
        return math.ceil(len(self.cells) / len(self.starts))

    def length(self, order):
        """Steps needed to fly over the cells of `order` in that order."""
        return sum(self.steps(a)[b] for a, b in zip(order, order[1:]))

    def fly(self, order):
        """Turns an order of cells into a cell-by-cell route (not including the first cell), using shortest paths between them."""
        route = []
        for a, b in zip(order, order[1:]):
            toB = self.steps(b)
            cell = a
            while cell != b:
                cell = next(n for n in ((cell[0] + dx, cell[1] + dy) for dx, dy in NEIGHBOURS) if toB.get(n, FAR) == toB[cell] - 1)
                route.append(cell)
        return route

    def clashes(self, routes):
        """Ticks on which two drones would be in the same cell or swap places, if every route were flown at once."""
        ticks = max(len(route) for route in routes)
        paths = [[start] + route + [(route or [start])[-1]] * (ticks - len(route)) for start, route in zip(self.starts, routes)]
        count = 0
        for tick in range(1, ticks + 1):
            for i in range(len(paths)):
                for j in range(i + 1, len(paths)):
                    sameCell = paths[i][tick] == paths[j][tick]
                    swap = paths[i][tick] == paths[j][tick - 1] and paths[j][tick] == paths[i][tick - 1] and paths[i][tick] != paths[i][tick - 1]
                    count += sameCell or swap
        return count

    # ---------------- Solving

    def solve(self):
        """Returns one order per drone: the cells to fly over, starting with its start cell."""
        trip = self.round_trip(self.cells)
        self.log(f"  one round trip over all {len(trip)} cells: {self.length(trip + trip[:1])} steps")
        orders = [[start] + piece for start, piece in zip(self.starts, self.cut_in_pieces(trip))]
        self.log(f"  cut into pieces: route lengths {[self.length(o) for o in orders]}")
        for attempt in range(SWAP_ROUNDS):
            solved = [self.best_order(order[0], order[1:]) for order in orders]
            orders = [new if self.length(new) < self.length(old) else old for new, old in zip(solved, orders)]
            self.log(f"  each piece re-solved: route lengths {[self.length(o) for o in orders]}")
            moved = self.move_edge_cells(orders)
            self.log(f"  {moved} edge cells moved: route lengths {[self.length(o) for o in orders]}")
            if not moved:
                break
        return orders

    def lkh(self, matrix):
        self.lkhCalls += 1
        return elkai.DistanceMatrix(matrix).solve_tsp(runs=1)[:-1]  # the last entry repeats the first

    def round_trip(self, cells):
        """The order LKH picks for one round trip over `cells`."""
        return [cells[i] for i in self.lkh(self.matrix(cells))]

    def best_order(self, start, cells):
        """The order LKH picks for flying over `cells` from `start`, as a list of cells that begins with `start`.

        LKH looks for round trips, but a drone does not need to come back. So
        an extra end marker is added that costs nothing to reach from the
        start cell and FAR from anywhere else. The cheapest round trip then
        always passes start -> marker, and cutting the marker out leaves the
        shortest one-way route from the start.
        """
        nodes = [start] + list(cells)
        if len(nodes) < 3:
            return nodes
        marker = len(nodes)
        matrix = [row + [FAR] for row in self.matrix(nodes)]
        matrix[0][marker] = 0
        matrix.append([0] + [FAR] * (len(nodes) - 1) + [0])
        tour = self.lkh(matrix)
        at = tour.index(marker)
        tour = tour[at + 1:] + tour[:at]  # the round trip with the marker cut out
        if tour[0] != 0:
            tour.reverse()
        if tour[0] != 0:
            raise RuntimeError("LKH did not keep the end marker next to the start cell")
        return [nodes[i] for i in tour]

    def cut_in_pieces(self, trip):
        """Cuts a round trip into one run of consecutive cells per drone so the longest is as short as possible.

        A drone's cost for a piece is the steps along it plus the flight from
        its start cell to whichever end of the piece is nearer. For each of
        SPLIT_ROTATIONS places to begin the first piece, the smallest limit
        that lets four pieces cover the whole trip is found by trying limits
        (a binary search), giving each drone in turn the longest piece that
        fits under the limit.
        """
        count, drones = len(trip), len(self.starts)
        fromStart = [[self.steps(start)[cell] for cell in trip] for start in self.starts]
        best = None
        for rotation in range(0, count, max(1, count // SPLIT_ROTATIONS)):
            order = list(range(rotation, count)) + list(range(rotation))
            along = [0]  # steps along the trip from the first cell of this rotation to each later one
            for a, b in zip(order, order[1:]):
                along.append(along[-1] + self.steps(trip[a])[trip[b]])

            def pieces(limit):
                cuts, first = [], 0
                for drone in range(drones):
                    out = fromStart[drone]
                    last = first - 1
                    for end in range(first, count):
                        inside = along[end] - along[first]
                        if inside > limit:
                            break
                        if inside + min(out[order[first]], out[order[end]]) <= limit:
                            last = end
                    cuts.append((first, last + 1))
                    first = last + 1
                    if first >= count:
                        break
                return cuts if first >= count else None

            low, high = count // drones, along[-1] + max(max(out) for out in fromStart)
            if best is not None:
                if pieces(best[0] - 1) is None:
                    continue  # this rotation cannot beat the best so far
                high = best[0] - 1
            while low < high:
                middle = (low + high) // 2
                if pieces(middle) is None:
                    low = middle + 1
                else:
                    high = middle
            best = (low, [[trip[i] for i in order[a:b]] for a, b in pieces(low)])
        result = best[1] + [[] for _ in range(drones - len(best[1]))]
        # A piece is flown from whichever end is nearer the drone's start
        return [piece if not piece or self.steps(start)[piece[0]] <= self.steps(start)[piece[-1]] else piece[::-1]
                for start, piece in zip(self.starts, result)]

    def move_edge_cells(self, orders):
        """Moves single cells out of the longest route into a neighbouring drone's route while that shortens the longest route.

        Only cells that touch another drone's cells are tried. A cell is put
        into the other route at the place where it adds the fewest steps.
        Edits `orders` in place and returns how many cells moved.
        """
        lengths = [self.length(order) for order in orders]
        moved = 0
        while True:
            a = max(range(len(orders)), key=lambda i: lengths[i])
            owner = {cell: i for i, order in enumerate(orders) for cell in order[1:]}
            source = orders[a]
            best = None
            for at in range(1, len(source)):
                cell = source[at]
                toCell = self.steps(cell)
                others = {owner.get((cell[0] + dx, cell[1] + dy), a) for dx, dy in NEIGHBOURS} - {a}
                if not others:
                    continue
                before, after = source[at - 1], source[at + 1] if at + 1 < len(source) else None
                saved = toCell[before] + (toCell[after] - self.steps(before)[after] if after else 0)
                for b in others:
                    target = orders[b]
                    added, place = toCell[target[-1]], len(target)  # on the end
                    for i in range(1, len(target)):
                        extra = toCell[target[i - 1]] + toCell[target[i]] - self.steps(target[i - 1])[target[i]]
                        if extra < added:
                            added, place = extra, i
                    worst = max(lengths[a] - saved, lengths[b] + added)
                    if worst < lengths[a] and (best is None or (worst, added - saved) < best[0]):
                        best = ((worst, added - saved), at, b, place, saved, added)
            if best is None:
                return moved
            _, at, b, place, saved, added = best
            orders[b].insert(place, source.pop(at))
            lengths[a] -= saved
            lengths[b] += added
            moved += 1

def render(sim, routes, tile=12):
    """The drone view of a finished plan, drawn the way the visualizer draws it. Flies the routes on `sim`."""
    import edge_cases  # reuses its picture code; importing it sets pygame up without a window
    import main

    for row, worldRow in zip(sim.liveMap.cells, sim.world.cells):  # everything is known in advance
        for known, real in zip(row, worldRow):
            known.searched, known.isObstacle = True, real.isObstacle
    for drone, route in zip(sim.drones, routes):  # fly the routes so the trails are recorded
        drone.route = list(route)
        while drone.route:
            drone.advance()
    edge_cases.TILE, main.TRAIL_WIDTH = tile, 1.5
    return edge_cases.render(sim)


def side_by_side(pictures, path):
    """Saves the pictures in one row, each under its caption. `pictures` is a list of (caption, surface)."""
    import pygame

    from palette import BACKGROUND_COLOR, TEXT_COLOR

    font = pygame.font.SysFont("Arial", 20)
    gap, header = 24, 44
    width = sum(surface.get_width() for _, surface in pictures) + gap * (len(pictures) + 1)
    sheet = pygame.Surface((width, max(surface.get_height() for _, surface in pictures) + header + gap))
    sheet.fill(BACKGROUND_COLOR)
    x = gap
    for caption, surface in pictures:
        sheet.blit(font.render(caption, True, TEXT_COLOR), (x, 12))
        sheet.blit(surface, (x, header))
        x += surface.get_width() + gap
    pygame.image.save(sheet, path)


# name -> obstacle settings for the picture in the README
DEMO_MAPS = {
    "default": dict(obstacleRate=0.08, clusterWeight=3, obstacleRounds=3, seed=0),
    "dense, clustered": dict(obstacleRate=0.2, clusterWeight=10, obstacleRounds=8, seed=0),
    "every slider at its maximum": dict(obstacleRate=0.3, clusterWeight=20, obstacleRounds=12, seed=1),
}
DEMO_PATH = os.path.join("docs", "near_optimal.png")


def solve_map(settings):
    """Solves one map and prints the result. Returns (ticks, planner's ticks, lower bound, picture)."""
    truth = GroundTruth(Simulation(**settings))
    planner = Simulation(**settings)
    while planner.step():
        pass
    print(f"{truth.sim.world.width} x {truth.sim.world.height} field, {len(truth.cells) + len(truth.starts)} reachable free cells")
    began = time.perf_counter()
    routes = [truth.fly(order) for order in truth.solve()]
    ticks = max(len(route) for route in routes)
    print(f"known-map plan:  {ticks} ticks (routes {[len(r) for r in routes]}, {truth.clashes(routes)} clashes between drones not resolved)")
    print(f"planner:         {planner.tick} ticks, finding the obstacles as it goes ({planner.tick / ticks - 1:+.1%})")
    print(f"lower bound:     {truth.lower_bound()} ticks")
    print(f"time:            {time.perf_counter() - began:.0f} s, {truth.lkhCalls} LKH runs")
    return ticks, planner.tick, truth.lower_bound(), render(Simulation(**settings), routes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Near-best plan with every obstacle known in advance")
    parser.add_argument("--seed", type=int, default=0, help="obstacle layout (default 0, the visualizer's starting map)")
    parser.add_argument("--obstacles", type=float, default=OBSTACLE_RATE, help="share of cells that are obstacles, e.g. 0.2")
    parser.add_argument("--clustering", type=float, default=CLUSTER_WEIGHT, help="clustering multiplier, e.g. 10")
    parser.add_argument("--rounds", type=int, default=OBSTACLE_ROUNDS, help="obstacle placement rounds, e.g. 8")
    parser.add_argument("--width", type=int, help="cells across (default: the usual field)")
    parser.add_argument("--height", type=int, help="cells along (default: the usual field)")
    parser.add_argument("--demo", action="store_true", help=f"solve the three README maps and save them side by side to {DEMO_PATH}")
    parser.add_argument("--out", default="ground_truth.png", help="where to save the picture")
    parser.add_argument("--open", action="store_true", help="open the picture when done (macOS)")
    args = parser.parse_args()

    if args.demo:
        maps, out = DEMO_MAPS, DEMO_PATH
    else:
        maps, out = {"plan": dict(obstacleRate=args.obstacles, clusterWeight=args.clustering, obstacleRounds=args.rounds, seed=args.seed,
                                  width=args.width, height=args.height)}, args.out
    pictures = []
    for name, settings in maps.items():
        print(f"--- {name}")
        ticks, plannerTicks, bound, picture = solve_map(settings)
        pictures.append((f"{name}: {ticks} ticks (planner {plannerTicks}, bound {bound})", picture))
    side_by_side(pictures, out)
    print(f"picture saved to {out}")
    if args.open:
        subprocess.run(["open", out])
