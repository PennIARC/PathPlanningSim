"""A near-best plan for the four drones when every obstacle is known in advance, to compare the planner against.

    .venv/bin/python3 groundTruth.py                                             # default map, seed 0
    .venv/bin/python3 groundTruth.py --seed 3 --obstacles 0.3 --clustering 20 --rounds 12
    .venv/bin/python3 groundTruth.py --open                                      # also open the picture

It prints how many ticks the plan takes next to the planner's own result on
the same map, and saves a picture of the routes (ground_truth.png by default).

How it works:
  1. The free cells that can be reached are cut into one vertical strip per
     drone, each with the same number of cells.
  2. For each strip, LKH finds the shortest route that starts at the drone's
     start cell and flies over every cell of the strip. LKH is a solver for
     the travelling-salesman problem (the shortest route through a set of
     points); it comes with the `elkai` package. Distances are real step
     counts around the obstacles, and a route may pass through other strips.
  3. The strips are then nudged: cells move from the drone with the longest
     route towards its neighbours, the routes are solved again, and the best
     split seen is kept.

What it is not:
  - Not a proven optimum. Finding one is not feasible at this size. LKH is
    usually within a few percent of the best route for a strip, and the
    strips themselves are only balanced by trial.
  - Drones are planned one at a time, so two routes can cross the same cell
    on the same tick. The number of such clashes is printed. Fixing them
    would cost a few ticks, so the result is slightly optimistic.

The printed lower bound is exact: a drone flies over at most one new cell per
tick, so no plan can finish in fewer than (cells to cover) / (drones) ticks.

LKH is free for academic and non-commercial use only.
"""
import argparse
import collections
import math
import subprocess
import time

import elkai

from Pathing import NEIGHBOURS
from Sim import CLUSTER_WEIGHT, OBSTACLE_RATE, OBSTACLE_ROUNDS, Simulation

FAR = 1_000_000  # distance used to pin the end marker to the start cell; larger than any real route


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
    def __init__(self, sim):
        self.sim = sim
        world = sim.world
        self.starts = [drone.position for drone in sim.drones]
        free = {(x, y) for y in range(world.height) for x in range(world.width) if not world.cells[y][x].isObstacle}
        self.free = set(steps_from(self.starts[0], free))  # only cells that can actually be reached
        self.cells = sorted(self.free - set(self.starts))  # cells to share out, column by column from the left
        self.stepsCache = {}  # cell -> steps from it to every other cell
        self.lkhCalls = 0
        self.lkhSeconds = 0.0

    def steps(self, cell):
        if cell not in self.stepsCache:
            self.stepsCache[cell] = steps_from(cell, self.free)
        return self.stepsCache[cell]

    def lower_bound(self):
        """No plan can beat this: each drone flies over at most one new cell per tick."""
        return math.ceil(len(self.cells) / len(self.starts))

    # ---------------- One drone

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
        matrix = []
        for a in nodes:
            fromA = self.steps(a)
            matrix.append([fromA[b] for b in nodes] + [FAR])
        matrix[0][marker] = 0
        matrix.append([0] + [FAR] * (len(nodes) - 1) + [0])
        began = time.perf_counter()
        tour = elkai.DistanceMatrix(matrix).solve_tsp(runs=1)[:-1]  # the last entry repeats the first
        self.lkhSeconds += time.perf_counter() - began
        self.lkhCalls += 1
        at = tour.index(marker)
        tour = tour[at + 1:] + tour[:at]  # the round trip with the marker cut out
        if tour[0] != 0:
            tour.reverse()
        if tour[0] != 0:
            raise RuntimeError("LKH did not keep the end marker next to the start cell")
        return [nodes[i] for i in tour]

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

    # ---------------- All drones

    def routes_for(self, cuts):
        """Routes for the strips that `cuts` splits the cell list into: drone i gets cells[cuts[i]:cuts[i + 1]]."""
        return [self.fly(self.best_order(start, self.cells[cuts[i]:cuts[i + 1]])) for i, start in enumerate(self.starts)]

    def solve(self, balanceRounds=6, log=print):
        """Returns one route per drone. Starts from equal strips and tries `balanceRounds` adjusted splits, keeping the best."""
        count, drones = len(self.cells), len(self.starts)
        cuts = [round(i * count / drones) for i in range(drones + 1)]
        best = None
        for attempt in range(balanceRounds + 1):
            routes = self.routes_for(cuts)
            lengths = [len(route) for route in routes]
            log(f"  split {attempt}: cells per drone {[cuts[i + 1] - cuts[i] for i in range(drones)]}, route lengths {lengths}")
            if best is None or max(lengths) < max(len(route) for route in best):
                best = routes
            if max(lengths) - min(lengths) <= 1:
                break
            # Move each boundary towards the drone with the longer route, by about half the difference in cells
            stepsPerCell = sum(lengths) / count
            before = list(cuts)
            for b in range(1, drones):
                shift = round((lengths[b - 1] - lengths[b]) / 2 / stepsPerCell * 0.8)
                cuts[b] = max(cuts[b - 1] + 1, min(cuts[b + 1] - 1, cuts[b] - shift))
            if cuts == before:
                break  # nothing left to move
        return best

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


def render(sim, routes, path, tile=12):
    """Saves the drone view of the finished plan, drawn the way the visualizer draws it."""
    import pygame  # imported here so the solver works without a display

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
    pygame.image.save(edge_cases.render(sim), path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Near-best plan with every obstacle known in advance")
    parser.add_argument("--seed", type=int, default=0, help="obstacle layout (default 0, the visualizer's starting map)")
    parser.add_argument("--obstacles", type=float, default=OBSTACLE_RATE, help="share of cells that are obstacles, e.g. 0.2")
    parser.add_argument("--clustering", type=float, default=CLUSTER_WEIGHT, help="clustering multiplier, e.g. 10")
    parser.add_argument("--rounds", type=int, default=OBSTACLE_ROUNDS, help="obstacle placement rounds, e.g. 8")
    parser.add_argument("--width", type=int, help="cells across (default: the usual field)")
    parser.add_argument("--height", type=int, help="cells along (default: the usual field)")
    parser.add_argument("--balance", type=int, default=6, help="how many adjusted splits to try after the equal one")
    parser.add_argument("--out", default="ground_truth.png", help="where to save the picture")
    parser.add_argument("--open", action="store_true", help="open the picture when done (macOS)")
    args = parser.parse_args()

    settings = dict(obstacleRate=args.obstacles, clusterWeight=args.clustering, obstacleRounds=args.rounds, seed=args.seed,
                    width=args.width, height=args.height)
    began = time.perf_counter()
    truth = GroundTruth(Simulation(**settings))
    print(f"{truth.sim.world.width} x {truth.sim.world.height} field, {len(truth.cells) + len(truth.starts)} reachable free cells")
    routes = truth.solve(args.balance)
    seconds = time.perf_counter() - began

    planner = Simulation(**settings)
    while planner.step():
        pass
    ticks = max(len(route) for route in routes)
    print(f"known-map plan:  {ticks} ticks (routes {[len(route) for route in routes]}, {truth.clashes(routes)} clashes between drones not resolved)")
    print(f"planner:         {planner.tick} ticks, finding the obstacles as it goes ({planner.tick / ticks - 1:+.1%})")
    print(f"lower bound:     {truth.lower_bound()} ticks")
    print(f"time:            {seconds:.1f} s in all, {truth.lkhSeconds:.1f} s of it in {truth.lkhCalls} LKH runs")
    render(truth.sim, routes, args.out)
    print(f"picture saved to {args.out}")
    if args.open:
        subprocess.run(["open", args.out])
