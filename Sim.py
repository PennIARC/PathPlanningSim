"""The simulation itself, with no drawing. Run `python3 Sim.py` to run it headless to the end."""
import random

from Drones import NUM_DRONES, spawn_drones
from Pathing import GRID_HEIGHT, GRID_WIDTH, NEIGHBOURS, Grid, Planner

OBSTACLE_RATE = 0.08  # fraction of the real map's cells that are obstacles
OBSTACLE_ROUNDS = 3   # obstacles are placed in this many rounds; from the second round on they favour existing ones
CLUSTER_WEIGHT = 3.0  # in those later rounds a cell next to an obstacle is this many times as likely (1 = no clustering)


class Simulation:
    def __init__(self, numDrones=NUM_DRONES, obstacleRate=OBSTACLE_RATE, clusterWeight=CLUSTER_WEIGHT,
                 obstacleRounds=OBSTACLE_ROUNDS, seed=None, keepHistory=False, width=None, height=None, obstacles=None):
        self.numDrones = numDrones
        self.clusterWeight = clusterWeight
        self.obstacleRounds = obstacleRounds
        self.size = (width or GRID_WIDTH, height or GRID_HEIGHT)
        self.fixedObstacles = obstacles  # explicit list of (x, y) obstacle cells; None means place them at random
        self.keepHistory = keepHistory  # snapshot every tick so step_back() works; costs a few ms per tick
        self.obstacleRate = obstacleRate
        self.seed = random.randrange(1_000_000) if seed is None else seed
        self.version = 0  # goes up whenever anything changes, so the visualizer knows when to redraw
        self.reset()

    def reset(self):
        """Back to tick 0 on the same obstacle layout."""
        self.world = Grid(*self.size)    # the real map, including obstacles the drones have not found yet
        self.liveMap = Grid(*self.size)  # what the drones know so far, plus which drone each cell is assigned to
        self.knownObstacles = set()  # (x, y) obstacle cells seen so far, shared by every drone
        self.cellPasses = {}         # (cell, cell) -> times any drone has flown between them, shared by every drone
        self.drones = spawn_drones(self.world, self.knownObstacles, self.cellPasses, self.numDrones)
        self.planner = Planner(self.liveMap, self.drones, self.knownObstacles)
        self.tick = 0
        self.history = []  # one snapshot per tick taken so far, so step_back() can undo them
        self.place_obstacles()
        self.planner.assign()
        self.record_visits()
        self.sense()
        self.planner.plan()
        self.version += 1

    def randomize(self):
        """New random obstacle layout, and drones back to their starting cells."""
        self.seed = random.randrange(1_000_000)
        self.reset()

    def set_obstacle_rate(self, rate):
        """Changes how much of the map is obstacles (0 to 1), with a fresh random layout, and resets.

        Returns False if the number of obstacles did not change, in which case nothing is touched.
        """
        rate = max(0.0, min(1.0, rate))
        if self.obstacle_count(rate) == self.obstacle_count():
            self.obstacleRate = rate
            return False
        self.obstacleRate = rate
        self.randomize()
        return True

    def set_cluster_weight(self, weight):
        """Changes how strongly obstacles clump together, with a fresh random layout, and resets. False if unchanged."""
        weight = max(1.0, weight)
        if weight == self.clusterWeight:
            return False
        self.clusterWeight = weight
        self.randomize()
        return True

    def set_obstacle_rounds(self, rounds):
        """Changes how many rounds the obstacles are placed in, with a fresh random layout, and resets. False if unchanged."""
        rounds = max(1, int(round(rounds)))
        if rounds == self.obstacleRounds:
            return False
        self.obstacleRounds = rounds
        self.randomize()
        return True

    def obstacle_count(self, rate=None):
        rate = self.obstacleRate if rate is None else rate
        return round(rate * self.world.width * self.world.height)

    def place_obstacles(self):
        """Places the obstacles in `obstacleRounds` rounds.

        The first round is uniformly random. In each later round a free cell
        directly left, right, above or below an existing obstacle is
        `clusterWeight` times as likely to be picked as any other, so
        obstacles tend to form clusters. More rounds let clusters grow further.
        """
        if self.fixedObstacles is not None:
            for x, y in self.fixedObstacles:
                self.world.set_cell(x, y, isObstacle=True)
            return
        # Seeded, so reset() rebuilds the same layout until randomize() picks a new seed
        rng = random.Random(self.seed)
        spawnCells = {d.position for d in self.drones}
        free = [(x, y) for y in range(self.world.height) for x in range(self.world.width) if (x, y) not in spawnCells]
        total = min(self.obstacle_count(), len(free))
        obstacles = set()
        for turn in range(self.obstacleRounds):
            count = total // self.obstacleRounds + (1 if turn < total % self.obstacleRounds else 0)
            candidates = [c for c in free if c not in obstacles]

            def weight(cell):
                nextToObstacle = any((cell[0] + dx, cell[1] + dy) in obstacles for dx, dy in NEIGHBOURS)
                return self.clusterWeight if nextToObstacle else 1

            # Weighted pick without repeats: give each cell a random score that favours heavier cells, keep the top ones
            scored = sorted(candidates, key=lambda cell: rng.random() ** (1 / weight(cell)), reverse=True)
            obstacles.update(scored[:count])
        for x, y in obstacles:
            self.world.set_cell(x, y, isObstacle=True)

    def record_visits(self):
        """Notes on the live map which drones have flown over each cell."""
        for drone in self.drones:
            x, y = drone.position
            visitedBy = self.liveMap.cells[y][x].visitedBy
            if drone.index not in visitedBy:
                visitedBy.append(drone.index)
                self.liveMap.touch()

    def sense(self):
        for drone in self.drones:
            drone.sense(self.world, self.liveMap)

    def finished(self):
        return all(drone.finished() for drone in self.drones)

    def step(self):
        """Advances the sim by one tick: drones that can move take one step, everyone looks around, then re-plans.

        Returns False if every drone had already finished.
        """
        if self.finished():
            return False
        if self.keepHistory:
            self.history.append(self._snapshot())
        moving = self.planner.resolve_moves()
        for drone in self.drones:
            drone.ticksSinceProgress += 1
            if drone in moving:
                drone.advance()
            elif drone.route:
                drone.wait()
        self.record_visits()
        self.sense()
        self.planner.plan()
        self.tick += 1
        self.version += 1
        return True

    def _snapshot(self):
        """Everything step_back() needs to undo a tick. Hand-copied because a generic deep copy is too slow per tick."""
        cells = [[(c.droneAllocationIndex, c.searched, c.isObstacle, tuple(c.visitedBy)) for c in row] for row in self.liveMap.cells]
        return cells, set(self.knownObstacles), dict(self.cellPasses), [drone.clone() for drone in self.drones]

    def step_back(self):
        """Undoes the last tick. Needs keepHistory=True. Returns False if there is nothing to undo."""
        if not self.history:
            return False
        cells, self.knownObstacles, self.cellPasses, self.drones = self.history.pop()
        for row, savedRow in zip(self.liveMap.cells, cells):
            for cell, (allocation, searched, isObstacle, visitedBy) in zip(row, savedRow):
                cell.droneAllocationIndex, cell.searched, cell.isObstacle, cell.visitedBy = allocation, searched, isObstacle, list(visitedBy)
        self.liveMap.touch()
        for drone in self.drones:
            drone.obstacles, drone.passCounts = self.knownObstacles, self.cellPasses
        self.planner = Planner(self.liveMap, self.drones, self.knownObstacles)
        self.tick -= 1
        self.version += 1
        return True

    def searched_count(self):
        return sum(cell.searched for row in self.liveMap.cells for cell in row)

    def known_obstacle_count(self):
        return sum(cell.isObstacle for row in self.liveMap.cells for cell in row)


if __name__ == "__main__":
    sim = Simulation()
    while sim.step():
        pass
    total = sim.liveMap.width * sim.liveMap.height
    free = total - sim.obstacle_count()
    flownOver = len(set().union(*(d.visited for d in sim.drones)))
    print(f"finished in {sim.tick} ticks | flew over {flownOver} / {free} free cells | saw {sim.searched_count()} / {total} cells | "
          f"found {sim.known_obstacle_count()} / {sim.obstacle_count()} obstacles | seed {sim.seed}")
