"""The simulation itself, with no drawing. Run `python3 Sim.py` to run it headless to the end."""
import random

from Drones import NUM_DRONES, spawn_drones
from Pathing import Grid, boustrophedon_path, path_from, split_into_columns

OBSTACLE_COUNT = 20  # random obstacle cells placed on the real map


class Simulation:
    def __init__(self, numDrones=NUM_DRONES, obstacleCount=OBSTACLE_COUNT, seed=None):
        self.numDrones = numDrones
        self.obstacleCount = obstacleCount
        self.seed = random.randrange(1_000_000) if seed is None else seed
        self.version = 0  # goes up whenever anything changes, so the visualizer knows when to redraw
        self.reset()

    def reset(self):
        """Back to tick 0 on the same obstacle layout."""
        self.world = Grid()    # the real map, including obstacles the drones have not found yet
        self.liveMap = Grid()  # what the drones know so far, plus which drone each cell is assigned to
        self.drones = spawn_drones(self.world, self.liveMap, self.numDrones)
        self.tick = 0
        self.place_obstacles()
        self.plan()
        self.sense()
        self.version += 1

    def place_obstacles(self):
        rng = random.Random(self.seed)
        spawnCells = {d.position for d in self.drones}
        free = [(x, y) for y in range(self.world.height) for x in range(self.world.width) if (x, y) not in spawnCells]
        for x, y in rng.sample(free, min(self.obstacleCount, len(free))):
            self.world.set_cell(x, y, isObstacle=True)

    def plan(self):
        """Gives each drone a vertical strip of the field and a back-and-forth path that sweeps it.

        Obstacles are not avoided: drones fly over them and record them.
        """
        columns = split_into_columns(self.world.width, len(self.drones))
        for drone, columnXs in zip(self.drones, columns):
            for x in columnXs:
                for y in range(self.world.height):
                    self.liveMap.set_cell(x, y, droneAllocationIndex=drone.index)
                    drone.assignedCells.add((x, y))
            sweep = boustrophedon_path(columnXs, self.world.height, drone.position[0])
            drone.set_path(path_from(drone.position, sweep))

    def sense(self):
        for drone in self.drones:
            drone.sense(self.world, self.liveMap)

    def finished(self):
        return all(drone.finished() for drone in self.drones)

    def step(self):
        """Advances the sim by one tick: every drone moves one cell along its path, then looks around.

        Returns False if every drone had already finished its path.
        """
        if self.finished():
            return False
        for drone in self.drones:
            drone.step()
        self.sense()
        self.tick += 1
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
    print(f"finished in {sim.tick} ticks | searched {sim.searched_count()} / {total} cells | "
          f"found {sim.known_obstacle_count()} / {sim.obstacleCount} obstacles | seed {sim.seed}")
