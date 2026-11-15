"""The simulation itself, with no drawing. Run `python3 Sim.py [steps]` to run it headless."""
import sys

from Drones import NUM_DRONES, spawn_drones
from Pathing import Grid


class Simulation:
    def __init__(self, numDrones=NUM_DRONES):
        self.world = Grid()    # the real map, including obstacles the drones have not found yet
        self.liveMap = Grid()  # what the drones know so far
        self.drones = spawn_drones(self.world, self.liveMap, numDrones)
        self.tick = 0
        self.sense()

    def sense(self):
        for drone in self.drones:
            drone.sense(self.world, self.liveMap)

    def step(self):
        """Advances the sim by one tick. Path planning and drone movement will go here."""
        self.sense()
        self.tick += 1

    def searched_count(self):
        return sum(cell.searched for row in self.liveMap.cells for cell in row)


if __name__ == "__main__":
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    sim = Simulation()
    for _ in range(steps):
        sim.step()
    total = sim.liveMap.width * sim.liveMap.height
    print(f"ran {sim.tick} ticks | drones at {[d.position for d in sim.drones]} | searched {sim.searched_count()} / {total} cells")
