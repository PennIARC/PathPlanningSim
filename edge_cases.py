"""Regenerates the edge-case screenshots used in the README.

Each case is a small fixed map that makes one situation happen. The sim is run
to the end and the drone view (trails, drones, obstacles the drones found) is
saved to docs/edge_cases/<name>.png. Run from the project folder:

    .venv/bin/python3 edge_cases.py              # every picture
    .venv/bin/python3 edge_cases.py wall         # one picture, and print where each drone is on every tick

Most pictures carry two arrows at the cell where the rule changes what a drone
does: a white one for the step it takes and a black one for the step it would
have taken without the rule. A white dot means it waits there instead.

To document a new edge case: add an entry to CASES, run this, then describe
it in the "Edge cases" section of README.md.

The maps are tuned to the planner as it is. After changing a planner rule or
constant, re-run this and check each picture still shows what its README
entry says: a map can stop producing its situation.
"""
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # no window needed

import pygame

pygame.init()
pygame.display.set_mode((1, 1))

import main
from palette import BACKGROUND_COLOR
from Pathing import Planner
from Sim import Simulation

OUT_DIR = os.path.join("docs", "edge_cases")
TILE = 20  # px per cell before the 2x upscale, so cells come out 40 px wide
TAKEN_COLOR = (245, 245, 245)   # arrow for the step the drone takes
OTHER_COLOR = (15, 15, 20)      # arrow for the step it would have taken without the rule
OTHER_EDGE = (150, 150, 160)

OFF = float("inf")
LEFT, RIGHT, UP, DOWN = (-1, 0), (1, 0), (0, -1), (0, 1)

# Work sharing between drones and the side-row swap are switched off unless a case is about them, so each map shows one thing.
NO_SIDE_ROWS = {"SIDE_ROW_MIN": 0}
NO_SHARING = {"REBALANCE_EVERY": 0, "HANDOVER_WEIGHT": 0}
PLAIN = {**NO_SHARING, **NO_SIDE_ROWS}

# A pocket in yellow's column that opens only towards red's, with a short wall running up the boundary above it
POCKET = ([(x, 15) for x in range(5, 9)] + [(8, y) for y in range(16, 20)] + [(x, 20) for x in range(5, 9)]
          + [(5, y) for y in range(11, 15)])
WALL = [(3, y) for y in range(2, 8)]


def case(width, height, drones, obstacles, settings=PLAIN, without=None, mark=None):
    """One edge case. (0, 0) is the top-left cell; drones start on the bottom row.

    settings: planner constants for this run.
    without:  planner constants that switch the rule off. The arrows go where the two runs first differ.
    mark:     (drone, tick, direction) for rules that cannot be switched off: the arrows go on that drone at that
              tick, the black one pointing in `direction` (None for no black arrow).
    """
    return {"size": (width, height), "drones": drones, "obstacles": obstacles, "settings": settings, "without": without, "mark": mark}


# In the order the README describes them.
CASES = {
    # ---- One drone sweeping its column
    "blocked_cell": case(2, 6, 1, [(0, 3)], mark=(0, 3, UP)),
    "blocked_start": case(12, 5, 4, [(0, 4), (1, 4), (2, 4)]),
    "unreachable_cell": case(8, 8, 2, [(4, 0), (5, 1), (6, 0)]),
    "nearer_end": case(6, 6, 1, [(1, 3), (2, 3), (3, 3), (4, 3), (5, 3)]),
    "split_row": case(6, 6, 1, [(1, 3)], mark=(0, 17, RIGHT)),
    "wall": case(7, 10, 1, WALL, mark=(0, 20, UP)),
    "side_row": case(9, 8, 1, [(4, 3), (4, 4)], NO_SHARING, without=NO_SIDE_ROWS),
    "skipped_area": case(9, 16, 1, [(5, y) for y in range(10, 14)] + [(x, 9) for x in range(0, 6)], without={"BACKTRACK_WEIGHT": 0}),
    "stalled": case(7, 10, 1, WALL, {**PLAIN, "STALL_FACTOR": 0.2}, without={"STALL_FACTOR": 3}),  # limit cut to 3 ticks
    # ---- Crossing a neighbour's column
    "start_walk": case(12, 5, 4, []),
    "neighbour_cells": case(8, 8, 2, [(4, 5), (6, 6)]),
    "same_tick": case(12, 6, 4, [(2, 2), (3, 5), (4, 4), (5, 3), (8, 5)], mark=(1, 8, UP)),
    "kept_stop": case(12, 6, 4, [(4, 0), (5, 1), (6, 2), (7, 2), (8, 2), (8, 4), (9, 3)], mark=(2, 31, RIGHT)),
    "new_eyes": case(8, 7, 2, [(2, 5), (3, 4)], mark=(0, 20, LEFT)),
    # ---- Drones in each other's way
    "head_on": case(6, 7, 2, [(1, 3), (2, 1), (2, 2)], mark=(1, 17, UP)),
    "corridor": case(6, 7, 2, [(1, 0), (3, 1), (4, 1), (5, 1)], mark=(1, 18, UP)),
    "boxed_in_start": case(12, 6, 4, [(2, 5), (3, 4)], mark=(1, 1, LEFT)),
    "wait_then_go_round": case(12, 6, 4, [(1, 1), (6, 0), (6, 4), (7, 4), (8, 5)], mark=(3, 5, UP)),
    "occupied_target": case(12, 8, 4, [(0, 3), (1, 5), (1, 6), (3, 7), (4, 6), (5, 6), (6, 3), (7, 3), (8, 4), (8, 7)], mark=(2, 4, LEFT)),
    "parked_drone": case(6, 6, 2, [(3, 1), (4, 2)], mark=(0, 19, None)),
    # ---- Sharing the work
    "boundary_shift": case(9, 14, 2, [], {"HELP_MIN": OFF, "HANDOVER_WEIGHT": 0, **NO_SIDE_ROWS}, without={"REBALANCE_EVERY": 0}),
    "finished_drone_helps": case(9, 10, 2, [], {"REBALANCE_GAP": OFF, "HANDOVER_WEIGHT": 0, **NO_SIDE_ROWS}, without={"HELP_MIN": OFF}),
    "pocket_handover": case(10, 26, 2, POCKET, NO_SIDE_ROWS, without={"HANDOVER_WEIGHT": 0}),
}


def run(name, extra=None):
    """Runs a case to the end. Returns the sim and where every drone was on each tick."""
    spec = CASES[name]
    settings = {**spec["settings"], **(extra or {})}
    saved = {key: getattr(Planner, key) for key in settings}
    for key, value in settings.items():
        setattr(Planner, key, value)
    try:
        sim = Simulation(numDrones=spec["drones"], width=spec["size"][0], height=spec["size"][1], obstacles=spec["obstacles"])
        positions = [[d.position for d in sim.drones]]
        while sim.step():
            positions.append([d.position for d in sim.drones])
    finally:
        for key, value in saved.items():
            setattr(Planner, key, value)
    return sim, positions


def decision(name, positions):
    """(cell, cell stepped to, cell it would have stepped to or None) for the case's arrows; None if it has none."""
    spec = CASES[name]
    if spec["mark"]:
        drone, tick, direction = spec["mark"]
        cell = positions[tick][drone]
        return cell, positions[tick + 1][drone], direction and (cell[0] + direction[0], cell[1] + direction[1])
    if spec["without"]:
        _, other = run(name, spec["without"])
        for tick in range(1, min(len(positions), len(other))):
            for drone, (taken, alternative) in enumerate(zip(positions[tick], other[tick])):
                if taken != alternative:
                    return positions[tick - 1][drone], taken, alternative
    return None


def draw_arrow(image, start, end, color, outline):
    """A short arrow from one cell towards the next, in full-size pixels."""
    cell = TILE * main.SCALE_DOWN_FACTOR
    centre = pygame.Vector2((start[0] + 0.5) * cell - main.TILE_GAP, (start[1] + 0.5) * cell - main.TILE_GAP)
    along = pygame.Vector2(end[0] - start[0], end[1] - start[1])
    across = pygame.Vector2(-along.y, along.x)
    tail, neck, tip = centre + along * cell * 0.12, centre + along * cell * 0.62, centre + along * cell * 0.95
    shaft, head = cell * 0.11, cell * 0.27
    points = [tail + across * shaft, neck + across * shaft, neck + across * head, tip,
              neck - across * head, neck - across * shaft, tail - across * shaft]
    pygame.draw.polygon(image, color, points)
    pygame.draw.polygon(image, outline, points, 1)


def render(sim, arrows=None):
    """The drone view of the whole map, drawn the same way the visualizer draws it, plus the decision arrows."""
    scale = main.SCALE_DOWN_FACTOR
    width, height = sim.world.width, sim.world.height
    small = pygame.Surface((width * TILE, height * TILE))
    small.fill(BACKGROUND_COLOR)
    fill = TILE - main.TILE_GAP
    for y in range(height):
        for x in range(width):
            pygame.draw.rect(small, main.drone_view_color(sim, x, y), (x * TILE, y * TILE, fill, fill))
    image = pygame.transform.scale(small, (width * TILE * scale, height * TILE * scale))

    # A ring on each drone's start cell, so the direction of its trail can be read
    for drone in sim.drones:
        cx = (drone.trail[0][0] + 0.5) * TILE * scale - main.TILE_GAP
        cy = (drone.trail[0][1] + 0.5) * TILE * scale - main.TILE_GAP
        pygame.draw.circle(image, drone.color, (cx, cy), TILE * scale * 0.36, 2)

    layerScale = TILE * scale * main.TRAIL_SUPERSAMPLE
    layer = pygame.Surface((width * layerScale, height * layerScale))
    main.draw_drones(image, layer, layer.copy(), sim, TILE, (0, 0))

    if arrows:
        cell, taken, alternative = arrows
        if alternative and alternative != cell:
            draw_arrow(image, cell, alternative, OTHER_COLOR, OTHER_EDGE)
        if taken != cell:
            draw_arrow(image, cell, taken, TAKEN_COLOR, OTHER_COLOR)
        else:  # it waits
            centre = ((cell[0] + 0.5) * TILE * scale - main.TILE_GAP, (cell[1] + 0.5) * TILE * scale - main.TILE_GAP)
            pygame.draw.circle(image, TAKEN_COLOR, centre, TILE * scale * 0.2)
            pygame.draw.circle(image, OTHER_COLOR, centre, TILE * scale * 0.2, 2)
    return image


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    main.TRAIL_WIDTH = 2  # a little thicker than in the app, since these cells are drawn larger
    wanted = sys.argv[1:]
    for name in CASES:
        if wanted and name not in wanted:
            continue
        sim, positions = run(name)
        arrows = decision(name, positions)
        path = os.path.join(OUT_DIR, name + ".png")
        pygame.image.save(render(sim, arrows), path)
        print(f"{path}: {sim.tick} ticks" + (f", arrows at {arrows[0]}: goes to {arrows[1]}, not {arrows[2]}" if arrows else ""))
        if wanted:
            for tick, row in enumerate(positions):
                print(f"  tick {tick:3}: " + "  ".join(str(cell) for cell in row))
