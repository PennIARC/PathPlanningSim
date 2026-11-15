"""Regenerates the edge-case screenshots used in the README.

Each case is a small fixed map that makes one situation happen. The sim is run
to the end and the drone view (trails, drones, obstacles the drones found) is
saved to docs/edge_cases/<name>.png. Run from the project folder:

    .venv/bin/python3 edge_cases.py

To document a new edge case: add an entry to CASES, run this, then describe
it in the "Edge cases" section of README.md.

The maps are tuned to the planner as it is. After changing a planner rule or
constant, re-run this and check each picture still shows what its README
entry says: a map can stop producing its situation.
"""
import os

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

OFF = float("inf")

# Work sharing between drones and the side-row swap are switched off unless a case is about them, so each map shows one thing.
NO_SIDE_ROWS = {"SIDE_ROW_MIN": 0}
NO_SHARING = {"REBALANCE_EVERY": 0, "HANDOVER_WEIGHT": 0}
PLAIN = {**NO_SHARING, **NO_SIDE_ROWS}

# A pocket in yellow's column that opens only towards red's, with a short wall running up the boundary above it
POCKET = ([(x, 15) for x in range(5, 9)] + [(8, y) for y in range(16, 20)] + [(x, 20) for x in range(5, 9)]
          + [(5, y) for y in range(11, 15)])

# name -> (width, height, number of drones, obstacle cells, planner settings for this case).
# (0, 0) is the top-left cell; drones start on the bottom row.
CASES = {
    "blocked_cell": (2, 6, 1, [(0, 3)], PLAIN),
    "same_tick": (12, 6, 4, [(2, 2), (3, 5), (4, 4), (5, 3), (8, 5)], PLAIN),
    "neighbour_cells": (8, 8, 2, [(4, 5), (6, 6)], PLAIN),
    "unreachable_cell": (8, 8, 2, [(4, 0), (5, 1), (6, 0)], PLAIN),
    "head_on": (12, 6, 4, [(4, 2), (6, 0), (6, 3), (7, 4), (8, 5), (9, 4), (11, 1)], PLAIN),
    "corridor": (6, 7, 2, [(1, 0), (3, 1), (4, 1), (5, 1)], PLAIN),
    "boxed_in_start": (12, 6, 4, [(2, 5), (3, 4)], PLAIN),
    "parked_drone": (6, 7, 2, [(1, 3), (3, 1), (4, 0)], PLAIN),
    "start_walk": (12, 5, 4, [], PLAIN),
    "nearer_end": (6, 6, 1, [(1, 3), (2, 3), (3, 3), (4, 3), (5, 3)], PLAIN),
    "wall": (7, 10, 1, [(3, y) for y in range(2, 8)], PLAIN),
    "skipped_area": (9, 16, 1, [(5, y) for y in range(10, 14)] + [(x, 9) for x in range(0, 6)], PLAIN),
    "finished_drone_helps": (9, 10, 2, [], {"REBALANCE_GAP": OFF, "HANDOVER_WEIGHT": 0, **NO_SIDE_ROWS}),
    "boundary_shift": (9, 14, 2, [], {"HELP_MIN": OFF, "HANDOVER_WEIGHT": 0, **NO_SIDE_ROWS}),
    "pocket_handover": (10, 26, 2, POCKET, NO_SIDE_ROWS),
    "kept_stop": (12, 6, 4, [(4, 0), (5, 1), (6, 2), (7, 2), (8, 2), (8, 4), (9, 3)], PLAIN),
    "wait_then_go_round": (12, 6, 4, [(1, 1), (6, 0), (6, 4), (7, 4), (8, 5)], PLAIN),
    "occupied_target": (12, 8, 4, [(0, 3), (1, 5), (1, 6), (3, 7), (4, 6), (5, 6), (6, 3), (7, 3), (8, 4), (8, 7)], PLAIN),
    "new_eyes": (8, 7, 2, [(2, 5), (3, 4)], PLAIN),
    "blocked_start": (12, 5, 4, [(0, 4), (1, 4), (2, 4)], PLAIN),
    "split_row": (6, 6, 1, [(1, 3)], PLAIN),
    "stalled": (7, 10, 1, [(3, y) for y in range(2, 8)], {**PLAIN, "STALL_FACTOR": 0.2}),  # limit cut to 3 ticks
    "side_row": (9, 8, 1, [(4, 3), (4, 4)], NO_SHARING),
}


def run(case):
    width, height, numDrones, obstacles, settings = CASES[case]
    saved = {name: getattr(Planner, name) for name in settings}
    for name, value in settings.items():
        setattr(Planner, name, value)
    try:
        sim = Simulation(numDrones=numDrones, width=width, height=height, obstacles=obstacles)
        while sim.step():
            pass
    finally:
        for name, value in saved.items():
            setattr(Planner, name, value)
    return sim


def render(sim):
    """The drone view of the whole map, drawn the same way the visualizer draws it."""
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
    return image


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    main.TRAIL_WIDTH = 2  # a little thicker than in the app, since these cells are drawn larger
    for case in CASES:
        sim = run(case)
        path = os.path.join(OUT_DIR, case + ".png")
        pygame.image.save(render(sim), path)
        print(f"{path}: {sim.tick} ticks")
