"""Runs the planner headless on many random maps of very different kinds and checks it never breaks a rule.

    .venv/bin/python3 stress_test.py            # 20 maps of each kind
    .venv/bin/python3 stress_test.py 50         # 50 maps of each kind
    .venv/bin/python3 stress_test.py 20 dense   # only kinds whose name contains "dense"

Every tick it checks that no two drones share a cell, no two swap places, no
drone is on an obstacle, and nobody moves more than one cell. At the end it
checks the run finished and counts free cells that could have been reached
but were never flown over.

It also reports two signs of wasted effort, to help spot odd behaviour worth
watching in the visualizer: "extra flying" is how much further the drones flew
than one step per reachable cell, and "idle" is drone-ticks spent standing
still with work left. "Turns per drone" is how often a drone changed direction. The worst map of each kind is named so it can be
replayed with, for example:

    .venv/bin/python3 main.py --seed 7 --obstacles 0.3 --clustering 20 --rounds 12

The dense and clustered kinds are far harsher than a real field. They are here
to shake out edge cases, not to tune parameters against.
"""
import statistics
import sys
import time

from Pathing import NEIGHBOURS
from Sim import Simulation

TICK_LIMIT = 6000  # a run still going after this many ticks counts as stuck

# name -> (share of cells that are obstacles, clustering multiplier, placement rounds)
MAP_KINDS = {
    "open field": (0.0, 1, 1),
    "sparse, even": (0.04, 1, 1),
    "default": (0.08, 3, 3),
    "dense, even": (0.20, 1, 1),
    "clustered": (0.12, 6, 5),
    "dense, clustered": (0.20, 10, 8),
    "very dense, clustered": (0.30, 20, 12),
}


def count_turns(trail):
    """Times a drone changed direction along its trail."""
    steps = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(trail, trail[1:]) if a != b]
    return sum(a != b for a, b in zip(steps, steps[1:]))


def run_one(rate, weight, rounds, seed, settings=None):
    """Runs one map to the end. Returns a dict of results; 'problems' lists any broken rules."""
    sim = Simulation(obstacleRate=rate, clusterWeight=weight, obstacleRounds=rounds, seed=seed)
    for name, value in (settings or {}).items():
        setattr(sim.planner, name, value)
    world = sim.world
    obstacles = {(x, y) for y in range(world.height) for x in range(world.width) if world.cells[y][x].isObstacle}
    start = sim.drones[0].position
    problems = []
    idle = 0
    started = time.perf_counter()
    while sim.tick < TICK_LIMIT:
        before = [d.position for d in sim.drones]
        if not sim.step():
            break
        after = [d.position for d in sim.drones]
        idle += sum(a == b and not d.finished() for a, b, d in zip(before, after, sim.drones))
        if len(set(after)) != len(after):
            problems.append(f"tick {sim.tick}: two drones in one cell {after}")
        for i, (a, b) in enumerate(zip(before, after)):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) > 1:
                problems.append(f"tick {sim.tick}: drone {i} jumped {a} -> {b}")
            if b in obstacles:
                problems.append(f"tick {sim.tick}: drone {i} on obstacle {b}")
            for j in range(i + 1, len(after)):
                if a != b and after[i] == before[j] and after[j] == before[i]:
                    problems.append(f"tick {sim.tick}: drones {i} and {j} swapped places")
        if len(problems) > 5:
            break
    seconds = time.perf_counter() - started
    if sim.tick >= TICK_LIMIT:
        problems.append(f"still running after {TICK_LIMIT} ticks")

    # Free cells that can actually be reached from the start on the real map
    reachable, stack = set(), [start]
    while stack:
        cell = stack.pop()
        if cell in reachable or cell in obstacles or not world.in_bounds(*cell):
            continue
        reachable.add(cell)
        stack.extend((cell[0] + dx, cell[1] + dy) for dx, dy in NEIGHBOURS)
    flown = set().union(*(d.visited for d in sim.drones))
    return {
        "ticks": sim.tick,
        "path": sum(len(d.trail) - 1 for d in sim.drones),
        "seed": seed,
        "missed": len(reachable - flown),
        "reachable": len(reachable),
        "extra": sum(len(d.trail) - 1 for d in sim.drones) / max(1, len(reachable)) - 1,
        "idle": idle,
        "turns": sum(count_turns(d.trail) for d in sim.drones) / len(sim.drones),
        "msPerTick": 1000 * seconds / max(1, sim.tick),
        "problems": problems,
    }


def run_kind(name, count, settings=None, verbose=True):
    rate, weight, rounds = MAP_KINDS[name]
    results = []
    for seed in range(count):
        result = run_one(rate, weight, rounds, seed, settings)
        results.append(result)
        if verbose and result["problems"]:
            print(f"  PROBLEM in '{name}' seed {seed}: " + "; ".join(result["problems"][:3]))
    return results


def summary(results):
    bad = sum(bool(r["problems"]) for r in results)
    worst = max(results, key=lambda r: r["extra"])
    return (f"{len(results) - bad}/{len(results)} clean | ticks avg {statistics.mean(r['ticks'] for r in results):6.1f} "
            f"max {max(r['ticks'] for r in results):4} | missed avg {statistics.mean(r['missed'] for r in results):5.2f} "
            f"max {max(r['missed'] for r in results):3} | extra flying avg {statistics.mean(r['extra'] for r in results):5.1%} "
            f"worst {worst['extra']:5.1%} (seed {worst['seed']}) | idle avg {statistics.mean(r['idle'] for r in results):5.1f} "
            f"max {max(r['idle'] for r in results):3} | turns per drone avg {statistics.mean(r['turns'] for r in results):5.1f} | {statistics.mean(r['msPerTick'] for r in results):.2f} ms/tick")


if __name__ == "__main__":
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    wanted = sys.argv[2:]
    allClean = True
    for name in MAP_KINDS:
        if wanted and not any(w in name for w in wanted):
            continue
        results = run_kind(name, count)
        allClean &= not any(r["problems"] for r in results)
        print(f"{name:22} {summary(results)}", flush=True)
    print("all rules held on every map" if allClean else "SOME MAPS BROKE A RULE (see above)")
    sys.exit(0 if allClean else 1)
