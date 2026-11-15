import pygame

from Sim import Simulation
from text import drawText
from fontDict import fonts

# ---------------- Configuration
FPS = 60
SCALE_DOWN_FACTOR = 2  # draw at 1/2 resolution and scale up, as in GridTesting
PADDING = 20           # px (in scaled-down space) kept clear around each grid
TILE_GAP = 1           # px between cells, which is what makes the grid visible
UNSEEN_OPACITY = 0.3   # live map: how strongly a cell's colour shows before any drone has seen it


class Endesga:
    black = [19, 19, 19]
    white = [255, 255, 255]
    my_blue = [32, 36, 46]
    darker_my_blue = [16, 18, 21]
    grey_blue = [66, 76, 110]
    debug_red = [255, 96, 141]


OBSTACLE_COLOR = [0, 0, 0]
UNALLOCATED_COLOR = Endesga.grey_blue


def world_cell_color(cell):
    """Top grid (the real map): colouring unchanged for now."""
    return Endesga.my_blue


def make_live_cell_color(drones):
    """Bottom grid (the live map): colour = allocated drone, opacity = seen, black = known obstacle."""
    background = Endesga.darker_my_blue

    def live_cell_color(cell):
        if cell.isObstacle:
            return OBSTACLE_COLOR
        base = UNALLOCATED_COLOR if cell.droneAllocationIndex is None else drones[cell.droneAllocationIndex].color
        opacity = 1.0 if cell.searched else UNSEEN_OPACITY
        return [int(background[i] + (base[i] - background[i]) * opacity) for i in range(3)]

    return live_cell_color


def compute_tile_size(region_size, grid):
    """Largest whole-pixel cell size that fits the grid inside a region."""
    avail_w = region_size[0] - PADDING * 2
    avail_h = region_size[1] - PADDING * 2
    return int(max(2, min(avail_w / grid.width, avail_h / grid.height)))


def layout_grids(surface_size, grids):
    """Splits the surface into equal-height bands, one per grid, top to bottom.

    Returns the shared tile size and the top-left draw position of each grid,
    centred inside its band.
    """
    band_h = surface_size[1] // len(grids)
    tile_size = min(compute_tile_size((surface_size[0], band_h), g) for g in grids)
    positions = []
    for i, g in enumerate(grids):
        w, h = g.width * tile_size, g.height * tile_size
        positions.append(((surface_size[0] - w) // 2, i * band_h + (band_h - h) // 2))
    return tile_size, positions


class GridView:
    """Cached drawing of one Grid. Cells are only redrawn when the grid flags them dirty."""

    def __init__(self, grid, tile_size, pos, color_fn):
        self.grid = grid
        self.tile_size = tile_size
        self.pos = pos
        self.color_fn = color_fn
        self.surface = pygame.Surface((grid.width * tile_size, grid.height * tile_size))
        self.surface.fill(Endesga.darker_my_blue)  # shows through as the gap between cells
        grid.mark_all_dirty()

    def refresh(self):
        """Redraws dirty cells onto the cached surface. Returns True if anything changed."""
        if not self.grid.dirty:
            return False
        t = self.tile_size
        fill = t - TILE_GAP
        for x, y in self.grid.dirty:
            pygame.draw.rect(self.surface, self.color_fn(self.grid.cells[y][x]), (x * t, y * t, fill, fill))
        self.grid.dirty.clear()
        return True


def main():
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    clock = pygame.time.Clock()

    screen_width = int(screen.get_width() / SCALE_DOWN_FACTOR)
    screen_height = int(screen.get_height() / SCALE_DOWN_FACTOR)
    screen2 = pygame.Surface((screen_width, screen_height)).convert()
    screen2_scaled = pygame.Surface(screen.get_size()).convert()  # full-size copy, rebuilt only when a grid changes
    screenUI = pygame.Surface((screen_width, screen_height)).convert_alpha()
    font_regular = fonts[f"regular{int(25 / (SCALE_DOWN_FACTOR ** (1 / 1.5)))}"]

    sim = Simulation()
    world, liveMap, drones = sim.world, sim.liveMap, sim.drones  # world is drawn on top, liveMap below it

    tile_size, (world_pos, live_pos) = layout_grids((screen_width, screen_height), [world, liveMap])
    views = [
        GridView(world, tile_size, world_pos, world_cell_color),
        GridView(liveMap, tile_size, live_pos, make_live_cell_color(drones)),
    ]
    # Drones are drawn straight onto the full-size screen, so convert the grid's place and cell size to real pixels
    world_origin_px = (world_pos[0] * SCALE_DOWN_FACTOR, world_pos[1] * SCALE_DOWN_FACTOR)
    tile_size_px = tile_size * SCALE_DOWN_FACTOR

    pygame.mouse.set_visible(False)
    toggle = True

    running = True
    while running:
        mx, my = pygame.mouse.get_pos()
        mx, my = mx / SCALE_DOWN_FACTOR, my / SCALE_DOWN_FACTOR

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    toggle = not toggle

        # ---------------- Simulation
        sim.step()

        # ---------------- Grids: recompose and rescale only when a cell changed
        changed = [view.refresh() for view in views]
        if any(changed):
            screen2.fill(Endesga.darker_my_blue)
            for view in views:
                screen2.blit(view.surface, view.pos)
            pygame.transform.scale(screen2, screen.get_size(), screen2_scaled)

        # ---------------- UI overlay (redrawn every frame)
        screenUI.fill((0, 0, 0, 0))
        if toggle:
            drawText(screenUI, Endesga.debug_red, font_regular, 5, screen_height - 30 / (SCALE_DOWN_FACTOR ** (1 / 1.8)),
                     str(round(clock.get_fps())), Endesga.black, 1, antiAliasing=False)
            pygame.draw.circle(screenUI, Endesga.black, (mx + 1, my + 1), 2, 1)
            pygame.draw.circle(screenUI, Endesga.white, (mx, my), 2, 1)

        screen.blit(screen2_scaled, (0, 0))
        for drone in drones:
            drone.draw(screen, world_origin_px, tile_size_px)
        screen.blit(pygame.transform.scale(screenUI, screen.get_size()), (0, 0))

        pygame.display.update()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
