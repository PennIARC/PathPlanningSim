import time

import pygame

from Sim import Simulation
from palette import (BACKGROUND_COLOR, BUTTON_COLOR, BUTTON_HOVER_COLOR, CURSOR_COLOR, FPS_COLOR, OBSTACLE_COLOR,
                     SEEN_COLOR, SLIDER_HANDLE_COLOR, TEXT_COLOR, UNSEEN_COLOR)
from text import drawText
from fontDict import fonts

# ---------------- Configuration
FPS = 60
SCALE_DOWN_FACTOR = 2   # draw at 1/2 resolution and scale up, as in GridTesting
PADDING = 20            # px (in scaled-down space) kept clear around each grid
TITLE_H = 16            # px above each grid for its title
BAR_H = 36              # px at the bottom for the buttons
TILE_GAP = 1            # px between cells, which is what makes the grid visible
STEPS_PER_SECOND = 40   # auto-step speed while playing
ALLOCATION_OPACITY = 0.6   # middle grid: how strongly a drone's colour tints its cells
VISIT_OUTLINE = 2          # middle grid: thickness in scaled-down px of the outline a visiting drone leaves on a cell
MAX_OBSTACLE_RATE = 0.2    # right-hand end of the obstacle slider
OBSTACLE_RATE_STEP = 0.005 # the slider moves in steps of this size (0.5%)
MAX_CLUSTER_WEIGHT = 10.0  # right-hand end of the clustering slider; its left-hand end is 1x (no clustering)
CLUSTER_WEIGHT_STEP = 0.5
MAX_OBSTACLE_ROUNDS = 8    # right-hand end of the rounds slider; its left-hand end is 1 round
HOLD_DELAY = 0.2           # seconds a step key or button must be held before it starts repeating
TRAIL_SUPERSAMPLE = 2      # trails and drones are drawn this many times larger, then shrunk, to smooth their edges
TRAIL_WIDTH = 1.5          # trail thickness, in scaled-down px
TRAIL_OPACITY = 0.55       # how strongly a trail shows over the grid


def blend(base, over, opacity):
    return [int(base[i] + (over[i] - base[i]) * opacity) for i in range(3)]


# ---------------- Cell colours, one function per grid
def drone_view_color(sim, x, y):
    """Left grid: lighter where a drone has seen the cell, purple for obstacles the drones have found."""
    cell = sim.liveMap.cells[y][x]
    if cell.isObstacle:
        return OBSTACLE_COLOR
    return SEEN_COLOR if cell.searched else UNSEEN_COLOR


def allocation_color(sim, x, y):
    """Middle grid: each cell tinted with the colour of the drone it is assigned to.

    Cells found to be obstacles are assigned to nobody and are left blank, so
    they cannot be mistaken for free cells a drone skipped.
    """
    index = sim.liveMap.cells[y][x].droneAllocationIndex
    if index is None:
        return BACKGROUND_COLOR
    return blend(UNSEEN_COLOR, sim.drones[index].color, ALLOCATION_OPACITY)


def obstacles_color(sim, x, y):
    """Right grid: every obstacle on the real map, whether or not a drone has seen it."""
    return OBSTACLE_COLOR if sim.world.cells[y][x].isObstacle else UNSEEN_COLOR


def draw_visit_outlines(surface, sim, x, y, rect):
    """Middle grid: one outline per drone that has flown over the cell, in that drone's colour.

    The first visitor's outline is outermost and later ones nest inside it.
    With three or more the outlines thin out so they all still fit.
    """
    visitedBy = sim.liveMap.cells[y][x].visitedBy
    if not visitedBy:
        return
    thickness = VISIT_OUTLINE if len(visitedBy) <= 2 else max(1, rect.width // (2 * len(visitedBy) + 1))
    for i, index in enumerate(visitedBy):
        ring = rect.inflate(-2 * i * thickness, -2 * i * thickness)
        if ring.width <= 0:
            break
        pygame.draw.rect(surface, sim.drones[index].color, ring, thickness if ring.width > 2 * thickness else 0)


# (title, cell colour function, optional function that draws extra detail on top of each cell)
VIEWS = [
    ("Drone view", drone_view_color, None),
    ("Allocation", allocation_color, draw_visit_outlines),
    ("Obstacles", obstacles_color, None),
]


def layout_grids(surface_size, grid_w, grid_h, count):
    """Splits the surface into equal-width columns, one per grid, left to right.

    Returns the shared tile size and the top-left position of each grid,
    leaving room for titles above and the button bar below.
    """
    column_w = surface_size[0] // count
    avail_w = column_w - PADDING * 2
    avail_h = surface_size[1] - PADDING * 2 - TITLE_H - BAR_H
    tile_size = int(max(2, min(avail_w / grid_w, avail_h / grid_h)))
    w, h = grid_w * tile_size, grid_h * tile_size
    top = PADDING + TITLE_H + (avail_h - h) // 2
    return tile_size, [(i * column_w + (column_w - w) // 2, top) for i in range(count)]


def draw_grids(surface, sim, tile_size, positions, font):
    """Draws all three grids and their titles at the scaled-down resolution."""
    surface.fill(BACKGROUND_COLOR)
    fill = tile_size - TILE_GAP
    width, height = sim.world.width, sim.world.height
    for (title, color_fn, detail_fn), (ox, oy) in zip(VIEWS, positions):
        drawText(surface, TEXT_COLOR, font, ox + width * tile_size / 2, oy - TITLE_H, title, justify="center")
        for y in range(height):
            for x in range(width):
                rect = pygame.Rect(ox + x * tile_size, oy + y * tile_size, fill, fill)
                pygame.draw.rect(surface, color_fn(sim, x, y), rect)
                if detail_fn:
                    detail_fn(surface, sim, x, y, rect)


def draw_drones(surface, layer, trail_layer, sim, tile_size, position):
    """Draws trails and drones over the left grid of the full-size surface.

    The grid is copied onto `layer` enlarged, the trails and drones are drawn
    on top, and the result is shrunk back with smoothing. That anti-aliases
    them so every curved corner looks the same. Both layers are opaque on
    purpose: a per-pixel transparent one comes out solid white on the macOS
    display. Trails get their opacity by drawing them on a second copy of the
    grid and fading that whole copy over the first.
    """
    scale = SCALE_DOWN_FACTOR * TRAIL_SUPERSAMPLE
    size = (layer.get_width() // TRAIL_SUPERSAMPLE, layer.get_height() // TRAIL_SUPERSAMPLE)
    area = pygame.Rect((position[0] * SCALE_DOWN_FACTOR, position[1] * SCALE_DOWN_FACTOR), size)
    pygame.transform.scale(surface.subsurface(area), layer.get_size(), layer)
    # Cell centres are offset by half the gap, since each cell is drawn one gap short of its tile
    origin = (-TILE_GAP / 2 * scale, -TILE_GAP / 2 * scale)
    trail_layer.blit(layer, (0, 0))
    for drone in sim.drones:
        drone.draw_trail(trail_layer, origin, tile_size * scale, width=round(TRAIL_WIDTH * scale))
    trail_layer.set_alpha(round(TRAIL_OPACITY * 255))
    layer.blit(trail_layer, (0, 0))
    for drone in sim.drones:
        drone.draw(layer, origin, tile_size * scale)
    surface.blit(pygame.transform.smoothscale(layer, size), area)


class Button:
    def __init__(self, rect, label):
        self.rect = pygame.Rect(rect)
        self.label = label

    def hit(self, pos):
        return self.rect.collidepoint(pos)

    def draw(self, surface, font, mouse_pos):
        pygame.draw.rect(surface, BUTTON_HOVER_COLOR if self.hit(mouse_pos) else BUTTON_COLOR, self.rect, 0, 3)
        pygame.draw.rect(surface, BUTTON_HOVER_COLOR, self.rect, 1, 3)
        drawText(surface, TEXT_COLOR, font, self.rect.centerx, self.rect.centery - font.get_height() / 2, self.label, justify="center")


class Slider:
    """Horizontal slider. `value` runs from min_value at the left end to max_value at the right, in multiples of step.

    `apply(value)` is called while dragging and returns True if the value changed something;
    `label()` gives the text shown to the right of the slider.
    """

    def __init__(self, rect, value, min_value, max_value, step, apply, label):
        self.rect = pygame.Rect(rect)
        self.value = value
        self.min_value = min_value
        self.max_value = max_value
        self.step = step
        self.apply = apply
        self.label = label
        self.dragging = False

    def hit(self, pos):
        return self.rect.inflate(8, 8).collidepoint(pos)

    def drag_to(self, x):
        """Moves the handle to pixel x. Returns the new value."""
        fraction = max(0.0, min(1.0, (x - self.rect.left) / self.rect.width))
        span = self.max_value - self.min_value
        self.value = self.min_value + round(fraction * span / self.step) * self.step
        return self.value

    def draw(self, surface):
        track_y = self.rect.centery
        handle_x = self.rect.left + self.rect.width * (self.value - self.min_value) / (self.max_value - self.min_value)
        pygame.draw.line(surface, BUTTON_HOVER_COLOR, (self.rect.left, track_y), (self.rect.right, track_y), 2)
        pygame.draw.line(surface, TEXT_COLOR, (self.rect.left, track_y), (handle_x, track_y), 2)
        pygame.draw.circle(surface, SLIDER_HANDLE_COLOR, (handle_x, track_y), 4)


def main():
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    clock = pygame.time.Clock()

    screen_width = int(screen.get_width() / SCALE_DOWN_FACTOR)
    screen_height = int(screen.get_height() / SCALE_DOWN_FACTOR)
    screen2 = pygame.Surface((screen_width, screen_height)).convert()
    screen2_scaled = pygame.Surface(screen.get_size()).convert()  # full-size copy, rebuilt only when the sim changes
    screenUI = pygame.Surface((screen_width, screen_height)).convert_alpha()
    font_regular = fonts[f"regular{int(25 / (SCALE_DOWN_FACTOR ** (1 / 1.5)))}"]
    font_small = fonts[f"regular{int(18 / (SCALE_DOWN_FACTOR ** (1 / 1.5)))}"]

    sim = Simulation(keepHistory=True)
    tile_size, positions = layout_grids((screen_width, screen_height), sim.world.width, sim.world.height, len(VIEWS))
    drone_layer_scale = tile_size * SCALE_DOWN_FACTOR * TRAIL_SUPERSAMPLE
    drone_layer = pygame.Surface((sim.world.width * drone_layer_scale, sim.world.height * drone_layer_scale)).convert()
    trail_layer = drone_layer.copy()

    # ---------------- Bottom bar: Play, Back, Step, Reset, Randomize, the three obstacle sliders, then the tick counter
    button_h, gap = 20, 6
    widths = [44, 44, 44, 46, 74]
    slider_w, status_w = 56, 60
    # (start value, left end, right end, step, what to call while dragging, label text, label width)
    slider_specs = [
        (sim.obstacleRate, 0.0, MAX_OBSTACLE_RATE, OBSTACLE_RATE_STEP, sim.set_obstacle_rate,
         lambda: f"obstacles {sim.obstacleRate * 100:.1f}%", 90),
        (sim.clusterWeight, 1.0, MAX_CLUSTER_WEIGHT, CLUSTER_WEIGHT_STEP, sim.set_cluster_weight,
         lambda: f"clustering {sim.clusterWeight:.1f}x", 82),
        (sim.obstacleRounds, 1, MAX_OBSTACLE_ROUNDS, 1, sim.set_obstacle_rounds,
         lambda: f"rounds {sim.obstacleRounds}", 64),
    ]
    bar_w = sum(widths) + gap * len(widths) + sum(slider_w + gap + 10 + spec[-1] for spec in slider_specs) + status_w
    x = (screen_width - bar_w) // 2
    bar_y = screen_height - BAR_H + (BAR_H - button_h) // 2 - 4
    buttons = []
    for label, w in zip(["Play", "Back", "Step", "Reset", "Randomize"], widths):
        buttons.append(Button((x, bar_y, w, button_h), label))
        x += w + gap
    play_button, back_button, step_button, reset_button, randomize_button = buttons
    sliders = []  # (slider, x of its label)
    for value, low, high, step, apply, label, label_w in slider_specs:
        slider = Slider((x + 6, bar_y, slider_w, button_h), value, low, high, step, apply, label)
        sliders.append((slider, slider.rect.right + gap + 2))
        x = slider.rect.right + gap + 4 + label_w
    status_x = x
    text_y = bar_y + (button_h - font_small.get_height()) / 2

    pygame.mouse.set_visible(False)
    playing = False
    step_timer = 0.0
    held_action = None  # sim.step or sim.step_back while its key or button is held down
    hold_time = 0.0     # how long it has been held
    drawn_version = None
    last_time = time.time()

    running = True
    while running:
        now = time.time()
        dt = now - last_time
        last_time = now

        mx, my = pygame.mouse.get_pos()
        mx, my = mx / SCALE_DOWN_FACTOR, my / SCALE_DOWN_FACTOR

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE and playing:
                    playing = False
                elif event.key in (pygame.K_SPACE, pygame.K_RIGHT, pygame.K_LEFT):
                    playing = False
                    held_action = sim.step_back if event.key == pygame.K_LEFT else sim.step
                    held_action()
                    hold_time, step_timer = 0.0, 0.0
            if event.type == pygame.KEYUP and event.key in (pygame.K_SPACE, pygame.K_RIGHT, pygame.K_LEFT):
                held_action = None
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if play_button.hit((mx, my)):
                    playing = not playing and not sim.finished()
                    step_timer = 0.0
                elif step_button.hit((mx, my)) or back_button.hit((mx, my)):
                    playing = False
                    held_action = sim.step if step_button.hit((mx, my)) else sim.step_back
                    held_action()
                    hold_time, step_timer = 0.0, 0.0
                elif reset_button.hit((mx, my)):
                    sim.reset()
                    playing = False
                elif randomize_button.hit((mx, my)):
                    sim.randomize()
                    playing = False
                else:
                    for slider, _ in sliders:
                        slider.dragging = slider.hit((mx, my))
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                for slider, _ in sliders:
                    slider.dragging = False
                held_action = None

        # Dragging a slider changes the obstacles, which puts the drones back at the start
        for slider, _ in sliders:
            if slider.dragging and slider.apply(slider.drag_to(mx)):
                playing = False

        # ---------------- Simulation: auto-step while playing, or repeat a step once it has been held long enough
        if held_action:
            hold_time += dt
        if playing or (held_action and hold_time >= HOLD_DELAY):
            step_timer += dt
            while step_timer >= 1 / STEPS_PER_SECOND:
                step_timer -= 1 / STEPS_PER_SECOND
                (held_action or sim.step)()
            if sim.finished():
                playing = False
        play_button.label = "Pause" if playing else "Play"

        # ---------------- Grids: redraw and rescale only when the sim changed
        version = (sim.version, sim.world.version, sim.liveMap.version)
        if version != drawn_version:
            draw_grids(screen2, sim, tile_size, positions, font_small)
            pygame.transform.scale(screen2, screen.get_size(), screen2_scaled)
            draw_drones(screen2_scaled, drone_layer, trail_layer, sim, tile_size, positions[0])
            drawn_version = version

        # ---------------- UI overlay (redrawn every frame)
        screenUI.fill((0, 0, 0, 0))
        for button in buttons:
            button.draw(screenUI, font_small, (mx, my))
        for slider, label_x in sliders:
            slider.draw(screenUI)
            drawText(screenUI, TEXT_COLOR, font_small, label_x, text_y, slider.label())
        drawText(screenUI, TEXT_COLOR, font_small, status_x, text_y, f"tick {sim.tick}" + ("  done" if sim.finished() else ""))
        drawText(screenUI, FPS_COLOR, font_regular, 5, screen_height - 30 / (SCALE_DOWN_FACTOR ** (1 / 1.8)),
                 str(round(clock.get_fps())), BACKGROUND_COLOR, 1, antiAliasing=False)
        pygame.draw.circle(screenUI, BACKGROUND_COLOR, (mx + 1, my + 1), 2, 1)
        pygame.draw.circle(screenUI, CURSOR_COLOR, (mx, my), 2, 1)

        screen.blit(screen2_scaled, (0, 0))
        screen.blit(pygame.transform.scale(screenUI, screen.get_size()), (0, 0))

        pygame.display.update()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
