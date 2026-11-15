"""The project's colours, as [r, g, b] lists, plus what each one is used for."""


def _rgb(hex_code):
    hex_code = hex_code.lstrip("#")
    return [int(hex_code[i:i + 2], 16) for i in (0, 2, 4)]


class Palette:
    """The five colours from the bottom row of orbs in the Super Plum 21 palette."""
    magenta = _rgb("#A63A82")
    rust = _rgb("#A64B4B")
    teal = _rgb("#15788C")
    blue = _rgb("#266EFF")
    purple = _rgb("#6F66CC")


class Neutral:
    """Greys and dark blues for everything that is not a drone or an obstacle."""
    background = [16, 18, 21]
    cell = [32, 36, 46]
    cell_light = [56, 63, 82]
    grey_blue = [66, 76, 110]
    text = [200, 200, 200]
    white = [255, 255, 255]
    pink = [255, 96, 141]


BACKGROUND_COLOR = Neutral.background  # behind everything, and the gaps between cells
UNSEEN_COLOR = Neutral.cell            # default cell
SEEN_COLOR = Neutral.cell_light        # cell a drone has seen
OBSTACLE_COLOR = Palette.purple
TEXT_COLOR = Neutral.text
FPS_COLOR = Neutral.pink
CURSOR_COLOR = Neutral.white
SLIDER_HANDLE_COLOR = Neutral.text
BUTTON_COLOR = Neutral.cell
BUTTON_HOVER_COLOR = Neutral.grey_blue

# Ordered so that drones in neighbouring strips contrast with each other
DRONE_COLORS = [Palette.magenta, Palette.teal, Palette.rust, Palette.blue]
