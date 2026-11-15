"""The project's colours, as [r, g, b] lists, plus what each one is used for."""


MUTE = 0.25  # how far the drone and obstacle colours are pulled towards grey (0 = full colour, 1 = fully grey)


def _muted(r, g, b):
    grey = 0.299 * r + 0.587 * g + 0.114 * b  # perceived brightness, so muting keeps each colour as light as it was
    return [round(c + (grey - c) * MUTE) for c in (r, g, b)]


class Palette:
    """Drone and obstacle colours, slightly desaturated."""
    pink = _muted(255, 96, 141)
    amber = _muted(237, 171, 80)
    green = _muted(99, 199, 77)
    cyan = _muted(44, 232, 245)
    purple = _muted(150, 90, 220)


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

DRONE_COLORS = [Palette.pink, Palette.amber, Palette.green, Palette.cyan]
