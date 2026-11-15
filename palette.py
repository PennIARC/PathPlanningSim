"""The project's colour palette, as [r, g, b] lists, plus what each colour is used for."""


def _rgb(hex_code):
    hex_code = hex_code.lstrip("#")
    return [int(hex_code[i:i + 2], 16) for i in (0, 2, 4)]


class Palette:
    teal = _rgb("#3F8985")
    teal_mid = _rgb("#29727B")
    teal_dark = _rgb("#1B576F")
    blue = _rgb("#0B3C60")
    navy = _rgb("#051D48")
    navy_dark = _rgb("#010434")
    purple_dark = _rgb("#1C0728")
    purple = _rgb("#4B0A2E")
    maroon_dark = _rgb("#512431")
    maroon = _rgb("#723033")
    red = _rgb("#B73A29")
    orange = _rgb("#BD6C43")


BACKGROUND_COLOR = Palette.navy_dark   # behind everything, and the gaps between cells
UNSEEN_COLOR = Palette.navy            # default cell
SEEN_COLOR = Palette.blue              # cell a drone has seen
OBSTACLE_COLOR = Palette.purple
TEXT_COLOR = Palette.teal
ACCENT_COLOR = Palette.orange          # FPS counter, cursor, slider handle
BUTTON_COLOR = Palette.navy
BUTTON_HOVER_COLOR = Palette.teal_dark

# Ordered so that drones in neighbouring strips contrast with each other
DRONE_COLORS = [Palette.red, Palette.teal, Palette.maroon, Palette.orange]
