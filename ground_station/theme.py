"""Shared visual constants, matching the approved ground-station wireframe."""

BG = "#14171c"
PANEL = "#1c2028"
BORDER = "#2f3542"
TEXT = "#e6e8ec"
TEXT_DIM = "#7d8590"
ACCENT = "#e8823c"
OK = "#3fb950"
OFF = "#545d6b"
# The one colour that means something went wrong rather than something
# needs watching (ACCENT). Used for a refused or failed map command.
BAD = "#f85149"

# Hover/pressed variants of PANEL, for the shared button style below.
PANEL_LIGHT = "#232833"
PANEL_DARK = "#171b22"

FONT_FAMILY = "IBM Plex Sans"
MONO_FONT_FAMILY = "IBM Plex Mono"

FONT_SIZE_SMALL = 11
FONT_SIZE_BODY = 13
FONT_SIZE_TITLE = 15


def card_style() -> str:
    """Shared card look: panel background, 1px border, rounded corners,
    consistent internal padding. Applied to the video panel, MAP row,
    DRIVE row, the DRIVE/TWIST card and the node list, so the dashboard
    reads as a stack of cards rather than flat rows."""
    return (
        f"background-color: {PANEL}; border: 1px solid {BORDER}; "
        f"border-radius: 8px; padding: 12px;"
    )


def section_title_style() -> str:
    """Shared section-title look (CAMERA/…, DRIVE/TWIST, SYSTEM NODES):
    small, letter-spaced, semi-bold, dimmed, uppercase."""
    return (
        f"color: {TEXT_DIM}; font-size: {FONT_SIZE_SMALL}px; font-weight: 600; "
        f"letter-spacing: 1px; border: none; background: transparent;"
    )


def button_style() -> str:
    """Shared button look: filled panel, 1px border, rounded corners,
    hover slightly lighter, pressed slightly darker, disabled dimmed."""
    return (
        f"QPushButton {{ background-color: {PANEL}; color: {TEXT}; "
        f"border: 1px solid {BORDER}; border-radius: 6px; padding: 6px 14px; }} "
        f"QPushButton:hover {{ background-color: {PANEL_LIGHT}; }} "
        f"QPushButton:pressed {{ background-color: {PANEL_DARK}; }} "
        f"QPushButton:disabled {{ color: {TEXT_DIM}; border-color: {PANEL_DARK}; }}"
    )


def pill_style(bg: str, fg: str = TEXT) -> str:
    """A small rounded status pill: a coloured background chip carrying
    plain text (no rich text needed inside a pill)."""
    return (
        f"background-color: {bg}; color: {fg}; border: none; "
        f"border-radius: 8px; padding: 2px 10px; font-weight: 600; "
        f"font-size: {FONT_SIZE_SMALL}px;"
    )
