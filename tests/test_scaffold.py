def test_theme_constants_are_strings():
    from ground_station import theme

    for name in ("BG", "PANEL", "BORDER", "TEXT", "TEXT_DIM", "ACCENT", "OK", "OFF",
                 "FONT_FAMILY", "MONO_FONT_FAMILY"):
        assert isinstance(getattr(theme, name), str)
        assert getattr(theme, name) != ""
