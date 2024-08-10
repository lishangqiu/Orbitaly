"""The console's theme: a config key that has to reach <html data-theme>.

The palette itself lives in style.css, but the wiring is server-side — the
page is rewritten per request — so it is checked here rather than by eye.
"""
import re

import pytest
from fastapi.testclient import TestClient

from orbitaly.app import create_app
from orbitaly.config import Config, load_config


def serve(theme=None):
    config = Config()
    config.tle.sources = []
    config.tle.cache_path = "/nonexistent/ignore.json"
    if theme is not None:
        config.ui.theme = theme
    return TestClient(create_app(config))


def html_tag(body):
    match = re.search(r"<html[^>]*>", body)
    assert match, "the page must still have an <html> tag"
    return match.group(0)


def test_the_console_is_dark_unless_asked_otherwise():
    with serve() as client:
        assert 'data-theme="dark"' in html_tag(client.get("/").text)


def test_a_light_station_gets_the_light_palette_in_the_first_response():
    """Stamped server-side: no dark flash before a script can fix it up."""
    with serve("light") as client:
        body = client.get("/").text
        assert 'data-theme="light"' in html_tag(body)
        assert "dark" not in html_tag(body)
        # Only the <html> attribute is rewritten, not every "dark" in the file.
        assert "<canvas id=\"worldmap\">" in body


def test_index_html_is_themed_too_not_just_the_bare_root():
    with serve("light") as client:
        assert 'data-theme="light"' in html_tag(client.get("/index.html").text)


def test_the_stylesheet_defines_the_light_palette():
    """The attribute is inert without a rule that keys off it."""
    with serve("light") as client:
        assert ':root[data-theme="light"]' in client.get("/style.css").text


def test_an_unknown_theme_is_a_config_error(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("ui:\n  theme: solarized\n")
    with pytest.raises(ValueError, match="ui.theme"):
        load_config(path)


def test_a_known_theme_loads(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("ui:\n  theme: light\n")
    assert load_config(path).ui.theme == "light"
