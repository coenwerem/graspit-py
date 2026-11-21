"""Sphinx configuration for the graspit-py manual."""

from markupsafe import Markup

project = "graspit-py"
extensions = ["myst_parser"]
exclude_patterns = ["_build", ".DS_Store"]
myst_enable_extensions = ["dollarmath"]
nitpicky = True

html_theme = "shibuya"
html_title = "graspit-py Documentation"
html_theme_options = {
    "globaltoc_expand_depth": 1,
    "show_ai_links": False,
    "color_mode": "auto",
    "nav_links": [
        {"title": "GitHub", "url": "https://github.com/coenwerem/graspit-py"},
    ],
}
html_show_sourcelink = False
html_static_path = ["_static"]
html_favicon = "_static/favicon.svg"
html_css_files = ["hand-gallery.css"]

html_context = {
    "copyright": Markup('2026, <a href="https://clintonenwerem.com/">Clinton Enwerem</a>'),
}
