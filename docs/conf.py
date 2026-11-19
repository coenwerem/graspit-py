"""Sphinx configuration for the GraspIt! Python wrapper manual."""

project = "GraspIt! Python Wrapper"
extensions = ["myst_parser"]
exclude_patterns = ["_build", ".DS_Store"]
myst_enable_extensions = ["dollarmath"]
nitpicky = True

html_theme = "furo"
html_title = project
html_show_sourcelink = False
html_static_path = ["_static"]
html_css_files = ["hand-gallery.css"]

html_theme_options = {
    "footer_icons": [
        {
            "name": "Clinton Enwerem’s Homepage",
            "url": "https://clintonenwerem.com/",
            "html": "Clinton Enwerem",
            "class": "author-homepage-link",
        },
    ],
}
