# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

from importlib.metadata import version as get_version

project = "Pion"
copyright = "2025, Michael Solovev"
author = "Michael Solovev"
release: str = get_version("pionsdk")

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

pygments_style = "sphinx"

extensions = [
    "sphinx.ext.autodoc",
]

templates_path = ["_templates"]
exclude_patterns = []

language = "ru"

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "furo"
html_static_path = ["_static"]
