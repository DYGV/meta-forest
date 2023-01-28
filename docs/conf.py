# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import sys
import sphinx_material
sys.path.insert(0, os.path.abspath('../'))
import meta_forest
project = 'meta-FOrEST'
copyright = '2023, Eisuke Okazaki'
author = 'Eisuke Okazaki'
release = 'v0.3.2'

extensions = [
    "sphinx.ext.doctest",
    "sphinx.ext.intersphinx",
    "sphinx.ext.mathjax",
    "sphinx.ext.napoleon",
    "sphinx.ext.autosummary",
    "sphinx.ext.viewcode",
    "sphinx_copybutton",
    "sphinx_material",
    'myst_parser'
]
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

html_title = "meta-FOrEST documentation"
html_theme = 'sphinx_rtd_theme'
html_show_sourcelink = True

templates_path = ['_templates']

autodoc_mock_imports = ["importlib"]
language = 'en'

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'templates/']

autodoc_default_options = {"members": True, "undoc-members": True, "private-members": True}


html_theme_path = sphinx_material.html_theme_path()
html_context = sphinx_material.get_html_context()

html_static_path = ['_static']

#autoclass_content = 'both'
#master_doc = 'index'

version="v0.3.2"

html_sidebars = {
    "**": ["logo-text.html", "globaltoc.html", "localtoc.html", "searchbox.html", "globaltoc.html"]
}
