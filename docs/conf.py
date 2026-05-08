# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import sys, os
sys.path.insert(0, os.path.abspath('..'))  # adjust to where your scripts live

project = 'lar'
copyright = '2026, Ondřej Vosáhlo, Václav Kučera, Jakub Kecek'
author = 'Ondřej Vosáhlo, Václav Kučera, Jakub Kecek'
release = '8.5.2026'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',   # pulls docstrings automatically
]
autodoc_mock_imports = [
    "numpy",
    "robolab_turtlebot",
    "scipy",
    "cv2"
]
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
