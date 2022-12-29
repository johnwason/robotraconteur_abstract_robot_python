# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'robotraconteur_abstract_robot'
copyright = '2022, Wason Technology LLC'
author = 'John Wason'

import robotraconteur_abstract_robot

# The short X.Y version.
# version = abb_motion_program_exec.__version__
# The full version, including alpha/beta/rc tags.
# release = abb_motion_program_exec.__version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
     "sphinx.ext.autodoc",
     "sphinx_autodoc_typehints",
     "recommonmark",
     "sphinx_rst_builder"
]

source_suffix = [".rst", ".md"]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ['_static']

