# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'STARK'
copyright = '2026, Jose Antonio Fernandez-Fernandez'
author = 'Jose Antonio Fernandez-Fernandez'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_static_path = ['_static']

# -- General configuration ---------------------------------------------------
extensions += [
    "myst_parser",            # enables Markdown
    "sphinx.ext.mathjax",     # recommended for HTML
    "sphinxcontrib.mermaid",  # Mermaid diagram rendering
]

# mermaid (diagrams) config
mermaid_init_config = {
    "startOnLoad": False,
    "themeVariables": {
        "fontSize": "40px",
    },
    "flowchart": {
        "curve": "linear",
        "nodeSpacing": 80,
        "rankSpacing": 90,
    },
}



# Let Sphinx read both .md and .rst
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# Enable a few extra MyST goodies (fenced-colon & definition lists)
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "dollarmath",
    "amsmath",
]

# -- HTML output -------------------------------------------------------------
html_theme = "furo"
# html_css_files = ["custom.css"]

# Logo in the sidebar
html_logo = "_static/stark250.png"
html_title = "STARK docs"      # title tag & window caption


# -- Pygments (syntax highlighting) -------------------------------------------------------------
# Make Pygments output stable classes; colors will come from CSS.
# pygments_style = "sphinx"


# -- Pygments lexer -------------------------------------------------------------
# import os, sys
# sys.path.append(os.path.abspath("_ext"))

# from symx_pygments import SymXCppLexer

# def setup(app):
#     app.add_lexer("cpp", SymXCppLexer)
#     app.add_lexer("c++", SymXCppLexer)  # optional alias

# from pygments_vs_code_palette import VSCodeDarkPlusStyle

# pygments_style = "default"            # for light mode, or make your own light style too
# pygments_dark_style = VSCodeDarkPlusStyle    


# -- Pygments (syntax highlighting) ------------------------------------------
import os, sys, types
sys.path.append(os.path.abspath("_ext"))

from pygments_vs_code_palette import VSCodeDarkPlusStyle

modname = "pygments.styles.vscode_dark_plus"
m = types.ModuleType(modname)

# Pygments expects this exact class name for style "vscode_dark_plus"
m.Vscode_Dark_PlusStyle = VSCodeDarkPlusStyle

# (Optional) keep your original class name too
m.VSCodeDarkPlusStyle = VSCodeDarkPlusStyle

sys.modules[modname] = m

pygments_style = "github-light"
pygments_dark_style = "vscode_dark_plus"

# --- Custom lexer (your types/functions filter) ---
from symx_pygments import SymXCppLexer

def setup(app):
    app.add_lexer("cpp", SymXCppLexer)
    app.add_lexer("c++", SymXCppLexer)

