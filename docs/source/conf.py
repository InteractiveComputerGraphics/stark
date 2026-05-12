# Configuration file for the Sphinx documentation builder.
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
project = 'Stark'
copyright = '2026, Jose Antonio Fernandez-Fernandez'
author = 'Jose Antonio Fernandez-Fernandez'

# -- General configuration ---------------------------------------------------
extensions = []

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
html_static_path = ['_static', '../images']

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

# Enable MyST extensions
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "dollarmath",
    "amsmath",
]

# -- HTML output -------------------------------------------------------------
html_theme = "furo"

# Sidebar logo – replace with a branded Stark logo when available
# html_logo = "_static/stark_logo.png"
html_title = "Stark docs"
