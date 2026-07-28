import subprocess
import sys

from pathlib import Path

sys.path.append(Path('../../avt_341/avt_341'))


def get_git_revision_short_hash() -> str:
    """Get the short hash of the Git repository in the parent folder."""
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'],
                                   cwd="../").decode('ascii').strip()


# General
project = "NATO AVT-341 Autonomy Stack"
copyright = "2024, NATO Applied Vehicle Technology (AVT)"
author = "The AVT-341 Autonomy Stack Developers Team"
version = "main"
release = get_git_revision_short_hash()
extensions = [
    "breathe", "sphinx.ext.autodoc", "sphinx_autodoc_typehints",
    "sphinx.ext.napoleon", "sphinx.ext.todo"
]
templates_path = ["_templates"]
exclude_patterns = []
numfig = True

# HTML build
html_theme = "pydata_sphinx_theme"
html_theme_options = {
    "navigation_with_keys": False,
    "show_nav_level": 2,
    "show_toc_level": 3
}
html_static_path = ["_static"]
html_logo = "_static/img/logo.png"
html_css_files = [
    "css/custom.css",
]
html_show_sphinx = False

# LaTeX PDF build
latex_engine = "pdflatex"
latex_elements = {
    'passoptionstopackages': r'\PassOptionsToPackage{table}{xcolor}',
    "preamble": r"\usepackage{nato-theme}",
    'pointsize':'10pt',
    "papersize": "a4paper",
    "extraclassoptions": "openany,oneside",
    "releasename": "Commit hash",
}
latex_additional_files = ["_static/latex/nato-theme.sty"]
latex_authors = r"Tulga Ersal \and Christopher T. Goodin \and Dario Sirangelo"
latex_documents = [("index", "nato-avt-341-stack.tex", project, latex_authors,
                    "manual")]
pdf_documents = [("index", "nato-avt-341-stack", "title", latex_authors)]

# Autodoc extension
autodoc_default_options = {
    "members": True,
    "private-members": True,
    "special-members": "__init__",
    "show-inheritance": True,
    "member-order": "bysource",
}

# Autodoc typehints extension
always_document_param_types = True

# Breathe extension
breathe_projects = {"nato-avt-341-stack": "../build/xml"}
breathe_default_project = "nato-avt-341-stack"
breathe_default_members = (
    "members",
    "protected-members",
    "private-members",
)

# Napoleon extension
napoleon_google_docstring = True
napoleon_use_rtype = True

# Todo extension
todo_include_todos = True
