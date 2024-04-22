import subprocess

def get_git_revision_short_hash() -> str:
    """Get the short hash of the Git repository in the parent folder."""
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd="../").decode('ascii').strip()

# General
project = "NATO AVT-341 Autonomy Stack"
copyright = "2024, NATO Applied Vehicle Technology (AVT)"
author = "The AVT-341 Autonomy Stack Developers Team"
version = "main"
release = get_git_revision_short_hash()
extensions = ["breathe"]
templates_path = ["_templates"]
exclude_patterns = []

# HTML build
html_theme = "pydata_sphinx_theme"
html_theme_options = {
    "navigation_with_keys": False,
}
html_static_path = ["_static"]
html_logo = "_static/img/logo.png"
html_css_files = [
    "css/custom.css",
]
html_show_sphinx = False

# LaTeX PDF build
latex_elements = {
    'preamble': r'\usepackage{nato-theme}',
    'extraclassoptions': 'openany,oneside',
    "releasename": "Commit hash",
}
latex_additional_files = ["_static/latex/nato-theme.sty"]
latex_authors = r"Tulga Ersal \and Christopher T. Goodin \and Dario Sirangelo"
latex_documents = [("index", "nato-avt-341-stack.tex", project, latex_authors, "manual")]
pdf_documents = [("index", "nato-avt-341-stack", "title", latex_authors)]

# Breathe extension
breathe_projects = {"nato-avt-341-stack": "../build/xml"}
breathe_default_project = "nato-avt-341-stack"
breathe_default_members = (
    "members",
    "protected-members",
    "private-members",
    "undoc-members",
)
