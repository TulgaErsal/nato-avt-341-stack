project = "NATO AVT-341 Autonomy Stack"
copyright = "2024, NATO AVT-341 Stack Developers"
author = "NATO AVT-341 Stack Developers"
release = "2024-02-26"
extensions = ["breathe"]
templates_path = ["_templates"]
exclude_patterns = []
html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_logo = "_static/img/logo.png"
html_css_files = [
    "css/custom.css",
]
html_show_sphinx = False
breathe_projects = {"nato-avt-341-stack": "../build/xml"}
breathe_default_project = "nato-avt-341-stack"
breathe_default_members = (
    "members",
    "protected-members",
    "private-members",
    "undoc-members",
)
