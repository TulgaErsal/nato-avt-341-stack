# NATO AVT-341 Autonomy Stack

ROS package with autonomy algorithms for the NATO AVT-341.

## Documentation

Latest documentation can be found here:
  - [Website](https://d1nxz9z3nv7fn1.cloudfront.net)
  - [PDF](https://d1nxz9z3nv7fn1.cloudfront.net/nato-avt-341-stack.pdf)

> [!NOTE]
> Documentation is automatically built and published by a Github Action whenever contents under `./docs` changes in the `main` branch.

### Building and editing documentation locally

To build the documentation from source, you will need:

* A working Doxygen binary in your `PATH` (on Ubuntu, `sudo apt install -y
  doxygen`).
* A Python dependency manager, i.e. `Poetry` (see [Build using
  Poetry](#build-using-poetry)) or `pip` (see [Build using
  pip](#build-using-pip))

#### Build using Poetry

> This method assumes you have a working [Poetry](https://python-poetry.org/)
> installation. Follow the [installation
> instructions](https://python-poetry.org/docs/#installation) for your platform
> to get Poetry running on your system.

From the root of the repository, install and activate the Poetry environment for
the current session:

```shell
poetry install --with documentation
poetry shell
```

Change directory to the `docs` folder and run the Sphinx Makefile with `cd docs
&& make html`.

Before sourcing your ROS distribution to build and/or run the stack, make sure
the documentation environment is deactivated by exiting the Poetry shell with
`exit`.

#### Build using pip

Then install the required Python dependencies by issuing the following commands
from the root of the repository:

```shell
python -m venv docs/.nato-avt-341-docs-env
source docs/.nato-avt-341-docs-env/bin/activate
pip install -r requirements.txt
```

Finally, run the build process by running `cd docs && make html`.

---

Once the build process is complete, the documentation will be available under
`build/html/index.html`.

#### Editing documentation with live updates

Run `sphinx-autobuild source build` in the `docs` folder to receive a live html updates when editing the documentation.  

> [!WARNING]
> Before sourcing your ROS distribution to build and/or run the stack, make sure
> the documentation environment is deactivated (`exit` for Poetry, `deactivate`
> for pip).

## Acknowledgements

This project is made possible by technical and financial support of the
Mississippi State University Center for Advanced Vehicular Systems as well as
the Automotive Research Center (ARC) in accordance with Cooperative Agreement
W56HZV 14 2 0001 U.S. Army CCDC Ground Vehicle Systems Center (GVSC) Warren, MI.
