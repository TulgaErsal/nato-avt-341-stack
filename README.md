# NATO AVT-341 Autonomy Stack

ROS package with autonomy algorithms for the NATO AVT-341.

## Documentation

### Accessing the latest release

The latest released documentation in PDF format can be accessed [at this
link](https://www.dropbox.com/scl/fi/swf6yi9j3yf84bh7go59j/nato-avt-341-stack.pdf?rlkey=2vm9q1yyanwebyf3sjnyc7ku7&e=1&st=sw1z7q8h&dl=0).

> [!NOTE]
> The features documented in the PDF version of the documentation may differ
> from the ones available in the latest commit. Refer to the title page for the
> hash of the commit the documentation was built against.

If you would like to access the documentation for the latest commit or would
like a browseable web version instead, follow the instructions in the [Building
the documentation from source](#building-the-documentation-from-source) section.

### Building the documentation from source

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
