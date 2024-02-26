# AVT-341

ROS package with autonomy algorithms for the NATO AVT-341.

The MPC plugin is available at [https://github.com/TulgaErsal/AVT-341-MPC](https://github.com/TulgaErsal/AVT-341-MPC)

## Building the documentation

The NATO AVT-341 autonomy stack comes with documentation that may be built and
browsed locally. The documentation build has been tested on Ubuntu 22.04, but a
similar procedure should work provided a working Python installation is
available.

First, install Doxygen:

```bash
sudo apt install -y doxygen
```

Then install the required Python dependencies by issuing the following commands from
the root of the repository:

```bash
python -m venv docs/.nato-avt-341-docs-env
source docs/.nato-avt-341-docs-env/bin/activate
pip install -r docs/requirements.txt
```

Finally, run the build process:

```bash
cd docs
make html
```

Before sourcing your ROS distribution to run the stack, make sure the
documentation build environment is deactivated by issuing `deactivate` in the
current terminal or by opening a new terminal. The prefix
`.nato-avt-341-docs-env` should no longer be visible in your terminal before
sourcing the ROS distribution.

## Acknowledgements

This project is made possible by technical and financial support of the
Mississippi State University Center for Advanced Vehicular Systems as well as
the Automotive Research Center (ARC) in accordance with Cooperative Agreement
W56HZV 14 2 0001 U.S. Army CCDC Ground Vehicle Systems Center (GVSC) Warren, MI.
