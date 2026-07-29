Editing Documentation
===================================

Dependencies
------------------

To build and edit the documentation from source, you will need:

- A working Doxygen binary in your ``PATH`` (on Ubuntu, ``sudo apt install -y
  doxygen``).
- A Python dependency manager such as ``Poetry`` or ``pip`` (see instructions below).

Build using Poetry
^^^^^^^^^^^^^^^^^^^^

.. note::
    This method assumes you have a working `Poetry <https://python-poetry.org/>`_
    installation. Follow the `installation
    instructions <https://python-poetry.org/docs/#installation>`_ for your platform
    to get Poetry running on your system.

From the root of the repository, install and activate the Poetry environment for
the current session:

.. code-block:: shell

    poetry install --with documentation
    poetry shell

Change directory to the ``docs`` folder and run the Sphinx Makefile with `cd docs
&& make html`.


Build using pip
^^^^^^^^^^^^^^^^^^^^

Then install the required Python dependencies by issuing the following commands
from the root of the repository:

.. code-block:: shell

    python -m venv docs/.nato-avt-341-docs-env
    source docs/.nato-avt-341-docs-env/bin/activate
    pip install -r docs/requirements.txt

Finally, run the build process by running `cd docs && make html`.

Once the build process is complete, the documentation will be available under
``build/html/index.html``.

Editing documentation with live updates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Running ``sphinx-autobuild source build`` in the ``docs`` folder will run a local
html webpage which also receives live updates when edits are made. By default the documentation will be available at `http://127.0.0.1:8000 <http://127.0.0.1:8000>`_.

.. warning::

    Before sourcing your ROS distribution to build and/or run the stack, make sure
    the documentation environment is deactivated (``exit`` for Poetry, ``deactivate``
    for pip).
