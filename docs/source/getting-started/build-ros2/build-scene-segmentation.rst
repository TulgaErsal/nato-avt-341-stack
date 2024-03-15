Build scene segmentation node
=============================

Prerequisites
-------------

Installing the Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Python dependendencies are managed with `pipenv`. To install the
dependencies run the following command inside the 'src/ia/segmentation' folder
under the root of the `avt_341` package:

.. code-block:: shell

    pipenv install

Setting the shebang
^^^^^^^^^^^^^^^^^^^

Once the dependencies are installed, you can set the 'shebang' of the
**avt_341_depth_estimation_node** for your system with the following command:

.. code-block:: python

    python3 shebang_util.py