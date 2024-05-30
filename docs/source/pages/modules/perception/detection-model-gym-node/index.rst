Detection model gym node
========================

The detection model gym node provides a conveniente interface to train your
custom detection model and integrate it directly into the autonomy stack. The
node is a wrapper around the Ultralytics YOLOv8 framework, and relies on the
Python Ultralytics package for the training, validation and export of YOLOv8
models to be used in the autonomy stack.

.. note::

  At the current time, only models based on the YOLOv8 architecture are supported
  for inference in the autonomy stack detection nodes, despite the model gym node
  interface allowing to load models from previous YOLO versions.

Setup
-----

From the root of the repository, run the following commands to install all the
required dependencies:

.. code-block::

  poetry lock
  poetry install --with object-detection

After all dependencies are installed, drop into a shell in the generated virtual
environment using :code`poetry shell`. Then, source the local ROS workspace and run the node (make sure /usr/env/python3 points to your Poetry virtual environment!).


ROS interface
-------------

Parameters
^^^^^^^^^^

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1
   :widths: 17, 20, 63  

Troubleshooting
---------------

* *No module named `ultralytics`*  
  Make sure that the Poetry shell is activated before running the node!