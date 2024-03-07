Running with MAVS
=================

To run an example simulation with MAVS, first `install and build MAVS
<https://mavs-documentation.readthedocs.io/en/latest/MavsBuildInstructions/>`_.

Next, install and build the `MAVS-ROS package
<https://github.com/CGoodin/mavs_ros>`_ (for ROS1) or the `MAVS-ROS2
<https://github.com/CGoodin/mavs-ros2>`_ package (for ROS2).

To test in ROS-1:

.. code-block:: shell

    roslaunch avt_341 mavs_example.launch

To test in ROS-2:

.. code-block:: shell

    ros2 launch avt_341 mavs_example.launch.py