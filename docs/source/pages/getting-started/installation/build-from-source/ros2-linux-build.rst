ROS 2 Linux - Build From Source
==================================

Prerequisites
------------------------------

- Installation of a `ROS2 distribution <https://docs.ros.org/en/rolling/Releases.html>`_.
- Working ROS2 `colcon workspace <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_

.. note::

    Ensure that ``<ros_ws>/install/setup.bash`` has been sourced in your workspace.
    Typically this command is added to ``~/.bashrc`` to automatically run on shell prompt startup.

    .. code-block:: shell

        source ~/<ros_ws>/install/setup.bash

Dependency Installation
----------------------------------

ROS Dependencies
"""""""""""""""""""""""""""""""""""""""""""""""

Install the following ROS packages via the command line:

.. code-block:: shell

    # For ROS2 Foxy
    sudo apt install ros-foxy-pcl-ros ros-foxy-ackermann-msgs ros-foxy-vision-msgs

    # For ROS2 Humble
    sudo apt install ros-humble-pcl-ros ros-humble-ackermann-msgs ros-humble-vision-msgs

    # For ROS2 Jazzy
    sudo apt install ros-jazzy-pcl-ros ros-jazzy-ackermann-msgs ros-jazzy-vision-msgs


Machine Learning Torch Dependency (Optional)
"""""""""""""""""""""""""""""""""""""""""""""""

Download the c++ torchlib interface (`based on instructions <https://pytorch.org/cppdocs/installing.html>`_).

.. code-block:: shell

    wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip
    unzip libtorch-shared-with-deps-latest.zip

Add the following to ``~/.bashrc``:

.. code-block:: shell

    export TORCH_DIR='<location_to_unzip>/libtorch'
    # ex: export TORCH_DIR='/home/username/libtorch'

In the previous steps, the CPU version of libtorch was downloaded and installed for ease of use.
For the GPU accelerated version,  `download the CUDA version <https://pytorch.org/get-started/locally/>`_, unzip the package and set the ``Torch_CUDA_DIR`` environment variable.

MPC Local Planner Dependencies (Optional)
"""""""""""""""""""""""""""""""""""""""""""""""
If using the stack's MPC local planner, additional dependencies must be installed. Refer to documentation at `TulgaErsal/AVT-341-MPC <https://github.com/TulgaErsal/AVT-341-MPC>`_ for more details.

Build Source Code
-------------------------------

Clone the repo into your catkin_ws/src directory with the following command:

.. code-block:: shell

    cd <ros_ws>/src
    git clone https://github.com/TulgaErsal/nato-avt-341-stack.git avt_341_stack

From the top level ROS workspace directory, execute:

.. code-block:: shell

    colcon build --packages-select avt_341_msgs avt_341

.. note::

    There may be some some warning messages on the first build about deprecated point cloud libraries.
    If this is the case, execute the build a second time.

.. note::

    The ``--event-handlers console_cohesion+`` argument can be used to obtain additional logs (for example ``colcon build --packages-select avt_341_msgs avt_341``).



Testing the installation
--------------------------

To test the installation, execute the following command. Rviz should appear with a mock scenario.

.. code-block:: shell

    ros2 launch avt_341 example.launch.py

.. image:: images/rviz_mock_test_scenario.png
    :alt: RVIZ Mock Scenario