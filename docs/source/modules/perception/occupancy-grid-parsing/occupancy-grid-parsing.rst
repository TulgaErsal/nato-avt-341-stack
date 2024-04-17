Occupancy grid parsing
======================

Overview
--------

The occupancy grid parsing node receives an occupancy grid messages and parses
the cells into individual obstacles for use in local planners. It provides
different methods for identifying, parsing and publishing obstacles from an
occupancy grid and provides a shared source for planners to receive processed
grid information.

The grid parsing is performed asynchronously based on the latest receive pair of
occupancy data and AGV odometry. The AGV odometry can be optionally used to only
filter obstacles in the vicinity of the vehicle and/or within its planning
horizon.

The field of view filtering does not support AGVs moving in reverse or
undergoing significant sliding, as only the longitudinal component of the
velocity vector is considered.

ROS Interface
-------------

Subscriptions
^^^^^^^^^^^^^

.. csv-table:: Subscribed topics
   :file: subscriptions.csv
   :header-rows: 1

Publishers
^^^^^^^^^^

.. csv-table:: Published topics
   :file: publishers.csv
   :header-rows: 1

Parameters
^^^^^^^^^^

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1
