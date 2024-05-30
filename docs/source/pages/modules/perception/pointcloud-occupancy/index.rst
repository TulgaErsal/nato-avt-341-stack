Point-cloud based occupancy
===========================

Overview
--------

The perception node uses a point cloud and odometry to create a map of obstacle
locations in the environment. The map is in the form of a ROS occupancy grid. 

The grid is stationary and centered on the starting position of the vehicle. The
grid size parameter must be set to encompass the entire operating extent for the
vehicle.

A map cell is determined to be occupied if the highest measured point in that
cell exceeds the lowest measured point in that cell by more than a user-defined
threshhold.

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