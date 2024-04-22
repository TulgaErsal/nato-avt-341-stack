Potential field planner
=======================

An alternative to the spline-based local planner is the potential field planning
algorithm. Although listed as a local planner, it can be used as either a local
or global planner. 

Unlike the spline planner, the potential field planner does not follow a
centerline. Therefore, when given a list of waypoints or points in a global
path, it will plan to the farthest available point.

The subscribed and published topics are identical to the spline planner, but the
parameters of the launch file are different, see below.

Subscriptions
-------------

.. csv-table:: Subscriptions
   :file: subscriptions.csv
   :header-rows: 1

Publishers
----------

.. csv-table:: Publishers
   :file: publishers.csv
   :header-rows: 1

Parameters
----------

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1
