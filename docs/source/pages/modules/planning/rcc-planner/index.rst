Road-centerline constrained (RCC) planner
=========================================

The local planning algorithm is a spline-based planner for local path planning. The algorithm is based on `"Dynamic path planning for autonomous driving on various roads with avoidance of static and moving obstacles" by Hu, Chen, Tang, Cao, and He <https://www.sciencedirect.com/science/article/pii/S0888327017303825?casa_token=jvXhlTgKFVQAAAAA:jnnYnooDkS3Tp8Sj0DoMQPNGtEGCB4Bp2IWNZrKKoTTTju5mxpaeBUGv6EVYKzqHKId_k1-caLIn>`_.

The planning algorithm requires a road centerline to follow. In the off-road case, the road centerline is assumed to be the "global path" published by the [global path node](wiki/avt_341_global_path.md).

Subscriptions
-------------

.. csv-table:: Subscriptions
   :file: subscriptions.csv
   :header-rows: 1

Published Topics
----------------

.. csv-table:: Published topics
   :file: publishers.csv
   :header-rows: 1

Parameters
----------

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1
