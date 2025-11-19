Inputs and Outputs
============================

.. note::
    In all case, the topic name is prefixed with the vehicle name but is not shown in the table for the sake of brevity.

Minimum Requirements
-----------------------------

**Subscribed Input to Stack**

======================================  =====================================   ===============
Topic                                   Type                                    Notes
======================================  =====================================   ===============
/avt_341/odometry                       nav_msgs/msg/Odometry                   Assumed twist linear and angular velocities in vehicle frame.
/avt_341/points                         sensor_msgs/msgs/PointCloud2            Vehicle lidar point cloud.
/avt_341/forward_speed                  std_msgs/msg/Float64                    Vehicle lateral speed.
======================================  =====================================   ===============

**Published Output from Stack**

======================================  =====================================   ===============
Topic                                   Type                                    Notes
======================================  =====================================   ===============
/avt_341/cmd_vel                        geometry_msgs/msg/Twist                 | linear.x = throttle  [0..1]
                                                                                | linear.y = brake     [0..1]
                                                                                | angular.z = steering [-1..1]
======================================  =====================================   ===============

.. todo::

    Other topics needed for other stack features
