Writing ROS Version-Independent Code
======================================

This page describes the syntax conventions for writing ROS version-independent code.

The repository supports interfacing with both ROS1 and ROS2 using a single code syntax. It does this by encapsulated
the ROS version specific code in separate files and classes which are abstracted away from the user client ROS code.


ROS Message Types
-------------------------

Use ``#include "avt_341/node/ros_types.h"`` to reference ROS types instead of the ROS version-specific message headers.
The message properties should be the same compared to ROS1 and ROS2 message types. For example:

.. code-block:: cpp

    #include "avt_341/node/ros_types.h"

    avt_341::msg::Odometry current_pose
    current_pose.pose.pose.position.x = 1.0;


Basic Node Setup
-------------------------

Use the node definition under ``#include "avt_341/node/node_proxy.h"`` to create the ros node. Below is a simple example:

.. note::

    Can use ``auto`` when possible to simplify verbose type syntax.

.. code-block:: cpp

    #include "avt_341/node/node_proxy.h"

    std::shared_ptr<avt_341::node::NodeProxy> node = nullptr;
    node = avt_341::node::init_node(argc, argv, "my_node");

    // Example declaring and reading node parameter
    float node_param;
    node->get_parameter("~node_param", node_param, 1.0f);


    // Example node main loop
    avt_341::node::Rate rate(10.0f);
    while (avt_341::node::ok()){
        ...
        node->spin_some();
        rate.sleep();
    }




ROS Publishers
-------------------------

Below is an example of creating ROS publishers using the ROS version-independent syntax.

.. code-block:: cpp

    #include "avt_341/node/ros_types.h"
    #include "avt_341/node/node_proxy.h"

    avt_341::node::Publisher<avt_341::msg::Odometry>::SharedPtr pub = nullptr;
    pub = node->create_publisher<avt_341::msg::Odometry>("odometry", qos);


ROS Subscribers
-------------------------

Below is an example of creating ROS subscribers using the ROS version-independent syntax both inside and outside of a class context.

.. code-block:: cpp

    #include "avt_341/node/ros_types.h"
    #include "avt_341/node/node_proxy.h"

    void Callback(avt_341::msg::OdometryPtr msg){
        ...
    }

    node::Subscriber<avt_341::msg::Odometry>::SharedPtr sub = nullptr;
    sub = node->create_subscription<avt_341::msg::Odometry>("odometry", qos, Callback);


**When in class:**

.. code-block:: cpp

    #include "avt_341/node/ros_types.h"
    #include "avt_341/node/node_proxy.h"


    class MyClass {

    public:

        MyClass(const std::shared_ptr<NodeProxy> &node, int qos){
            sub_ = node->create_subscription<msg::Odometry>(topic_name, qos,
                        std::bind(&MyClass::Callback, this, std::placeholders::_1));

        }

    private:

        void Callback(msg::OdometryPtr msg) {
                ...
        }

        node::Subscriber<avt_341::msg::Odometry>::SharedPtr sub_;

    };

