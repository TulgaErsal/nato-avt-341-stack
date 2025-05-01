
import os
if "ROS_DISTRO" not in os.environ or os.environ["ROS_DISTRO"] == "noetic": 
    ROSV = 1
elif os.environ["ROS_DISTRO"] == "foxy" or os.environ["ROS_DISTRO"] == "humble" or os.environ["ROS_DISTRO"] == "jazzy":
    ROSV = 2
else:
    print ("ROS_DISTRO not set, assuming ROS2")
    ROSV = 2

if ROSV == 1:
    import rospy
else:
    import rclpy
    from rclpy.node import Node

##################################################################
# ROS12 MSG IMPORTS 

import tf2_ros
if ROSV == 1:
    import sensor_msgs.point_cloud2 as pc2
else:
    import sensor_msgs_py.point_cloud2 as pc2

##################################################################
# ROS12 WRAPPERS

def loginfo(s):
    if ROSV == 1:
        rospy.loginfo(s)
    else:
        print(s)

def rosstamp(ts):
    if ROSV == 1:
        rtime = rospy.Duration(ts)
    else:
        ns = int(ts * 1e9)
        rtime = rclpy.time.Time(nanoseconds=ts).to_msg()
    return rtime  

def ros_to_sec(rtime):
    if ROSV == 1:
        secs = rtime.to_sec()
    else:
        secs = rtime.sec + rtime.nanosec/1e9
    return secs 

def rosinit(name="anonymous_node", anonymous=True, args=None):
    if ROSV == 1:
        rospy.init_node(name, anonymous=anonymous)
    else:
        rclpy.init(args=args)

if ROSV == 1:
    class NodeHelper:      
        def __init__(self, name):
            pass

        def _now(self):
            return rospy.Time.now().to_sec()

        def _create_broadcaster(self):
            return tf2_ros.TransformBroadcaster()

        def _create_publisher(self, topic, msgtype, queue_size=1):
            return rospy.Publisher(topic, msgtype, queue_size=queue_size)
            
        def _create_timer(self, period, callback):
            return rospy.Timer(period, callback)
            
        def _create_subscriber(self, topic, msgtype, callback, queue_size=1, buff_size=None):
           return rospy.Subscriber(topic, msgtype, callback, queue_size=queue_size)

        def _spin(self):
            rospy.spin()
            
        def _getparam(self, name, default = None):
            return rospy.get_param(name, default)
        
        def _loginfo(s):
            rospy.loginfo(s)
    
else:
    class NodeHelper(Node):      
        def __init__(self, name):
            super().__init__(name)
            pass

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _create_broadcaster(self):
            return tf2_ros.TransformBroadcaster(self)
        
        def _create_publisher(self, topic, msgtype, queue_size=1):
            return self.create_publisher(msgtype, topic, queue_size)
            
        def _create_timer(self, period, callback):
            hide_timer_event = lambda : callback(None)
            return self.create_timer(period.sec, hide_timer_event)
            
        def _create_subscriber(self, topic, msgtype, callback, queue_size=1, buff_size=None):
            return self.create_subscription(msgtype, topic, callback, queue_size)

        def _spin(self):
            rclpy.spin(self)

        def _getparam(self, name, default = None):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value
        
        def _loginfo(self, s):
            self.get_logger().info(s)

##################################################################
