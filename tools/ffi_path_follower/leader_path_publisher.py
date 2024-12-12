#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_from_euler
import math

class LeaderOdomPublisherNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('leader_odom_publisher_node')

        # Read the parameters
        self.follower_odom_sub_topic = rospy.get_param('~follower_odom_sub_topic', '~follower_odom')
        self.leader_odom_sub_topic = rospy.get_param('~leader_odom_sub_topic', '~leader_odom')
        self.follow_point_pub_topic = rospy.get_param('~follow_point_pub_topic', '~follow_point_output')
        node_rate = rospy.get_param('~node_rate', 1)
        self.dist_x = rospy.get_param('~min_distance', 10)

        # Set node rate
        self.rate = rospy.Rate(node_rate)

        # Subscribers
        self.follower_odom_sub = rospy.Subscriber(self.follower_odom_sub_topic, Odometry, self.follower_callback)
        self.leader_odom_sub = rospy.Subscriber(self.leader_odom_sub_topic, Odometry, self.leader_callback)

        # Publishers
        self.follow_point_pub = rospy.Publisher(self.follow_point_pub_topic, Path, queue_size=1)

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.vel = 0.25
        self.dist_y = 0.0
        self.N = 2500

        self.follower_odom = Odometry()
        self.leader_odom = Odometry()
        self.leader_path = []

    def leader_callback(self, leader_odom_msg):
        self.leader_odom = leader_odom_msg

    def point_on_line_with_distance(self, point1, point2, L):
        # Extract coordinates of the points
        x1, y1 = point1
        x2, y2 = point2

        # Calculate the direction vector from point1 to point2
        dx = x2 - x1
        dy = y2 - y1

        # Calculate the distance between the two points
        dist = math.sqrt(dx**2 + dy**2)

        # Normalize the direction vector (get a unit vector)
        dx /= dist
        dy /= dist

        # Calculate the new point at distance L from point2 along the line
        new_x = x2 - L * dx
        new_y = y2 - L * dy

        return [new_x, new_y]

    def follower_callback(self, follower_odom_msg):
        # Publish leaders driven path
        self.leader_path.append([self.leader_odom.pose.pose.position.x, self.leader_odom.pose.pose.position.y, self.leader_odom.pose.pose.position.z, self.leader_odom.pose.pose.orientation.x, self.leader_odom.pose.pose.orientation.y, self.leader_odom.pose.pose.orientation.z, self.leader_odom.pose.pose.orientation.w])
        if len(self.leader_path) > self.N:
            self.leader_path.pop(0)


        # Publish point behind UGV X meters along path
        length = 0
        path_pose_to_use = PoseStamped()
        path_pose_to_use.header.seq = self.leader_odom.header.seq
        path_pose_to_use.header.stamp = self.leader_odom.header.stamp
        path_pose_to_use.header.frame_id = self.leader_odom.header.frame_id
        for i in range(len(self.leader_path)-1, -1, -1):
            if i <= 0:
                path_pose_to_use.pose.position.x = self.leader_path[i][0]
                path_pose_to_use.pose.position.y = self.leader_path[i][1]
                path_pose_to_use.pose.position.z = self.leader_path[i][2]
                path_pose_to_use.pose.orientation.x = self.leader_path[i][3]
                path_pose_to_use.pose.orientation.y = self.leader_path[i][4]
                path_pose_to_use.pose.orientation.z = self.leader_path[i][5]
                path_pose_to_use.pose.orientation.w = self.leader_path[i][6]
                break
            else:
                length = length + np.sqrt(pow(self.leader_path[i][0]-self.leader_path[i-1][0], 2) + pow(self.leader_path[i][1]-self.leader_path[i-1][1], 2))
                if length >= self.dist_x:
                    # overshoot = length - self.dist_x
                    path_pose_to_use.pose.position.x = self.leader_path[i][0]
                    path_pose_to_use.pose.position.y = self.leader_path[i][1]
                    path_pose_to_use.pose.position.z = self.leader_path[i][2]
                    path_pose_to_use.pose.orientation.x = self.leader_path[i][3]
                    path_pose_to_use.pose.orientation.y = self.leader_path[i][4]
                    path_pose_to_use.pose.orientation.z = self.leader_path[i][5]
                    path_pose_to_use.pose.orientation.w = self.leader_path[i][6]
                    break

        delta_x = path_pose_to_use.pose.position.x - follower_odom_msg.pose.pose.position.x
        delta_y = path_pose_to_use.pose.position.y - follower_odom_msg.pose.pose.position.y
        distance = np.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if (abs(distance) < self.dist_x):
            path_pose_to_use.pose = follower_odom_msg.pose.pose

        Foo = Path()
        Foo.header = path_pose_to_use.header
        Foo.poses.append(path_pose_to_use)
        self.follow_point_pub.publish(Foo)



    def spin(self):
        # while not rospy.is_shutdown():
        rospy.spin()
        # self.rate.sleep()

if __name__ == '__main__':
    try:
        node = LeaderOdomPublisherNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass






# # Publish UGV random walk odometry
# odom = Odometry()
# odom.header.frame_id = 'map'
# odom.pose.pose.position.x = self.x
# odom.pose.pose.position.y = self.y
# odom.pose.pose.position.z = 0.0
#
# quat = quaternion_from_euler(0.0, 0.0, self.yaw)
# odom.pose.pose.orientation.x = quat[0]
# odom.pose.pose.orientation.y = quat[1]
# odom.pose.pose.orientation.z = quat[2]
# odom.pose.pose.orientation.w = quat[3]
# self.leader_odom_pub.publish(odom)
#
#
# self.yaw = self.yaw + 0.25 * (random.random() - 0.5)
# while self.yaw >= 2*np.pi:
#     self.yaw = self.yaw - 2*np.pi
# while self.yaw < 0:
#     self.yaw = self.yaw + 2*np.pi
# self.vel = self.vel + 0.05 * (random.random() - 0.5)
# self.vel = min(max(self.vel, self.vel*0.5), self.vel*1.5)
# self.x = self.x + self.vel*np.cos(self.yaw)
# self.y = self.y + self.vel*np.sin(self.yaw)


# # Publish point X meters directly behind UGV
# line_point = PoseStamped()
# line_point.header.frame_id = 'map'
# line_point.pose.position.x = self.x - self.dist_x*np.cos(self.yaw)
# line_point.pose.position.y = self.y - self.dist_x*np.sin(self.yaw)
# self.follow_point_pub.publish(line_point)
