#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import numpy as np
import time

class GlobalPathTestNode(Node):
    def __init__(self):
        super().__init__('global_path_test_driver')
        self.declare_parameter('verbose', True)
        
        self.grid_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid_low_res', 10)
        self.odom_pub = self.create_publisher(Odometry, 'avt_341/odometry', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'avt_341/goal_pose', 10)
        self.nav_state_pub = self.create_publisher(Int32, 'avt_341/nav_command_state', 10)
        self.path_sub = self.create_subscription(Path, 'avt_341/global_path', self.path_callback, 10)
        
        self.timer = self.create_timer(1.0, self.publish_test_data)
        self.path_received = False
        self.goal_published = False
        self.nav_state_published = False
        self.get_logger().info("Global Path Test Driver Node Started")

    def publish_test_data(self):
        verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # Create Occupancy Grid
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = 1.0
        grid.info.width = 100
        grid.info.height = 100
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        
        # Random obstacles
        data = np.zeros(100 * 100, dtype=np.int8)
        np.random.seed(0)
        
        start_x, start_y = 10, 10
        goal_x, goal_y = 30, 53
        
        count = 0
        while count < 20:
            w = np.random.randint(5, 15)
            h = np.random.randint(5, 15)
            x = np.random.randint(0, 85)
            y = np.random.randint(0, 85)
            
            # Buffer check
            if (y - 5 < start_y < y + h + 5 and x - 5 < start_x < x + w + 5) or \
               (y - 5 < goal_y < y + h + 5 and x - 5 < goal_x < x + w + 5):
                continue
            
            for i in range(y, min(y + h, 100)):
                for j in range(x, min(x + w, 100)):
                    data[i * 100 + j] = 100
            count += 1
            
        grid.data = data.tolist()
        self.grid_pub.publish(grid)
        if verbose: self.get_logger().info('Published OccupancyGrid')

        # Publish Odometry (Start point)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = float(start_x)
        odom.pose.pose.position.y = float(start_y)
        self.odom_pub.publish(odom)
        if verbose: self.get_logger().info('Published Odometry')

        # Publish Goal
        if not self.goal_published:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = float(goal_x)
            goal.pose.position.y = float(goal_y)
            self.goal_pub.publish(goal)
            self.goal_published = True
            if verbose: self.get_logger().info('Published Goal')

        # Publish Nav Command State
        if not self.nav_state_published:
            nav_state = Int32()
            nav_state.data = 1
            self.nav_state_pub.publish(nav_state)
            self.nav_state_published = True
            if verbose: self.get_logger().info('Published Nav Command State')

    def path_callback(self, msg):
        verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        if not self.path_received:
            self.get_logger().info(f'RECEIVED PATH from Planner! Size: {len(msg.poses)}')
            self.path_received = True

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
