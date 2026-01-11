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
        resolution = 1.0  # Change this to test resolution independence
        map_width_m = 100.0
        map_height_m = 100.0
        
        width = int(map_width_m / resolution)
        height = int(map_height_m / resolution)
        
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        
        # Random obstacles
        data = np.zeros(width * height, dtype=np.int8)
        np.random.seed(0)
        
        # Start and Goal in Meters
        start_x_m, start_y_m = 10.0, 10.0
        goal_x_m, goal_y_m = 90.0, 90.0
        
        count = 0
        while count < 20:
            # Random size in meters (5m to 15m)
            w_m = np.random.uniform(5.0, 15.0)
            h_m = np.random.uniform(5.0, 15.0)
            # Random position in meters
            x_m = np.random.uniform(0.0, 85.0)
            y_m = np.random.uniform(0.0, 85.0)
            
            # Check buffer in meters
            if (y_m - 5.0 < start_y_m < y_m + h_m + 5.0 and x_m - 5.0 < start_x_m < x_m + w_m + 5.0) or \
               (y_m - 5.0 < goal_y_m < y_m + h_m + 5.0 and x_m - 5.0 < goal_x_m < x_m + w_m + 5.0):
                continue
            
            # Convert to grid indices
            ix = int(x_m / resolution)
            iy = int(y_m / resolution)
            iw = int(w_m / resolution)
            ih = int(h_m / resolution)
            
            # Fill grid
            for i in range(iy, min(iy + ih, height)):
                for j in range(ix, min(ix + iw, width)):
                    data[i * width + j] = 100
            count += 1
            
        grid.data = data.tolist()
        self.grid_pub.publish(grid)
        if verbose: self.get_logger().info('Published OccupancyGrid')

        # Publish Odometry (Start point)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = float(start_x_m)
        odom.pose.pose.position.y = float(start_y_m)
        self.odom_pub.publish(odom)
        if verbose: self.get_logger().info('Published Odometry')

        # Publish Goal
        if not self.goal_published:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = float(goal_x_m)
            goal.pose.position.y = float(goal_y_m)
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
