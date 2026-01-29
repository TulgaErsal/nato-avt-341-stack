#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import numpy as np

class GlobalPathTestNode(Node):
    def __init__(self):
        super().__init__('global_path_test_driver')
        
        # Parameters
        self.declare_parameter('scenario', 'random')
        self.declare_parameter('width_m', 100.0)
        self.declare_parameter('height_m', 100.0)
        self.declare_parameter('cell_size_m', 1.0)
        self.declare_parameter('add_terrain_cost', True)
        self.declare_parameter('num_patches', 12)
        self.declare_parameter('verbose', True)
        self.declare_parameter('num_random', 10)
        self.declare_parameter('obs_min', 5.0)
        self.declare_parameter('obs_max', 15.0)
        self.declare_parameter('seed', 0)

        # Publishers
        self.grid_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid_low_res', 10)
        self.terrain_pub = self.create_publisher(OccupancyGrid, 'avt_341/normal_segmentation_grid', 10)
        self.odom_pub = self.create_publisher(Odometry, 'avt_341/odometry', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'avt_341/goal_pose', 10)
        self.nav_state_pub = self.create_publisher(Int32, 'avt_341/nav_command_state', 10)
        
        # Subscriber
        self.path_sub = self.create_subscription(Path, 'avt_341/global_path', self.path_callback, 10)
        
        self.timer = self.create_timer(2.0, self.publish_test_data)
        self.goal_published = False

    def setup_grids(self):
        # Fetch all parameters correctly
        scenario = self.get_parameter('scenario').value
        width_m = self.get_parameter('width_m').value
        height_m = self.get_parameter('height_m').value
        cell_size_m = self.get_parameter('cell_size_m').value
        add_terrain = self.get_parameter('add_terrain_cost').value
        num_random = self.get_parameter('num_random').value
        obs_min = self.get_parameter('obs_min').value
        obs_max = self.get_parameter('obs_max').value
        seed = self.get_parameter('seed').value
        
        res = 1.0 / cell_size_m
        size_y, size_x = int(height_m * res), int(width_m * res)
        
        occ_grid = np.zeros((size_y, size_x), dtype=np.int8)
        terrain_grid = np.full((size_y, size_x), 100, dtype=np.int8)
        
        def m_to_px(val): return int(val * res)
        
        start_m = (0.1 * width_m, 0.1 * height_m)
        goal_m = (0.9 * width_m, 0.9 * height_m)
        start_px = (m_to_px(start_m[0]), m_to_px(start_m[1]))
        goal_px = (m_to_px(goal_m[0]), m_to_px(goal_m[1]))

        # 1. Populate Hard Obstacles (Occupancy Grid)
        if scenario == "gate":
            occ_grid[m_to_px(40):m_to_px(60), 0:m_to_px(45)] = 100
            occ_grid[m_to_px(40):m_to_px(60), m_to_px(55):size_x] = 100
        elif scenario == "box":
            occ_grid[m_to_px(30):m_to_px(60), m_to_px(40):m_to_px(60)] = 100
        elif scenario == "random":
            np.random.seed(seed)
            count = 0
            while count < num_random:
                w_m, h_m = np.random.uniform(obs_min, obs_max, size=2)
                x_m = np.random.uniform(0, width_m - w_m)
                y_m = np.random.uniform(0, height_m - h_m)
                w, h = m_to_px(w_m), m_to_px(h_m)
                x, y = m_to_px(x_m), m_to_px(y_m)
                buffer = m_to_px(2.0)
                
                # Check collision with start/goal pixels
                if (y - buffer < start_px[1] < y + h + buffer and x - buffer < start_px[0] < x + w + buffer) or \
                   (y - buffer < goal_px[1] < y + h + buffer and x - buffer < goal_px[0] < x + w + buffer):
                    continue 
                occ_grid[y:y+h, x:x+w] = 100
                count += 1
        elif scenario == "narrow_and_wide":
            occ_grid[m_to_px(40):m_to_px(60), m_to_px(20):m_to_px(49)] = 100
            occ_grid[m_to_px(40):m_to_px(60), m_to_px(51):size_x] = 100
        elif scenario == "narrow_gate":
            occ_grid[m_to_px(40):m_to_px(60), 0:m_to_px(49)] = 100
            occ_grid[m_to_px(40):m_to_px(60), m_to_px(51):size_x] = 100

        # 2. Populate Terrain Costs
        if add_terrain:
            np.random.seed(seed + 1) 
            num_patches = self.get_parameter('num_patches').value
            for _ in range(num_patches):
                cx, cy = np.random.randint(0, size_x), np.random.randint(0, size_y)
                radius = np.random.randint(m_to_px(4.0), m_to_px(12.0))
                
                y_indices, x_indices = np.ogrid[:size_y, :size_x]
                mask = (x_indices - cx)**2 + (y_indices - cy)**2 <= radius**2
                
                traversability_value = int(np.random.uniform(20, 50))
                
                terrain_grid[mask] = np.minimum(terrain_grid[mask], traversability_value)       
                
        return occ_grid, terrain_grid, start_m, goal_m

    def create_grid_msg(self, data_np, frame_id, resolution):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.width = data_np.shape[1]
        msg.info.height = data_np.shape[0]
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.data = data_np.flatten().astype(int).tolist()
        return msg

    def publish_test_data(self):
        verbose = self.get_parameter('verbose').value
        occ_np, terr_np, start_m, goal_m = self.setup_grids()
        res = self.get_parameter('cell_size_m').value
        
        # Publish both grids
        self.grid_pub.publish(self.create_grid_msg(occ_np, 'map', res))
        self.terrain_pub.publish(self.create_grid_msg(terr_np, 'map', res))

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = start_m[0]
        odom.pose.pose.position.y = start_m[1]
        self.odom_pub.publish(odom)

        # Publish Goal and State once
        if not self.goal_published:
            goal = PoseStamped()
            goal.header = odom.header
            goal.pose.position.x = goal_m[0]
            goal.pose.position.y = goal_m[1]
            self.goal_pub.publish(goal)
            
            nav_state = Int32()
            nav_state.data = 1
            self.nav_state_pub.publish(nav_state)
            
            self.goal_published = True
            if verbose: self.get_logger().info("Test environment published.")

    def path_callback(self, msg):
        self.get_logger().info(f"Received path: {len(msg.poses)} poses")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()