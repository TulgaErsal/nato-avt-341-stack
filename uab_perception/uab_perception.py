#!/usr/bin/env python3
from avt_341_types import Image, PointCloud2, OccupancyGrid, Odometry
from avt_341_node_proxy import NodeProxy
import numpy
import perception_wrapper

record_bin_file = False
node = None 
pub_occupancy_grid = None
pub_vis_occupancy_grid = None
raw_image = None 
raw_lidar = None 
odom_data = None
queue_size = 10

grid_width = 510
grid_height = 160
grid_res = 0.5
grid_llx = -252
grid_lly = -41

def listener_callback_camera(image_msg):
    """listen for camera messages.
       sensor_msgs/msg/Image
    """
    print(f"camera callback  {type(image_msg)}" )
    global raw_image
    raw_image = image_msg
    
def listener_callback_lidar(lidar_msg):
    """listen for lidar messages.
       sensor_msgs/msg/PointCloud2
    """
    print(f"lidar callback  {type(lidar_msg)}")
    global raw_lidar
    raw_lidar = lidar_msg

def listener_callback_odom(odom_msg):
    """listen for odom messages.
       nav_msgs/msg/Odometry
    """
    print(f"odom callback  {type(odom_msg)}")
    global odom_data
    odom_data = odom_msg
 
def publish_occupancy_grid(data):
    """publish Occupancy Grid messages.
       nav_msgs/msg/OccupancyGrid
    """
    print("sending occupancy grid")
    
    # align in RVIZ
    grd = numpy.array(data)
    grd = numpy.reshape(grd, (grid_width, grid_height))
    grd = numpy.fliplr(grd)
    aligned_grid = numpy.rot90(grd)
    
    # this grid is used by the global/local planners
    col_major_grid = numpy.transpose(aligned_grid) # AVT 341 stack expects column major order
    int_grid = flatten_and_scale_grid(col_major_grid)
    grid = build_occupancy_grid_msg(int_grid)
    pub_occupancy_grid.publish(grid)
    
    # this grid is *ONLY* used for visualization in RIVZ
    int_vis_grid = flatten_and_scale_grid(aligned_grid)
    vis_grid = build_occupancy_grid_msg(int_vis_grid)
    pub_vis_occupancy_grid.publish(vis_grid)
    
def flatten_and_scale_grid(grid: list) -> list:
    # The 'data' field must be a set or sequence and each value of type 'int' and each integer in [-128, 127]
    # flatten 510 x 160 array to 81600 element 1D array, scale from [0,1] to [0, 100], and cast to int
    return [int(j * 100) for i in list(grid) for j in i]

def build_occupancy_grid_msg(data: list) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = grid_res
    grid.info.width = grid_width
    grid.info.height = grid_height
    grid.info.origin.position.x = float(grid_llx)
    grid.info.origin.position.y = float(grid_lly)
    grid.info.origin.orientation.w = 1.0
    grid.info.origin.orientation.x = 0.0
    grid.info.origin.orientation.y = 0.0
    grid.info.origin.orientation.z = 0.0
    grid.data = data
    
    return grid

def setup_ros():
    global node
    global pub_occupancy_grid
    global pub_vis_occupancy_grid

    node = NodeProxy('Perception_sem_seg')
    logger = node.get_logger()

    # start subscriptions
    sub_cam   = node.create_subscription(Image, '/camera/rgb/image_raw', listener_callback_camera, queue_size)
    sub_lidar = node.create_subscription(PointCloud2,  '/avt_341/points', listener_callback_lidar, queue_size)
    sub_odom  = node.create_subscription(Odometry,  'avt_341/odometry', listener_callback_odom, queue_size)

    # start publishers
    pub_occupancy_grid = node.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid', queue_size)
    pub_vis_occupancy_grid = node.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid_vis', queue_size)

    logger.info("uab perception node running")

def parse_odom(odom_msg):
    pose_point_x = odom_msg.pose.pose.position.x
    pose_point_y = odom_msg.pose.pose.position.y
    pose_point_z = odom_msg.pose.pose.position.z
    pose_quat_x = odom_msg.pose.pose.orientation.x
    pose_quat_y = odom_msg.pose.pose.orientation.y
    pose_quat_z = odom_msg.pose.pose.orientation.z
    pose_quat_w = odom_msg.pose.pose.orientation.w

    return pose_point_x, pose_point_y, pose_point_z, pose_quat_w, pose_quat_x, pose_quat_y, pose_quat_z

def main():
    global node 
    flgLoadNet = 1

    # set up ROS
    setup_ros()
    # set up MATLAB connection
    matlab_perception = perception_wrapper.initialize()

    try:
        while True:
            node.spin()
            # make sure we have good data before we try to process it
            if raw_image != None and raw_lidar != None and odom_data != None:
                print("calling matlab PerceptionAlgorithm")
                # get the occupancy_grid from MATLAB code
                [x, y, z, qw, qx, qy, qz] = parse_odom(odom_data)
                occupancy_grid = matlab_perception.perception_wrapper(flgLoadNet, list(raw_image.data), list(raw_lidar.data), x, y, z, qw, qx, qy, qz)
                # publish occupancy grid
                publish_occupancy_grid(occupancy_grid)
            else:
                print("no ros data from subscriptions yet")
            

    except Exception as e:
        print(e)
    finally:
        matlab_perception.terminate()
        node.shutdown()
         
if __name__ == '__main__':
    main()
