from avt_341_types import Image, PointCloud2, OccupancyGrid, Odometry
from avt_341_node_proxy import NodeProxy
import numpy
import matlab.engine

matlab_code_folder = "C:\\path\\to\\nato-avt-314-stack\\semantic_segmentation"
matlab_code_folder = "C:\\Users\\Nic\\Documents\\git\\nato-avt-341-stack\\uab-perception\\semantic_segmentation"

record_bin_file = False
node = None 
pub_occupancy_grid = None
pub_vis_occupancy_grid = None
rawImage = None 
rawLidar = None 
odom_data = None
queue_size = 10

grid_width = 510
grid_height = 160
grid_res = 0.5
grid_llx = -252
grid_lly = -41

class MatlabWrapper():
    def __init__(self, parameters):
        self.matlab_code_folder = parameters['matlab_code_folder']
        print(f"code folder {self.matlab_code_folder}")
        print(f"code folder type {type(self.matlab_code_folder)}")

    def init_matlab_engine(self, connect):
        """runs matlab in the background"""
        if connect == True:
            print("connecting to matlab")
            self.eng = matlab.engine.connect_matlab()
        else:
            print("spawning matlab instance")
            self.eng = matlab.engine.start_matlab()
        s = self.eng.genpath(self.matlab_code_folder)
        self.eng.addpath(s, nargout=0)
               
    def stop_process(self):
        self.disp_msg("entered stop_process")
        self.eng.quit()

    def call_ex_PerceptionAlgorithm(self,flgLoadNNet,rawImage,rawLidar,rawOdom):
        
        print("calling image_to_array")
        ar_image = self.image_to_array(rawImage)
        ar_lidar = self.point_cloud_to_array(rawLidar)
        # parse the odometry variables from the rawOdom message
        pose_point_x, pose_point_y, pose_point_z, pose_quat_w, pose_quat_x, pose_quat_y, pose_quat_z = self.parse_odom(rawOdom)

        print("calling self.eng.ex_PerceptionAlgorithm")
        return self.eng.perception_wrapper(flgLoadNNet, ar_image, ar_lidar, pose_point_x, pose_point_y, pose_point_z, pose_quat_w, pose_quat_x, pose_quat_y, pose_quat_z )

    def call_ex_record_bin_file(self,rawImage,rawLidar):
        
        print("calling image_to_array")
        ar_image = self.image_to_array(rawImage)
        ar_lidar = self.point_cloud_to_array(rawLidar)

        print("calling self.eng.ex_record_bin_file")
        return self.eng.ex_record_bin_file(ar_image, ar_lidar)

    def parse_odom(self, odom_msg):
        pose_point_x = odom_msg.pose.pose.position.x
        pose_point_y = odom_msg.pose.pose.position.y
        pose_point_z = odom_msg.pose.pose.position.z
        pose_quat_x = odom_msg.pose.pose.orientation.x
        pose_quat_y = odom_msg.pose.pose.orientation.y
        pose_quat_z = odom_msg.pose.pose.orientation.z
        pose_quat_w = odom_msg.pose.pose.orientation.w

        return pose_point_x, pose_point_y, pose_point_z, pose_quat_w, pose_quat_x, pose_quat_y, pose_quat_z


    def image_to_array(self, rawImage):
        print(f"{rawImage.encoding}")
        print(f"{rawImage.header}")
        print(f"{rawImage.height}")
        print(f"{rawImage.is_bigendian}")
        print(f"{rawImage.step}")
        print(f"{rawImage.width}")
        ar_image = matlab.uint8(rawImage.data)
        return ar_image

    def point_cloud_to_array(self, rawPointCloud):
        print(f"{rawPointCloud.height}")
        print(f"{rawPointCloud.width}")
        print(f"{rawPointCloud.fields}")
        print(f"{rawPointCloud.is_bigendian}")
        print(f"{rawPointCloud.point_step}")
        print(f"{rawPointCloud.row_step}")
        print(f"{len(rawPointCloud.data)}")
        print(f"{rawPointCloud.is_dense}")
        ar_point_cloud = matlab.uint8(rawPointCloud.data)
        return ar_point_cloud

    def list_props(self, obj):
        props = dir(obj)
        for prop in props:
            if len(prop) > 100:
                prop = prop[0:100]
            print(prop)

    def call_test(self):
        return self.eng.ex_test(nargout=1)

def start_matlab_proc(matlab_code_folder, connect):
    parameters = {}
    parameters['matlab_code_folder'] = matlab_code_folder
    proc = MatlabWrapper(parameters)
    proc.init_matlab_engine(connect)
    return proc 

def listener_callback_camera(image_msg):
    """listen for camera messages.
       sensor_msgs/msg/Image
    """
    print(f"camera callback  {type(image_msg)}" )
    global rawImage
    rawImage = image_msg
    
def listener_callback_lidar(lidar_msg):
    """listen for lidar messages.
       sensor_msgs/msg/PointCloud2
    """
    print(f"lidar callback  {type(lidar_msg)}")
    global rawLidar
    rawLidar = lidar_msg

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

    logger.info("node running")

def setup_matlab():
    proc = start_matlab_proc(matlab_code_folder, connect=False)   
    return proc   

def main():
    global node 
    flgLoadNet = 1

    # set up ROS
    setup_ros()

    # set up MATLAB connection
    proc = setup_matlab()

    print("Starting main loop")
    try:
        if record_bin_file == False:
            while True:
                node.spin()
                # make sure we have good data before we try to process it
                if rawImage != None and rawLidar != None and odom_data != None:
                    print("calling matlab PerceptionAlgorithm")
                    # get the occupancy_grid from MATLAB code
                    occupancy_grid = proc.call_ex_PerceptionAlgorithm(flgLoadNet, rawImage, rawLidar, odom_data)
                    # publish the og to the network
                    publish_occupancy_grid(occupancy_grid)
                else:
                    print("no ros data from subscriptions yet")
        else:
            while True:
                node.spin()
                # make sure we have good data before we try to process it
                if rawImage != None and rawLidar != None:
                    print("calling matlab call_ex_record_bin_file")
                    # get the occupancy_grid from MATLAB code
                    occupancy_grid = proc.call_ex_record_bin_file(rawImage, rawLidar)
                    # we only need one frame
                    break

                else:
                    print("no ros data from subscriptions yet")
            

    except Exception as e:
        print(e)
    finally:
        node.shutdown()
         
if __name__ == '__main__':
    main()
