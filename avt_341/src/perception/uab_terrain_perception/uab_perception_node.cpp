#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/lib_uab_perception_wrapper.h"
#include "mclcppclass.h"
#include "mclmcrrt.h"
#include <matrix.h>
#include <vector>
#include <array>
#include <math.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

const uint8_t TERRAIN_GRID_DEFAULT_VAL = 50;
const uint8_t OBSTACLE_GRID_DEFAULT_VAL = 0;

geometry_msgs::msg::TransformStamped lidar_to_vbox_tf;
geometry_msgs::msg::TransformStamped lidar_to_camera_tf;

avt_341::msg::Odometry current_pose;
bool odom_received = false;

avt_341::msg::PointCloud2 pc;
bool pc_received = false;

avt_341::msg::Image img;
bool img_received = false;

sensor_msgs::msg::CameraInfo cam_info;
bool cam_info_received = false;

std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom)
{
    current_pose = *rcv_odom;
    odom_received = true;
}

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_pc)
{
    pc = *rcv_pc;
    pc_received = true;
}

void ImageCallback(avt_341::msg::ImagePtr rcv_img)
{
    img = *rcv_img;
    img_received = true;
}

void CameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    cam_info = *msg;
    cam_info_received = true;
}

bool allMsgsReceived()
{
    return odom_received && pc_received && img_received && cam_info_received;
}

static mxArray* toDoubleColumn(const std::vector<double> &vec)
{
    mxArray *array = mxCreateDoubleMatrix(vec.size(), 1, mxREAL);
    double *data = mxGetPr(array);
    //indexing starts at 1 for matlab arrays
    for (size_t i = 1; i < vec.size(); i++)
    {
        data[i] = vec[i];
    }

    return array;
}

static mxArray* toDoubleColumn(const std::array<double, 4>& vec)
{
    mxArray* array = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* data = mxGetPr(array);
    for (size_t i = 0; i < 4; ++i) 
    {
        data[i] = vec[i];
    }

    return array;
}

//packages up an Image message into matlab-friendly struct
static mwArray imageToMwArray(const avt_341::msg::Image &img)
{
    const size_t num_fields = 4;
    const char *field_names[]
    {
        "width",
        "height",
        "encoding",
        "data"
    };
    
    const mwSize height = static_cast<mwSize>(img.height);
    const mwSize width = static_cast<mwSize>(img.width);
    mxClassID encoding = mxUINT8_CLASS;

    mwArray mw_img(1, 1, num_fields, field_names);
    mw_img("width", 1, 1) = mwArray((double)img.height);
    mw_img("height", 1, 1) = mwArray((double)img.width);
    mw_img("encoding", 1, 1) = mwArray(img.encoding.c_str());

    std::vector<uint8_t> img_data(std::begin(img.data), std::end(img.data));
    mwArray mw_img_data(1, img_data.size(), mxUINT8_CLASS);
    mw_img_data.SetData(&img_data.front(), img_data.size());
    mw_img("data", 1, 1) = mw_img_data;

    return mw_img;
}

//packages up PointCloud2 message into matlab-friendly struct
static mwArray pcToMwArray(const avt_341::msg::PointCloud2 &pc)
{
    const size_t num_fields = 5;
    const char* field_names[]
    {
        "height",
        "width",
        "point_step",
        "row_step",
        "data"
    };

    mwArray mw_point_cloud(1, 1, num_fields, field_names);
    mw_point_cloud("height", 1, 1) = mwArray((double)pc.height);
    mw_point_cloud("width", 1, 1) = mwArray((double)pc.width);
    mw_point_cloud("point_step", 1, 1) = mwArray((double)pc.point_step);
    mw_point_cloud("row_step", 1, 1) = mwArray((double)pc.row_step);
    
    std::vector<uint8_t> pc_data(std::begin(pc.data), std::end(pc.data));
    mwArray pcData(1, pc_data.size(), mxUINT8_CLASS);
    pcData.SetData(&pc_data.front(), pc_data.size());

    mw_point_cloud("data", 1, 1) = pcData;

    return mw_point_cloud;
}

//packages up CameraInfo message into matlab-friendly struct
static mwArray cameraInfoToMwArray(const sensor_msgs::msg::CameraInfo &cam_info)
{
    const size_t num_fields = 11;
    const char *field_names[] =
    {
        "height",
        "width",
        "distortion_model",
        "d",
        "k",
        "r",
        "p",
        "binning_x",
        "binning_y",
        "roi",
        "header"
    };

    mwArray mw_camera_info(1, 1, num_fields, field_names);
    mw_camera_info("height", 1, 1) = mwArray((double)cam_info.height);
    mw_camera_info("width", 1, 1) = mwArray((double)cam_info.width);
    mw_camera_info("binning_x", 1, 1) = mwArray((double)cam_info.binning_x);
    mw_camera_info("binning_y", 1, 1) = mwArray((double)cam_info.binning_y);
    mw_camera_info("distortion_model", 1, 1) = mwArray(cam_info.distortion_model.c_str());
    mwArray d((mwSize)cam_info.d.size(), 1, mxDOUBLE_CLASS);
    if (!cam_info.d.empty())
    {
        d.SetData(const_cast<double*>(cam_info.d.data()), cam_info.d.size());
    }
    mw_camera_info("d", 1, 1) = d;

    mwArray k(9, 1, mxDOUBLE_CLASS);
    k.SetData(const_cast<double*>(cam_info.k.data()), 9);
    mw_camera_info("k", 1, 1) = k;

    mwArray r(9, 1, mxDOUBLE_CLASS);
    r.SetData(const_cast<double*>(cam_info.r.data()), 9);
    mw_camera_info("r", 1, 1) = r;

    mwArray p(12, 1, mxDOUBLE_CLASS);
    p.SetData(const_cast<double*>(cam_info.p.data()), 12);
    mw_camera_info("p", 1, 1) = p;

    return mw_camera_info;
}

//packages up a point and quaternion message into matlab-friendly struct
static mwArray translationOrientationToMwArray(
    const geometry_msgs::msg::Point &translation,
    const geometry_msgs::msg::Quaternion &orientation)
{
    const char *translation_field_names[] = {"x", "y", "z"};
    mwArray mw_translation(1, 1, 3, translation_field_names);
    mw_translation(translation_field_names[0], 1, 1) = mwArray((double)translation.x);
    mw_translation(translation_field_names[1], 1, 1) = mwArray((double)translation.y);
    mw_translation(translation_field_names[2], 1, 1) = mwArray((double)translation.z);

    const char *orientation_field_names[] = {"w", "x", "y", "z"};
    mwArray mw_orientation(1, 1, 4, orientation_field_names);
    mw_orientation(orientation_field_names[0], 1, 1) = mwArray((double)orientation.w);
    mw_orientation(orientation_field_names[1], 1, 1) = mwArray((double)orientation.x);
    mw_orientation(orientation_field_names[2], 1, 1) = mwArray((double)orientation.y);
    mw_orientation(orientation_field_names[3], 1, 1) = mwArray((double)orientation.z);

    const char *field_names[]
    {
        "translation",
        "orientation"
    };

    mwArray mwTform(1, 1, 2, field_names);
    mwTform(field_names[0], 1, 1) = mw_translation;
    mwTform(field_names[1], 1, 1) = mw_orientation;

    return mwTform;
}

static mwArray odomToMwArray(const avt_341::msg::Odometry &odom)
{
    return translationOrientationToMwArray(odom.pose.pose.position, odom.pose.pose.orientation);
}

//packages up transform message into matlab-friendly struct
static mwArray tfToMwArray(const geometry_msgs::msg::TransformStamped &tf)
{
    const char *translation_field_names[] = {"x", "y", "z"};
    mwArray mwTranslation(1, 1, 3, translation_field_names);
    mwTranslation(translation_field_names[0], 1, 1) = mwArray((double)tf.transform.translation.x);
    mwTranslation(translation_field_names[1], 1, 1) = mwArray((double)tf.transform.translation.y);
    mwTranslation(translation_field_names[2], 1, 1) = mwArray((double)tf.transform.translation.z);

    const char *orientation_field_names[] = {"w", "x", "y", "z"};
    mwArray mwOrientation(1, 1, 4, orientation_field_names);
    mwOrientation(orientation_field_names[0], 1, 1) = mwArray((double)tf.transform.rotation.w);
    mwOrientation(orientation_field_names[1], 1, 1) = mwArray((double)tf.transform.rotation.x);
    mwOrientation(orientation_field_names[2], 1, 1) = mwArray((double)tf.transform.rotation.y);
    mwOrientation(orientation_field_names[3], 1, 1) = mwArray((double)tf.transform.rotation.z);

    const char *fieldNames[]
    {
        "translation",
        "rotation"
    };

    mwArray mwTform(1, 1, 2, fieldNames);
    mwTform(fieldNames[0], 1, 1) = mwTranslation;
    mwTform(fieldNames[1], 1, 1) = mwOrientation;

    return mwTform;
}

/*
* GetCostmapFromMatlab packages up all the data from ROS (odometry, pointcloud, and image), calls
* the semantic segmentation model in Matlab, and populates an array of cost values based on
* terrain traversability
*/
void GetCostmapFromMatlab(float width,
                            float height,
                            float res,
                            float grid_llx,
                            float grid_lly,
                            bool invert_lidar_z_rot,
                            std::vector<int8_t> &terrain_grid,
                            std::vector<double> &terrain_grid_modified_idxs,
                            std::vector<int8_t> &obstacle_grid,
                            std::vector<double> &obstacle_grid_modified_idxs,
                            std::string lidar_frame_id,
                            std::string odom_frame_id,
                            std::string camera_frame_id)
{
    bool received_tform = false;
    while (!received_tform)
    {
        try
        {
            if (tf_buffer->canTransform(odom_frame_id, lidar_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0)))
            {
                lidar_to_vbox_tf = tf_buffer->lookupTransform(odom_frame_id, lidar_frame_id, tf2::TimePointZero);
                lidar_to_camera_tf = tf_buffer->lookupTransform(camera_frame_id, lidar_frame_id, tf2::TimePointZero);
                received_tform = true;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            std::cout << "TF lookup failed: " << ex.what() << std::endl;
        }
    }

    //odometry
    mwArray mw_odom = odomToMwArray(current_pose);

    //raw pointcloud
    mwArray mw_pc = pcToMwArray(pc);

    //raw image
    mwArray mw_img = imageToMwArray(img);
    //camera intrinsics
    mwArray mw_camera_info = cameraInfoToMwArray(cam_info);

    //lidar to camera transform
    mwArray mw_lidar_to_camera_tform = tfToMwArray(lidar_to_camera_tf);
    //transform from lidar to vbox for mapping pointcloud points to the grid
    mwArray mw_lidar_to_vbox_tform = tfToMwArray(lidar_to_vbox_tf);

    //used to invert pointcloud in the case of backwards mounting
    mwArray mw_invert_lidar_z_rot(invert_lidar_z_rot);

    //grid parameters
    mwArray mw_grid_width(width);
    mwArray mw_grid_height(height);
    mwArray mw_grid_res(res);
    //lower left corner grid offset in meters (x/east direction)
    mwArray mw_grid_llx(grid_llx);
    //lower right corner grid offset in meters (y/north direction)
    mwArray mw_grid_lly(grid_lly);

    try
    {
        //output variables
        mwArray mw_terrain_costmap;
        mwArray mw_terrain_sub_grid_size;
        mwArray mw_terrain_grid_modified_idxs;
        mwArray mw_obstacle_grid;
        mwArray mw_obstacle_grid_size;
        mwArray mw_obstacle_grid_modified_idxs;
        perception_wrapper(
                            6, //number of output arguments
                            mw_terrain_costmap, //terrain costmap output
                            mw_terrain_sub_grid_size, //size of terrain costmap output
                            mw_terrain_grid_modified_idxs, //cell indices of grid that were updated this iteration
                            mw_obstacle_grid, //obstacle grid output
                            mw_obstacle_grid_size, //size of obstacle grid output
                            mw_obstacle_grid_modified_idxs, //cell indices of obstacle grid that were updated this iteration
                            mw_img, //image struct
                            mw_pc, //pointcloud struct
                            mw_odom, //odometry struct
                            mw_camera_info, //camera info struct
                            mw_lidar_to_camera_tform, //transform from camera to lidar
                            mw_lidar_to_vbox_tform, //transform from lidar to vbox
                            mw_invert_lidar_z_rot, //handle backwards lidar mounting by inverting rotation about z axis
                            mw_grid_width, mw_grid_height, mw_grid_res, mw_grid_llx, mw_grid_lly //grid parameters
                        );

        // parse terrain sub grid
        uint32_t terrain_sub_grid_size = mw_terrain_sub_grid_size;
        terrain_grid_modified_idxs.resize(terrain_sub_grid_size);
        mw_terrain_grid_modified_idxs.GetData(terrain_grid_modified_idxs.data(), terrain_grid_modified_idxs.size());

        terrain_grid.resize(terrain_sub_grid_size);
        mw_terrain_costmap.GetData(terrain_grid.data(), terrain_grid.size());

        // parse obstacle sub grid
        uint32_t obstacle_sub_grid_size = mw_obstacle_grid_size;
        obstacle_grid_modified_idxs.resize(obstacle_sub_grid_size);
        mw_obstacle_grid_modified_idxs.GetData(obstacle_grid_modified_idxs.data(), obstacle_grid_modified_idxs.size());

        obstacle_grid.resize(obstacle_sub_grid_size);
        mw_obstacle_grid.GetData(obstacle_grid.data(), obstacle_grid.size());
    }
    catch(const mwException& e)
    {
        std::cerr << "Failed to execute Matlab perception_wrapper. " << e.what() << std::endl;
    }
}

void BuildOccupancyGrid(avt_341::msg::OccupancyGrid &grid,
                        float width,
                        float height,
                        float grid_llx,
                        float grid_lly,
                        float res,
                        float default_cell_value)
{
    grid.header.frame_id = "map";
    grid.info.resolution = res;
    grid.info.height = height;
    grid.info.width = width;
    grid.info.origin.position.x = grid_llx;
    grid.info.origin.position.y = grid_lly;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    std::vector<int8_t> initVals(width * height, default_cell_value);
    grid.data = initVals;
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg)
{
    if (msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos)
    {
        reset_called = true;
    }
}

int main(int argc, char *argv[])
{
    auto node = avt_341::node::init_node(argc, argv, "uab_perception_node");

    auto odom_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odom", 10, OdometryCallback);
    auto pc_sub = node->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 2, PointCloudCallback);
    auto img_sub = node->create_subscription<avt_341::msg::Image>("avt_341/camera/image_raw", 10, ImageCallback);
    auto reset_sub = node->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
    auto camera_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("avt_341/camera/camera_info", 10, CameraInfoCallback);
    
    auto seg_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);
    auto occ_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);
    auto reset_ack_pub = node->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_raw_node()->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    float width;
    node->get_parameter("~grid_width", width, 100.0f);
    float height;
    node->get_parameter("~grid_height", height, 100.0f);
    float grid_llx;
    node->get_parameter("~grid_llx", grid_llx, 0.0f);
    float grid_lly;
    node->get_parameter("~grid_lly", grid_lly, 0.0f);
    float grid_res;
    node->get_parameter("~grid_res", grid_res, 1.0f);
    bool publish_occupancy_grid;
    node->get_parameter("~publish_uab_occupancy_grid", publish_occupancy_grid, false);
    bool invert_lidar_z_rot = false;
    node->get_parameter("~invert_lidar_z_rot", invert_lidar_z_rot, false);
    std::string lidar_frame_id;
    node->get_parameter("~lidar_frame_id", lidar_frame_id, std::string("lidar_link"));
    std::string odom_frame_id;
    node->get_parameter("~odom_frame_id", odom_frame_id, std::string("base_link"));
    std::string camera_frame_id;
    node->get_parameter("~camera_frame_id", camera_frame_id, std::string("camera_link"));

    width = width/grid_res;
    height = height/grid_res;

    //initialize matlab runtime
    if (!mclInitializeApplication(NULL, 0))
    {
        std::cerr << "Failed to initialize Matlab Runtime" << std::endl;
        return -1;
    }

    //initialize uab matlab perception model
    if(!lib_uab_perception_wrapperInitialize())
    {
        std::cerr << "Failed to initialize perception_wrapper package" << std::endl;
        return -1;
    }

    //initialize terrain grid with default cell values
    avt_341::msg::OccupancyGrid terrain_grid;
    BuildOccupancyGrid(terrain_grid, width, height, grid_llx, grid_lly, grid_res, TERRAIN_GRID_DEFAULT_VAL);

    //initialize obstacle grid with default cell values
    avt_341::msg::OccupancyGrid obstacle_grid;
    BuildOccupancyGrid(obstacle_grid, width, height, grid_llx, grid_lly, grid_res, OBSTACLE_GRID_DEFAULT_VAL);
    
    avt_341::node::Rate rate(100.0);
    //number of seconds to wait for messages before exiting
    uint16_t timeout_sec = 20;
    while (avt_341::node::ok())
    {
        if(reset_called){
            avt_341::msg::String reset_ack_msg;
            reset_ack_msg.data = avt_341::node::NodeType::Perception;
            reset_ack_pub->publish(reset_ack_msg);
            reset_called = false;
        }
        
        //wait until we've received all necessary messages
        if (!allMsgsReceived())
        {
            std::string waiting_on = "waiting on ";
            if (!odom_received) waiting_on += "odom ";
            if (!pc_received) waiting_on += "pointcloud ";
            if (!img_received) waiting_on += "image ";
            if (!cam_info_received) waiting_on += "camera_info";
            std::cout << waiting_on << std::endl;
            
            avt_341::node::Rate wait(1.0);
            wait.sleep();

            if (--timeout_sec == 0) break;
        }
        else
        {
            //get terrain and obstacle probabilistic costmaps from matlab
            //this returns a sub grid of the full costmap with the indices that were updated
            //to reduce the amount of data that needs to be passed between matlab wrapper and this node
            std::vector<int8_t> terrain_sub_grid;
            std::vector<double> terrain_sub_grid_idxs;
            std::vector<int8_t> obstacle_sub_grid;
            std::vector<double> obstacle_sub_grid_idxs;
            GetCostmapFromMatlab(width,
                height,
                grid_res,
                grid_llx,
                grid_lly,
                invert_lidar_z_rot,
                terrain_sub_grid,
                terrain_sub_grid_idxs,
                obstacle_sub_grid,
                obstacle_sub_grid_idxs,
                lidar_frame_id,
                odom_frame_id,
                camera_frame_id);

            //update the terrain grid with the new cell values
            int c = 0;
            for (auto i : terrain_sub_grid_idxs)
            {
                terrain_grid.data[i] = terrain_sub_grid[c++];
            }

            seg_grid_pub->publish(terrain_grid);

            if (publish_occupancy_grid)
            {
                //update the obstacle grid with the new cell values
                c = 0;
                for (auto i : obstacle_sub_grid_idxs)
                {
                    //thresholding value to clean up obstacle grid
                    //only cell values in the obstacle grid with a probability above this threshold
                    //will be marked as occupied in the published occupancy grid
                    double obstacle_probability_threshold = 95.0;
                    double obstacle_probability_at_cell = obstacle_sub_grid[c++];
                    if (obstacle_probability_at_cell >= obstacle_probability_threshold)
                    {
                        obstacle_grid.data[i] = obstacle_probability_at_cell;
                    }
                }

                occ_grid_pub->publish(obstacle_grid);
            }
        }
        node->spin_some();
        rate.sleep();
    }

    mclTerminateApplication();
    return 0;
}
