// The MATLAB MCR headers pull in <windows.h> on Windows, which defines min/max as
// macros and breaks std::min/std::max (error C2589). Define NOMINMAX before any
// include so windows.h skips those macros.
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/core/frame_id_collection.hpp"
#include "avt_341_nav/node/node_types.h"
#include "avt_341_nav/node/node_utils.h"
#include "avt_341_nav/perception/lib_uab_perception_wrapper.h"
#include <avt_341_nav/uab_perception_params_service.hpp>
#include "mclcppclass.h"
#include "mclmcrrt.h"
#include <string>
#include <vector>
#include <array>
#include <math.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#ifdef GTE_ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#include "avt_341_nav/node/tf_interface.h"
#endif

const uint8_t TERRAIN_GRID_DEFAULT_VAL = 50;
const uint8_t OBSTACLE_GRID_DEFAULT_VAL = 0;

rclcpp::Node::SharedPtr node;
std::shared_ptr<avt_341_nav::node::TfInterface> tf;
geometry_msgs::msg::TransformStamped lidar_to_base_link_tf;
geometry_msgs::msg::TransformStamped lidar_to_camera_tf;

nav_msgs::msg::Odometry current_pose;
bool odom_received = false;

sensor_msgs::msg::PointCloud2 pc;
bool pc_received = false;

sensor_msgs::msg::Image img;
bool img_received = false;

sensor_msgs::msg::CameraInfo cam_info;
bool cam_info_received = false;

std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom)
{
    current_pose = *rcv_odom;
    odom_received = true;
}

void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr rcv_pc)
{
    pc = *rcv_pc;
    pc_received = true;
}

void ImageCallback(sensor_msgs::msg::Image::ConstSharedPtr rcv_img)
{
    img = *rcv_img;
    img_received = true;
}

void CameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
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
    for (size_t i = 0; i < vec.size(); i++)
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
static mwArray imageToMwArray(const sensor_msgs::msg::Image &img)
{
    const size_t num_fields = 5;
    const char *field_names[]
    {
        "width",
        "height",
        "encoding",
        "data",
        "step"
    };

    const mwSize height = static_cast<mwSize>(img.height);
    const mwSize width = static_cast<mwSize>(img.width);
    mxClassID encoding = mxUINT8_CLASS;

    mwArray mw_img(1, 1, num_fields, field_names);
    mw_img("width", 1, 1) = mwArray((double)img.width);
    mw_img("height", 1, 1) = mwArray((double)img.height);
    mw_img("encoding", 1, 1) = mwArray(img.encoding.c_str());
    mw_img("step", 1,1 ) = mwArray((double)img.step);

    std::vector<uint8_t> img_data(std::begin(img.data), std::end(img.data));
    mwArray mw_img_data(1, img_data.size(), mxUINT8_CLASS);
    mw_img_data.SetData(&img_data.front(), img_data.size());
    mw_img("data", 1, 1) = mw_img_data;

    return mw_img;
}

//extracts the xyz channels of a PointCloud2 into an Nx3 (x|y|z columns) matlab
//matrix. Decoding here is cheaper than serializing the whole cloud into matlab
//and de-serializing it on the matlab side.
static mwArray pcToXyzMwArray(const sensor_msgs::msg::PointCloud2 &pc)
{
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : pc.fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
    }

    int num_points = pc.width * pc.height;
    std::vector<double> xyz_data(num_points * 3, 0.0);
    if (x_offset >= 0 && y_offset >= 0 && z_offset >= 0) {
        for (int i = 0; i < num_points; i++) {
            int base = i * pc.point_step;
            float x, y, z;
            memcpy(&x, &pc.data[base + x_offset], sizeof(float));
            memcpy(&y, &pc.data[base + y_offset], sizeof(float));
            memcpy(&z, &pc.data[base + z_offset], sizeof(float));
            xyz_data[i] = x;
            xyz_data[i + num_points] = y;
            xyz_data[i + 2 * num_points] = z;
        }
    }
    mwArray mwXYZ(num_points, 3, mxDOUBLE_CLASS);
    mwXYZ.SetData(xyz_data.data(), xyz_data.size());

    return mwXYZ;
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

static mwArray odomToMwArray(const nav_msgs::msg::Odometry &odom)
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

//converts the MATLAB segmentation image (uint8 [height x width x 3], stored
//column-major and planar) into an interleaved rgb8 cv::Mat
static cv::Mat segMaskToMat(mwArray &mw_mask)
{
    mwArray dims = mw_mask.GetDimensions();
    if (dims.NumberOfElements() != 3)
    {
        throw cv::Exception(0, "expected a 3-D HxWx3 segmentation mask",
                             "segMaskToMat", __FILE__, __LINE__);
    }
    mwArray height_arr = dims(1, 1);
    mwArray width_arr = dims(1, 2);
    mwArray channels_arr = dims(1, 3);
    const int height = height_arr;
    const int width = width_arr;
    const int num_channels = channels_arr;
    if (num_channels != 3 || height <= 0 || width <= 0)
    {
        throw cv::Exception(0, "expected a 3-D HxWx3 segmentation mask",
                             "segMaskToMat", __FILE__, __LINE__);
    }
    const int num_pixels = height * width;

    std::vector<uint8_t> buf(static_cast<size_t>(num_pixels) * 3);
    mw_mask.GetData(buf.data(), buf.size());

    //each MATLAB channel plane is column-major, i.e. the transpose of the image
    //when read row-major; transpose each plane and merge into an rgb image
    std::vector<cv::Mat> channels;
    channels.reserve(3);
    for (int ch = 0; ch < 3; ch++)
    {
        cv::Mat plane_t(width, height, CV_8UC1, buf.data() + static_cast<size_t>(ch) * num_pixels);
        channels.push_back(plane_t.t());
    }

    cv::Mat rgb;
    cv::merge(channels, rgb);
    return rgb;
}

//builds an rgb8 image message from an rgb cv::Mat, carrying the same header
//(stamp/frame) as the source camera image
static sensor_msgs::msg::Image matToRgbMsg(const sensor_msgs::msg::Image &reference, const cv::Mat &rgb)
{
    return *cv_bridge::CvImage(reference.header, "rgb8", rgb).toImageMsg();
}

//derives a sibling debug topic from the camera topic by replacing the final path
//segment, e.g. "flir_camera/image_raw" + "segmentation" -> "flir_camera/segmentation"
static std::string makeSiblingTopic(const std::string &camera_topic, const std::string &suffix)
{
    size_t last_slash = camera_topic.find_last_of('/');
    std::string base = (last_slash == std::string::npos) ? std::string("") : camera_topic.substr(0, last_slash + 1);
    return base + suffix;
}

/*
* GetCostmapFromMatlab packages up all the data from ROS (odometry, pointcloud, and image), calls
* the semantic segmentation model in Matlab, and populates an array of cost values based on
* terrain traversability
*/
void GetCostmapFromMatlab(float width_cells,
                            float height_cells,
                            float res,
                            float grid_llx,
                            float grid_lly,
                            double brightness_offset,
                            std::vector<int8_t> &terrain_grid,
                            std::vector<double> &terrain_grid_modified_idxs,
                            std::vector<int8_t> &obstacle_grid,
                            std::vector<double> &obstacle_grid_modified_idxs,
                            std::string lidar_frame_id,
                            std::string odom_frame_id,
                            std::string camera_frame_id,
                            bool debug_vis_segmentation,
                            cv::Mat &seg_img)
{
    auto isValidTransform = [](const geometry_msgs::msg::TransformStamped &tf) -> bool
    {
        const auto &t = tf.transform.translation;
        const auto &q = tf.transform.rotation;
        if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            return false;
        }
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        //unit-norm quaternion expected; e.g. 0 from a default-constructed transform is invalid
        return std::abs(norm - 1.0) < 0.05;
    };

    bool received_tform = false;
    auto last_wait_log = std::chrono::steady_clock::now();
    while (!received_tform && rclcpp::ok())
    {
        try
        {
            if (tf_buffer->canTransform(odom_frame_id, lidar_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0)) &&
                tf_buffer->canTransform(camera_frame_id, lidar_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0)))
            {
                geometry_msgs::msg::TransformStamped new_lidar_to_base_link_tf =
                    tf_buffer->lookupTransform(odom_frame_id, lidar_frame_id, tf2::TimePointZero);
                geometry_msgs::msg::TransformStamped new_lidar_to_camera_tf =
                    tf_buffer->lookupTransform(camera_frame_id, lidar_frame_id, tf2::TimePointZero);

                if (isValidTransform(new_lidar_to_base_link_tf) && isValidTransform(new_lidar_to_camera_tf))
                {
                    lidar_to_base_link_tf = new_lidar_to_base_link_tf;
                    lidar_to_camera_tf = new_lidar_to_camera_tf;
                    received_tform = true;
                }
                else
                {
                    std::cerr << "Rejected degenerate lidar_to_base_link/lidar_to_camera transform "
                                  "(non-finite or non-unit-norm rotation); retrying." << std::endl;
                }
            }
            else
            {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration<double>(now - last_wait_log).count() >= 5.0)
                {
                    std::cerr << "Still waiting on transform " << lidar_frame_id << " -> " << odom_frame_id
                              << " and " << lidar_frame_id << " -> " << camera_frame_id << std::endl;
                    last_wait_log = now;
                }
            }
        }
        catch (const tf2::TransformException &ex)
        {
            std::cout << "TF lookup failed: " << ex.what() << std::endl;
        }
    }
    if (!rclcpp::ok())
    {
        return;
    }

    //odometry
    mwArray mw_odom = odomToMwArray(current_pose);

    //raw pointcloud xyz, expected to be in lidar coordinate frame
    mwArray mwXYZ;
    if (pc.header.frame_id != lidar_frame_id) {
        sensor_msgs::msg::PointCloud2 pc_out;
        if (!tf->transform_cloud(pc, pc_out, lidar_frame_id)) {
            return;
        }
        mwXYZ = pcToXyzMwArray(pc_out);
    }else {
        mwXYZ = pcToXyzMwArray(pc);
    }

    //raw image
    mwArray mw_img = imageToMwArray(img);
    //camera intrinsics
    mwArray mw_camera_info = cameraInfoToMwArray(cam_info);

    //lidar to camera transform
    mwArray mw_lidar_to_camera_tform = tfToMwArray(lidar_to_camera_tf);
    //transform from lidar to robot base link for mapping pointcloud points to the grid
    mwArray mw_lidar_to_base_link_tform = tfToMwArray(lidar_to_base_link_tf);

    //used to correct color to match model expectations
    mwArray mw_brightness_offset(brightness_offset);

    //grid parameters
    mwArray mw_grid_width(width_cells);
    mwArray mw_grid_height(height_cells);
    mwArray mw_grid_res(res);
    //lower left corner grid offset in meters (x/east direction)
    mwArray mw_grid_llx(grid_llx);
    //lower right corner grid offset in meters (y/north direction)
    mwArray mw_grid_lly(grid_lly);

    //when set, the wrapper returns the (small) segmentation image for debug visualization
    mwArray mw_debug_vis_segmentation(static_cast<double>(debug_vis_segmentation));

    std::atomic<bool> call_finished{false};
    std::mutex watchdog_mutex;
    std::condition_variable watchdog_cv;
    std::thread watchdog([&]()
    {
        std::unique_lock<std::mutex> lock(watchdog_mutex);
        int waited = 0;
        while (!watchdog_cv.wait_for(lock, std::chrono::seconds(5), [&] { return call_finished.load(); }))
        {
            waited += 5;
            std::cerr << "perception_wrapper has not returned after " << waited << "s" << std::endl;
        }
    });

    try
    {
        //output variables
        mwArray mw_terrain_costmap;
        mwArray mw_terrain_sub_grid_size;
        mwArray mw_terrain_grid_modified_idxs;
        mwArray mw_obstacle_grid;
        mwArray mw_obstacle_grid_size;
        mwArray mw_obstacle_grid_modified_idxs;
        //only requested (and only populated by the wrapper) when debug visualization is enabled
        mwArray mw_debug_segmentation_mask;
        perception_wrapper(
                             7, //number of output arguments
                            mw_terrain_costmap, //terrain costmap output
                            mw_terrain_sub_grid_size, //size of terrain costmap output
                            mw_terrain_grid_modified_idxs, //cell indices of grid that were updated this iteration
                            mw_obstacle_grid, //obstacle grid output
                            mw_obstacle_grid_size, //size of obstacle grid output
                            mw_obstacle_grid_modified_idxs, //cell indices of obstacle grid that were updated this iteration
                            mw_debug_segmentation_mask, //small segmentation image (only when debug_vis_segmentation)
                            mw_img, //image struct
                            mwXYZ, //pointcloud xyz (Nx3 matrix)
                            mw_odom, //odometry struct
                            mw_camera_info, //camera info struct
                            mw_lidar_to_camera_tform, //transform from camera to lidar
                            mw_lidar_to_base_link_tform, //transform from lidar to robot base link
                            mw_brightness_offset, //used to correct color to match model expectations
                            mw_grid_width, mw_grid_height, mw_grid_res, mw_grid_llx, mw_grid_lly, //grid parameters
                            mw_debug_vis_segmentation //return the segmentation image for debug visualization
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

        // decode the (small) segmentation image for debug visualization
        if (debug_vis_segmentation && !mw_debug_segmentation_mask.IsEmpty())
        {
            seg_img = segMaskToMat(mw_debug_segmentation_mask);
        }
    }
    catch(const mwException& e)
    {
        std::cerr << "Failed to execute Matlab perception_wrapper. " << e.what() << std::endl;
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "Failed to decode debug segmentation mask. " << e.what() << std::endl;
        seg_img = cv::Mat();
    }
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex);
        call_finished = true;
    }
    watchdog_cv.notify_one();
    watchdog.join();
}

void BuildOccupancyGrid(nav_msgs::msg::OccupancyGrid &grid,
                        float width_cells,
                        float height_cells,
                        float grid_llx,
                        float grid_lly,
                        float res,
                        float default_cell_value)
{
    grid.header.frame_id = "map";
    grid.info.resolution = res;
    grid.info.height = height_cells;
    grid.info.width = width_cells;
    grid.info.origin.position.x = grid_llx;
    grid.info.origin.position.y = grid_lly;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    std::vector<int8_t> initVals(width_cells * height_cells, default_cell_value);
    grid.data = initVals;
}

nav_msgs::msg::OccupancyGrid ExtractGridWindow(const nav_msgs::msg::OccupancyGrid &src,
                                              double center_x,
                                              double center_y,
                                              float max_width,
                                              float max_height,
                                              float default_cell_value)
{
    const float res = src.info.resolution;
    const int src_width_cells = src.info.width;
    const int src_height_cells = src.info.height;
    const double src_llx = src.info.origin.position.x;
    const double src_lly = src.info.origin.position.y;
    const int window_width_cells = static_cast<int>(max_width/res);
    const int window_height_cells = static_cast<int>(max_height/res);

    // Desired lower-left corner of the window in src cell coordinates (may fall outside src)
    int start_col = static_cast<int>(std::round((center_x - max_width / 2.0 - src_llx) / res));
    int start_row = static_cast<int>(std::round((center_y - max_height / 2.0 - src_lly) / res));
    int end_col = start_col + window_width_cells;
    int end_row = start_row + window_height_cells;

    // Clamp the corners to the src bounds so the window does not extend past the overall grid
    start_col = std::max(0, start_col);
    start_row = std::max(0, start_row);
    end_col = std::min(src_width_cells, end_col);
    end_row = std::min(src_height_cells, end_row);

    const int win_width_cells = std::max(0, end_col - start_col);
    const int win_height_cells = std::max(0, end_row - start_row);

    // Origin of the clamped window, snapped to the src cell grid
    const double win_llx = src_llx + start_col * res;
    const double win_lly = src_lly + start_row * res;

    nav_msgs::msg::OccupancyGrid window;
    BuildOccupancyGrid(window, win_width_cells, win_height_cells, win_llx, win_lly, res, default_cell_value);

    // Copy the overlapping cells from src into the window
    for (int r = 0; r < win_height_cells; r++)
    {
        for (int c = 0; c < win_width_cells; c++)
        {
            int src_idx = (start_row + r) * src_width_cells + (start_col + c);
            int dst_idx = r * win_width_cells + c;
            window.data[dst_idx] = src.data[src_idx];
        }
    }

    return window;
}

bool reset_called = false;
void ResetCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data.find(avt_341_nav::node::NodeType::Perception) != std::string::npos)
    {
        reset_called = true;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::on_shutdown([]()
    {
        std::thread([]()
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cerr << "uab_perception_node: still running 5s after shutdown was requested "
                         "(likely blocked in the Matlab perception call); forcing exit." << std::endl;
            std::_Exit(1);
        }).detach();
    });

    if (!mclInitializeApplication(NULL, 0))
    {
        std::cerr << "Failed to initialize Matlab Runtime" << std::endl;
        return -1;
    }

    if (!lib_uab_perception_wrapperInitialize())
    {
        std::cerr << "Failed to initialize perception_wrapper package" << std::endl;
        return -1;
    }

    try
    {
        node = rclcpp::Node::make_shared("uab_perception_node");
        tf = std::make_shared<avt_341_nav::node::TfInterface>(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "uab_perception_node: node construction failed (likely shutdown racing "
                     "startup): " << e.what() << std::endl;
        return rclcpp::ok() ? -1 : 0;
    }

    avt_341_nav::params::uab_perception::ParamsListener param_listener(node);
    const auto params = param_listener.get_params();

    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("avt_341/odom", 10, OdometryCallback);
    auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("avt_341/points", 2, PointCloudCallback);
    auto img_sub = node->create_subscription<sensor_msgs::msg::Image>(
        params.camera_topic, 10, ImageCallback);
    auto reset_sub = node->create_subscription<std_msgs::msg::String>("avt_341/reset", 10, ResetCallback);
    auto camera_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("avt_341/camera/camera_info", 10, CameraInfoCallback);

    auto seg_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);
    auto occ_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);
    auto reset_ack_pub = node->create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    const avt_341_nav::core::FrameIdCollection frame_ids(
        params.frames, avt_341_nav::node::GetLeadingNodeNamespace(node));
    const std::string lidar_frame_id = frame_ids.Lidar();
    const std::string odom_frame_id = frame_ids.BaseLink();
    const std::string camera_frame_id = frame_ids.Camera();

    //debug segmentation visualization topics, derived from the camera topic by
    //replacing its final path segment (e.g. flir_camera/image_raw ->
    //flir_camera/segmentation and flir_camera/segmentation_overlay)
    const std::string seg_topic =
        makeSiblingTopic(params.camera_topic, "segmentation");
    const std::string seg_overlay_topic =
        makeSiblingTopic(params.camera_topic, "segmentation_overlay");
    decltype(node->create_publisher<sensor_msgs::msg::Image>(seg_topic, 1)) seg_pub, seg_overlay_pub;
    if (params.debug_vis_segmentation)
    {
        seg_pub = node->create_publisher<sensor_msgs::msg::Image>(seg_topic, 1);
        seg_overlay_pub = node->create_publisher<sensor_msgs::msg::Image>(seg_overlay_topic, 1);
        std::cout << "debug_vis_segmentation enabled. Publishing debug topics: "
                  << seg_topic << ", " << seg_overlay_topic << std::endl;
    }

    const int width_cells =
        static_cast<int>(params.costmap.geometry.width / params.costmap.geometry.res);
    const int height_cells =
        static_cast<int>(params.costmap.geometry.height / params.costmap.geometry.res);

    //initialize terrain grid with default cell values
    nav_msgs::msg::OccupancyGrid terrain_grid;
    BuildOccupancyGrid(
        terrain_grid, width_cells, height_cells,
        static_cast<float>(params.costmap.geometry.llx),
        static_cast<float>(params.costmap.geometry.lly),
        static_cast<float>(params.costmap.geometry.res), TERRAIN_GRID_DEFAULT_VAL);

    //initialize obstacle grid with default cell values
    nav_msgs::msg::OccupancyGrid obstacle_grid;
    BuildOccupancyGrid(
        obstacle_grid, width_cells, height_cells,
        static_cast<float>(params.costmap.geometry.llx),
        static_cast<float>(params.costmap.geometry.lly),
        static_cast<float>(params.costmap.geometry.res), OBSTACLE_GRID_DEFAULT_VAL);

    rclcpp::Rate rate(100.0);
    //number of seconds to wait for messages before exiting
    uint16_t timeout_sec = 20;
    while (rclcpp::ok())
    {
        if(reset_called){
            std_msgs::msg::String reset_ack_msg;
            reset_ack_msg.data = avt_341_nav::node::NodeType::Perception;
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

            rclcpp::Rate wait(1.0);
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
            //small (un-upsampled) rgb segmentation image, populated only in debug mode
            cv::Mat seg_img;

            GetCostmapFromMatlab(
                width_cells,
                height_cells,
                static_cast<float>(params.costmap.geometry.res),
                static_cast<float>(params.costmap.geometry.llx),
                static_cast<float>(params.costmap.geometry.lly),
                params.brightness_offset,
                terrain_sub_grid,
                terrain_sub_grid_idxs,
                obstacle_sub_grid,
                obstacle_sub_grid_idxs,
                lidar_frame_id,
                odom_frame_id,
                camera_frame_id,
                params.debug_vis_segmentation,
                seg_img
                );

            //update the terrain grid with the new cell values. MATLAB's sub2ind (GridBuilder.m)
            //returns 1-based linear indices; convert to 0-based before indexing.
            int c = 0;
            for (auto i : terrain_sub_grid_idxs)
            {
                const double idx0 = i - 1;
                if (idx0 >= 0 && idx0 < static_cast<double>(terrain_grid.data.size()))
                {
                    terrain_grid.data[static_cast<size_t>(idx0)] = terrain_sub_grid[c];
                }
                c++;
            }


            if (params.publish_uab_occupancy_grid)
            {
                //update the obstacle grid with the new cell values. MATLAB's sub2ind (GridBuilder.m)
                //returns 1-based linear indices; convert to 0-based before indexing.
                c = 0;
                for (auto i : obstacle_sub_grid_idxs)
                {
                    //thresholding value to clean up obstacle grid
                    //only cell values in the obstacle grid with a probability above this threshold
                    //will be marked as occupied in the published occupancy grid
                    double obstacle_probability_threshold = 95.0;
                    double obstacle_probability_at_cell = obstacle_sub_grid[c++];
                    const double idx0 = i - 1;
                    if (obstacle_probability_at_cell >= obstacle_probability_threshold &&
                        idx0 >= 0 && idx0 < static_cast<double>(obstacle_grid.data.size()))
                    {
                        obstacle_grid.data[static_cast<size_t>(idx0)] = obstacle_probability_at_cell;
                    }
                }
            }

            nav_msgs::msg::OccupancyGrid terrain_grid_window, obstacle_grid_window;
            if (params.publish_window)
            {
                //extract a local grid centered at the vehicle pose, clamped so it does
                //not extend past the overall grid
                terrain_grid_window = ExtractGridWindow(terrain_grid,
                                                        current_pose.pose.pose.position.x,
                                                        current_pose.pose.pose.position.y,
                                                        params.costmap.publish.max_grid_width,
                                                        params.costmap.publish.max_grid_height,
                                                        TERRAIN_GRID_DEFAULT_VAL);
                obstacle_grid_window = ExtractGridWindow(obstacle_grid,
                                                         current_pose.pose.pose.position.x,
                                                         current_pose.pose.pose.position.y,
                                                         params.costmap.publish.max_grid_width,
                                                         params.costmap.publish.max_grid_height,
                                                         OBSTACLE_GRID_DEFAULT_VAL);
            }

            seg_grid_pub->publish(
                params.publish_window ? terrain_grid_window : terrain_grid);

            if (params.publish_uab_occupancy_grid)
            {
                occ_grid_pub->publish(
                    params.publish_window ? obstacle_grid_window :
                                            obstacle_grid);
            }

            if (params.debug_vis_segmentation && !seg_img.empty())
            {
                try
                {
                    //decode the original camera image to rgb (cv_bridge handles the
                    //source encoding, including bayer demosaicing, automatically)
                    cv::Mat original = cv_bridge::toCvCopy(img, "rgb8")->image;

                    //upsample the small segmentation mask to the original camera
                    //resolution (the resolution increase is done here, rather than in
                    //MATLAB, to keep the data returned over the MATLAB boundary small)
                    cv::Mat seg_full;
                    if (seg_img.size() != original.size())
                    {
                        cv::resize(seg_img, seg_full, original.size(), 0, 0, cv::INTER_NEAREST);
                    }
                    else
                    {
                        seg_full = seg_img;
                    }
                    seg_pub->publish(matToRgbMsg(img, seg_full));

                    //overlay the segmentation on the original camera image:
                    //(1 - 0.3) * original + 0.3 * segmentation
                    cv::Mat overlay;
                    cv::addWeighted(original, 0.7, seg_full, 0.3, 0.0, overlay);
                    seg_overlay_pub->publish(matToRgbMsg(img, overlay));
                }
                catch (const cv_bridge::Exception &e)
                {
                    std::cerr << "cv_bridge failed to convert debug segmentation image: " << e.what() << std::endl;
                }
                catch (const cv::Exception &e)
                {
                    std::cerr << "OpenCV failed to build debug segmentation image: " << e.what() << std::endl;
                }
            }
        }

        try
        {
            if (rclcpp::ok())
            {
                rclcpp::spin_some(node);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Ignoring exception from spin_some during shutdown: " << e.what() << std::endl;
        }
        rate.sleep();
    }

    tf.reset();

    tf_listener.reset();
    tf_buffer.reset();

    odom_sub.reset();
    pc_sub.reset();
    img_sub.reset();
    reset_sub.reset();
    camera_info_sub.reset();
    seg_grid_pub.reset();
    occ_grid_pub.reset();
    reset_ack_pub.reset();
    seg_pub.reset();
    seg_overlay_pub.reset();

    node.reset();

    mclTerminateApplication();
    return 0;
}
