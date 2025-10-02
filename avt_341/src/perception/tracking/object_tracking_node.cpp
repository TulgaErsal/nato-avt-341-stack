/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                      _    _    _    _    _    _    _                      +
 +                     / \  / \  / \  / \  / \  / \  / \                     +
 +                    ( A )( V )( T )( - )( 3 )( 4 )( 1 )                    +
 +                     \_/  \_/  \_/  \_/  \_/  \_/  \_/                     +
 +       _    _    _    _    _    _    _    _     _    _    _    _    _      +
 +      / \  / \  / \  / \  / \  / \  / \  / \   / \  / \  / \  / \  / \     +
 +     ( A )( U )( T )( O )( N )( O )( M )( Y ) ( S )( T )( A )( C )( K )    +
 +      \_/  \_/  \_/  \_/  \_/  \_/  \_/  \_/   \_/  \_/  \_/  \_/  \_/     +
 +                                                                           +
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +                                                                           +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      object_tracking_node.cpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Source file for the camera/LiDAR sensor fusion object tracker
             rclcpp ROS node.
* @copyright
  MIT License

  NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
  Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com)

  NOTE: The above copyright only applies to the contents of this file. The
  source code contained in this file is a direct port from the GitHub repository
  aarhus-robotics/navi, released by the copyright holder under the MIT license.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <avt_341/perception/tracking/object_tracking_node.hpp>

namespace avt_341 {
namespace perception {

ObjectTrackingNode::ObjectTrackingNode() : rclcpp::Node("object_tracker") {
    GetParameters();
    Initialize();
    CreateSubscriptions();
    CreateTimers();
    CreatePublishers();

    estimator_timer_->reset();
}

void ObjectTrackingNode::GetParameters() {
    declare_parameter("camera_frame", "camera_optical");
    camera_frame_ = get_parameter("camera_frame").as_string();

    declare_parameter("odometry_child_frame", "odom");
    odometry_child_frame_ = get_parameter("odometry_child_frame").as_string();

    declare_parameter("filters_downsampling_leaf_size", 0.3);
    leaf_size_ = get_parameter("filters_downsampling_leaf_size").as_double();

    declare_parameter("filters_passthrough_min", 3.0);
    passthrough_distance_min_ =
        get_parameter("filters_passthrough_min").as_double();

    declare_parameter("filters_passthrough_max", 30.0);
    passthrough_distance_max_ =
        get_parameter("filters_passthrough_max").as_double();

    declare_parameter("filters_clustering_tolerance", 0.6);
    clustering_tolerance_ =
        get_parameter("filters_clustering_tolerance").as_double();

    declare_parameter("filters_clustering_size_minimum", 50);
    cluster_size_min_ =
        get_parameter("filters_clustering_size_minimum").as_int();

    declare_parameter("filters_clustering_size_maximum", 500);
    cluster_size_max_ =
        get_parameter("filters_clustering_size_maximum").as_int();

    declare_parameter("filters_ground_max_iterations", 50);
    sac_segmentation_max_iterations_ =
        get_parameter("filters_ground_max_iterations").as_int();

    declare_parameter("filters_ground_threshold", 0.2);
    sac_segmentation_threshold_ =
        get_parameter("filters_ground_threshold").as_double();

    declare_parameter("filters_ground_angle", 3.0);
    sac_segmentation_angle_ = get_parameter("filters_ground_angle").as_double();

    declare_parameter("filters_kalman_rate", 20.0);
    estimator_rate_ = get_parameter("filters_kalman_rate").as_double();

    declare_parameter("filters_kalman_process", 0.01);
    filter_process_variance_ =
        get_parameter("filters_kalman_process").as_double();

    declare_parameter("filters_kalman_measurement", 1.0);
    filter_measurement_variance_ =
        get_parameter("filters_kalman_measurement").as_double();

    declare_parameter("tracker_target_class", 0);
    target_class_ =
        std::to_string(get_parameter("tracker_target_class").as_int());

    declare_parameter("tracker_timeout", 5.0);
    target_timeout_ = get_parameter("tracker_timeout").as_double();

    declare_parameter("sync_enable", true);
    sync_messages_ = get_parameter("sync_enable").as_bool();

    declare_parameter("sync_use_callback", false);
    use_callback_time_ = get_parameter("sync_use_callback").as_bool();

    declare_parameter("sync_detection", 0.1);
    max_detection_skew_ = get_parameter("sync_detection").as_double();

    declare_parameter("publish_clouds_fov", false);
    publish_fov_cloud_ = get_parameter("publish_clouds_fov").as_bool();

    declare_parameter("publish_clouds_roi", false);
    publish_roi_cloud_ = get_parameter("publish_clouds_roi").as_bool();

    declare_parameter("publish_clouds_ground", false);
    publish_ground_cloud_ = get_parameter("publish_clouds_ground").as_bool();

    declare_parameter("publish_clouds_cluster", false);
    publish_cluster_cloud_ = get_parameter("publish_clouds_cluster").as_bool();

    declare_parameter("publish_pose", true);
    publish_pose_ = get_parameter("publish_pose").as_bool();

    declare_parameter("filters_pose", true);
    use_filtered_pose_ = get_parameter("filters_pose").as_bool();

    declare_parameter("publish_odometry", false);
    publish_odometry_ = get_parameter("publish_odometry").as_bool();

    declare_parameter("filters_odometry", true);
    use_filtered_odometry_ = get_parameter("filters_odometry").as_bool();

    declare_parameter("publish_detection", false);
    publish_detection_3d_ = get_parameter("publish_detection").as_bool();

    declare_parameter("publish_image", false);
    publish_image_ = get_parameter("publish_image").as_bool();
}

void ObjectTrackingNode::CreateSubscriptions() {
    // Create the transform listener.
    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*transform_buffer_);

    detections_subscription_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection_2d",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::DetectionsCallback,
                      this,
                      std::placeholders::_1));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::ImageCallback,
                  this,
                  std::placeholders::_1));

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::CameraInfoCallback,
                      this,
                      std::placeholders::_1));

    point_cloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "input",
            RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::PointCloudCallback,
                      this,
                      std::placeholders::_1));
}

void ObjectTrackingNode::CreateTimers() {
    estimator_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / estimator_rate_),
        std::bind(&ObjectTrackingNode::EstimatorTimerCallback, this));
    estimator_timer_->cancel();
}

void ObjectTrackingNode::CreatePublishers() {
    if(publish_fov_cloud_) {
        fov_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/fov", 1);
    }

    if(publish_roi_cloud_) {
        roi_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/roi", 1);
    }

    if(publish_ground_cloud_) {
        ground_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/ground", 1);
    }

    if(publish_cluster_cloud_) {
        cluster_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/cluster",
                                                            1);
    }

    detection_publisher_ =
        create_publisher<vision_msgs::msg::Detection3D>("detection_3d", 1);

    if(publish_image_) {
        image_publisher_ =
            create_publisher<sensor_msgs::msg::Image>("out_image", 1);
    }

    if(publish_pose_) {
        pose_publisher_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "pose",
                1);
    }

    if(publish_odometry_) {
        odometry_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    }
}

void ObjectTrackingNode::PointCloudCallback(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Point cloud callback triggered!");

    if(!has_camera_info_) {
        RCLCPP_DEBUG(get_logger(),
                     "No camera info received, skipping tracking ...");
        return;
    }

    if(!has_first_detection_) {
        RCLCPP_DEBUG(get_logger(),
                     "No detection received, skipping tracking ...");
        return;
    }

    if(sync_messages_) {
        double detection_skew = (use_callback_time_)
            ? get_clock()->now().nanoseconds() / 1.0e9 -
                last_detection_callback_time_.nanoseconds() / 1.0e9
            : rclcpp::Time(point_cloud_message->header.stamp).nanoseconds() /
                    1.0e9 -
                last_detection_time_.nanoseconds() / 1.0e9;
        if(detection_skew > max_detection_skew_) {
            RCLCPP_DEBUG(
                get_logger(),
                "Last detection is too old %0.2lf, skipping tracking ...",
                detection_skew);
            has_detection_ = false;
            return;
        }
    }

    auto start_time = get_clock()->now();

    // Convert the ROS sensor_msgs/msg/PointCloud2 message to a PCL XYZ point
    // cloud.
    auto point_cloud = ToPCLCloud(point_cloud_message);

    // LIDAR TO CAMERA POINT CLOUD TRANSFORM
    // -------------------------------------

    // Transform the point cloud from its native frame to the camera frame. Note
    // that this only works when there is already a valid transform between the
    // two frames in the TF tree. Also note that this transform is performed
    // in-place, as the point coordinates from the LiDAR frame are not used
    // elsewhere in this node.
    if(point_cloud_message->header.frame_id.empty() ||
       !(transform_buffer_->canTransform(camera_frame_,
                                         point_cloud_message->header.frame_id,
                                         rclcpp::Time(0)))) {
        RCLCPP_ERROR(get_logger(),
                     "Could not lookup the transform between point cloud frame "
                     "and camera frame!");
        return;
    }

    if(tf2::getFrameId(*point_cloud_message) != camera_frame_) {
        RCLCPP_DEBUG(get_logger(),
                     "Transforming point cloud between LiDAR frame \"%s\" and "
                     "camera frame \"%s\" ...",
                     tf2::getFrameId(*point_cloud_message).c_str(),
                     camera_frame_.c_str());
        try {
            auto transform_message =
                TransformPointCloud(point_cloud_message, camera_frame_);

            pcl::transformPointCloud(
                *point_cloud,
                *point_cloud,
                tf2::transformToEigen(transform_message.transform).matrix());

        } catch(tf2::TransformException& exception) {
            RCLCPP_WARN(get_logger(), exception.what());
            return;
        }
    }

    // POINT CLOUD DOWNSAMPLING PIPELINE
    // ---------------------------------

    // Clamp the sensor distance.
    LimitSensorDistance(point_cloud);

    // Downsample the point cloud using a voxel filter to speed up the next
    // processing steps.
    DownsampleCloud(point_cloud);

    // POINT TO PIXEL COORDINATES PROJECTION
    // -------------------------------------

    // Find the mapping between the 3D coordinates (X, Y, Z) of a point and the
    // corresponding pixel coordinates in the camera image frame.
    const std::vector<PixelCoordinates> coordinates =
        ConvertPointCloudToPixelCoordinates(point_cloud, camera_info_message_);

    // Find the cloud points in the camera field of view and store them in a
    // separate point cloud. We keep the original downsampled point cloud to
    // account for objects that are not yet in the field-of-view.
    FindPointsInCameraFOV(point_cloud,
                          coordinates,
                          camera_info_message_->height,
                          camera_info_message_->width);

    if(publish_fov_cloud_) {
        PublishPointCloud(point_cloud,
                          point_cloud_message->header.stamp,
                          camera_frame_,
                          fov_cloud_publisher_);
    }

    // Find cloud points the region of interest defined by the first detection.

    RCLCPP_DEBUG(get_logger(),
                 "Finding cloud points in the region of interest ...");

    unsigned int x_min =
        detections_message_.detections[0].bbox.center.position.x -
        detections_message_.detections[0].bbox.size_x / 2;
    unsigned int x_max =
        detections_message_.detections[0].bbox.center.position.x +
        detections_message_.detections[0].bbox.size_x / 2;

    unsigned int y_min =
        detections_message_.detections[0].bbox.center.position.y -
        detections_message_.detections[0].bbox.size_y / 2;
    unsigned int y_max =
        detections_message_.detections[0].bbox.center.position.y +
        detections_message_.detections[0].bbox.size_y / 2;

    RCLCPP_DEBUG(
        get_logger(),
        "Trimming region of interest to the camera image frame bounds ...");
    if(x_min < 0) x_min = 0;
    if(x_max > camera_info_message_->width) x_max = camera_info_message_->width;
    if(y_min < 0) y_min = 0;
    if(y_max > camera_info_message_->height)
        y_max = camera_info_message_->height;

    RCLCPP_DEBUG(get_logger(),
                 "Selected the following bounding box as region of interest: "
                 "[X_MIN: %i, X_MAX: %i Y_MIN: %i Y_MAX: %i]",
                 x_min,
                 x_max,
                 y_min,
                 y_max);

    // Find the points lying in the region of interest (ROI) defined by the
    // detection bounding box.
    FindPointsInROI(point_cloud, coordinates, x_min, x_max, y_min, y_max);

    if(publish_roi_cloud_) {
        PublishPointCloud(point_cloud,
                          point_cloud_message->header.stamp,
                          camera_frame_,
                          roi_cloud_publisher_);
    }

    RemoveNaNPoints(point_cloud);

    // GROUND PLANE SEGMENTATION
    // -------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
        new pcl::PointCloud<pcl::PointXYZ>);

    SegmentGroundPlane(point_cloud, cloud_plane);

    if(publish_ground_cloud_) {
        PublishPointCloud(cloud_plane,
                          point_cloud_message->header.stamp,
                          camera_frame_,
                          ground_cloud_publisher_);
    }

    RemoveNaNPoints(point_cloud);

    // EUCLIDEAN CLUSTERING
    // --------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster;
    try {
        auto clustering_result = ExtractEuclideanClusters(point_cloud);
        // Update the flag for centroid measurement.
        has_new_measurement_ = true;

        cloud_cluster = clustering_result.first;
        bounding_box_centroid_ = Eigen::Vector3d(clustering_result.second.x,
                                                 clustering_result.second.y,
                                                 clustering_result.second.z);
    } catch(const ClusteringException& exception) {
        RCLCPP_DEBUG(get_logger(), exception.what());
        return;
    }

    if(publish_cluster_cloud_) {
        PublishPointCloud(cloud_cluster,
                          point_cloud_message->header.stamp,
                          camera_frame_,
                          cluster_publisher_);
    }

    // ORIENTED BOUNDING BOX ESTIMATION
    // --------------------------------

    try {
        pcl::PointXYZ bounding_box_min, bounding_box_max, bounding_box_centroid;
        Eigen::Matrix3f bounding_box_rotation;

        GetOrientedBoundingBox(cloud_cluster,
                               bounding_box_min,
                               bounding_box_max,
                               bounding_box_centroid,
                               bounding_box_rotation);

        /*
        bounding_box_centroid_ = Eigen::Vector3d(bounding_box_centroid.x,
                                             bounding_box_centroid.y,
                                             bounding_box_centroid.z);
        */

        bounding_box_size_ =
            Eigen::Vector3d(bounding_box_max.x - bounding_box_min.x,
                            bounding_box_max.y - bounding_box_min.y,
                            bounding_box_max.z - bounding_box_min.z);

        // Convert the rotation matrix to an orientation quaternion.
        bounding_box_orientation_ =
            Eigen::Quaterniond(bounding_box_rotation.cast<double>());

        if(publish_image_) { PublishImage(); }

    } catch(const PCAException& exception) {
        RCLCPP_ERROR(get_logger(), exception.what());
    }

    auto execution_time = (get_clock()->now() - start_time).nanoseconds();
    RCLCPP_DEBUG(get_logger(),
                 "Tracker pipeline execution time: %0.2lf ms",
                 execution_time / 1.0e6);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectTrackingNode::ToPCLCloud(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_message, *point_cloud);

    RCLCPP_DEBUG(get_logger(),
                 "Number of points in the raw point cloud: %i",
                 int(point_cloud->points.size()));

    return point_cloud;
}

void ObjectTrackingNode::RemoveNaNPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    RCLCPP_DEBUG(get_logger(),
                 "Removing invalid values (NaN) from the point cloud ...");

    unsigned int points_before_filtering = point_cloud->points.size();

    std::vector<int> removed_indices;
    pcl::removeNaNFromPointCloud(*point_cloud, *point_cloud, removed_indices);

    unsigned int points_after_filtering = point_cloud->points.size();

    RCLCPP_DEBUG(get_logger(),
                 "Total points after NaN filtering: %i -> %i (%i)",
                 points_before_filtering,
                 points_after_filtering,
                 points_before_filtering - points_after_filtering);
}

void ObjectTrackingNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr image_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera image callback triggered!");

    try {
        // Store the sensor_msgs/msg/Image as an OpenCV image and mark the
        // camera image as received.
        latest_image_ = cv_bridge::toCvShare(image_message);
        has_image_ = true;
    } catch(const cv_bridge::Exception& exception) {
        RCLCPP_ERROR(get_logger(), "CVBridge exception: %s", exception.what());
    }
}

void ObjectTrackingNode::CameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera info callback triggered!!");

    // Store the sensor_msgs/msg/CameraInfo message and mark camera info as
    // received.
    camera_info_message_ = camera_info_message;
    has_camera_info_ = true;
}

void ObjectTrackingNode::DetectionsCallback(
    const vision_msgs::msg::Detection2DArray::SharedPtr detections_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Detections callback triggered!");

    if(!detections_message->detections.size() > 0) return;

    bool has_target = false;
    for(const auto& detection : detections_message->detections) {
        // We only consider the highest scoring result for each detection, under
        // the assumption that the hypotheses array is sorted from highest to
        // lowest scoring.
        if(detection.results[0].hypothesis.class_id == target_class_) {
            detection_score_ = detection.results[0].hypothesis.score;
            has_target = true;
            break;
        }
    }

    if(!has_target) {
        has_detection_ = false;
        return;
    }

    // Store the vision_msgs/msg/Detection2DArray message, keep track of its
    // timestamp and mark detections as received.
    detections_message_ = *detections_message;
    last_detection_time_ = detections_message->header.stamp;
    last_detection_callback_time_ = get_clock()->now();
    has_first_detection_ = true;

    has_detection_ = true;
}

void ObjectTrackingNode::DownsampleCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    RCLCPP_DEBUG(get_logger(),
                 "Applying voxel grid downsampling with leaf size %0.3f ...",
                 leaf_size_);

    // Filter the point cloud in-place.
    voxel_grid_filter_.setInputCloud(point_cloud);
    voxel_grid_filter_.filter(*point_cloud);

    RCLCPP_DEBUG(get_logger(),
                 "Number of points in the processed point cloud after voxel "
                 "grid downsampling: %i",
                 int(point_cloud->points.size()));
}

geometry_msgs::msg::TransformStamped ObjectTrackingNode::TransformPointCloud(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message,
    const std::string target_frame) {
    // Retrieve the transform from the point cloud message native frame to the
    // target frame from the published transform tree.
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        return transform_buffer_->lookupTransform(
            target_frame,
            tf2::getFrameId(*point_cloud_message),
            tf2::TimePointZero);
    } catch(tf2::TransformException& exception) {
        RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
        throw;
    }
}

PixelCoordinates ObjectTrackingNode::ConvertPointToPixelCoordinates(
    const pcl::PointXYZ& point,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    return PixelCoordinates(camera_info_message->k[0] * point.x / point.z +
                                camera_info_message->k[2],
                            camera_info_message->k[4] * point.y / point.z +
                                camera_info_message->k[5]);
}

std::vector<PixelCoordinates>
ObjectTrackingNode::ConvertPointCloudToPixelCoordinates(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    std::vector<PixelCoordinates> coordinates;
    coordinates.reserve(point_cloud->size());

    for(auto& point : point_cloud->points) {
        coordinates.emplace_back(
            ConvertPointToPixelCoordinates(point, camera_info_message));
    }

    return coordinates;
}

void ObjectTrackingNode::FindPointsInCameraFOV(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const std::vector<PixelCoordinates>& coordinates,
    const int height,
    const int width) {
    pcl::PointIndices::Ptr fov_points_indices(new pcl::PointIndices());
    fov_points_indices->indices.reserve(point_cloud->size());

    for(unsigned int index = 0; index < coordinates.size(); ++index) {
        if(point_cloud->points[index].z > 0.0 && coordinates[index].x_ >= 0 &&
           coordinates[index].x_ <= width && coordinates[index].y_ >= 0 &&
           coordinates[index].y_ <= height) {
            fov_points_indices->indices.push_back(index);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract_fov_indices;
    extract_fov_indices.setInputCloud(point_cloud);
    extract_fov_indices.setIndices(fov_points_indices);
    extract_fov_indices.setNegative(false);
    extract_fov_indices.setKeepOrganized(true);
    extract_fov_indices.filterDirectly(point_cloud);
}

void ObjectTrackingNode::FindPointsInROI(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const std::vector<PixelCoordinates>& coordinates,
    const unsigned int x_min,
    const unsigned int x_max,
    const unsigned int y_min,
    const unsigned int y_max) {
    pcl::PointIndices::Ptr roi_points_indices(new pcl::PointIndices());

    for(unsigned int index = 0; index < coordinates.size(); ++index) {
        if(coordinates[index].x_ >= x_min && coordinates[index].x_ < x_max &&
           coordinates[index].y_ >= y_min && coordinates[index].y_ < y_max) {
            roi_points_indices->indices.push_back(index);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract_fov_indices;
    extract_fov_indices.setInputCloud(point_cloud);
    extract_fov_indices.setIndices(roi_points_indices);
    extract_fov_indices.setNegative(false);
    extract_fov_indices.setKeepOrganized(true);
    extract_fov_indices.filterDirectly(point_cloud);
}

void ObjectTrackingNode::LimitSensorDistance(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    passthrough_filter_.setInputCloud(point_cloud);
    passthrough_filter_.filter(*point_cloud);
}

const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointXYZ>
ObjectTrackingNode::ExtractEuclideanClusters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    // Instantiate a k-d tree to speed up the clustering process.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(point_cloud);

    // Configure the Euclidean cluster extraction agent.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_clustering_;
    euclidean_clustering_.setClusterTolerance(clustering_tolerance_);
    euclidean_clustering_.setMinClusterSize(cluster_size_min_);
    euclidean_clustering_.setMaxClusterSize(cluster_size_max_);
    euclidean_clustering_.setSearchMethod(kd_tree);

    // Run the cluster extraction algorithm and store the results in a vector of
    // vectors of cluster indices.
    euclidean_clustering_.setInputCloud(point_cloud);
    std::vector<pcl::PointIndices> clusters_indices;
    euclidean_clustering_.extract(clusters_indices);

    // Throw an exception to be caught upstream f clustering was unsuccessful.
    if(int(clusters_indices.size()) < 1) {
        throw ClusteringException(
            "Could not find any valid clusters in the provided point cloud.");
    }
    RCLCPP_DEBUG(get_logger(),
                 "Euclidean clustering found %i clusters.",
                 int(clusters_indices.size()));

    // Create a shared pointer to a new point cloud to store the closest cluster
    // and keep track of the cluster distances from the origin as the clustering
    // results are traversed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ closest_cluster_centroid(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity());

    for(int cluster_index = 0; cluster_index < int(clusters_indices.size());
        ++cluster_index) {
        // Temporarily instantiate a point cloud to the current cluster.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        for(const auto& idx : clusters_indices[cluster_index].indices) {
            cloud_cluster->push_back((*point_cloud)[idx]);
        }

        // Compute the distance to the centroid of the current cluster from the
        // point cloud origin.
        pcl::PointXYZ cloud_centroid;
        pcl::computeCentroid(*cloud_cluster, cloud_centroid);
        auto distance = cloud_centroid.getVector3fMap().norm();

        // Compare the distance to the current cluster with the shortest
        // distance so far and update the closest cluster index accordingly.
        if(distance < closest_cluster_centroid.getVector3fMap().norm()) {
            closest_cluster_centroid = cloud_centroid;
            closest_cloud_cluster = cloud_cluster;
        }

        RCLCPP_DEBUG(get_logger(),
                     "Cluster %i: %i points, centroid %0.2f meters away.",
                     cluster_index,
                     int(cloud_cluster->points.size()),
                     distance);
    }

    return std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr,
                     const pcl::PointXYZ>(closest_cloud_cluster,
                                          closest_cluster_centroid);
}

void ObjectTrackingNode::SegmentGroundPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane) {
    // Segment the largest planar component from the remaining cloud
    sac_segmentation_.setInputCloud(point_cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    sac_segmentation_.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0) {
        RCLCPP_DEBUG(get_logger(),
                     "Could not estimate a planar model for the given "
                     "dataset.");
        return;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(point_cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(false);
    extract_indices.setKeepOrganized(true);

    // Get the points associated with the planar surface
    extract_indices.filter(*cloud_plane);

    RCLCPP_DEBUG(
        get_logger(),
        "Point cloud representing the planar component: %i data points.",
        int(cloud_plane->size()));

    // Remove the planar inliers, extract the rest
    extract_indices.setNegative(true);
    extract_indices.setKeepOrganized(true);
    extract_indices.filterDirectly(point_cloud);
}

void ObjectTrackingNode::EstimatorTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Estimator timer callback triggered!");

    // Skip the target state estimation if no valid target detection has been
    // performed since the last reset.
    if(!has_first_detection_) return;

    // Reset the target state estimator if the target is currently being
    // tracked but no valid detections have been received during the timeout
    // window.
    if((get_clock()->now() - last_detection_time_).seconds() >
       target_timeout_) {
        if(is_tracking_) {
            RCLCPP_WARN(get_logger(),
                        "Target timeout, resetting estimator ...");
            filter_->SetInitialPosition(Eigen::Vector<double, 3>::Zero());
            is_tracking_ = false;
            has_detection_ = false;
            has_first_detection_ = false;
        }

        return;
    }

    // If you made it this far, then it means you are ready to track the target.
    is_tracking_ = true;

    // Run the "Predict" step of the Kalman filter.
    filter_->Predict();

    // Check if a new measurement is available to be parse, then provide a
    // matching measurement vector z_n.
    if(has_new_measurement_) {
        // The measurement vector contains the position vector components, each
        // followed by its first (velocity) and second (acceleration)
        // derivatives, hence (0, 0 + 3, 0 + 3 + 3) are the corresponding to the
        // position vector.
        Eigen::Vector<double, 9> measurement_vector =
            Eigen::Vector<double, 9>::Zero();
        measurement_vector(0) = bounding_box_centroid_.x();
        measurement_vector(3) = bounding_box_centroid_.y();
        measurement_vector(6) = bounding_box_centroid_.z();

        // Run the "Update" step of the Kalman filter and mark the latest
        // measurement as processed.
        filter_->Update(measurement_vector);
        has_new_measurement_ = false;
    }

    // Get the filtered state and publish a matching odometry message.
    const auto state_filtered = filter_->GetState();
    bounding_box_centroid_filtered_.x() = state_filtered(0);
    bounding_box_centroid_filtered_.y() = state_filtered(3);
    bounding_box_centroid_filtered_.z() = state_filtered(6);

    if(publish_odometry_) { PublishOdometry(); }
    if(publish_pose_) { PublishPose(); }
    if(publish_detection_3d_) { PublishDetection3D(); }
}

void ObjectTrackingNode::PublishPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) {
    // Convert the PCL XYZ point cloud to a ROS sensor_msgs/PointCloud2 message.
    sensor_msgs::msg::PointCloud2 point_cloud_message;
    pcl::toROSMsg(*point_cloud, point_cloud_message);

    // Stamp the point cloud message with the provided header.
    point_cloud_message.header.stamp = stamp;
    point_cloud_message.header.frame_id = frame_id;

    publisher->publish(point_cloud_message);
}

void ObjectTrackingNode::Initialize() {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Configure the voxel grid filter.
    voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    // Configure the passthrough filter.
    passthrough_filter_.setFilterFieldName("z");
    passthrough_filter_.setFilterLimits(passthrough_distance_min_,
                                        passthrough_distance_max_);

    // Configure the SAC segmentation filter.
    sac_segmentation_.setOptimizeCoefficients(true);
    sac_segmentation_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    sac_segmentation_.setEpsAngle((M_PI / 180.0) * sac_segmentation_angle_);
    sac_segmentation_.setAxis(Eigen::Vector3f{0.0, 1.0, 0.0});
    sac_segmentation_.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation_.setMaxIterations(sac_segmentation_max_iterations_);
    sac_segmentation_.setDistanceThreshold(sac_segmentation_threshold_);
    sac_segmentation_.setNumberOfThreads(0);

    // Initialize the Kalman filter.
    filter_ = std::make_shared<avt_341::perception::filtering::CVFilter<3>>(
        1.0 / estimator_rate_,
        filter_process_variance_,
        filter_measurement_variance_);
    filter_->SetInitialPosition(Eigen::Vector<double, 3>::Zero());
}

void ObjectTrackingNode::PublishImage() {
    if(!has_image_) return;

    auto image_copy = latest_image_->image.clone();

    cv::Vec3b& color = image_copy.at<cv::Vec3b>(0, 0);
    color[2] = 13;

    // Publish the detection image
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = get_clock()->now();
    cv_image.header.frame_id = camera_frame_;
    cv_image.encoding = "bgr8";
    cv_image.image = image_copy;
    image_publisher_->publish(*cv_image.toImageMsg());
}

void ObjectTrackingNode::GetOrientedBoundingBox(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    pcl::PointXYZ& bounding_box_min,
    pcl::PointXYZ& bounding_box_max,
    pcl::PointXYZ& bounding_box_centroid,
    Eigen::Matrix3f& bounding_box_rotation) {
    moi_estimation_.setInputCloud(point_cloud);
    moi_estimation_.compute();

    if(!moi_estimation_.getOBB(bounding_box_min,
                               bounding_box_max,
                               bounding_box_centroid,
                               bounding_box_rotation)) {
        throw PCAException("Could not estimate the oriented bounding box for "
                           "the provided point cloud cluster.");
    }
}

void ObjectTrackingNode::PublishDetection3D() {
    vision_msgs::msg::ObjectHypothesisWithPose object_hypothesis_message;

    object_hypothesis_message.hypothesis.class_id = target_class_;
    object_hypothesis_message.hypothesis.score = detection_score_;

    vision_msgs::msg::Detection3D detection_message;

    detection_message.header.stamp = get_clock()->now();
    detection_message.header.frame_id = camera_frame_;

    detection_message.results.push_back(object_hypothesis_message);

    detection_message.bbox.size.x = bounding_box_size_.x();
    detection_message.bbox.size.y = bounding_box_size_.y();
    detection_message.bbox.size.z = bounding_box_size_.z();

    detection_message.bbox.center.position.x = bounding_box_centroid_.x();
    detection_message.bbox.center.position.y = bounding_box_centroid_.y();
    detection_message.bbox.center.position.z = bounding_box_centroid_.z();

    detection_message.bbox.center.orientation.w = bounding_box_orientation_.w();
    detection_message.bbox.center.orientation.x = bounding_box_orientation_.x();
    detection_message.bbox.center.orientation.y = bounding_box_orientation_.y();
    detection_message.bbox.center.orientation.z = bounding_box_orientation_.z();

    detection_publisher_->publish(detection_message);
}

void ObjectTrackingNode::PublishPose() {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_message;

    pose_message.header.stamp = get_clock()->now();
    pose_message.header.frame_id = camera_frame_;

    if(use_filtered_pose_) {
        pose_message.pose.pose.position.x = bounding_box_centroid_filtered_.x();
        pose_message.pose.pose.position.y = bounding_box_centroid_filtered_.y();
        pose_message.pose.pose.position.z = bounding_box_centroid_filtered_.z();
    } else {
        pose_message.pose.pose.position.x = bounding_box_centroid_.x();
        pose_message.pose.pose.position.y = bounding_box_centroid_.y();
        pose_message.pose.pose.position.z = bounding_box_centroid_.z();
    }

    pose_message.pose.pose.orientation.w = bounding_box_orientation_.w();
    pose_message.pose.pose.orientation.x = bounding_box_orientation_.x();
    pose_message.pose.pose.orientation.y = bounding_box_orientation_.y();
    pose_message.pose.pose.orientation.z = bounding_box_orientation_.z();

    pose_publisher_->publish(pose_message);
}

void ObjectTrackingNode::PublishOdometry() {
    nav_msgs::msg::Odometry odometry_message;

    odometry_message.header.stamp = get_clock()->now();
    odometry_message.header.frame_id = camera_frame_;
    odometry_message.child_frame_id = odometry_child_frame_;

    if(use_filtered_odometry_) {
        odometry_message.pose.pose.position.x =
            bounding_box_centroid_filtered_.x();
        odometry_message.pose.pose.position.y =
            bounding_box_centroid_filtered_.y();
        odometry_message.pose.pose.position.z =
            bounding_box_centroid_filtered_.z();
    } else {
        odometry_message.pose.pose.position.x = bounding_box_centroid_.x();
        odometry_message.pose.pose.position.y = bounding_box_centroid_.y();
        odometry_message.pose.pose.position.z = bounding_box_centroid_.z();
    }

    // Note that the filter does not predicted orientation, hence the
    // orientation fields in the odometry message pose entry are not populated.

    odometry_publisher_->publish(odometry_message);
}

} // namespace perception
} // namespace avt_341