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
* @brief     Source file for the camera/LiDAR sensor fusion object tracker rclcpp ROS node.
* @copyright MIT License

             NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
             Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).

             NOTE: The above copyright only applies to the contents of this file. The source code contained in this file
             is a direct port from the GitHub repository aarhus-robotics/navi, released by the copyright holder under
             the MIT license.

             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
             associated documentation files (the "Software"), to deal in the Software without restriction, including
             without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
             copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
             following conditions:

             The above copyright notice and this permission notice shall be included in all copies or substantial
             portions of the Software.

             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
             LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
             EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
             IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
             THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <avt_341/perception/tracking/object_tracking_node.hpp>

namespace avt_341 {
namespace perception {

ObjectTrackingNode::ObjectTrackingNode() : rclcpp::Node("object_tracker") {
    GetParameters();
    CreateSubscriptions();
    CreateTimers();
    CreatePublishers();

    filter_ = std::make_shared<avt_341::perception::filtering::CVFilter<3>>(estimator_rate_,
                                                                            filter_process_variance_,
                                                                            filter_measurement_variance_);
    Eigen::Vector<double, 3> initial_position = Eigen::Vector<double, 3>::Zero();
    filter_->SetInitialPosition(initial_position);

    estimator_timer_->reset();

    estimated_orientation_ = Eigen::Quaternionf::Identity();
}

void ObjectTrackingNode::GetParameters() {
    declare_parameter("camera.frame", "camera_optical");
    camera_frame_ = get_parameter("camera.frame").as_string();

    declare_parameter("cloud.leaf_size", 0.01);
    leaf_size_ = get_parameter("cloud.leaf_size").as_double();

    declare_parameter("filters.clustering.tolerance", 0.4);
    cluster_tolerance_ = get_parameter("filters.clustering.tolerance").as_double();

    declare_parameter("filters.clustering.size.minimum", 250);
    minimum_cluster_size_ = get_parameter("filters.clustering.size.minimum").as_int();

    declare_parameter("filters.clustering.size.maximum", 750);
    maximum_cluster_size_ = get_parameter("filters.clustering.size.maximum").as_int();

    declare_parameter("filters.ground.max_iterations", 100);
    sac_segmentation_max_iterations_ = get_parameter("filters.ground.max_iterations").as_int();

    declare_parameter("filters.ground.threshold", 0.2);
    sac_segmentation_threshold_ = get_parameter("filters.ground.threshold").as_double();

    declare_parameter("filters.kalman.rate", 100.0);
    estimator_rate_ = get_parameter("filters.kalman.rate").as_double();

    declare_parameter("filters.kalman.process", 0.2);
    filter_process_variance_ = get_parameter("filters.kalman.process").as_double();

    declare_parameter("filters.kalman.measurement", 0.02);
    filter_measurement_variance_ = get_parameter("filters.kalman.measurement").as_double();

    declare_parameter("sync.enable", false);
    sync_messages_ = get_parameter("sync.enable").as_bool();

    declare_parameter("sync.detection", 0.1);
    max_detection_skew_ = get_parameter("sync.detection").as_double();

    declare_parameter("publish.clouds.fov", true);
    publish_fov_cloud_ = get_parameter("publish.clouds.fov").as_bool();

    declare_parameter("publish.clouds.ground", true);
    publish_ground_cloud_ = get_parameter("publish.clouds.ground").as_bool();

    declare_parameter("publish.clouds.cluster", true);
    publish_cluster_cloud_ = get_parameter("publish.clouds.cluster").as_bool();
}

void ObjectTrackingNode::CreateSubscriptions() {
    // Create the transform listener.
    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*transform_buffer_);

    detections_subscription_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "detection_2d",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::DetectionsCallback, this, std::placeholders::_1));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::ImageCallback, this, std::placeholders::_1));

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::CameraInfoCallback, this, std::placeholders::_1));

    point_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::PointCloudCallback, this, std::placeholders::_1));
}

void ObjectTrackingNode::CreateTimers() {
    estimator_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / estimator_rate_),
                                         std::bind(&ObjectTrackingNode::EstimatorTimerCallback, this));
    estimator_timer_->cancel();
}

void ObjectTrackingNode::CreatePublishers() {
    if(publish_fov_cloud_) { fov_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points/fov", 1); }

    if(publish_ground_cloud_) {
        ground_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points/ground", 1);
    }

    if(publish_cluster_cloud_) {
        cluster_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points/cluster", 1);
    }

    detection_publisher_ = create_publisher<vision_msgs::msg::Detection3D>("detection_3d", 1);

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>("out_image", 1);

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 1);

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
}

void ObjectTrackingNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr image_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera image callback triggered!");

    try {
        // Store the sensor_msgs/msg/Image as an OpenCV image and mark the camera image as received.
        latest_image_ = cv_bridge::toCvShare(image_message);
        has_image_ = true;
    } catch(const cv_bridge::Exception& exception) {
        RCLCPP_ERROR(get_logger(), "CVBridge exception: %s", exception.what());
    }
}

void ObjectTrackingNode::DetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Detections callback triggered!");

    // Store the vision_msgs/msg/Detection2DArray message, keep track of its timestamp and mark detections as received.
    detections_message_ = *detections_message;
    last_detection_time_ = detections_message->header.stamp;
    number_of_detections_ = int(detections_message->detections.size());
    has_detection_ = true;
}

void ObjectTrackingNode::PointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "LiDAR point cloud callback triggered!");

    cloud_time_ = point_cloud_message->header.stamp;

    if(!has_camera_info_) {
        RCLCPP_WARN(get_logger(), "No camera info received, skipping tracking ...");
        return;
    }

    if(!has_detection_) {
        RCLCPP_WARN(get_logger(), "No detection received, skipping tracking ...");
        return;
    }

    if(sync_messages_) {
        double detection_skew = rclcpp::Time(point_cloud_message->header.stamp).nanoseconds() / 1.0e9 -
            last_detection_time_.nanoseconds() / 1.0e9;
        if(detection_skew < max_detection_skew_) {
            RCLCPP_WARN(get_logger(), "Last detection is too old %0.2lf, skipping tracking ...", detection_skew);
            return;
        }
    }

    // Transform the point cloud from its native frame to the camera frame. Note that this transform is performed
    // in-place, as the point coordinates from the LiDAR frame are not used elsewhere in this node.
    if(point_cloud_message->header.frame_id.empty() ||
       !(transform_buffer_->canTransform(camera_frame_, point_cloud_message->header.frame_id, rclcpp::Time(0)))) {
        RCLCPP_ERROR(get_logger(),
                     "Could not transform between camera point cloud frame and "
                     "camera frame!");
        return;
    }
    if(tf2::getFrameId(*point_cloud_message) != camera_frame_) {
        RCLCPP_DEBUG(get_logger(),
                     "Transforming point cloud between LiDAR frame \"%s\" and "
                     "camera frame \"%s\" ...",
                     tf2::getFrameId(*point_cloud_message).c_str(),
                     camera_frame_.c_str());
        try {
            TransformPointCloud(point_cloud_message, camera_frame_);
        } catch(tf2::TransformException& exception) {
            RCLCPP_WARN(get_logger(), exception.what());
            return;
        }
    }

    // Convert the ROS sensor_msgs/msg/PointCloud2 message to an XYZ PCL point cloud.
    auto point_cloud = ToPCLCloud(point_cloud_message);

    // Densify the point cloud.
    DensifyCloud(point_cloud);

    // Clamp the sensor maximum distance.
    LimitSensorDistance(point_cloud, 25.0);

    // Downsample the point cloud using a voxel filter to speed up the next processing steps.
    DownsampleCloud(point_cloud, leaf_size_);

    // Find the mapping between the 3D coordinates (X, Y, Z) of a point and the corresponding pixel coordinates in the
    // camera image frame.
    const std::vector<PixelCoordinates> coordinates =
        ConvertPointCloudToPixelCoordinates(point_cloud, camera_info_message_);

    // Find the cloud points in the camera field of view and store them in a separate point cloud. We keep the original
    // downsampled point cloud to account for objects that are not yet in the field-of-view.
    const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_fov =
        FindPointsInCameraFOV(point_cloud, coordinates, camera_info_message_->height, camera_info_message_->width);

    // Find cloud points the region of interest defined by the first detection.
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_roi;
    if(has_detection_ && detections_message_.detections.size() > 0) {
        RCLCPP_DEBUG(get_logger(), "Finding cloud points in the region of interest ...");

        int x_min = -1;
        int x_max = -1;
        int y_min = -1;
        int y_max = -1;

        x_min = detections_message_.detections[0].bbox.center.position.x -
            detections_message_.detections[0].bbox.size_x / 2;
        x_max = detections_message_.detections[0].bbox.center.position.x +
            detections_message_.detections[0].bbox.size_x / 2;

        y_min = detections_message_.detections[0].bbox.center.position.y -
            detections_message_.detections[0].bbox.size_y / 2;
        y_max = detections_message_.detections[0].bbox.center.position.y +
            detections_message_.detections[0].bbox.size_y / 2;

        RCLCPP_DEBUG(get_logger(), "Trimming region of interest to the camera image frame bounds ...");
        if(x_min < 0) x_min = 0;
        if(x_max > camera_info_message_->width) x_max = camera_info_message_->width;
        if(y_min < 0) y_min = 0;
        if(y_max > camera_info_message_->height) y_max = camera_info_message_->height;

        RCLCPP_DEBUG(get_logger(),
                     "Selected the following bounding box as region of interest: "
                     "[X_MIN: %i, X_MAX: %i Y_MIN: %i Y_MAX: %i]",
                     x_min,
                     x_max,
                     y_min,
                     y_max);

        point_cloud_roi = FindPointsInROI(point_cloud_fov, coordinates, x_min, x_max, y_min, y_max);
    }

    // Segment the ground plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    SegmentGroundPlane(point_cloud_fov, cloud_plane, sac_segmentation_max_iterations_, sac_segmentation_threshold_);

    if(publish_ground_cloud_) {
        PublishPointCloud(cloud_plane, point_cloud_message->header.stamp, camera_frame_, ground_cloud_publisher_);
    }

    if(number_of_detections_ > 0) {
        auto clustering_result =
            ExtractEuclideanClusters(point_cloud_fov, cluster_tolerance_, minimum_cluster_size_, maximum_cluster_size_);

        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = clustering_result.first;
        centroid_ = clustering_result.second;

        if(publish_cluster_cloud_) {
            PublishPointCloud(cloud_cluster, point_cloud_message->header.stamp, camera_frame_, cluster_publisher_);
        }

        auto moi_estimation = pcl::MomentOfInertiaEstimation<pcl::PointXYZ>();
        moi_estimation.setInputCloud(cloud_cluster);
        moi_estimation.compute();
        pcl::PointXYZ minimum_point;
        pcl::PointXYZ maximum_point;
        pcl::PointXYZ position;
        Eigen::Matrix3f rotation_matrix;
        auto result = moi_estimation.getOBB(minimum_point, maximum_point, position, rotation_matrix);

        if(!result) { RCLCPP_ERROR(get_logger(), "COULD NOT COMPUTE ORIENTED BOUNDING BOX!"); }

        Eigen::Quaternion<float> q(rotation_matrix);
        estimated_orientation_ = q;

        vision_msgs::msg::Detection3D detection_message;
        detection_message.id = "0";
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = "0";
        hyp.hypothesis.score = 0.9;
        detection_message.results.push_back(hyp);
        detection_message.header = point_cloud_message->header;
        detection_message.bbox.size.x = maximum_point.x - minimum_point.x;
        detection_message.bbox.size.y = maximum_point.y - minimum_point.y;
        detection_message.bbox.size.z = maximum_point.z - minimum_point.z;
        detection_message.bbox.center.position.x = position.x;
        detection_message.bbox.center.position.y = position.y;
        detection_message.bbox.center.position.z = position.z;
        detection_message.bbox.center.orientation.w = q.w();
        detection_message.bbox.center.orientation.x = q.x();
        detection_message.bbox.center.orientation.y = q.y();
        detection_message.bbox.center.orientation.z = q.z();
        detection_publisher_->publish(detection_message);
    }

    if(publish_fov_cloud_) {
        PublishPointCloud(point_cloud_fov, point_cloud_message->header.stamp, camera_frame_, fov_cloud_publisher_);
    }

    if(has_image_) {
        auto image_copy = std::make_shared<cv_bridge::CvImage>(*latest_image_);
        auto myimage = image_copy->image;
        cv::Vec3b& color = myimage.at<cv::Vec3b>(0, 0);
        color[2] = 13;

        // Publish the detection image
        cv_bridge::CvImage image;
        image.header.stamp = get_clock()->now();
        image.header.frame_id = camera_frame_;
        image.encoding = "bgr8";
        image.image = myimage;
        image_publisher_->publish(*image.toImageMsg());
    }

    if(number_of_detections_ > 0) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_message;
        pose_message.header.stamp = point_cloud_message->header.stamp;
        pose_message.header.frame_id = camera_frame_;
        pose_message.pose.pose.position.x = centroid_.x;
        pose_message.pose.pose.position.y = centroid_.y;
        pose_message.pose.pose.position.z = centroid_.z;

        pose_message.pose.pose.orientation.w = estimated_orientation_.w();
        pose_message.pose.pose.orientation.x = estimated_orientation_.x();
        pose_message.pose.pose.orientation.y = estimated_orientation_.y();
        pose_message.pose.pose.orientation.z = estimated_orientation_.z();

        pose_publisher_->publish(pose_message);
    }
}

void ObjectTrackingNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera info callback triggered!!");

    // Store the sensor_msgs/msg/CameraInfo message and mark camera info as
    // received.
    camera_info_message_ = camera_info_message;
    has_camera_info_ = true;
}

sensor_msgs::msg::PointCloud2
ObjectTrackingNode::TransformPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_message,
                                        const std::string target_frame) {
    // Retrieve the transform from the point cloud message native frame to the
    // target frame from the published transform tree.
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        transform_message =
            transform_buffer_->lookupTransform(target_frame, tf2::getFrameId(*point_cloud_message), tf2::TimePointZero);
    } catch(tf2::TransformException& exception) {
        RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
        throw;
    }

    sensor_msgs::msg::PointCloud2 pcl_msg;

    tf2::doTransform(*point_cloud_message, pcl_msg, transform_message);

    return pcl_msg;
}

void ObjectTrackingNode::TransformPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message,
                                             const std::string target_frame) {
    // Retrieve the transform from the point cloud message native frame to the
    // target frame from the published transform tree.
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        transform_message =
            transform_buffer_->lookupTransform(target_frame, tf2::getFrameId(*point_cloud_message), tf2::TimePointZero);
    } catch(tf2::TransformException& exception) {
        RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
        throw;
    }

    tf2::doTransform(*point_cloud_message, *point_cloud_message, transform_message);
}

PixelCoordinates
ObjectTrackingNode::ConvertPointToPixelCoordinates(pcl::PointXYZ& point,
                                                   const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    PixelCoordinates coordinates(camera_info_message->k[0] * point.x / point.z + camera_info_message->k[2],
                                 camera_info_message->k[4] * point.y / point.z + camera_info_message->k[5],
                                 point.z);
    return coordinates;
}

std::vector<PixelCoordinates> ObjectTrackingNode::ConvertPointCloudToPixelCoordinates(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    std::vector<PixelCoordinates> coordinates;
    coordinates.reserve(point_cloud->size());

    for(auto& point : point_cloud->points) {
        coordinates.push_back(ConvertPointToPixelCoordinates(point, camera_info_message));
    }

    return coordinates;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ObjectTrackingNode::FindPointsInCameraFOV(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                          const std::vector<PixelCoordinates>& coordinates,
                                          const int height,
                                          const int width) {
    pcl::PointIndices::Ptr fov_points_indices(new pcl::PointIndices());
    fov_points_indices->indices.reserve(point_cloud->size());

    for(int index = 0; index < int(coordinates.size()); ++index) {
        if(coordinates[index].z_ > 0 && coordinates[index].x_ >= 0 && coordinates[index].x_ <= width &&
           coordinates[index].y_ >= 0 && coordinates[index].y_ <= height) {
            fov_points_indices->indices.push_back(index);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr fov_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_fov_indices;
    extract_fov_indices.setInputCloud(point_cloud);
    extract_fov_indices.setIndices(fov_points_indices);
    extract_fov_indices.setNegative(false);
    extract_fov_indices.filter(*fov_cloud);

    return fov_cloud;
}

void ObjectTrackingNode::DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const float& leaf_size) {
    RCLCPP_DEBUG(get_logger(), "Applying voxel grid downsampling with leaf size %0.3f ...", leaf_size);

    // Initialize the voxel grid filter and set the leaf size.
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    // Filter the point cloud in-place.
    voxel_grid_filter.setInputCloud(point_cloud);
    voxel_grid_filter.filter(*point_cloud);

    RCLCPP_DEBUG(get_logger(),
                 "Number of points in the processed point cloud after voxel "
                 "grid downsampling: %i",
                 int(point_cloud->points.size()));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ObjectTrackingNode::FindPointsInROI(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                    const std::vector<PixelCoordinates>& coordinates,
                                    const unsigned int x_min,
                                    const unsigned int x_max,
                                    const unsigned int y_min,
                                    const unsigned int y_max) {
    pcl::PointIndices::Ptr roi_points_indices(new pcl::PointIndices());
    roi_points_indices->indices.reserve(point_cloud->size());

    for(int index = 0; index < int(coordinates.size()); ++index) {
        if(coordinates[index].x_ >= x_min && coordinates[index].x_ < x_max && coordinates[index].y_ >= y_min &&
           coordinates[index].y_ < y_max) {
            roi_points_indices->indices.push_back(index);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr fov_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_fov_indices;
    extract_fov_indices.setInputCloud(point_cloud);
    extract_fov_indices.setIndices(roi_points_indices);
    extract_fov_indices.setNegative(false);
    extract_fov_indices.filter(*fov_cloud);

    return fov_cloud;
}

void ObjectTrackingNode::SegmentGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                            const int& max_iterations,
                                            const float& threshold) {
    // Create the segmentation object for the planar model and set all the
    // parameters

    pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    sac_segmentation.setOptimizeCoefficients(true);
    sac_segmentation.setModelType(pcl::SACMODEL_PLANE);
    sac_segmentation.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation.setMaxIterations(max_iterations);
    sac_segmentation.setDistanceThreshold(threshold);

    int number_of_iterations = 0;
    // Segment the largest planar component from the remaining cloud
    sac_segmentation.setInputCloud(point_cloud);
    sac_segmentation.segment(*inliers, *coefficients);
    number_of_iterations++;

    if(inliers->indices.size() == 0)

    {
        RCLCPP_WARN(get_logger(),
                    "Could not estimate a planar model for the given "
                    "dataset at iteration %i",
                    number_of_iterations);
        return;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(point_cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(false);
    // Get the points associated with the planar surface
    extract_indices.filter(*cloud_plane);

    RCLCPP_DEBUG(get_logger(),
                 "PointCloud representing the planar component: %i data points.",
                 int(cloud_plane->size()));

    // Remove the planar inliers, extract the rest
    extract_indices.setNegative(true);
    extract_indices.filter(*point_cloud);
}

const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointXYZ>
ObjectTrackingNode::ExtractEuclideanClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                             const float tolerance,
                                             const int cluster_size_min,
                                             const int cluster_size_max) {
    // Instantiate and configure the Euclidean cluster extraction agent.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_clustering;
    euclidean_clustering.setClusterTolerance(tolerance);
    euclidean_clustering.setMinClusterSize(cluster_size_min);
    euclidean_clustering.setMaxClusterSize(cluster_size_max);

    // Instantiate a k-d tree to speed up the clustering process.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(point_cloud);
    euclidean_clustering.setSearchMethod(kd_tree);

    // Run the cluster extraction algorithm and store the results in a vector of vectors of cluster indices.
    euclidean_clustering.setInputCloud(point_cloud);
    std::vector<pcl::PointIndices> clusters_indices;
    euclidean_clustering.extract(clusters_indices);

    // Quit early if clustering was unsuccessful.
    // if(int(clusters_indices.size()) < 1) {
    //    RCLCPP_DEBUG(get_logger(), "No clusters found.");
    //    return;
    //}

    RCLCPP_DEBUG(get_logger(), "Euclidean clustering found %i clusters!", int(clusters_indices.size()));

    // Create a shared pointer to a new point cloud to store the closest cluster and keep track of the cluster distances
    // from the origin as the clustering results are traversed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ closest_cluster_centroid(std::numeric_limits<float>::infinity(),
                                           std::numeric_limits<float>::infinity(),
                                           std::numeric_limits<float>::infinity());

    for(int cluster_index = 0; cluster_index < int(clusters_indices.size()); ++cluster_index) {
        // Temporarily instantiate a point cloud to the current cluster.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(const auto& idx : clusters_indices[cluster_index].indices) {
            cloud_cluster->push_back((*point_cloud)[idx]);
        }

        // Compute the distance to the centroid of the current cluster from the point cloud origin.
        pcl::PointXYZ cloud_centroid;
        pcl::computeCentroid(*cloud_cluster, cloud_centroid);
        auto distance = cloud_centroid.getVector3fMap().norm();

        // Compare the distance to the current cluster with the shortest distance so far and update the closest cluster
        // index accordingly.
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

    // Update the flag for centroid measurement.
    has_new_measurement_ = true;

    return std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointXYZ>(closest_cloud_cluster,
                                                                                     closest_cluster_centroid);
}

void ObjectTrackingNode::DensifyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    if(point_cloud->is_dense == false) {
        RCLCPP_ERROR(get_logger(),
                     "Point cloud is not in dense format, please remove NaN "
                     "points first!");
    }

    RCLCPP_DEBUG(get_logger(), "Removing invalid values (NaN) from the point cloud ...");

    std::vector<int> removed_indices;
    pcl::removeNaNFromPointCloud(*point_cloud, *point_cloud, removed_indices);

    RCLCPP_INFO(get_logger(), "Total points after NaN filtering: %i", int(point_cloud->points.size()));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ObjectTrackingNode::ToPCLCloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*point_cloud_message, *point_cloud);

    RCLCPP_DEBUG(get_logger(), "Number of points in the raw point cloud: %i", int(point_cloud->points.size()));

    return point_cloud;
}

void ObjectTrackingNode::PublishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                           rclcpp::Time stamp,
                                           std::string frame_id,
                                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) {
    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*point_cloud, pcl_msg);
    pcl_msg.header.stamp = stamp;
    pcl_msg.header.frame_id = frame_id;
    publisher->publish(pcl_msg);
}

void ObjectTrackingNode::EstimatorTimerCallback() {
    filter_->Predict();

    if(has_new_measurement_) {
        Eigen::Vector<double, 9> measurement = Eigen::Vector<double, 9>::Zero();
        measurement(0) = centroid_.getArray3fMap().cast<double>().x();
        measurement(3) = centroid_.getArray3fMap().cast<double>().y();
        measurement(6) = centroid_.getArray3fMap().cast<double>().z();

        filter_->Update(measurement);

        has_new_measurement_ = false;
    }

    nav_msgs::msg::Odometry filtered_odometry_message;
    auto filtered_state = filter_->GetState();
    filtered_odometry_message.header.stamp = cloud_time_;
    filtered_odometry_message.header.frame_id = camera_frame_;
    filtered_odometry_message.pose.pose.position.x = filtered_state(0);
    filtered_odometry_message.pose.pose.position.y = filtered_state(3);
    filtered_odometry_message.pose.pose.position.z = filtered_state(6);

    filtered_odometry_message.pose.pose.orientation.w = estimated_orientation_.w();
    filtered_odometry_message.pose.pose.orientation.x = estimated_orientation_.x();
    filtered_odometry_message.pose.pose.orientation.y = estimated_orientation_.y();
    filtered_odometry_message.pose.pose.orientation.z = estimated_orientation_.z();

    odometry_publisher_->publish(filtered_odometry_message);
}

void ObjectTrackingNode::LimitSensorDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const float distance) {
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
    passthrough_filter.setInputCloud(point_cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(2.0, distance);
    passthrough_filter.filter(*point_cloud);
}

} // namespace perception
} // namespace avt_341