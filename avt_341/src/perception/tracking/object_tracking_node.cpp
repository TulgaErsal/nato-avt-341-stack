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
             rclcpp ROS node. The node owns the shared sensor inputs (camera,
             LiDAR obstacle detection, TF) and replicates one ObjectTracker
             instance per tracked target class.
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
#include <avt_341/node/node_proxy.h>

#include <avt_341/core/eigen_dto_conversion.hpp>

namespace avt_341 {
namespace perception {
TrackerSensorContext context_;
ObjectTrackingNode::ObjectTrackingNode() : rclcpp::Node("object_tracking_node") {
    GetParameters();
    Initialize();
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();
    SpawnAutostartTrackers();

    estimator_timer_->reset();
    tracking_timer_->reset();
    info_timer_->reset();
}

void ObjectTrackingNode::GetParameters() {
    settings_ = ObjectTrackerSettings::Load(*this);

    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ObjectTrackingNode::SetParametersCallback, this,
                  std::placeholders::_1));
}

void ObjectTrackingNode::Initialize() {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


}

void ObjectTrackingNode::CreateSubscriptions() {
    // Create the transform listener.
    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*transform_buffer_);
    coord_transformer_ = std::make_unique<core::CoordTransformer>(*transform_buffer_, get_logger());

    detections_subscription_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection_2d", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::DetectionsCallback, this,
                      std::placeholders::_1));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::ImageCallback, this,
                  std::placeholders::_1));

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::CameraInfoCallback, this,
                      std::placeholders::_1));

    point_cloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/input", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::PointCloudCallback, this,
                      std::placeholders::_1));

    if (settings_.target_selection.use_mission_manager) {
        task_status_subscription_ =
            create_subscription<avt_341_msgs::msg::MissionTaskStatus>(
                "task", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
                std::bind(&ObjectTrackingNode::TaskStatusCallback, this,
                        std::placeholders::_1));
    }

    reset_subscription_ = create_subscription<std_msgs::msg::String>(
        "avt_341/reset", 10,
        std::bind(&ObjectTrackingNode::ResetCallback, this, std::placeholders::_1));

    // Initialize the integrated obstacle detector.
    obstacle_detector_ =
        std::make_shared<avt_341::perception::LidarObstacleDetector<pcl::PointXYZ>>();
    obstacle_id_ = 0;
}

void ObjectTrackingNode::CreateServices() {
    if (!settings_.target_selection.use_mission_manager) {
        set_target_service_server_ =
            create_service<avt_341_msgs::srv::SetTarget>(
                "set_target",
                std::bind(&ObjectTrackingNode::SetTargetServiceCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
    }
}

void ObjectTrackingNode::CreateTimers() {
    estimator_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / settings_.filter.estimator_rate),
        std::bind(&ObjectTrackingNode::EstimatorTimerCallback, this));
    estimator_timer_->cancel();

    tracking_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / settings_.tracking.tracking_rate),
        std::bind(&ObjectTrackingNode::TrackingTimerCallback, this));
    tracking_timer_->cancel();

    info_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / settings_.tracking.info_rate),
        std::bind(&ObjectTrackingNode::TrackerInfoCallback, this));
    info_timer_->cancel();
}

void ObjectTrackingNode::CreatePublishers() {
    if (settings_.publish.image) {
        image_publisher_ =
            create_publisher<sensor_msgs::msg::Image>("out_image", 1);
    }

    info_publisher_ =
        create_publisher<avt_341_msgs::msg::TrackerInfo>("info", 1);

    reset_ack_publisher_ =
        create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);

    target_contacts_publisher_ =
        create_publisher<nav_msgs::msg::Path>("avt_341/target_contacts", 1);
	
	leader_odom_publisher_ =
		create_publisher<nav_msgs::msg::Odometry>("avt_341/odometry_estimate/leader", 1);

    // Integrated obstacle detector publishers.
    obstacle_bboxes_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "lidar_detector/bboxes", 1);

    if (settings_.obstacle_detector.publish_ground_cloud) {
        obstacle_ground_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_detector/cloud_ground", 1);
    }

    if (settings_.obstacle_detector.publish_cluster_cloud) {
        obstacle_clusters_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_detector/cloud_clusters", 1);
    }
}

void ObjectTrackingNode::SpawnAutostartTrackers() {
    if (!settings_.target_selection.use_autostart) {
        return;
    }

    for (const auto& target_class :
         settings_.target_selection.autostart_target_classes) {
        if (target_class.empty()) {
            continue;
        }
        AddOrResetTracker(target_class);
        if (!settings_.target_selection.use_multi_tracking) {
            RCLCPP_INFO(get_logger(),
                        "Single-tracking mode: autostarting only target "
                        "class \"%s\".",
                        target_class.c_str());
            break;
        }
    }
}

ObjectTracker& ObjectTrackingNode::AddOrResetTracker(
    const std::string& target_class) {
    auto existing = trackers_.find(target_class);
    if (existing != trackers_.end()) {
        existing->second->Reset();
        return *existing->second;
    }

    if (!settings_.target_selection.use_multi_tracking && !trackers_.empty()) {
        // Remove and delete existing trackers
        trackers_.clear();
    }

    auto tracker = std::make_unique<ObjectTracker>(
        this, target_class, settings_, *coord_transformer_,
		target_contacts_publisher_, leader_odom_publisher_	);
    ObjectTracker& tracker_ref = *tracker;
    trackers_.emplace(target_class, std::move(tracker));
    RCLCPP_INFO(get_logger(), "Created tracker for target class \"%s\".",
                target_class.c_str());
    return tracker_ref;
}

void ObjectTrackingNode::TrackerInfoCallback() {
    if (trackers_.empty()) {
        // Preserve topic liveness for consumers while no target is selected.
        avt_341_msgs::msg::TrackerInfo info_message;
        info_message.header.stamp = get_clock()->now();
        info_message.header.frame_id = "none";
        info_message.state = TrackerState::UNINITIALIZED;
        info_publisher_->publish(info_message);
        return;
    }

    for (const auto& [target_class, tracker] : trackers_) {
        avt_341_msgs::msg::TrackerInfo info_message;
        info_message.header.stamp = get_clock()->now();
        info_message.header.frame_id = target_class;
        info_message.state = tracker->GetTrackerState();
        info_publisher_->publish(info_message);
    }
}

void ObjectTrackingNode::TrackingTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Tracking timer callback triggered!");

    if (reset_called_) {
        for (auto& [target_class, tracker] : trackers_) {
            tracker->Reset();
        }
        std_msgs::msg::String ack;
        ack.data = avt_341::node::NodeType::Perception;
        reset_ack_publisher_->publish(ack);
        reset_called_ = false;
        RCLCPP_INFO(get_logger(), "Reset complete.");
    }

    auto start_time = get_clock()->now();

    context_.obstacle_markers = &latest_obstacle_markers_;
    context_.has_obstacle_markers = has_obstacle_markers_;
    context_.camera_info = has_camera_info_ ? camera_info_message_ : nullptr;

    for (auto& [target_class, tracker] : trackers_) {
        tracker->TrackingTick(context_);
    }

    // Note: the debug image is target-independent and is republished once per
    // tracking tick (previously it was only republished when the single
    // tracker got past its early-return guards).
    if (settings_.publish.image) {
        PublishImage();
    }

    execution_time_ = (get_clock()->now() - start_time).nanoseconds() / 1.0e6;
    RCLCPP_DEBUG(get_logger(), "Tracker pipeline execution time: %0.2lf ms",
                 execution_time_);
}

void ObjectTrackingNode::EstimatorTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Estimator timer callback triggered!");

    for (auto& [target_class, tracker] : trackers_) {
        tracker->EstimatorTick();
    }
}

void ObjectTrackingNode::PublishObstacleDeleteAll(
    const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header = header;
    marker_array.markers.push_back(delete_marker);
    latest_obstacle_markers_ = marker_array;
    has_obstacle_markers_ = true;
    obstacle_bboxes_publisher_->publish(marker_array);
}

void ObjectTrackingNode::PublishObstacleMarkers(
    const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Always prepend a DELETEALL to clear stale markers from previous frames.
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header = header;
    marker_array.markers.push_back(delete_marker);

    for (const auto& box : curr_boxes_) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "lidar_bboxes";
        marker.id = box.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = core::ToPointMsg(box.position.cast<double>());
        marker.pose.orientation = core::ToQuaternionMsg(box.quaternion.cast<double>());
        marker.scale = core::ToVector3Msg(box.dimension.cast<double>());
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
    }

    latest_obstacle_markers_ = marker_array;
    has_obstacle_markers_ = true;
    obstacle_bboxes_publisher_->publish(marker_array);
}

void ObjectTrackingNode::RunObstacleDetection(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    const auto& od = settings_.obstacle_detector;
    std_msgs::msg::Header header = cloud_msg->header;

    // Transform to robot base link if needed, using the cloud's own timestamp
    // so that the TF lookup matches the moment the scan was captured (same as
    // the original standalone obstacle detector node).
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (cloud_msg->header.frame_id != od.robot_base_link) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = transform_buffer_->lookupTransform(
                od.robot_base_link, cloud_msg->header.frame_id,
                cloud_msg->header.stamp, tf2::durationFromSec(0.2));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(),
                        "RunObstacleDetection: TF %s -> %s failed: %s",
                        cloud_msg->header.frame_id.c_str(),
                        od.robot_base_link.c_str(), ex.what());
            PublishObstacleDeleteAll(header);
            return;
        }
        tf2::doTransform(*cloud_msg, transformed_cloud, tf);
        transformed_cloud.header.frame_id = od.robot_base_link;
    } else {
        transformed_cloud = *cloud_msg;
    }
    header.frame_id = od.robot_base_link;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *raw_cloud);

    // Downsample, crop ROI, remove ego-vehicle body.
    auto filtered_cloud = obstacle_detector_->filterCloud(
        raw_cloud, od.voxel_grid_size,
        od.roi_min_point, od.roi_max_point,
        od.body_min_point, od.body_max_point);

    if (static_cast<int>(filtered_cloud->size()) < od.cluster_min_size) {
        PublishObstacleDeleteAll(header);
        return;
    }

    // Rotation-only transform to fixed frame for normal estimation.
    // Use the cloud's timestamp to match the original standalone node behavior.
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixed_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::msg::TransformStamped fixed_tf;
    try {
        fixed_tf = transform_buffer_->lookupTransform(
            settings_.frames.world_frame, od.robot_base_link,
            cloud_msg->header.stamp, tf2::durationFromSec(0.2));
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(),
                    "RunObstacleDetection: fixed-frame TF not available: %s",
                    ex.what());
        PublishObstacleDeleteAll(header);
        return;
    }

    const Eigen::Quaternionf q = core::ToEigen(fixed_tf.transform.rotation).cast<float>();
    if (q.norm() < 1e-6f) {
        RCLCPP_WARN(get_logger(),
                    "RunObstacleDetection: fixed-frame TF quaternion is zero.");
        PublishObstacleDeleteAll(header);
        return;
    }
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    Eigen::Affine3f transform_fixed;
    transform_fixed.matrix() = mat4;
    pcl::transformPointCloud(*filtered_cloud, *fixed_cloud, transform_fixed);

    if (static_cast<int>(fixed_cloud->size()) < od.cluster_min_size) {
        PublishObstacleDeleteAll(header);
        return;
    }

    // Separate ground and obstacle points via normal filtering.
    pcl::PointCloud<pcl::PointXYZ>::Ptr norm_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    obstacle_detector_->pclFilterNorms(
        filtered_cloud, fixed_cloud,
        norm_filtered, ground_filtered,
        od.ground_normal, od.ground_normal_threshold,
        od.obstacle_scale, od.obstacle_min_neighbors);

    if (od.publish_ground_cloud && obstacle_ground_cloud_publisher_) {
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_filtered, ground_msg);
        ground_msg.header = header;
        obstacle_ground_cloud_publisher_->publish(ground_msg);
    }

    if (od.publish_cluster_cloud && obstacle_clusters_cloud_publisher_) {
        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::toROSMsg(*norm_filtered, cluster_msg);
        cluster_msg.header = header;
        obstacle_clusters_cloud_publisher_->publish(cluster_msg);
    }

    if (norm_filtered->empty()) {
        PublishObstacleDeleteAll(header);
        return;
    }
    context_.current_cluster = norm_filtered; // global version of norm_filtered
    // Cluster and build bounding boxes.
    auto cloud_clusters = obstacle_detector_->clustering(
        norm_filtered, od.cluster_threshold,
        od.cluster_min_size, od.cluster_max_size);

    curr_boxes_.clear();
    for (auto& cluster : cloud_clusters) {
        Box box = od.use_pca_box
            ? obstacle_detector_->pcaBoundingBox(cluster, obstacle_id_)
            : obstacle_detector_->axisAlignedBoundingBox(cluster, obstacle_id_);
        obstacle_id_ = (obstacle_id_ < SIZE_MAX) ? obstacle_id_ + 1 : 0;
        curr_boxes_.emplace_back(box);
    }

    if (od.use_tracking) {
        obstacle_detector_->obstacleTracking(
            prev_boxes_, curr_boxes_,
            od.displacement_threshold, od.iou_threshold);
    }

    PublishObstacleMarkers(header);

    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
}

void ObjectTrackingNode::PointCloudCallback(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Point cloud callback triggered!");

    // Run the integrated obstacle detector synchronously so that
    // latest_obstacle_markers_ is up-to-date before the next tracking tick.
    RunObstacleDetection(point_cloud_message);
}

void ObjectTrackingNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr image_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera image callback triggered!");

    try {
        // Store the sensor_msgs/msg/Image as an OpenCV image and mark the
        // camera image as received.
        latest_image_ = cv_bridge::toCvShare(image_message);
        has_image_ = true;
    } catch (const cv_bridge::Exception& exception) {
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

    if (trackers_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "No target selected, ignoring detections ...");
        return;
    }

    if (detections_message->detections.empty()) {
        RCLCPP_DEBUG(get_logger(),
                     "No detections in the current frame, skipping ...");
        for (auto& [target_class, tracker] : trackers_) {
            tracker->MarkDetectionMiss();
        }
        return;
    }

    const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info =
        has_camera_info_ ? camera_info_message_ : nullptr;

    for (auto& [target_class, tracker] : trackers_) {
        // We only consider the highest scoring result for each detection,
        // under the assumption that the hypotheses array is sorted from
        // highest to lowest scoring.
        const vision_msgs::msg::Detection2D* match = nullptr;
        for (const auto& detection : detections_message->detections) {
            if (!detection.results.empty() &&
                detection.results[0].hypothesis.class_id == target_class) {
                match = &detection;
                break;
            }
        }

        if (match) {
            tracker->IngestDetection(*match, detections_message->header.stamp,
                                     camera_info);
        } else {
            RCLCPP_INFO(get_logger(),
                        "Target %s not found in the current detection, skipping ...",
                        target_class.c_str());
            tracker->MarkDetectionMiss();
        }
    }
}

void ObjectTrackingNode::PublishImage() {
    if (!has_image_)
        return;

    auto image_copy = latest_image_->image.clone();

    cv::Vec3b& color = image_copy.at<cv::Vec3b>(0, 0);
    color[2] = 13;

    // Publish the detection image
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = get_clock()->now();
    cv_image.header.frame_id = settings_.frames.camera_frame;
    cv_image.encoding = "bgr8";
    cv_image.image = image_copy;
    image_publisher_->publish(*cv_image.toImageMsg());
}

void ObjectTrackingNode::TaskStatusCallback(
    avt_341_msgs::msg::MissionTaskStatus::SharedPtr task_status_message) {
    const std::string& target_class = task_status_message->tracked_vehicle;

    if (target_class.empty()) {
        // No follow target assigned (e.g. the ego-vehicle is the formation
        // leader). Leave the existing trackers undisturbed.
        RCLCPP_DEBUG(get_logger(),
                     "Task status without tracked vehicle, ignoring ...");
        return;
    }

    AddOrResetTracker(target_class);

    RCLCPP_INFO(get_logger(), "Target selection set to \"%s\".",
                target_class.c_str());
}

void ObjectTrackingNode::ResetCallback(std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.find(avt_341::node::NodeType::Perception) == std::string::npos) return;
    reset_called_ = true;
}

rcl_interfaces::msg::SetParametersResult
ObjectTrackingNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    if (settings_.UpdateFromParameters(parameters)) {
        for (auto& [target_class, tracker] : trackers_) {
            tracker->UpdateSettings(settings_);
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void ObjectTrackingNode::SetTargetServiceCallback(
    const std::shared_ptr<avt_341_msgs::srv::SetTarget::Request> request,
    std::shared_ptr<avt_341_msgs::srv::SetTarget::Response> response) {
    if (request->id.empty()) {
        response->success = false;
        response->message = "Target ID must not be empty.";
        return;
    }

    const bool existed = trackers_.find(request->id) != trackers_.end();
    AddOrResetTracker(request->id);

    const std::string message =
        "Target \"" + request->id +
        (existed ? "\" re-targeted (state reset)." : "\" added.");
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    response->success = true;
    response->message = message;
}

}  // namespace perception
}  // namespace avt_341
