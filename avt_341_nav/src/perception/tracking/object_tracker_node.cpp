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

* @file      object_tracker_node.cpp
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

#include <avt_341_nav/perception/tracking/object_tracker_node.hpp>
#include <avt_341_nav/object_tracker_params_service.hpp>
#include <avt_341_nav/node/node_types.h>
#include <avt_341_nav/node/node_utils.h>

#include <algorithm>
#include <regex>

#include <avt_341_nav/core/eigen_dto_conversion.hpp>
#include <avt_341_nav/perception/tracking/formation_vehicle_tracker.hpp>
#include <avt_341_nav/perception/tracking/toi_tracker.hpp>

namespace avt_341_nav {
namespace perception {
TrackerSensorContext context_;
namespace {
// Compute time section ids. Obstacle detection is recorded as a real parent rather than being
// synthesized from its children, so that the work not covered by a child (transforming the incoming
// cloud, fromROSMsg, the transform to the fixed frame) shows up as parent minus children.
const std::string OBSTACLE_DETECTION_SECTION_ID = "obstacle_detection";
const std::string FILTER_CLOUD_SECTION_ID = OBSTACLE_DETECTION_SECTION_ID + "/filter_cloud";
const std::string FILTER_NORMS_SECTION_ID = OBSTACLE_DETECTION_SECTION_ID + "/filter_norms";
const std::string CLUSTERING_SECTION_ID = OBSTACLE_DETECTION_SECTION_ID + "/clustering";
const std::string TRACKING_TICK_SECTION_ID = "tracking_tick";
const std::string ESTIMATOR_TICK_SECTION_ID = "estimator_tick";

}

ObjectTrackerNode::ObjectTrackerNode() : rclcpp::Node("object_tracker_node") {
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

void ObjectTrackerNode::GetParameters() {
    param_listener_ =
        std::make_shared<avt_341_nav::params::object_tracker::ParamsListener>(
            get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();
    frame_ids_ = std::make_unique<core::FrameIdCollection>(
        params_.frames, node::GetLeadingNodeNamespace(get_namespace()));
    param_listener_->setUserCallback(
        [this](const ObjectTrackerSettings& updated_params) {
            ApplyUpdatedParameters(updated_params);
        });
}

const std::shared_ptr<core::ComputeTimeRecorder>& ObjectTrackerNode::Recorder() {
    if (compute_time_recorder_ == nullptr) {
        compute_time_recorder_ = std::make_shared<core::ComputeTimeRecorder>(
            shared_from_this(), core::ComputeTimeRecorder::MakeNodeTag(shared_from_this()));

        core::RunningStatsConfig section_config;
        section_config.window_num_samples = 40;
        for (const auto& section_id : {OBSTACLE_DETECTION_SECTION_ID, FILTER_CLOUD_SECTION_ID,
                                       FILTER_NORMS_SECTION_ID, CLUSTERING_SECTION_ID,
                                       TRACKING_TICK_SECTION_ID, ESTIMATOR_TICK_SECTION_ID}) {
            compute_time_recorder_->Configure(section_id, section_config);
        }
    }
    return compute_time_recorder_;
}

void ObjectTrackerNode::Initialize() {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


}

void ObjectTrackerNode::CreateSubscriptions() {
    // Create the transform listener.
    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*transform_buffer_);
    coord_transformer_ = std::make_unique<core::CoordTransformer>(*transform_buffer_, get_logger());

    detections_subscription_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection_2d", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackerNode::DetectionsCallback, this,
                      std::placeholders::_1));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackerNode::ImageCallback, this,
                  std::placeholders::_1));

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackerNode::CameraInfoCallback, this,
                      std::placeholders::_1));

    point_cloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/input", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackerNode::PointCloudCallback, this,
                      std::placeholders::_1));

    if (params_.target_selection.use_mission_manager) {
        task_status_subscription_ =
            create_subscription<avt_341_msgs::msg::MissionModuleStatus>(
                "task", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
                std::bind(&ObjectTrackerNode::TaskChangedCallback, this,
                        std::placeholders::_1));
    }

    reset_subscription_ = create_subscription<std_msgs::msg::String>(
        "avt_341/reset", 10,
        std::bind(&ObjectTrackerNode::ResetCallback, this, std::placeholders::_1));

    // Initialize the integrated obstacle detector.
    obstacle_detector_ =
        std::make_shared<avt_341_nav::perception::LidarObstacleDetector<pcl::PointXYZ>>();
    obstacle_id_ = 0;
}

void ObjectTrackerNode::CreateServices() {
    if (!params_.target_selection.use_mission_manager) {
        set_target_service_server_ =
            create_service<avt_341_msgs::srv::SetTarget>(
                "set_target",
                std::bind(&ObjectTrackerNode::SetTargetServiceCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
    }
}

void ObjectTrackerNode::CreateTimers() {
    estimator_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.filter.estimator_rate),
        std::bind(&ObjectTrackerNode::EstimatorTimerCallback, this));
    estimator_timer_->cancel();

    tracking_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.tracking.tracking_rate),
        std::bind(&ObjectTrackerNode::TrackingTimerCallback, this));
    tracking_timer_->cancel();

    info_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.tracking.info_rate),
        std::bind(&ObjectTrackerNode::TrackerInfoCallback, this));
    info_timer_->cancel();
}

void ObjectTrackerNode::CreatePublishers() {
    if (params_.publish.image) {
        image_publisher_ =
            create_publisher<sensor_msgs::msg::Image>("out_image", 1);
    }

    info_publisher_ =
        create_publisher<avt_341_msgs::msg::TrackerModuleStatus>("avt_341/tracker/state", 1);

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

    if (params_.obstacle_detector.publish_ground_cloud) {
        obstacle_ground_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_detector/cloud_ground", 1);
    }

    if (params_.obstacle_detector.publish_cluster_cloud) {
        obstacle_clusters_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_detector/cloud_clusters", 1);
    }
}

void ObjectTrackerNode::SpawnAutostartTrackers() {
    if (!params_.target_selection.use_autostart) {
        return;
    }

    for (const auto& target_class :
         params_.target_selection.autostart_target_classes) {
        if (target_class.empty()) {
            continue;
        }
        // The autostart list may contain the ego vehicle itself; it cannot
        // track itself. Other AddOrResetTracker callers are trusted to never
        // pass the ego vehicle id.
        if (IsEgoVehicle(target_class)) {
            RCLCPP_WARN(get_logger(),
                        "Skipping autostart target \"%s\": the ego vehicle "
                        "cannot track itself.",
                        target_class.c_str());
            continue;
        }
        if (AddOrResetTracker(target_class) == nullptr) {
            // Rejected (e.g. generic tracking disabled): try the next class.
            continue;
        }
        if (!params_.target_selection.use_multi_tracking) {
            RCLCPP_INFO(get_logger(),
                        "Single-tracking mode: autostarting only target "
                        "class \"%s\".",
                        target_class.c_str());
            break;
        }
    }
}

ObjectTracker* ObjectTrackerNode::AddOrResetTracker(
    const std::string& target_class) {
    auto existing = trackers_.find(target_class);
    if (existing != trackers_.end()) {
        existing->second->Reset();
        return existing->second.get();
    }

    std::unique_ptr<ObjectTracker> tracker = CreateTracker(target_class);
    if (!tracker) {
        // Rejected target (CreateTracker already logged why). Checked before
        // the single-tracking clear so a rejected target cannot destroy the
        // currently tracked one.
        return nullptr;
    }

    if (!params_.target_selection.use_multi_tracking && !trackers_.empty()) {
        // Remove and delete existing trackers
        trackers_.clear();
    }

    ObjectTracker* tracker_ptr = tracker.get();
    trackers_.emplace(target_class, std::move(tracker));
    RCLCPP_INFO(get_logger(),
        "Created %s tracker for target class \"%s\".",
        ToString(tracker_ptr->GetTrackerType()).c_str(),
        target_class.c_str());
    return tracker_ptr;
}

std::unique_ptr<ObjectTracker> ObjectTrackerNode::CreateTracker(
    const std::string& target_class) {

    // Formation vehicle tracker
    // ------------------------------------------------------------------------------------------
    const auto& formation_ids = params_.target_selection.formation_vehicle_ids;
    if (std::find(formation_ids.begin(), formation_ids.end(), target_class) !=
        formation_ids.end()) {
        return std::make_unique<FormationVehicleTracker>(
            this, target_class, params_, *coord_transformer_,
            leader_odom_publisher_);
    }

    // Toi tracker
    // ------------------------------------------------------------------------------------------
    if (MatchesToiRegex(target_class)) {
        return std::make_unique<ToiTracker>(
            this, target_class, params_, *coord_transformer_,
            leader_odom_publisher_, target_contacts_publisher_);
    }

    // Generic tracker
    // ------------------------------------------------------------------------------------------
    if (!params_.target_selection.allow_generic) {
        RCLCPP_INFO(get_logger(),
            "Ignoring target \"%s\": not a formation vehicle or TOI "
            "match, and generic tracking is disabled "
            "(target_selection.allow_generic=false).",
            target_class.c_str());
        return nullptr;
    }
    return std::make_unique<ObjectTracker>(
        this, target_class, params_, *coord_transformer_,
        leader_odom_publisher_);
}

bool ObjectTrackerNode::IsEgoVehicle(const std::string& target_class) const {
    // The node runs under the ego vehicle's namespace (e.g. "/agv1") and
    // target ids are the bare vehicle namespaces (e.g. "agv1"): compare the
    // top-level namespace token against the target id.
    const std::string ego = node::GetLeadingNodeNamespace(get_namespace());
    return !ego.empty() && ego == target_class;
}

bool ObjectTrackerNode::MatchesToiRegex(
    const std::string& target_class) const {
    const std::string& toi_regex = params_.target_selection.toi_regex;
    if (toi_regex.empty()) {
        return false;
    }
    try {
        return std::regex_search(target_class, std::regex(toi_regex));
    } catch (const std::regex_error& e) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid target_selection.toi_regex \"%s\": %s",
                     toi_regex.c_str(), e.what());
        return false;
    }
}

void ObjectTrackerNode::MaybeSpawnToiTrackers(
    const vision_msgs::msg::Detection2DArray& detections_message) {

    const bool single_tracking = !params_.target_selection.use_multi_tracking;

    // In single-tracking mode never displace a tracked formation vehicle for TOI sighting
    if (single_tracking && HasTrackerOfType(ObjectTrackerType::FormationVehicle)) {
        return;
    }

    for (const auto& detection : detections_message.detections) {
        if (detection.results.empty()) {
            continue;
        }
        const std::string& target_class = detection.results[0].hypothesis.class_id;
        if (trackers_.count(target_class) > 0 || !MatchesToiRegex(target_class)) {
            continue;
        }
        AddOrResetTracker(target_class);
        if (single_tracking) {
            break;  // Only one tracker slot.
        }
    }
}

bool ObjectTrackerNode::HasTrackerOfType(const ObjectTrackerType type) const {
    return std::any_of(trackers_.begin(), trackers_.end(),
                       [type](const auto& entry) {
                           return entry.second->GetTrackerType() == type;
                       });
}

void ObjectTrackerNode::RemoveStaleToiTrackers() {
    for (auto it = trackers_.begin(); it != trackers_.end();) {
        if (it->second->GetTrackerType() == ObjectTrackerType::Toi &&
            !MatchesToiRegex(it->first)) {
            RCLCPP_INFO(get_logger(),
                        "Removing TOI tracker \"%s\": target no longer "
                        "matches target_selection.toi_regex \"%s\".",
                        it->first.c_str(),
                        params_.target_selection.toi_regex.c_str());
            it = trackers_.erase(it);
        } else {
            ++it;
        }
    }
}

void ObjectTrackerNode::TrackerInfoCallback() {
    // Publish a single module status aggregating every child tracker's status.
    // The module is UNINITIALIZED while no target is selected (and the message
    // still publishes to preserve topic liveness for consumers), ACTIVE once at
    // least one tracker exists.
    avt_341_msgs::msg::TrackerModuleStatus module_status;
    module_status.header.stamp = get_clock()->now();
    module_status.header.frame_id = frame_ids_->Map();

    if (trackers_.empty()) {
        module_status.module_state =
            avt_341_msgs::msg::TrackerModuleStatus::MODULE_STATE_UNINITIALIZED;
    } else {
        module_status.module_state =
            avt_341_msgs::msg::TrackerModuleStatus::MODULE_STATE_ACTIVE;
        module_status.trackers.reserve(trackers_.size());
        for (const auto& [target_class, tracker] : trackers_) {
            module_status.trackers.push_back(tracker->GetTrackerStatus());
        }
    }

    info_publisher_->publish(module_status);

    // Publishing rides the info timer, so the compute time publish rate follows tracking.info_rate.
    Recorder()->PublishSummary();
}

void ObjectTrackerNode::TrackingTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Tracking timer callback triggered!");

    if (reset_called_) {
        for (auto& [target_class, tracker] : trackers_) {
            tracker->Reset();
        }
        std_msgs::msg::String ack;
        ack.data = avt_341_nav::node::NodeType::Perception;
        reset_ack_publisher_->publish(ack);
        reset_called_ = false;
        RCLCPP_INFO(get_logger(), "Reset complete.");
    }

    context_.obstacle_markers = &latest_obstacle_markers_;
    context_.has_obstacle_markers = has_obstacle_markers_;
    context_.obstacle_markers = &latest_obstacle_markers_;
    context_.has_obstacle_markers = has_obstacle_markers_;
    context_.camera_info = has_camera_info_ ? camera_info_message_ : nullptr;

    {
        auto recording = Recorder()->RecordScope(TRACKING_TICK_SECTION_ID);
        for (auto& [target_class, tracker] : trackers_) {
        tracker->TrackingTick(context_);
        }
    }

    // Note: the debug image is target-independent and is republished once per
    // tracking tick (previously it was only republished when the single
    // tracker got past its early-return guards).
    if (params_.publish.image) {
        PublishImage();
    }
}

void ObjectTrackerNode::EstimatorTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Estimator timer callback triggered!");

    auto recording = Recorder()->RecordScope(ESTIMATOR_TICK_SECTION_ID);
    for (auto& [target_class, tracker] : trackers_) {
        tracker->EstimatorTick();
    }
}

void ObjectTrackerNode::PublishObstacleDeleteAll(
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

void ObjectTrackerNode::PublishObstacleMarkers(
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

void ObjectTrackerNode::RunObstacleDetection(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    // Recorded over the whole body, including the early returns: they all do real work (a TF lookup
    // and often a full cloud transform) before bailing out. This runs synchronously from the point
    // cloud callback on the same single threaded executor as the timers, so whatever it costs is
    // taken directly out of the tracking and estimator ticks.
    auto detection_recording = Recorder()->RecordScope(OBSTACLE_DETECTION_SECTION_ID);

    const auto& od = params_.obstacle_detector;
    const std::string robot_base_link = frame_ids_->BaseLink();
    const Eigen::Vector4f roi_min_point = ToEigenPoint4f(od.roi_min_point);
    const Eigen::Vector4f roi_max_point = ToEigenPoint4f(od.roi_max_point);
    const Eigen::Vector4f body_min_point = ToEigenPoint4f(od.body_min_point);
    const Eigen::Vector4f body_max_point = ToEigenPoint4f(od.body_max_point);
    const Eigen::Vector3f ground_normal = ToEigenVector3f(od.ground_normal);
    std_msgs::msg::Header header = cloud_msg->header;

    // Transform to robot base link if needed, using the cloud's own timestamp
    // so that the TF lookup matches the moment the scan was captured (same as
    // the original standalone obstacle detector node).
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (cloud_msg->header.frame_id != robot_base_link) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = transform_buffer_->lookupTransform(
                robot_base_link, cloud_msg->header.frame_id,
                cloud_msg->header.stamp, tf2::durationFromSec(0.2));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(),
                        "RunObstacleDetection: TF %s -> %s failed: %s",
                        cloud_msg->header.frame_id.c_str(),
                        robot_base_link.c_str(), ex.what());
            PublishObstacleDeleteAll(header);
            return;
        }
        tf2::doTransform(*cloud_msg, transformed_cloud, tf);
        transformed_cloud.header.frame_id = robot_base_link;
    } else {
        transformed_cloud = *cloud_msg;
    }
    header.frame_id = robot_base_link;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *raw_cloud);

    // Downsample, crop ROI, remove ego-vehicle body.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    {
        auto recording = Recorder()->RecordScope(FILTER_CLOUD_SECTION_ID);
        filtered_cloud = obstacle_detector_->filterCloud(
            raw_cloud, static_cast<float>(od.voxel_grid_size),
            roi_min_point, roi_max_point, body_min_point, body_max_point);
    }

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
            frame_ids_->Map(), robot_base_link,
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
    {
        auto recording = Recorder()->RecordScope(FILTER_NORMS_SECTION_ID);
        obstacle_detector_->pclFilterNorms(
            filtered_cloud, fixed_cloud,
            norm_filtered, ground_filtered,
            ground_normal, static_cast<float>(od.ground_normal_threshold),
            static_cast<float>(od.obstacle_scale),
            static_cast<int>(od.obstacle_min_neighbors));
    }

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
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters;
    {
        auto recording = Recorder()->RecordScope(CLUSTERING_SECTION_ID);
        cloud_clusters = obstacle_detector_->clustering(
            norm_filtered, static_cast<float>(od.cluster_threshold),
            static_cast<int>(od.cluster_min_size),
            static_cast<int>(od.cluster_max_size));
    }

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
            static_cast<float>(od.displacement_threshold),
            static_cast<float>(od.iou_threshold));
    }

    PublishObstacleMarkers(header);

    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
}

void ObjectTrackerNode::PointCloudCallback(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Point cloud callback triggered!");

    // Run the integrated obstacle detector synchronously so that
    // latest_obstacle_markers_ is up-to-date before the next tracking tick.
    RunObstacleDetection(point_cloud_message);
}

void ObjectTrackerNode::ImageCallback(
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

void ObjectTrackerNode::CameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera info callback triggered!!");

    // Store the sensor_msgs/msg/CameraInfo message and mark camera info as
    // received.
    camera_info_message_ = camera_info_message;
    has_camera_info_ = true;
}

void ObjectTrackerNode::DetectionsCallback(
    const vision_msgs::msg::Detection2DArray::SharedPtr detections_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Detections callback triggered!");

    if (detections_message->detections.empty()) {
        RCLCPP_DEBUG(get_logger(),
                     "No detections in the current frame, skipping ...");
        for (auto& [target_class, tracker] : trackers_) {
            tracker->MarkDetectionMiss();
        }
        return;
    }

    // Newly detected TOI classes spawn their own tracker on sight; it then
    // ingests its detection in the matching loop below.
    MaybeSpawnToiTrackers(*detections_message);

    if (trackers_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "No target selected, ignoring detections ...");
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

void ObjectTrackerNode::PublishImage() {
    if (!has_image_)
        return;

    auto image_copy = latest_image_->image.clone();

    cv::Vec3b& color = image_copy.at<cv::Vec3b>(0, 0);
    color[2] = 13;

    // Publish the detection image
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = get_clock()->now();
    cv_image.header.frame_id = frame_ids_->Camera();
    cv_image.encoding = "bgr8";
    cv_image.image = image_copy;
    image_publisher_->publish(*cv_image.toImageMsg());
}

void ObjectTrackerNode::TaskChangedCallback(
    avt_341_msgs::msg::MissionModuleStatus::SharedPtr task_status_message) {
    const std::string& target_class = task_status_message->active_task.tracked_vehicle;

    if (target_class.empty()) {
        // No follow target assigned (e.g. the ego-vehicle is the formation
        // leader). Leave the existing trackers undisturbed.
        RCLCPP_DEBUG(get_logger(),
                     "Task status without tracked vehicle, ignoring ...");
        return;
    }

    if (AddOrResetTracker(target_class) == nullptr) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Target selection set to \"%s\".",
                target_class.c_str());
}

void ObjectTrackerNode::ResetCallback(std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.find(avt_341_nav::node::NodeType::Perception) == std::string::npos) return;
    reset_called_ = true;
}

void ObjectTrackerNode::ApplyUpdatedParameters(
    const ObjectTrackerSettings& updated_params) {
    const std::string old_toi_regex = params_.target_selection.toi_regex;
    if (ApplyRuntimeParameters(params_, updated_params)) {
        if (params_.target_selection.toi_regex != old_toi_regex) {
            RemoveStaleToiTrackers();
        }
        for (auto& [target_class, tracker] : trackers_) {
            tracker->UpdateSettings(params_);
        }
    }
}

void ObjectTrackerNode::SetTargetServiceCallback(
    const std::shared_ptr<avt_341_msgs::srv::SetTarget::Request> request,
    std::shared_ptr<avt_341_msgs::srv::SetTarget::Response> response) {
    if (request->id.empty()) {
        response->success = false;
        response->message = "Target ID must not be empty.";
        return;
    }

    const bool existed = trackers_.find(request->id) != trackers_.end();
    if (AddOrResetTracker(request->id) == nullptr) {
        response->success = false;
        response->message = "Target \"" + request->id +
                            "\" rejected: generic tracking is disabled.";
        return;
    }

    const std::string message =
        "Target \"" + request->id +
        (existed ? "\" re-targeted (state reset)." : "\" added.");
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    response->success = true;
    response->message = message;
}

}  // namespace perception
}  // namespace avt_341_nav
