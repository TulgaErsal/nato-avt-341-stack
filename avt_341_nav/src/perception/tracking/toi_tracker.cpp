/**
* @file      toi_tracker.cpp
* @brief     ObjectTracker specialization for targets of interest (TOI).
             See toi_tracker.hpp.
*/

#include <avt_341_nav/perception/tracking/toi_tracker.hpp>

#include <cmath>
#include <utility>

#include <avt_341_nav/core/eigen_dto_conversion.hpp>

namespace avt_341_nav {
namespace perception {

ToiTracker::ToiTracker(
    rclcpp::Node* node, const std::string& target_class,
    const ObjectTrackerSettings& params,
    const core::CoordTransformer& coord_transformer,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher)
    : ObjectTracker(node, target_class, params, coord_transformer,
                    std::move(leader_odom_publisher)),
      target_contacts_publisher_(std::move(target_contacts_publisher)) {
}

void ToiTracker::Reset() {
    ObjectTracker::Reset();
    contact_update_counter_ = 0;
}

void ToiTracker::OnOtherDetections(const TrackerSensorContext& context,
                                   const vision_msgs::msg::Detection2D* selected) {
    const rclcpp::Time now = node_->get_clock()->now();
    if (last_sightings_time_.nanoseconds() != 0 &&
        (now - last_sightings_time_).seconds() <
            contact_update_interval_ticks_ / params_.tracking.tracking_rate) {
        return;
    }
    nav_msgs::msg::Path sightings;
    sightings.header.stamp = now;
    sightings.header.frame_id = "map";
    for (const auto& detection : candidate_detections_) {
        if (selected != nullptr &&
            detection.bbox.center.position.x == selected->bbox.center.position.x &&
            detection.bbox.center.position.y == selected->bbox.center.position.y) {
            continue;
        }
        const Eigen::Vector3d position = EstimateMapPosition(detection, context);
        if (IsExcludedDetection(detection, context, true) ||
            std::hypot(position.x() - bounding_box_centroid_filtered_.x(),
                       position.y() - bounding_box_centroid_filtered_.y()) <=
                params_.target_selection.investigated_toi_radius) {
            continue;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = target_class_;
        pose.pose.position = core::ToPointMsg(position);
        pose.pose.orientation.w = 1.0;
        sightings.poses.push_back(pose);
    }
    if (sightings.poses.empty()) {
        return;
    }
    target_contacts_publisher_->publish(sightings);
    last_sightings_time_ = now;
    for (const auto& pose : sightings.poses) {
        RCLCPP_INFO(logger_, "Published sighting of another \"%s\" at (%.2f, %.2f).",
                    target_class_.c_str(), pose.pose.position.x, pose.pose.position.y);
    }
}

void ToiTracker::PublishTargetContact() {
    geometry_msgs::msg::PoseStamped contact_pose;
    contact_pose.header.stamp = rclcpp::Time(track_start_ns_, node_->get_clock()->get_clock_type());
    contact_pose.header.frame_id = target_class_;
    contact_pose.pose.position = core::ToPointMsg(bounding_box_centroid_filtered_);
    contact_pose.pose.orientation = core::YawToQuaternionMsg(last_reliable_yaw_);

    nav_msgs::msg::Path contact_msg;
    contact_msg.header.stamp = node_->get_clock()->now();
    contact_msg.header.frame_id = "map";
    contact_msg.poses.push_back(contact_pose);

    target_contacts_publisher_->publish(contact_msg);
    RCLCPP_INFO(logger_,
                "Published target contact \"%s\" at (%.2f, %.2f, %.2f).",
                target_class_.c_str(),
                bounding_box_centroid_filtered_.x(),
                bounding_box_centroid_filtered_.y(),
                bounding_box_centroid_filtered_.z());
}

void ToiTracker::MaybePublishContactUpdate() {
    const bool is_actively_tracking =
        filter_initialized_ && IsActiveTrackerState(state_);

    if (!is_actively_tracking) return;

    if (published_track_start_ns_ != track_start_ns_) {
        if (camera_detection_count_ <
            params_.target_selection.toi_min_detections) {
            return;
        }
        PublishTargetContact();
        published_track_start_ns_ = track_start_ns_;
        contact_update_counter_ = 0;
    } else if (++contact_update_counter_ >= contact_update_interval_ticks_) {
        PublishTargetContact();
        contact_update_counter_ = 0;
    }
}

}  // namespace perception
}  // namespace avt_341_nav
