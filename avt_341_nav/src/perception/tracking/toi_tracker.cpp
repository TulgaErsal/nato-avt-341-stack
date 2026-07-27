/**
* @file      toi_tracker.cpp
* @brief     ObjectTracker specialization for targets of interest (TOI).
             See toi_tracker.hpp.
*/

#include <avt_341_nav/perception/tracking/toi_tracker.hpp>

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
    encircle_triggered_ = false;
    contact_update_counter_ = 0;
}

void ToiTracker::PublishTargetContact() {
    geometry_msgs::msg::PoseStamped contact_pose;
    contact_pose.header.stamp = node_->get_clock()->now();
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

    if (!encircle_triggered_) {
        PublishTargetContact();
        encircle_triggered_ = true;
        contact_update_counter_ = 0;
    } else if (++contact_update_counter_ >= contact_update_interval_ticks_) {
        PublishTargetContact();
        contact_update_counter_ = 0;
    }
}

}  // namespace perception
}  // namespace avt_341_nav
