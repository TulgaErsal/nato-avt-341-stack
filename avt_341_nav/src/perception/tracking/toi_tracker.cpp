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
    rclcpp::Publisher<avt_341_msgs::msg::TargetReports>::SharedPtr target_reports_publisher,
    rclcpp::Publisher<avt_341_msgs::msg::TargetReports>::SharedPtr target_released_publisher)
    : ObjectTracker(node, target_class, params, coord_transformer,
                    std::move(leader_odom_publisher)),
      target_reports_publisher_(std::move(target_reports_publisher)),
      target_released_publisher_(std::move(target_released_publisher)) {
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
    avt_341_msgs::msg::TargetReports sightings;
    sightings.header.stamp = now;
    sightings.header.frame_id = "map";
    const Eigen::Vector3d observer = CameraPositionInMap();
    for (const auto& detection : candidate_detections_) {
        if (selected != nullptr &&
            detection.bbox.center.position.x == selected->bbox.center.position.x &&
            detection.bbox.center.position.y == selected->bbox.center.position.y) {
            continue;
        }
        if (IsExcludedDetection(detection, context, true)) {
            continue;
        }
        Eigen::Vector3d position;
        bool lidar = false;
        std::optional<MarkerMatch> match;
        if (context.has_obstacle_markers && context.obstacle_markers) {
            match = AssociateMarker(detection, *context.obstacle_markers,
                                    params_.tracking.target_association_max_dist);
        }
        if (match) {
            position = coord_transformer_.Transform(frame_ids_.Camera(), frame_ids_.Map(), match->pos_cam);
            lidar = BoxMatchesDetection(detection, position, match->size);
        }
        if (!lidar) {
            position = CameraMapEstimate(detection);
        }
        if (std::hypot(position.x() - bounding_box_centroid_filtered_.x(),
                       position.y() - bounding_box_centroid_filtered_.y()) <=
            params_.target_selection.investigated_toi_radius) {
            continue;
        }
        sightings.reports.push_back(Report(position, lidar, 0, observer));
    }
    if (sightings.reports.empty()) {
        return;
    }
    target_reports_publisher_->publish(sightings);
    last_sightings_time_ = now;
    for (const auto& report : sightings.reports) {
        RCLCPP_INFO(logger_, "Published sighting of another \"%s\" at (%.2f, %.2f), %s-ranged.",
                    target_class_.c_str(), report.position.x, report.position.y,
                    report.ranging == avt_341_msgs::msg::TargetReport::RANGING_LIDAR ? "LiDAR" : "camera");
    }
}

avt_341_msgs::msg::TargetReport ToiTracker::Report(const Eigen::Vector3d& position, bool lidar,
                                                    int64_t track_start_ns,
                                                    const Eigen::Vector3d& observer) const {
    avt_341_msgs::msg::TargetReport report;
    report.target_class = target_class_;
    report.track_start = rclcpp::Time(track_start_ns, node_->get_clock()->get_clock_type());
    report.position = core::ToPointMsg(position);
    report.yaw = last_reliable_yaw_;
    report.ranging = lidar ? avt_341_msgs::msg::TargetReport::RANGING_LIDAR
                           : avt_341_msgs::msg::TargetReport::RANGING_CAMERA;
    report.observer = core::ToPointMsg(observer);
    return report;
}

avt_341_msgs::msg::TargetReports ToiTracker::TrackReport(bool lidar) const {
    avt_341_msgs::msg::TargetReports message;
    message.header.stamp = node_->get_clock()->now();
    message.header.frame_id = "map";
    message.reports.push_back(
        Report(bounding_box_centroid_filtered_, lidar, track_start_ns_, CameraPositionInMap()));
    return message;
}

void ToiTracker::PublishTrackReport(bool lidar) {
    target_reports_publisher_->publish(TrackReport(lidar));
    RCLCPP_INFO(logger_,
                "Published target contact \"%s\" at (%.2f, %.2f, %.2f), %s-ranged.",
                target_class_.c_str(),
                bounding_box_centroid_filtered_.x(),
                bounding_box_centroid_filtered_.y(),
                bounding_box_centroid_filtered_.z(),
                lidar ? "LiDAR" : "camera");
}

void ToiTracker::OnExcludedTargetReleased() {
    if (published_track_start_ns_ == track_start_ns_ && track_start_ns_ != 0) {
        const bool lidar = ReportLidarRanged();
        PublishTrackReport(lidar);
        if (target_released_publisher_) {
            target_released_publisher_->publish(TrackReport(lidar));
        }
    }
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
        PublishTrackReport(ReportLidarRanged());
        published_track_start_ns_ = track_start_ns_;
        contact_update_counter_ = 0;
    } else if (++contact_update_counter_ >= contact_update_interval_ticks_) {
        PublishTrackReport(ReportLidarRanged());
        contact_update_counter_ = 0;
    }
}

}  // namespace perception
}  // namespace avt_341_nav
