/**
* @file      formation_vehicle_tracker.cpp
* @brief     ObjectTracker specialization for formation vehicles.
             See formation_vehicle_tracker.hpp.
*/

#include <avt_341_nav/perception/tracking/formation_vehicle_tracker.hpp>

#include <utility>

#include <avt_341_nav/core/eigen_dto_conversion.hpp>

namespace avt_341_nav {
namespace perception {

FormationVehicleTracker::FormationVehicleTracker(
    rclcpp::Node* node, const std::string& target_class,
    const ObjectTrackerSettings& params,
    const core::CoordTransformer& coord_transformer,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher)
    : ObjectTracker(node, target_class, params, coord_transformer,
                    std::move(leader_odom_publisher)) {
    recovery_monitor_ = std::make_unique<TrackerRecoveryMonitor>(
        node_, target_class_, params_.recovery, logger_);
}

void FormationVehicleTracker::EstimatorTick() {
    if (RecoveryMonitorTick()) {
        // While LOST the last (ground-truth-recovered) estimate keeps
        // publishing so consumers can still plan a path to the target.
        if (filter_initialized_) {
            if (params_.publish.odometry) {
                PublishOdometry();
            }
            if (params_.publish.detection_3d) {
                PublishDetection3D();
            }
        }
        return;
    }
    ObjectTracker::EstimatorTick();
}

void FormationVehicleTracker::Reset() {
    ObjectTracker::Reset();
    recovery_monitor_->Reset();
}

void FormationVehicleTracker::UpdateSettings(
    const ObjectTrackerSettings& params) {
    ObjectTracker::UpdateSettings(params);
    recovery_monitor_->UpdateSettings(params.recovery);
}

bool FormationVehicleTracker::RecoveryMonitorTick() {
    TrackerRecoveryMonitor::TickInput input;
    input.state = state_;
    input.filter_initialized = filter_initialized_;
    input.has_tracked_target = has_tracked_target_;
    input.ctr_speed = filter_->GetCTRSpeed();
    const Eigen::Matrix<double, 5, 5> p = filter_->GetCTRCovariance();
    // Same x/y covariance block as published in PublishOdometry.
    input.xy_covariance << p(0, 0), p(0, 2), p(2, 0), p(2, 2);
    input.time_since_valid_target =
        (node_->get_clock()->now() - last_valid_target_time_).seconds();
    input.target_timeout = params_.tracking.target_timeout;

    const TrackerRecoveryMonitor::UpdateResult result =
        recovery_monitor_->Update(input);

    if (result.recovery_odom) {
        ApplyRecoveryOdometry(*result.recovery_odom);
        return true;
    }
    if (result.mark_lost && state_ != TrackerState::LOST) {
        RCLCPP_WARN(logger_,
                    "Tracker lost; requesting ground-truth recovery from "
                    "\"%s\" ...",
                    target_class_.c_str());
        state_ = TrackerState::LOST;
        has_tracked_target_ = false;
        has_detection_ = false;
        has_new_measurement_ = false;
    }
    return state_ == TrackerState::LOST;
}

void FormationVehicleTracker::ApplyRecoveryOdometry(const nav_msgs::msg::Odometry& odom) {

    const Eigen::Vector3d pos = core::ToEigen(odom.pose.pose.position);

    // Transform linear velocity to world coordinates expected by filter
    const Eigen::Quaterniond q = core::ToEigen(odom.pose.pose.orientation);
    Eigen::Vector3d vel = q.normalized() * core::ToEigen(odom.twist.twist.linear);

    // ResetCovariance() zeroes the fused position/velocity, so it must run
    // before the initial position/velocity are set.
    filter_->ResetCovariance();
    filter_->SetInitialPosition(pos);
    filter_->SetInitialVelocity(vel);
    filter_initialized_ = true;

    // Deliberately stays LOST (estimate is ground-truth-sourced, not
    // sensor-tracked); a fresh detection in TrackingTick ends LOST.
    ResetTrackingState(pos, TrackerState::LOST);

    RCLCPP_INFO(logger_,
                "Recovered tracker from ground-truth odometry of \"%s\" at "
                "(%.2f, %.2f, %.2f).",
                target_class_.c_str(), pos.x(), pos.y(), pos.z());
}

}  // namespace perception
}  // namespace avt_341_nav
