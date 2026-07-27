/**
* @file      tracker_recovery_monitor.cpp
* @brief     Lost-tracker detection and ground-truth recovery for formation
             vehicle targets. See tracker_recovery_monitor.hpp.
*/

#include <avt_341/perception/tracking/tracker_recovery_monitor.hpp>

#include <algorithm>
#include <cmath>

#include <avt_341/core/math_utils.hpp>

namespace avt_341 {
namespace perception {

TrackerRecoveryMonitor::TrackerRecoveryMonitor(
    rclcpp::Node* node, const std::string& target_class,
    const RecoverySettings& params, const rclcpp::Logger& logger)
    : node_(node),
      logger_(logger),
      target_class_(target_class),
      params_(params) {
    // Both services live on the tracked vehicle's own mission manager, so
    // they are addressed absolutely under the target's namespace.
    check_speed_client_ = node_->create_client<avt_341_msgs::srv::CheckSpeed>(
        "/" + target_class_ + "/avt_341/check_speed");
    get_odometry_client_ = node_->create_client<avt_341_msgs::srv::GetOdometry>(
        "/" + target_class_ + "/avt_341/get_odometry");

    speed_stats_ = MakeSpeedStats();
    uncertainty_stats_ = MakeUncertaintyStats();
}

TrackerRecoveryMonitor::UpdateResult TrackerRecoveryMonitor::Update(
    const TickInput& input) {
    UpdateResult result;
    const double now = node_->get_clock()->now().seconds();

    // Consume async results latched since the last tick.
    if (latch_->recovery_odom) {
        const nav_msgs::msg::Odometry odom = *latch_->recovery_odom;
        latch_->recovery_odom.reset();
        recovery_state_ = RequestState::IDLE;

        // A default-constructed odometry means the mission manager has not
        // received any odometry itself yet.
        const bool is_valid = !odom.header.frame_id.empty() ||
                              odom.header.stamp.sec != 0 ||
                              odom.header.stamp.nanosec != 0u;
        if (!is_valid) {
            RCLCPP_WARN(logger_,
                        "Recovery odometry from \"%s\" is empty, retrying ...",
                        target_class_.c_str());
            recovery_retry_after_ = now + kRetryBackoff;
            return result;
        }

        ResetWindows();
        confirm_backoff_until_ = now + params_.no_movement_backoff_time;
        // Refresh pace of the ground truth while the tracker stays LOST.
        recovery_retry_after_ = now + params_.no_movement_backoff_time;
        result.recovery_odom = odom;
        return result;
    }

    if (latch_->has_check_speed_result) {
        latch_->has_check_speed_result = false;
        confirm_state_ = RequestState::IDLE;
        if (latch_->target_actually_moving) {
            // The target refutes the tracker's no-movement belief: lost.
            RCLCPP_WARN(logger_,
                        "\"%s\" reports it is moving while the tracker sees "
                        "no movement: tracker is lost.",
                        target_class_.c_str());
            ResetWindows();
            result.mark_lost = true;
            return result;
        }
        // Genuinely stationary: keep the speed window so the still-low mean
        // can re-confirm right after the backoff.
        RCLCPP_INFO(logger_,
                    "\"%s\" confirmed stationary; tracker is correct. Next "
                    "no-movement confirmation in %.1f s.",
                    target_class_.c_str(),
                    params_.no_movement_backoff_time);
        confirm_backoff_until_ = now + params_.no_movement_backoff_time;
    }

    if (input.state == TrackerState::LOST) {
        RunRecovery(now);
        return result;
    }

    // Only sustained tracking arms the timeout check and clears its give-up
    // counter (see kSustainedTrackingTime).
    if (input.filter_initialized && input.has_tracked_target &&
        IsActiveTrackerState(input.state)) {
        if (active_streak_start_ < 0.0) {
            active_streak_start_ = now;
        }
        if (now - active_streak_start_ >= kSustainedTrackingTime) {
            has_been_active_ = true;
            timeout_recovery_attempts_ = 0;
        }
    } else {
        active_streak_start_ = -1.0;
    }

    // Measurement timeout: measurements starved after active tracking (the
    // other two checks cannot catch a target leaving the sensor view).
    // allow_never_tracked extends it to targets never acquired at all.
    if (params_.timeout_enabled &&
        (has_been_active_ || params_.timeout_allow_never_tracked) &&
        input.target_timeout > 0.0 &&
        input.time_since_valid_target > input.target_timeout) {
        if (timeout_recovery_attempts_ >= params_.timeout_max_attempts) {
            RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(),
                                 kServiceWarnPeriodMs,
                                 "Measurement timeout for \"%s\", but %d "
                                 "recovery attempts were made without "
                                 "re-acquisition; allowing the tracker to "
                                 "deactivate.",
                                 target_class_.c_str(),
                                 timeout_recovery_attempts_);
        } else {
            timeout_recovery_attempts_++;
            RCLCPP_WARN(logger_,
                        "No valid measurement of \"%s\" for %.1f s after "
                        "active tracking: tracker is lost (recovery attempt "
                        "%d/%d).",
                        target_class_.c_str(),
                        input.time_since_valid_target,
                        timeout_recovery_attempts_,
                        static_cast<int>(params_.timeout_max_attempts));
            AbortPendingConfirm();
            ResetWindows();
            result.mark_lost = true;
            return result;
        }
    }

    // No-movement window: samples accumulate only in the configured states;
    // leaving them resets the window and any pending confirmation. A
    // disabled pathway never fills its window, so its check never fires.
    if (params_.no_movement_enabled && input.filter_initialized &&
        IsInNoMovementCheckState(input.state)) {
        if (speed_window_start_ < 0.0) {
            speed_window_start_ = now;
        }
        speed_stats_.AddSample(std::abs(input.ctr_speed), now);
    } else {
        ResetSpeedWindow();
    }

    // Uncertainty window: samples accumulate only while actively tracking
    // (a covariance reset seeds large values that would poison the mean).
    if (params_.uncertainty_enabled && input.filter_initialized &&
        IsActiveTrackerState(input.state)) {
        if (uncertainty_window_start_ < 0.0) {
            uncertainty_window_start_ = now;
        }
        uncertainty_stats_.AddSample(
            core::MajorAxisStdDev2x2(input.xy_covariance), now);
    } else {
        ResetUncertaintyWindow();
    }

    // Uncertainty above the threshold over a full window: lost.
    if (uncertainty_window_start_ >= 0.0 &&
        now - uncertainty_window_start_ >= params_.uncertainty_window_time) {
        const core::StatsSnapshot stats = uncertainty_stats_.GetStats(now);
        if (uncertainty_stats_.GetConfig().IsThresholdMet(stats.mean)) {
            RCLCPP_WARN(logger_,
                        "Mean position uncertainty (major-axis std dev) "
                        "%.2f m over the last %.1f s exceeds %.2f m: tracker "
                        "is lost.",
                        stats.mean, params_.uncertainty_window_time,
                        params_.uncertainty_threshold);
            AbortPendingConfirm();
            ResetWindows();
            result.mark_lost = true;
            return result;
        }
    }

    RunNoMovementCheck(now);
    return result;
}

void TrackerRecoveryMonitor::RunNoMovementCheck(const double now) {
    if (confirm_state_ == RequestState::WAITING_RESPONSE) {
        if (now - confirm_sent_time_ > kServiceResponseTimeout) {
            RCLCPP_WARN(logger_,
                        "check_speed response from \"%s\" timed out.",
                        target_class_.c_str());
            AbortPendingConfirm();
            confirm_backoff_until_ = now + kRetryBackoff;
        }
        return;
    }

    if (now < confirm_backoff_until_) {
        return;
    }
    if (speed_window_start_ < 0.0 ||
        now - speed_window_start_ < params_.no_movement_window_time) {
        return;
    }
    if (!speed_stats_.GetConfig().IsThresholdMet(
            speed_stats_.GetStats(now).mean)) {
        return;
    }

    if (!check_speed_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), kServiceWarnPeriodMs,
                             "check_speed service of \"%s\" unavailable; "
                             "cannot confirm no movement.",
                             target_class_.c_str());
        return;
    }
    SendCheckSpeedRequest(now);
}

void TrackerRecoveryMonitor::RunRecovery(const double now) {
    if (recovery_state_ == RequestState::WAITING_RESPONSE) {
        if (now - recovery_sent_time_ > kServiceResponseTimeout) {
            RCLCPP_WARN(logger_,
                        "get_odometry response from \"%s\" timed out.",
                        target_class_.c_str());
            AbortPendingRecovery();
            recovery_retry_after_ = now + kRetryBackoff;
        }
        return;
    }

    if (now < recovery_retry_after_) {
        return;
    }
    if (!get_odometry_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), kServiceWarnPeriodMs,
                             "get_odometry service of \"%s\" unavailable; "
                             "cannot recover lost tracker.",
                             target_class_.c_str());
        return;
    }
    SendGetOdometryRequest(now);
}

void TrackerRecoveryMonitor::SendCheckSpeedRequest(const double now) {
    auto request = std::make_shared<avt_341_msgs::srv::CheckSpeed::Request>();
    // "Is your actual longitudinal speed above the no-movement threshold?"
    request->speed = params_.no_movement_threshold;
    request->operation = "gt";
    request->threshold = 0.0;

    const std::uint64_t generation = latch_->check_speed_generation;
    const std::shared_ptr<AsyncLatch> latch = latch_;
    check_speed_client_->async_send_request(
        request,
        [latch, generation](
            rclcpp::Client<avt_341_msgs::srv::CheckSpeed>::SharedFuture
                future) {
            if (generation != latch->check_speed_generation) {
                return;  // Aborted request; drop the response.
            }
            latch->target_actually_moving = future.get()->is_true;
            latch->has_check_speed_result = true;
        });
    confirm_sent_time_ = now;
    confirm_state_ = RequestState::WAITING_RESPONSE;
    RCLCPP_INFO(logger_,
                "No movement of \"%s\" detected; confirming via its "
                "check_speed service ...",
                target_class_.c_str());
}

void TrackerRecoveryMonitor::SendGetOdometryRequest(const double now) {
    auto request = std::make_shared<avt_341_msgs::srv::GetOdometry::Request>();
    request->vehicle_id = "";  // Empty: the target's own ground-truth odometry.

    const std::uint64_t generation = latch_->get_odometry_generation;
    const std::shared_ptr<AsyncLatch> latch = latch_;
    get_odometry_client_->async_send_request(
        request,
        [latch, generation](
            rclcpp::Client<avt_341_msgs::srv::GetOdometry>::SharedFuture
                future) {
            if (generation != latch->get_odometry_generation) {
                return;  // Aborted request; drop the response.
            }
            latch->recovery_odom = future.get()->odom;
        });
    recovery_sent_time_ = now;
    recovery_state_ = RequestState::WAITING_RESPONSE;
    RCLCPP_INFO(logger_,
                "Requesting ground-truth odometry from \"%s\" for tracker "
                "recovery ...",
                target_class_.c_str());
}

void TrackerRecoveryMonitor::AbortPendingConfirm() {
    latch_->check_speed_generation++;
    latch_->has_check_speed_result = false;
    check_speed_client_->prune_pending_requests();
    confirm_state_ = RequestState::IDLE;
}

void TrackerRecoveryMonitor::AbortPendingRecovery() {
    latch_->get_odometry_generation++;
    latch_->recovery_odom.reset();
    get_odometry_client_->prune_pending_requests();
    recovery_state_ = RequestState::IDLE;
}

void TrackerRecoveryMonitor::ResetSpeedWindow() {
    if (speed_window_start_ < 0.0 && confirm_state_ == RequestState::IDLE) {
        return;
    }
    speed_stats_ = MakeSpeedStats();
    speed_window_start_ = -1.0;
    AbortPendingConfirm();
}

void TrackerRecoveryMonitor::ResetUncertaintyWindow() {
    if (uncertainty_window_start_ < 0.0) {
        return;
    }
    uncertainty_stats_ = MakeUncertaintyStats();
    uncertainty_window_start_ = -1.0;
}

void TrackerRecoveryMonitor::ResetWindows() {
    ResetSpeedWindow();
    ResetUncertaintyWindow();
}

void TrackerRecoveryMonitor::Reset() {
    ResetWindows();
    AbortPendingRecovery();
    confirm_backoff_until_ = 0.0;
    recovery_retry_after_ = 0.0;
    has_been_active_ = false;
    active_streak_start_ = -1.0;
    timeout_recovery_attempts_ = 0;
}

void TrackerRecoveryMonitor::UpdateSettings(const RecoverySettings& params) {
    params_ = params;
    // The window/threshold configuration may have changed.
    ResetWindows();
}

core::RunningStats TrackerRecoveryMonitor::MakeSpeedStats() const {
    core::RunningStatsConfig config;
    config.window_time = params_.no_movement_window_time;
    config.threshold_check = params_.no_movement_threshold;
    config.threshold_greater_than = false;  // No movement: mean BELOW threshold.
    return core::RunningStats(config);
}

core::RunningStats TrackerRecoveryMonitor::MakeUncertaintyStats() const {
    core::RunningStatsConfig config;
    config.window_time = params_.uncertainty_window_time;
    config.threshold_check = params_.uncertainty_threshold;
    config.threshold_greater_than = true;  // Uncertain: mean ABOVE threshold.
    return core::RunningStats(config);
}

bool TrackerRecoveryMonitor::IsInNoMovementCheckState(
    const TrackerState state) const {
    return IsConfiguredTrackerState(params_, state);
}

}  // namespace perception
}  // namespace avt_341
