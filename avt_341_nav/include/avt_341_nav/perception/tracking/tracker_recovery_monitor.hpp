/**
* @file      tracker_recovery_monitor.hpp
* @brief     Lost-tracker detection (no-movement, uncertainty and
             measurement-timeout checks) and ground-truth recovery via the
             tracked formation vehicle's mission manager services.
*/

#ifndef AVT_341_TRACKER_RECOVERY_MONITOR_H
#define AVT_341_TRACKER_RECOVERY_MONITOR_H

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avt_341_msgs/srv/check_speed.hpp>
#include <avt_341_msgs/srv/get_odometry.hpp>

#include <avt_341_nav/core/running_stats.hpp>
#include <avt_341_nav/perception/tracking/tracker_dto.hpp>
#include <avt_341_nav/perception/tracking/tracker_params.hpp>

namespace avt_341_nav {
namespace perception {

/**
 * @brief Lost-tracker detection and recovery advisor for a single
 * formation-vehicle target. Owns the service plumbing and window statistics;
 * never mutates the owning tracker directly — Update() returns the actions
 * to apply. Not thread safe (single-threaded executor only).
 */
class TrackerRecoveryMonitor {
   public:
    /** @brief Per-estimator-tick snapshot of the owning tracker's state. */
    struct TickInput {
        TrackerState state = TrackerState::UNINITIALIZED;
        bool filter_initialized = false;
        /** @brief A real measurement association is currently held. */
        bool has_tracked_target = false;
        /** @brief Longitudinal target speed reported by the filter [m/s]. */
        double ctr_speed = 0.0;
        /** @brief x/y 2x2 block of the filter position covariance. */
        Eigen::Matrix2d xy_covariance = Eigen::Matrix2d::Zero();
        /** @brief Time [s] since the last valid measurement from any sensor. */
        double time_since_valid_target = 0.0;
        /** @brief Measurement timeout [s] ("tracking.target_timeout"). */
        double target_timeout = 0.0;
    };

    /** @brief Actions for the owning tracker to apply after Update(). */
    struct UpdateResult {
        /** @brief The tracker should transition to TrackerState::LOST. */
        bool mark_lost = false;
        /** @brief Ground-truth odometry to re-seed the filter from. */
        std::optional<nav_msgs::msg::Odometry> recovery_odom;
    };

    /** @param target_class Vehicle namespace of the tracked formation
     *         vehicle; used to address its mission manager services. */
    TrackerRecoveryMonitor(rclcpp::Node* node,
                           const std::string& target_class,
                           const RecoverySettings& params,
                           const rclcpp::Logger& logger);

    /** @brief Run one monitoring tick. Call once per estimator tick, in
     *         every tracker state (LOST drives the recovery call). */
    UpdateResult Update(const TickInput& input);

    /** @brief Full reset: windows, backoff timing and pending requests. */
    void Reset();

    /** @brief Replace the parameters; resets the statistics windows. */
    void UpdateSettings(const RecoverySettings& params);

   private:
    enum class RequestState { IDLE, WAITING_RESPONSE };

    /** @brief Shared with in-flight response callbacks so a late response
     *         outlives this monitor safely; generations invalidate aborted
     *         requests. */
    struct AsyncLatch {
        std::uint64_t check_speed_generation = 0;
        std::uint64_t get_odometry_generation = 0;
        bool has_check_speed_result = false;
        bool target_actually_moving = false;
        std::optional<nav_msgs::msg::Odometry> recovery_odom;
    };

    void RunRecovery(double now);
    void RunNoMovementCheck(double now);
    void SendCheckSpeedRequest(double now);
    void SendGetOdometryRequest(double now);
    void AbortPendingConfirm();
    void AbortPendingRecovery();
    void ResetSpeedWindow();
    void ResetUncertaintyWindow();
    void ResetWindows();
    core::RunningStats MakeSpeedStats() const;
    core::RunningStats MakeUncertaintyStats() const;
    bool IsInNoMovementCheckState(TrackerState state) const;

    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    std::string target_class_;
    RecoverySettings params_;

    rclcpp::Client<avt_341_msgs::srv::CheckSpeed>::SharedPtr
        check_speed_client_;
    rclcpp::Client<avt_341_msgs::srv::GetOdometry>::SharedPtr
        get_odometry_client_;

    /** @brief Windowed target speed; threshold met when mean below
     *         no_movement_threshold. */
    core::RunningStats speed_stats_;
    /** @brief Windowed major-axis position std dev [m]; threshold met when
     *         mean above uncertainty_threshold. */
    core::RunningStats uncertainty_stats_;
    /** @brief First-sample time of each window; negative while empty. Used
     *         to require a full window before evaluating thresholds. */
    double speed_window_start_ = -1.0;
    double uncertainty_window_start_ = -1.0;

    RequestState confirm_state_ = RequestState::IDLE;
    double confirm_sent_time_ = 0.0;
    double confirm_backoff_until_ = 0.0;

    RequestState recovery_state_ = RequestState::IDLE;
    double recovery_sent_time_ = 0.0;
    double recovery_retry_after_ = 0.0;

    /** @brief Sustained tracking observed; arms the timeout check. */
    bool has_been_active_ = false;
    /** @brief Start of the current tracking streak; negative when broken. */
    double active_streak_start_ = -1.0;
    /** @brief Timeout-triggered recoveries since the last re-acquisition. */
    int timeout_recovery_attempts_ = 0;

    std::shared_ptr<AsyncLatch> latch_ = std::make_shared<AsyncLatch>();

    /** @brief Give up on an unanswered service request after this long [s]. */
    static constexpr double kServiceResponseTimeout = 2.0;
    /** @brief Wait between retries after a timeout/invalid response [s]. */
    static constexpr double kRetryBackoff = 1.0;
    /** @brief Throttle period for "service unavailable" warnings [ms]. */
    static constexpr int kServiceWarnPeriodMs = 5000;
    /** @brief Tracking must persist this long [s] to count as (re)acquired,
     *         so brief post-recovery association blips do not count. */
    static constexpr double kSustainedTrackingTime = 1.0;
};

}  // namespace perception
}  // namespace avt_341_nav

#endif  // AVT_341_TRACKER_RECOVERY_MONITOR_H
