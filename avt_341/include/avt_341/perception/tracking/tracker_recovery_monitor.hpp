/**
* @file      tracker_recovery_monitor.hpp
* @brief     Lost-tracker detection and ground-truth recovery for formation
             vehicle targets. Detects poor tracking performance via moving
             time-window statistics (no-movement and uncertainty checks) and
             a measurement-timeout check (measurements starved after active
             tracking), confirms suspected no-movement with the tracked
             vehicle's mission manager over the check_speed service, and
             fetches ground-truth odometry over the get_odometry service to
             re-seed the tracker's filter once lost.
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

#include <avt_341/core/running_stats.hpp>
#include <avt_341/perception/tracking/tracker_dto.hpp>
#include <avt_341/perception/tracking/tracker_params.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief Lost-tracker detection and recovery advisor for a single
 * formation-vehicle target.
 *
 * Owns all ROS service plumbing (clients on the tracked vehicle's mission
 * manager, async request bookkeeping, backoff timing) and the moving-window
 * statistics. It never mutates the owning ObjectTracker directly: Update()
 * returns the actions the tracker should apply (mark itself LOST and/or
 * re-seed its filter from ground-truth odometry).
 *
 * Not thread safe: intended for a single-threaded executor where service
 * response callbacks and the estimator timer never run concurrently.
 */
class TrackerRecoveryMonitor {
   public:
    /** @brief Per-estimator-tick snapshot of the owning tracker's state. */
    struct TickInput {
        TrackerState state = TrackerState::UNINITIALIZED;
        bool filter_initialized = false;
        /** @brief Whether the tracker currently has a valid and recent
         *         tracked target centroid (a real measurement association,
         *         as opposed to a state set without one). */
        bool has_tracked_target = false;
        /** @brief Longitudinal target speed reported by the filter [m/s]. */
        double ctr_speed = 0.0;
        /** @brief x/y 2x2 block of the filter position covariance. */
        Eigen::Matrix2d xy_covariance = Eigen::Matrix2d::Zero();
        /** @brief Time [s] since the last valid measurement from any sensor. */
        double time_since_valid_target = 0.0;
        /** @brief The tracker's measurement timeout [s] ("tracker_timeout");
         *         the measurement-timeout check fires on the same boundary
         *         as the tracker's own timeout demotions. */
        double target_timeout = 0.0;
    };

    /** @brief Actions for the owning tracker to apply after Update(). */
    struct TickResult {
        /** @brief The tracker should transition to TrackerState::LOST. */
        bool mark_lost = false;
        /** @brief Ground-truth odometry to re-seed the filter from; set once
         *         a recovery service call completes while LOST. */
        std::optional<nav_msgs::msg::Odometry> recovery_odom;
    };

    /**
     * @param node Owning node (used for clocks and service clients).
     * @param target_class Tracked target id; must be the vehicle namespace of
     *        a formation vehicle (used to address its mission manager).
     * @param settings Recovery thresholds and windows.
     * @param logger Per-target logger of the owning tracker.
     */
    TrackerRecoveryMonitor(rclcpp::Node* node,
                           const std::string& target_class,
                           const RecoverySettings& settings,
                           const rclcpp::Logger& logger);

    /**
     * @brief Run one monitoring tick. Call once per estimator tick, in every
     * tracker state (including LOST, where it drives the recovery call).
     */
    TickResult Update(const TickInput& input);

    /** @brief Full reset: windows, backoff timing and pending requests.
     *         Service clients are kept. */
    void Reset();

    /** @brief Replace the settings (dynamic parameter propagation). Resets
     *         the statistics windows since their configuration may change. */
    void UpdateSettings(const RecoverySettings& settings);

   private:
    enum class RequestState { IDLE, WAITING_RESPONSE };

    /** @brief Shared with in-flight service response callbacks so a response
     *         arriving after this monitor (and its owning tracker) has been
     *         destroyed only touches this struct. The generation counters
     *         invalidate responses of aborted/timed-out requests. */
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
    static double MajorAxisStdDev2x2(const Eigen::Matrix2d& p);

    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    std::string target_class_;
    RecoverySettings settings_;

    rclcpp::Client<avt_341_msgs::srv::CheckSpeed>::SharedPtr
        check_speed_client_;
    rclcpp::Client<avt_341_msgs::srv::GetOdometry>::SharedPtr
        get_odometry_client_;

    /** @brief Windowed target speed; threshold met when mean BELOW
     *         no_movement_threshold. */
    core::RunningStats speed_stats_;
    /** @brief Windowed position uncertainty: standard deviation [m] along
     *         the axis of largest variance of the x/y covariance block;
     *         threshold met when mean ABOVE uncertainty_threshold. */
    core::RunningStats uncertainty_stats_;
    /** @brief Time [s] of the first sample since the last window reset;
     *         negative while the window is empty. Used to require a full
     *         window before evaluating thresholds. */
    double speed_window_start_ = -1.0;
    double uncertainty_window_start_ = -1.0;

    RequestState confirm_state_ = RequestState::IDLE;
    double confirm_sent_time_ = 0.0;
    /** @brief No new check_speed confirmation before this time [s]. */
    double confirm_backoff_until_ = 0.0;

    RequestState recovery_state_ = RequestState::IDLE;
    double recovery_sent_time_ = 0.0;
    double recovery_retry_after_ = 0.0;

    /** @brief True once the tracker has genuinely tracked the target for
     *         kSustainedTrackingTime; arms the measurement-timeout check. */
    bool has_been_active_ = false;
    /** @brief Start time [s] of the current uninterrupted genuine-tracking
     *         streak; negative while not genuinely tracking. */
    double active_streak_start_ = -1.0;
    /** @brief Consecutive timeout-triggered recoveries without a sustained
     *         re-acquisition; capped by timeout_max_attempts. */
    int timeout_recovery_attempts_ = 0;

    std::shared_ptr<AsyncLatch> latch_ = std::make_shared<AsyncLatch>();

    /** @brief Give up on an unanswered service request after this long [s]. */
    static constexpr double kServiceResponseTimeout = 2.0;
    /** @brief Wait between retries after a timeout/invalid response [s]. */
    static constexpr double kRetryBackoff = 1.0;
    /** @brief Throttle period for "service unavailable" warnings [ms]. */
    static constexpr int kServiceWarnPeriodMs = 5000;
    /** @brief Genuine tracking must persist this long [s] to arm the
     *         measurement-timeout check and clear its give-up counter. Keeps
     *         the brief post-recovery association attempts against stale
     *         detections from counting as a re-acquisition. */
    static constexpr double kSustainedTrackingTime = 1.0;
};

}  // namespace perception
}  // namespace avt_341

#endif  // AVT_341_TRACKER_RECOVERY_MONITOR_H
