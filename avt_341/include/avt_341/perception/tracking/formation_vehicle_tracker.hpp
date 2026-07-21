/**
* @file      formation_vehicle_tracker.hpp
* @brief     ObjectTracker specialization for formation vehicles: targets
             whose id is one of the "formation_vehicle_ids". Adds
             lost-detection (no-movement and uncertainty checks) and
             ground-truth recovery via the tracked vehicle's mission manager
             services, on top of the base tracking behavior.
*/

#ifndef AVT_341_FORMATION_VEHICLE_TRACKER_H
#define AVT_341_FORMATION_VEHICLE_TRACKER_H

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avt_341/perception/tracking/object_tracker.hpp>
#include <avt_341/perception/tracking/tracker_recovery_monitor.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief Tracker for a vehicle in our own formation. The owning node creates
 * this type (instead of the Generic base) when the target id is in
 * "formation_vehicle_ids". Only this tracker type owns a
 * TrackerRecoveryMonitor: formation vehicles host the check_speed /
 * get_odometry services the monitor relies on. Formation vehicles never
 * publish target contacts.
 */
class FormationVehicleTracker : public ObjectTracker {
   public:
    FormationVehicleTracker(rclcpp::Node* node,
                            const std::string& target_class,
                            const ObjectTrackerSettings& settings,
                            const core::CoordTransformer& coord_transformer,
                            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher);

    ObjectTrackerType GetTrackerType() const override {
        return ObjectTrackerType::FormationVehicle;
    }

    /** @brief Base estimator tick preceded by one lost-detection/recovery
     *         monitoring tick. While LOST the base estimation is suspended,
     *         but the last (ground-truth-recovered) estimate keeps
     *         publishing; a fresh camera detection ends LOST. */
    void EstimatorTick() override;

    void Reset() override;

    void UpdateSettings(const ObjectTrackerSettings& settings) override;

   private:
    /**
     * @brief Run one lost-detection/recovery monitoring tick.
     *
     * Feeds the recovery monitor and applies its verdict: transitions to
     * LOST when the monitor reports the tracker lost, and re-seeds the
     * filter from ground-truth odometry once a recovery completes.
     *
     * @return True when this estimator tick is consumed by LOST handling
     *         (the normal estimation must be skipped).
     */
    bool RecoveryMonitorTick();

    /** @brief Re-seed the filter from ground-truth target odometry and
     *         re-bootstrap the tracking state machine. */
    void ApplyRecoveryOdometry(const nav_msgs::msg::Odometry& odom);

    /** @brief Lost-detection/recovery monitor. */
    std::unique_ptr<TrackerRecoveryMonitor> recovery_monitor_;
};

}  // namespace perception
}  // namespace avt_341

#endif  // AVT_341_FORMATION_VEHICLE_TRACKER_H
