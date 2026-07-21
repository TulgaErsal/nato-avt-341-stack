/**
* @file      formation_vehicle_tracker.hpp
* @brief     ObjectTracker specialization for formation vehicles: adds
             lost-detection and ground-truth recovery via the tracked
             vehicle's mission manager services.
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
 * @brief Tracker for a vehicle in our own formation (target id in
 * "formation_vehicle_ids"). The only tracker type with a
 * TrackerRecoveryMonitor: formation vehicles host the services it relies
 * on. Never publishes target contacts.
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

    /** @brief Base estimator tick preceded by a recovery monitoring tick.
     *         While LOST the base estimation is suspended but the recovered
     *         estimate keeps publishing. */
    void EstimatorTick() override;

    void Reset() override;

    void UpdateSettings(const ObjectTrackerSettings& settings) override;

   private:
    /** @brief Feed the recovery monitor and apply its verdict (mark LOST /
     *         re-seed from ground truth). Returns true when this tick is
     *         consumed by LOST handling. */
    bool RecoveryMonitorTick();

    /** @brief Re-seed the filter from ground-truth target odometry. */
    void ApplyRecoveryOdometry(const nav_msgs::msg::Odometry& odom);

    /** @brief Lost-detection/recovery monitor. */
    std::unique_ptr<TrackerRecoveryMonitor> recovery_monitor_;
};

}  // namespace perception
}  // namespace avt_341

#endif  // AVT_341_FORMATION_VEHICLE_TRACKER_H
