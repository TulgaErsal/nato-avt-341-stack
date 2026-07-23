/**
* @file      toi_tracker.hpp
* @brief     ObjectTracker specialization for targets of interest (TOI):
             targets whose id matches the "tracker_toi_regex" parameter.
             Publishes target contacts while actively tracking so the mission
             manager can trigger an investigation task.
*/

#ifndef AVT_341_TOI_TRACKER_H
#define AVT_341_TOI_TRACKER_H

#include <string>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avt_341/perception/tracking/object_tracker.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief Tracker for a target of interest. The owning node creates this type
 * (instead of the Generic base) when the target id matches
 * "tracker_toi_regex". Adds target-contact publishing on top of the base
 * tracking behavior.
 */
class ToiTracker : public ObjectTracker {
   public:
    ToiTracker(rclcpp::Node* node,
               const std::string& target_class,
               const ObjectTrackerSettings& settings,
               const core::CoordTransformer& coord_transformer,
               rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher,
               rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher);

    ObjectTrackerType GetTrackerType() const override {
        return ObjectTrackerType::Toi;
    }

    void Reset() override;

   protected:
    /** @brief Publish a contact on the first active-tracking tick (encircle
     *         trigger) and then every contact_update_interval_ticks_. */
    void MaybePublishContactUpdate() override;

   private:
    void PublishTargetContact();

    /** @brief Shared target contacts publisher owned by the node. */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher_;

    // Target contacts (encircle trigger)
    // -------------------------------------------------------------------------

    bool encircle_triggered_ = false;

    int contact_update_counter_ = 0;

    static constexpr int contact_update_interval_ticks_ = 10;
};

}  // namespace perception
}  // namespace avt_341

#endif  // AVT_341_TOI_TRACKER_H
