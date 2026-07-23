#ifndef SET_FORMATION_TOOL_H
#define SET_FORMATION_TOOL_H

#ifndef Q_MOC_RUN
#include <mutex>
#include <string>
#include <vector>

#include <QObject>

#include <avt_341_msgs/msg/communication.hpp>
#include <avt_341_msgs/msg/map_marker.hpp>
#include <avt_341_msgs/msg/map_marker_list.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <rviz_common/tool.hpp>
#endif

namespace rviz_common {
namespace properties {
class Property;
class StringProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
}
}

namespace avt_341 {
namespace rviz_plugins {

/// RViz tool "Set Formation": publishes an avt_341_msgs/Communication FORM
/// message whose objective is the map marker clicked in the scene.
///
/// This carves the FORM half out of MissionCommandPanel: every Communication
/// field except msg_id (always -1), type (always "FORM") and objective_name is
/// exposed as a tool property. objective_name is filled from the marker_id of
/// the MapMarker clicked on the map. The marker visuals are not individually
/// pickable, so the tool subscribes to the MapMarker / MapMarkerList topics
/// itself and, on click, matches the clicked ground point to the nearest known
/// marker within a configurable tolerance.
class SetFormationTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    SetFormationTool();
    ~SetFormationTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent( rviz_common::ViewportMouseEvent& event ) override;

private Q_SLOTS:
    /// (Re)create the Communication publisher when the command topic changes.
    void updateCommandTopic();
    /// (Re)create the marker subscriptions when either marker topic changes.
    void updateMarkerSubscriptions();

private:
    /// A marker we know about, kept in its own frame so it can be transformed to
    /// the fixed frame at click time.
    struct CachedMarker
    {
        std::string marker_id;
        std::string frame_id;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    void onMarker( avt_341_msgs::msg::MapMarker::ConstSharedPtr msg );
    void onMarkerList( avt_341_msgs::msg::MapMarkerList::ConstSharedPtr msg );

    /// Finds the known marker nearest (in the fixed frame's ground plane) to the
    /// point (\p px, \p py). Returns false if none is within the pick tolerance.
    bool findNearestMarker( double px, double py, std::string& marker_id_out ) const;

    /// Builds and publishes the FORM Communication for the given objective.
    void publishFormation( const std::string& objective_name );

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<avt_341_msgs::msg::Communication>::SharedPtr publisher_;
    rclcpp::Subscription<avt_341_msgs::msg::MapMarker>::SharedPtr marker_sub_;
    rclcpp::Subscription<avt_341_msgs::msg::MapMarkerList>::SharedPtr marker_list_sub_;

    // Marker caches, guarded because the subscription callbacks may run on a
    // different thread than processMouseEvent().
    mutable std::mutex markers_mutex_;
    std::vector<CachedMarker> single_markers_;   // from the MapMarker topic
    std::vector<CachedMarker> list_markers_;      // from the MapMarkerList topic

    // Topic + matching properties.
    rviz_common::properties::StringProperty* command_topic_property_;
    rviz_common::properties::StringProperty* marker_topic_property_;
    rviz_common::properties::StringProperty* marker_list_topic_property_;
    rviz_common::properties::FloatProperty* pick_tolerance_property_;

    // Communication-field properties (all fields except msg_id / type / objective).
    rviz_common::properties::StringProperty* sender_name_property_;
    rviz_common::properties::StringProperty* formation_property_;
    rviz_common::properties::StringProperty* receiver_name_property_;
    rviz_common::properties::StringProperty* leader_name_property_;
    rviz_common::properties::StringProperty* follower1_property_;
    rviz_common::properties::StringProperty* follower2_property_;
    rviz_common::properties::StringProperty* follower3_property_;
    rviz_common::properties::FloatProperty* desired_speed_property_;
    rviz_common::properties::EnumProperty* priority_type_property_;
    rviz_common::properties::EnumProperty* termination_method_property_;
    rviz_common::properties::FloatProperty* x_scale_property_;
    rviz_common::properties::FloatProperty* y_scale_property_;
    rviz_common::properties::FloatProperty* x_offset_property_;
    rviz_common::properties::FloatProperty* y_offset_property_;
    rviz_common::properties::FloatProperty* distance_property_;
    rviz_common::properties::IntProperty* target_msg_id_property_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // SET_FORMATION_TOOL_H
