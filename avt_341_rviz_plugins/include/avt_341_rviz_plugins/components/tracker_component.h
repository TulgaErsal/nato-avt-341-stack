#ifndef TRACKER_COMPONENT_H
#define TRACKER_COMPONENT_H

#ifndef Q_MOC_RUN
#include <cstdint>
#include <memory>
#include <vector>

#include <QString>
#include <QStringList>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_msgs/msg/tracker_module_status.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

class QLabel;
class QVBoxLayout;

namespace avt_341 {
namespace rviz_plugins {

class AccordionGroup;
class MatrixField;
class VectorField;

/// Per-vehicle "Tracker" tab content. Subscribes to the vehicle's
/// TrackerModuleStatus topic and shows one collapsible "Tracked <id>" sub-group
/// per active child tracker, each with that target's state, the x/y/yaw
/// covariance matrix and the target x/y/yaw pose. A placeholder message is
/// shown while the module has no active trackers.
///
/// Like the other components it subscribes using the panel's node, which the
/// panel spins on the UI thread, so the callback updates the widgets directly.
class TrackerComponent: public QWidget
{

Q_OBJECT
public:
    TrackerComponent( const QString& vehicle_id, rclcpp::Node::SharedPtr node,
                      const TopicConfig& topics, QWidget* parent = nullptr );

protected:
    // Builds the static scaffold: the empty-state label and the (indented)
    // container that holds the per-target sub-groups.
    void buildUi();

    // Creates the topic subscription (a no-op without a node).
    void subscribe();

    // Refreshes the sub-groups and their values from a module-status message.
    void updateFromMessage( const avt_341_msgs::msg::TrackerModuleStatus& msg );

    // One collapsible "Tracked <id>" sub-group plus the widgets it updates.
    struct TargetView
    {
        QString object_id;
        AccordionGroup* group = nullptr;
        QLabel* state_value = nullptr;
        MatrixField* covariance_field = nullptr;
        VectorField* target_field = nullptr;
    };

    // Tears down and recreates the sub-groups so there is exactly one per id in
    // \p ids. Called only when the set/order of tracked targets changes, so
    // steady-state value updates preserve each group's expand/collapse state.
    void rebuildTargets( const QStringList& ids );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    TopicConfig topics_;

    // QT Widgets
    QLabel* empty_label_ = nullptr;          // shown when the module has no trackers
    QVBoxLayout* targets_layout_ = nullptr;  // holds the indented per-target groups
    std::vector<TargetView> target_views_;

    rclcpp::SubscriptionBase::SharedPtr tracker_sub_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // TRACKER_COMPONENT_H
