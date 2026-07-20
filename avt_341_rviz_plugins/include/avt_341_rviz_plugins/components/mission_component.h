#ifndef MISSION_COMPONENT_H
#define MISSION_COMPONENT_H

#ifndef Q_MOC_RUN
#include <memory>

#include <QString>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_msgs/msg/mission_module_status.hpp>
#include <avt_341_msgs/msg/mission_task_status.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

class QLabel;
class QTableWidget;

namespace avt_341 {
namespace rviz_plugins {

/// Per-vehicle "Mission" tab content: subscribes to that vehicle's
/// MissionTaskStatus topic and shows the task fields as a column of
/// "<Label>: <Value>" rows. The pose field is intentionally not shown.
/// Additionally subscribes to the latched MissionModuleStatus topic to show
/// the descriptions of the queued (non-active) tasks.
///
/// Like ComputeComponent this subscribes itself using the panel's node; the
/// panel spins that node on the UI thread, so the callback updates the labels
/// directly.
class MissionComponent: public QWidget
{

Q_OBJECT
public:
    MissionComponent( const QString& vehicle_id, rclcpp::Node::SharedPtr node,
                      const TopicConfig& topics, QWidget* parent = nullptr );

protected:
    // Refreshes the value labels from a newly received status message.
    void updateFromMessage( const avt_341_msgs::msg::MissionTaskStatus& msg );

    // Refreshes the queued-tasks table from a newly received module status.
    void updateQueuedTasks( const avt_341_msgs::msg::MissionModuleStatus& msg );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    TopicConfig topics_;
    rclcpp::Subscription<avt_341_msgs::msg::MissionTaskStatus>::SharedPtr subscription_;
    rclcpp::Subscription<avt_341_msgs::msg::MissionModuleStatus>::SharedPtr task_change_subscription_;

    // QT Widgets — the value side of each "<Label>: <Value>" row.
    QLabel* task_id_value_;
    QLabel* task_description_value_;
    QLabel* tracked_vehicle_value_;
    QLabel* formation_type_value_;
    QLabel* formation_vehicles_value_;

    // Full-width table of queued (non-active) task descriptions; the vertical
    // header provides the 1-based queue position.
    QTableWidget* queued_tasks_table_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MISSION_COMPONENT_H
