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
class QStackedLayout;
class QTableWidget;

namespace avt_341 {
namespace rviz_plugins {

/// Per-vehicle "Mission" tab content: shows a task's fields as a column of
/// "<Label>: <Value>" rows plus the vehicle's full task list. The pose field is
/// intentionally not shown.
///
/// The detail rows describe the active task, or -- once it finishes and nothing
/// replaces it -- the last one that ran, so the panel keeps reporting what the
/// vehicle was doing after it goes idle. The task list below them shows only
/// what is really outstanding: the active task first (highlighted) followed by
/// the queue behind it, and nothing at all when the vehicle is idle.
///
/// The single feed is the latched MissionModuleStatus topic, which is
/// authoritative: the mission manager publishes it on every task-list change,
/// including the change that empties the list, and it carries the active task's
/// full status alongside the queue. Its sibling MissionTaskStatus topic
/// re-sends the same fields at loop rate, but it is not subscribed: the only
/// field shown here that can change within a task is the task speed (via a
/// SET_SPEED command), and the mission manager republishes the module status
/// on that as well.
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

Q_SIGNALS:
    // Emitted on every module status with whether a task is now running, so the
    // panel can tag this vehicle's accordion header.
    void taskActiveChanged( bool active );

protected:
    // Applies an authoritative module status: detail rows and task list.
    void updateFromModuleStatus( const avt_341_msgs::msg::MissionModuleStatus& msg );

    // Writes the value labels from a task already known to be active.
    void setActiveTaskFields( const avt_341_msgs::msg::MissionTaskStatus& msg );

    // Refreshes the task-list table from a newly received module status.
    void updateTaskList( const avt_341_msgs::msg::MissionModuleStatus& msg );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    TopicConfig topics_;
    rclcpp::Subscription<avt_341_msgs::msg::MissionModuleStatus>::SharedPtr task_change_subscription_;

    // Whether the last module status named an active task. Until one arrives
    // there is none, which is also the correct display for an idle vehicle.
    bool has_active_task_ = false;

    // QT Widgets — the value side of each "<Label>: <Value>" row, plus the one
    // label whose text changes: the task-id row is titled "Task ID" while a
    // task is running and "Last Task ID" once the vehicle goes idle.
    QLabel* task_id_label_;
    QLabel* task_id_value_;
    QLabel* task_description_value_;
    QLabel* task_speed_value_;
    QLabel* tracked_vehicle_value_;
    QLabel* formation_type_value_;
    QLabel* formation_vehicles_value_;

    // Full-width table of task descriptions, active task first; the vertical
    // header provides the 1-based execution position. The table stays visible
    // when there is nothing to show, with an overlay stacked over its empty body
    // carrying the "No tasks." message.
    QTableWidget* task_list_table_;
    QStackedLayout* task_list_stack_;
    QWidget* task_list_empty_overlay_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MISSION_COMPONENT_H
