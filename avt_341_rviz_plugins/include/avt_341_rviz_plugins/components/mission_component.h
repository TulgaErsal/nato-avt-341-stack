#ifndef MISSION_COMPONENT_H
#define MISSION_COMPONENT_H

#ifndef Q_MOC_RUN
#include <cstdint>
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

/// Per-vehicle "Mission" tab content: shows the active task's fields as a
/// column of "<Label>: <Value>" rows plus a table of the queued (non-active)
/// task descriptions. The pose field is intentionally not shown.
///
/// Two topics feed this, with distinct roles:
///  - The latched MissionModuleStatus topic is *authoritative*. It is published
///    on every task-list change and names the active task (or an empty one when
///    nothing is running), so it alone decides what the rows show.
///  - The MissionTaskStatus topic is a live refresh for fields that mutate
///    within a task (a MoveTo description embeds its goal position, which the
///    mission manager rewrites when a contact is refined). It is silent while
///    no task is running, so a message arriving on it proves nothing about what
///    is currently active; it is applied only when its task id matches the
///    authoritative one.
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
    // Applies an authoritative module status: active task rows and queued table.
    void updateFromModuleStatus( const avt_341_msgs::msg::MissionModuleStatus& msg );

    // Live refresh; ignored unless it describes the current authoritative task.
    void updateFromMessage( const avt_341_msgs::msg::MissionTaskStatus& msg );

    // Writes the value labels from a task already known to be active.
    void setActiveTaskFields( const avt_341_msgs::msg::MissionTaskStatus& msg );

    // Blanks the value labels back to the empty marker.
    void clearActiveTask();

    // Refreshes the queued-tasks table from a newly received module status.
    void updateQueuedTasks( const avt_341_msgs::msg::MissionModuleStatus& msg );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    TopicConfig topics_;
    rclcpp::Subscription<avt_341_msgs::msg::MissionTaskStatus>::SharedPtr subscription_;
    rclcpp::Subscription<avt_341_msgs::msg::MissionModuleStatus>::SharedPtr task_change_subscription_;

    // The active task the last module status named. Until one arrives there is
    // no active task, which is also the correct display for an idle vehicle.
    bool has_active_task_ = false;
    std::int32_t active_task_id_ = -1;

    // QT Widgets — the value side of each "<Label>: <Value>" row.
    QLabel* task_id_value_;
    QLabel* task_description_value_;
    QLabel* tracked_vehicle_value_;
    QLabel* formation_type_value_;
    QLabel* formation_vehicles_value_;

    // Full-width table of queued (non-active) task descriptions; the vertical
    // header provides the 1-based queue position. The label replaces the table
    // while the queue is empty, so a bare table is never ambiguous between
    // "queue empty" and "nothing received yet".
    QTableWidget* queued_tasks_table_;
    QLabel* queued_tasks_empty_label_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MISSION_COMPONENT_H
