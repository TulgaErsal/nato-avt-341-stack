#ifndef NAV_STATE_COMPONENT_H
#define NAV_STATE_COMPONENT_H

#ifndef Q_MOC_RUN
#include <memory>

#include <QColor>
#include <QString>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

class QLabel;

namespace avt_341 {
namespace rviz_plugins {

class VectorField;

/// Per-vehicle "Nav State" tab content. Subscribes to the vehicle's odometry,
/// nav-state, command-velocity and desired-speed topics and shows pose,
/// velocity, command, run state, goal and goal duration.
///
/// Like the other components it subscribes using the panel's node, which the
/// panel spins on the UI thread, so the callbacks update the widgets directly.
class NavStateComponent: public QWidget
{

Q_OBJECT
public:
    NavStateComponent( const QString& vehicle_id, rclcpp::Node::SharedPtr node,
                       const TopicConfig& topics, QWidget* parent = nullptr );

Q_SIGNALS:
    // Emitted whenever the run-state row changes, carrying the same text and cell
    // color shown in the Nav State field. The Setup table mirrors it.
    void navStateChanged( const QString& text, const QColor& color );

protected:
    // Builds the row widgets and lays them out with a shared label column.
    void buildUi();

    // Creates the topic subscriptions (a no-op without a node).
    void subscribe();

    // Refreshes the velocity row from the cached linear velocity (odometry) and
    // desired speed, whose components arrive on two different topics.
    void updateVelocityField();

    // Sets the run-state row's text and background color from a run_state value.
    void setNavStateStatus( int run_state );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    TopicConfig topics_;

    // Velocity components cached so either source topic can refresh the row.
    double linear_velocity_x_ = 0.0;  // from odometry
    double linear_velocity_y_ = 0.0;  // from odometry
    double desired_speed_ = 0.0;      // from desired_speed

    // QT Widgets
    VectorField* pose_field_ = nullptr;
    VectorField* velocity_field_ = nullptr;
    VectorField* command_field_ = nullptr;
    VectorField* goal_field_ = nullptr;
    QLabel* nav_state_label_ = nullptr;
    QLabel* nav_state_value_ = nullptr;
    QLabel* duration_label_ = nullptr;
    QLabel* duration_value_ = nullptr;

    // Type-erased so the header needs no message includes; the typed
    // subscriptions are created in the .cpp.
    rclcpp::SubscriptionBase::SharedPtr odometry_sub_;
    rclcpp::SubscriptionBase::SharedPtr nav_state_sub_;
    rclcpp::SubscriptionBase::SharedPtr cmd_vel_sub_;
    rclcpp::SubscriptionBase::SharedPtr desired_speed_sub_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_STATE_COMPONENT_H
