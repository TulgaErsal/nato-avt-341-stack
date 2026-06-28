#ifndef AUTONOMY_PANEL_H
#define AUTONOMY_PANEL_H

#ifndef Q_MOC_RUN
#include <functional>
#include <memory>

#include <QStringList>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

class SetupComponent;

class AutonomyPanel: public rviz_common::Panel
{

Q_OBJECT
public:
    AutonomyPanel( QWidget* parent = 0 );

    virtual void onInitialize() override;

    // Persist / restore the vehicle list, topic configuration and the compute
    // components' shared monitored-topic configuration across RViz sessions.
    void save( rviz_common::Config config ) const override;
    void load( const rviz_common::Config& config ) override;

protected Q_SLOTS:
    // Rebuilds the per-vehicle group boxes in every tab when the Setup tab's
    // vehicle list changes.
    void onVehiclesChanged( const QStringList& vehicles );

    // Re-creates the components in the affected tab when a configurable topic
    // changes, so they re-subscribe using the new topic name.
    void onTopicConfigChanged( avt_341::rviz_plugins::TopicGroup group );

protected:
    // Builds a scrollable tab and returns it; the layout used to hold the
    // per-vehicle group boxes is returned through out_content_layout.
    QWidget* createVehicleTab( QVBoxLayout*& out_content_layout );

    // Clears and repopulates content_layout with one group box per vehicle,
    // each wrapping the component produced by make_component( vehicle_id ).
    void rebuildVehicleTab( QVBoxLayout* content_layout, const QStringList& vehicles,
                            const std::function<QWidget*( const QString& )>& make_component );

    // Per-vehicle component factories, each reading the live topic config.
    QWidget* makeNavStateComponent( const QString& vehicle_id );
    QWidget* makeMissionComponent( const QString& vehicle_id );
    QWidget* makeTrackerComponent( const QString& vehicle_id );
    QWidget* makeComputeComponent( const QString& vehicle_id );

    // QT Widgets
    QTabWidget* tab_widget_;
    SetupComponent* setup_component_;
    QVBoxLayout* nav_state_layout_;
    QVBoxLayout* mission_layout_;
    QVBoxLayout* tracker_layout_;
    QVBoxLayout* compute_layout_;

    // The current vehicle list, kept so a single tab can be rebuilt when a topic
    // changes without waiting for the next vehicle-list change.
    QStringList current_vehicles_;

    // The ROS node, pumped from the Qt event loop by spin_timer_ so subscription
    // callbacks (e.g. in the per-vehicle compute components) are delivered.
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    QTimer* spin_timer_ = nullptr;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // AUTONOMY_PANEL_H
