#ifndef AUTONOMY_PANEL_H
#define AUTONOMY_PANEL_H

#ifndef Q_MOC_RUN
#include <functional>
#include <memory>

#include <QStringList>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>
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

protected Q_SLOTS:
    // Rebuilds the per-vehicle group boxes in every tab when the Setup tab's
    // vehicle list changes.
    void onVehiclesChanged( const QStringList& vehicles );

protected:
    // Builds a scrollable tab and returns it; the layout used to hold the
    // per-vehicle group boxes is returned through out_content_layout.
    QWidget* createVehicleTab( QVBoxLayout*& out_content_layout );

    // Clears and repopulates content_layout with one group box per vehicle,
    // each wrapping the component produced by make_component( vehicle_id ).
    void rebuildVehicleTab( QVBoxLayout* content_layout, const QStringList& vehicles,
                            const std::function<QWidget*( const QString& )>& make_component );

    // QT Widgets
    QTabWidget* tab_widget_;
    SetupComponent* setup_component_;
    QVBoxLayout* nav_state_layout_;
    QVBoxLayout* mission_layout_;
    QVBoxLayout* tracker_layout_;
    QVBoxLayout* compute_layout_;

    // The ROS node.
    rclcpp::Node::SharedPtr node_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // AUTONOMY_PANEL_H
