#ifndef TRACKER_COMPONENT_H
#define TRACKER_COMPONENT_H

#ifndef Q_MOC_RUN
#include <cstdint>
#include <memory>

#include <QString>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

class QLabel;

namespace avt_341 {
namespace rviz_plugins {

class MatrixField;
class VectorField;

/// Per-vehicle "Tracker" tab content. Subscribes to the vehicle's tracker-info
/// topic and shows the tracker state as a colored status label, plus a
/// covariance matrix and the target's range/bearing (placeholder values for now).
///
/// Like the other components it subscribes using the panel's node, which the
/// panel spins on the UI thread, so the callback updates the widget directly.
class TrackerComponent: public QWidget
{

Q_OBJECT
public:
    TrackerComponent( const QString& vehicle_id, rclcpp::Node::SharedPtr node,
                      const TopicConfig& topics, QWidget* parent = nullptr );

protected:
    // Builds the row widgets and lays them out.
    void buildUi();

    // Creates the topic subscription (a no-op without a node).
    void subscribe();

    // Sets the state row's text and background color from a TrackerInfo state.
    void setTrackerStatus( uint8_t state );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    TopicConfig topics_;

    // QT Widgets
    QLabel* state_label_ = nullptr;
    QLabel* state_value_ = nullptr;

    // Covariance matrix and target range/bearing rows; placeholder values until
    // the tracker message carries them.
    MatrixField* covariance_field_ = nullptr;
    VectorField* target_field_ = nullptr;

    // Type-erased so the header needs no message includes; the typed
    // subscription is created in the .cpp.
    rclcpp::SubscriptionBase::SharedPtr tracker_sub_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // TRACKER_COMPONENT_H
