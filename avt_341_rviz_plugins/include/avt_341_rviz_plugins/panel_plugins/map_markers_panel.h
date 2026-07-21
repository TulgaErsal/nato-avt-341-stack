#ifndef MAP_MARKERS_PANEL_H
#define MAP_MARKERS_PANEL_H

#ifndef Q_MOC_RUN
#include <memory>

#include <QString>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_msgs/msg/map_marker_list.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#endif

class QLineEdit;
class QPushButton;
class QTableWidget;
class QTimer;

namespace avt_341 {
namespace rviz_plugins {

/// RViz panel that subscribes (latched) to an avt_341_msgs/MapMarkerList topic
/// and lists every marker's id, x/y position and yaw in a table. The topic is
/// editable in the panel and persisted with the RViz config.
class MapMarkersPanel: public rviz_common::Panel
{

Q_OBJECT
public:
    MapMarkersPanel( QWidget* parent = nullptr );

    void onInitialize() override;

    // Persist / restore the configured topic with the RViz display config.
    void save( rviz_common::Config config ) const override;
    void load( const rviz_common::Config& config ) override;

protected Q_SLOTS:
    // (Re)subscribe using the topic currently in the entry box.
    void updateTopic();

protected:
    // Replaces the subscription with a latched one for `topic`.
    void subscribe( const QString& topic );

    // Rebuilds the table rows from the latest MapMarkerList.
    void updateTable( const avt_341_msgs::msg::MapMarkerList& msg );

    // QT Widgets
    QLineEdit* topic_entry_;
    QPushButton* apply_button_;
    QTableWidget* table_;

    // The ROS node, pumped from the Qt event loop by spin_timer_ so subscription
    // callbacks arrive on the UI thread.
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    QTimer* spin_timer_ = nullptr;

    // Latched subscription to the configured MapMarkerList topic.
    rclcpp::Subscription<avt_341_msgs::msg::MapMarkerList>::SharedPtr subscription_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKERS_PANEL_H
