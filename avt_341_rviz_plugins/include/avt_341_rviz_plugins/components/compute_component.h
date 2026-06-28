#ifndef COMPUTE_COMPONENT_H
#define COMPUTE_COMPONENT_H

#ifndef Q_MOC_RUN
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <QString>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#endif

class QPushButton;
class QTableWidget;
class QTimer;

namespace avt_341 {
namespace rviz_plugins {

/// Per-vehicle "Compute" tab content: a small read-only table that reports the
/// live publish rate of monitored ROS topics against an expected rate, turning
/// a row red whenever its measured rate drops below an acceptable fraction of
/// that expected rate.
///
/// The monitored topics, their expected rates and the alert threshold are a
/// single per-process configuration shared across all vehicles, edited through a
/// "Configure Monitored Topics" popup. A change made on one vehicle's component
/// re-applies to every live component.
///
/// Rates are measured type-agnostically: each topic's type is discovered from
/// the ROS graph at runtime and subscribed via a generic (serialized)
/// subscription, so messages are counted without being deserialized and the
/// component needs no compile-time dependency on any message package.
class ComputeComponent: public QWidget
{

Q_OBJECT
public:
    ComputeComponent( const QString& vehicle_id, rclcpp::Node::SharedPtr node,
                      QWidget* parent = nullptr );
    ~ComputeComponent() override;

    /// One configurable monitored topic. Subscribed at "/<vehicle_id>/<suffix>",
    /// so the same spec applies to every vehicle.
    struct MonitoredTopicSpec
    {
        QString suffix;
        double expected_hz = 0.0;
    };

    /// Read the shared monitored-topic configuration (for save / restore).
    static const std::vector<MonitoredTopicSpec>& monitoredTopics() { return s_monitored_topics_; }
    static double thresholdFraction() { return s_threshold_fraction_; }

    /// Replaces the shared configuration and re-applies it to every live
    /// instance, so the change affects all vehicles.
    static void applyGlobalConfig( const std::vector<MonitoredTopicSpec>& specs,
                                   double threshold_fraction );

Q_SIGNALS:
    // Emitted when the overall health changes: healthy = no monitored topic is
    // below its threshold (i.e. no red row). The Setup table shows a success or
    // error icon from this.
    void healthChanged( bool healthy );

protected Q_SLOTS:
    // Turns the messages counted during the last window into per-topic rates and
    // refreshes the table (rate text + health coloring). Runs once per window.
    void updateRates();

    // Opens the configuration popup; on accept, applies the new configuration to
    // every vehicle's compute component.
    void onConfigureTopics();

protected:
    // A single monitored topic: how it is subscribed, the rate we expect of it,
    // and a rolling count of messages received during the current window.
    struct MonitoredTopic
    {
        QString label;            // shown in the "Topic" column
        std::string topic;        // fully-qualified topic name to subscribe
        double expected_hz = 0.0; // nominal publish rate
        rclcpp::SubscriptionBase::SharedPtr subscription; // generic; null until type is discovered
        bool subscribe_failed = false;          // true if the type could not be loaded
        std::atomic<std::uint64_t> count{ 0 };  // messages since last updateRates()
    };

    // Builds the table, the configure button and the layout (rows filled later).
    void buildUi();

    // Fills the table with one row per monitored topic.
    void populateTable();

    // Rebuilds this component's monitored topics from the shared configuration,
    // repopulates the table and (re-)subscribes.
    void applyConfig();

    // For each not-yet-subscribed topic, looks its type up in the ROS graph and,
    // once found, creates a generic (serialized) rate-counting subscription.
    void discoverAndSubscribe();

    // Applies (or clears) the below-threshold red highlight across a row.
    void setRowAlert( int row, bool alert );

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    std::vector<std::unique_ptr<MonitoredTopic>> topics_;

    // Last reported overall health and whether it has been reported yet, so
    // healthChanged() fires only on real transitions.
    bool healthy_ = true;
    bool health_known_ = false;

    // QT Widgets
    QTableWidget* table_ = nullptr;
    QPushButton* configure_button_ = nullptr;
    QTimer* measurement_timer_ = nullptr;

    // Shared (per-process) configuration and the live instances that mirror it,
    // so a change made on one vehicle applies to all of them.
    static std::vector<MonitoredTopicSpec> s_monitored_topics_;
    static double s_threshold_fraction_;
    static std::vector<ComputeComponent*> s_instances_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // COMPUTE_COMPONENT_H
