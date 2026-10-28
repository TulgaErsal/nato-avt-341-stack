#ifndef COMPUTE_COMPONENT_H
#define COMPUTE_COMPONENT_H

#ifndef Q_MOC_RUN
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <QString>
#include <QWidget>

#include <avt_341_msgs/msg/compute_time.hpp>
#include <avt_341_msgs/msg/compute_time_array.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

class QPushButton;
class QTableWidget;
class QTabWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

namespace avt_341 {
namespace rviz_plugins {

/// Per-vehicle "Compute" tab content. Its Topics sub-tab reports the live
/// publish rate of monitored ROS topics against an expected rate. Its Code
/// Sections sub-tab reports hierarchical timing summaries published by all
/// nodes on the vehicle's shared compute-times topic. Rows turn red whenever a
/// measured value crosses its configured threshold.
///
/// The monitored topics, their expected rates and the alert threshold are edited
/// through a "Configure Monitored Topics" popup. By default ("All vehicles") a
/// change replaces the shared per-process configuration and re-applies to every
/// live component; with that option off it is recorded as a per-vehicle override
/// and applied to this vehicle's component only. The shared configuration and
/// the overrides are static so the panel can save and restore them.
///
/// Topic rates are measured type-agnostically: each topic's type is discovered
/// from the ROS graph at runtime and subscribed via a generic (serialized)
/// subscription, so messages are counted without being deserialized.
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

    /// A complete configuration: the monitored topics plus the alert threshold
    /// (a fraction of each topic's expected rate).
    struct MonitoredTopicConfig
    {
        std::vector<MonitoredTopicSpec> topics;
        double threshold_fraction = 0.0;
    };

    /// Read the shared monitored-topic configuration (for save / restore).
    static const std::vector<MonitoredTopicSpec>& monitoredTopics()
    {
        return s_shared_config_.topics;
    }
    static double thresholdFraction() { return s_shared_config_.threshold_fraction; }

    /// Read the per-vehicle overrides, keyed by vehicle id (for save / restore).
    /// A vehicle is listed only after a change was applied to it alone.
    static const std::map<QString, MonitoredTopicConfig>& vehicleOverrides()
    {
        return s_vehicle_overrides_;
    }

    /// Replaces the shared configuration, drops every per-vehicle override and
    /// re-applies it to every live instance, so the change affects all vehicles.
    static void applyGlobalConfig( const std::vector<MonitoredTopicSpec>& specs,
                                   double threshold_fraction );

    /// Replaces the set of per-vehicle overrides. Live instances of the listed
    /// vehicles take their override and all others return to the shared
    /// configuration; instances created later for a listed vehicle start from it.
    static void applyVehicleOverrides(
        const std::map<QString, MonitoredTopicConfig>& overrides );

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
    // every vehicle's compute component or, with "All vehicles" unchecked, to
    // this one only.
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

    // Latest values for one code section. Entries persist across callbacks so
    // messages from one publisher cannot erase sections owned by another.
    struct CodeSection
    {
        float time = 0.0f;
        float time_std = 0.0f;
        std::int32_t window_num_samples = -1;
        float window_time = -1.0f;
        float warning_threshold = -1.0f;
        bool auto_parent_stats = true;
    };

    // Builds the table, the configure button and the layout (rows filled later).
    void buildUi();

    // Fills the table with one row per monitored topic.
    void populateTable();

    // Rebuilds this component's monitored topics from its current configuration,
    // repopulates the table and (re-)subscribes.
    void applyConfig();

    // Records a configuration as this vehicle's override and applies it to this
    // component only; the shared configuration and the other vehicles are left
    // untouched.
    void applyLocalConfig( const MonitoredTopicConfig& config );

    // Stores a configuration on this component and re-applies it, without
    // touching the shared configuration or the override map.
    void setConfig( const MonitoredTopicConfig& config );

    // For each not-yet-subscribed topic, looks its type up in the ROS graph and,
    // once found, creates a generic (serialized) rate-counting subscription.
    void discoverAndSubscribe();

    // Applies (or clears) the below-threshold red highlight across a row.
    void setTopicRowAlert( int row, bool alert );

    // Merges a compute-time summary into the persistent source/section map and
    // updates the corresponding hierarchical tree rows.
    void updateCodeSections( const avt_341_msgs::msg::ComputeTimeArray& msg );

    // Returns (and lazily creates) the tree item for a slash-delimited section
    // path beneath its publisher tag.
    QTreeWidgetItem* ensureCodeSectionItem( const std::string& source,
                                            const std::string& section_id );

    // Writes the latest statistics and alert styling into one section row.
    void updateCodeSectionItem( QTreeWidgetItem* item, const CodeSection& section );

    // Emits healthChanged when the combined topic/code-section health changes.
    void updateOverallHealth();

    QString vehicle_id_;
    rclcpp::Node::SharedPtr node_;
    std::vector<std::unique_ptr<MonitoredTopic>> topics_;

    // This component's configuration: the shared one, or this vehicle's override
    // when one exists.
    MonitoredTopicConfig config_;

    rclcpp::Subscription<avt_341_msgs::msg::ComputeTimeArray>::SharedPtr
        compute_times_subscription_;

    // Publisher tag -> section id -> latest section values. The tag is needed
    // because many nodes publish independently on the same per-vehicle topic.
    std::map<std::string, std::map<std::string, CodeSection>> code_sections_;
    std::map<std::string, QTreeWidgetItem*> code_source_items_;
    std::map<std::pair<std::string, std::string>, QTreeWidgetItem*>
        code_section_items_;

    // Last reported overall health and whether it has been reported yet, so
    // healthChanged() fires only on real transitions.
    bool healthy_ = true;
    bool health_known_ = false;
    bool topic_alert_ = false;
    bool code_section_alert_ = false;

    // QT Widgets
    QTabWidget* sub_tabs_ = nullptr;
    QTableWidget* topics_table_ = nullptr;
    QTreeWidget* code_sections_tree_ = nullptr;
    QPushButton* configure_button_ = nullptr;
    QTimer* measurement_timer_ = nullptr;

    // Per-process state the panel saves and restores: the shared configuration,
    // the per-vehicle overrides of it (keyed by vehicle id), and the live
    // instances a change is pushed to.
    static MonitoredTopicConfig s_shared_config_;
    static std::map<QString, MonitoredTopicConfig> s_vehicle_overrides_;
    static std::vector<ComputeComponent*> s_instances_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // COMPUTE_COMPONENT_H
