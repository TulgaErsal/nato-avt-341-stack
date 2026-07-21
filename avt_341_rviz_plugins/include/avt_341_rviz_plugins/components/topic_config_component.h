#ifndef TOPIC_CONFIG_COMPONENT_H
#define TOPIC_CONFIG_COMPONENT_H

#ifndef Q_MOC_RUN
#include <vector>

#include <QWidget>

#include <avt_341_rviz_plugins/components/topic_config.h>
#endif

class QLineEdit;

namespace avt_341 {
namespace rviz_plugins {

/// Setup-tab section that lists the configurable topic names as "<key>: <value>"
/// rows. Each value sits in a read-only field with an edit icon button to its
/// right; clicking the button opens a popup to change the value. After a change
/// it emits topicChanged() with the affected group so the panel can re-create
/// just the components that consume that topic.
class TopicConfigComponent: public QWidget
{

Q_OBJECT
public:
    TopicConfigComponent( QWidget* parent = nullptr );

    /// The current topic configuration.
    const TopicConfig& config() const { return config_; }

    /// Replace the topic configuration (e.g. when restoring saved panel state)
    /// and refresh the displayed value fields. Does not emit topicChanged().
    void setConfig( const TopicConfig& config );

Q_SIGNALS:
    /// Emitted after the user changes a topic; \p group identifies which tab's
    /// components consume it.
    void topicChanged( avt_341::rviz_plugins::TopicGroup group );

private:
    // Opens the edit popup for the descriptor at \p index and, on an accepted
    // change, updates the field and emits topicChanged().
    void onEditTopic( int index );

    TopicConfig config_;
    std::vector<QLineEdit*> value_fields_;   // one per descriptor, same order

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // TOPIC_CONFIG_COMPONENT_H
