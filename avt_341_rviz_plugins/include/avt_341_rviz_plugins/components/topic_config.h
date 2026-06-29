#ifndef TOPIC_CONFIG_H
#define TOPIC_CONFIG_H

#include <string>
#include <vector>

#include <QString>

namespace avt_341 {
namespace rviz_plugins {

/// Which per-vehicle tab a topic feeds, so the panel knows which components to
/// re-create when that topic changes.
enum class TopicGroup
{
    NavState,
    Mission,
    Tracker
};

/// The per-vehicle topic suffixes the Setup tab lets the user override. Each is
/// subscribed at "/<vehicle_id>/<suffix>". The defaults below are exactly what
/// the components used to hard-code.
struct TopicConfig
{
    QString odometry      = "avt_341/odometry";
    QString nav_state     = "avt_341/state";
    QString cmd_vel       = "avt_341/cmd_vel";
    QString desired_speed = "avt_341/desired_speed";
    QString task_status   = "avt_341/task_status";
    QString tracker_state = "avt_341/tracker/state";
};

/// Builds the fully-qualified per-vehicle topic name "/<vehicle_id>/<suffix>".
inline std::string makeTopicPath( const QString& vehicle_id, const QString& suffix )
{
    return ( "/" + vehicle_id + "/" + suffix ).toStdString();
}

/// Describes one editable topic: the tab that consumes it, the label shown in
/// the Setup table, and the TopicConfig member it reads / writes.
struct TopicDescriptor
{
    TopicGroup group;
    const char* key;
    QString TopicConfig::* member;
};

/// The editable topics, in display order. Single source of truth shared by the
/// Setup UI (to build the rows) and the panel (to map an edit back to a group).
inline const std::vector<TopicDescriptor>& topicDescriptors()
{
    static const std::vector<TopicDescriptor> descriptors = {
        { TopicGroup::NavState, "Odometry",         &TopicConfig::odometry },
        { TopicGroup::NavState, "Nav State",        &TopicConfig::nav_state },
        { TopicGroup::NavState, "Command Velocity", &TopicConfig::cmd_vel },
        { TopicGroup::NavState, "Desired Speed",    &TopicConfig::desired_speed },
        { TopicGroup::Mission,  "Task Status",      &TopicConfig::task_status },
        { TopicGroup::Tracker,  "Tracker State",    &TopicConfig::tracker_state },
    };
    return descriptors;
}

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // TOPIC_CONFIG_H
