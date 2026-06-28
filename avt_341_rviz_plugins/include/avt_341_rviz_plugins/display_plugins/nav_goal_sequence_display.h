#ifndef NAV_GOAL_SEQUENCE_DISPLAY_H
#define NAV_GOAL_SEQUENCE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <memory>
#include <vector>

#include <avt_341_msgs/msg/nav_goal_sequence.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <avt_341_rviz_plugins/display_plugins/nav_goal_properties.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

class NavGoalVisual;

/// RViz display for an avt_341_msgs/NavGoalSequence.
///
/// A NavGoalSequence is just an ordered list of NavGoals, so this reuses the
/// exact same rendering (NavGoalVisual) and property tree (NavGoalProperties) as
/// NavGoalDisplay, drawing one visual per goal. Each goal carries its own header
/// and is transformed independently.
class NavGoalSequenceDisplay
    : public rviz_common::MessageFilterDisplay<avt_341_msgs::msg::NavGoalSequence>
{
    Q_OBJECT

public:
    NavGoalSequenceDisplay();
    ~NavGoalSequenceDisplay() override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to every visual.
    void updateStyle();

private:
    void processMessage( avt_341_msgs::msg::NavGoalSequence::ConstSharedPtr msg ) override;

    std::vector<std::unique_ptr<NavGoalVisual>> visuals_;
    NavGoalProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_GOAL_SEQUENCE_DISPLAY_H
