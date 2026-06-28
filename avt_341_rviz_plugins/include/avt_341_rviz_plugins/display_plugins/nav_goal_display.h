#ifndef NAV_GOAL_DISPLAY_H
#define NAV_GOAL_DISPLAY_H

#ifndef Q_MOC_RUN
#include <memory>

#include <avt_341_msgs/msg/nav_goal.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <avt_341_rviz_plugins/display_plugins/nav_goal_properties.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

class NavGoalVisual;

/// RViz display for a single avt_341_msgs/NavGoal.
///
/// Draws the goal pose as a PoseStamped-style arrow and, around it, the
/// distance/yaw threshold region (see NavGoalVisual). The visual rendering and
/// the property tree are both shared with NavGoalSequenceDisplay.
class NavGoalDisplay : public rviz_common::MessageFilterDisplay<avt_341_msgs::msg::NavGoal>
{
    Q_OBJECT

public:
    NavGoalDisplay();
    ~NavGoalDisplay() override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to the current visual.
    void updateStyle();

private:
    void processMessage( avt_341_msgs::msg::NavGoal::ConstSharedPtr msg ) override;

    std::unique_ptr<NavGoalVisual> visual_;
    NavGoalProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_GOAL_DISPLAY_H
