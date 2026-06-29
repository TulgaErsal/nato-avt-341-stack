#ifndef NAV_GOAL_DISPLAY_H
#define NAV_GOAL_DISPLAY_H

#ifndef Q_MOC_RUN
#include <avt_341_msgs/msg/nav_goal.hpp>

#include <avt_341_rviz_plugins/display_plugins/nav_goal_properties.h>
#include <avt_341_rviz_plugins/display_plugins/visual_display_base.h>
#include <avt_341_rviz_plugins/primitives/nav_goal_visual.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// RViz display for a single avt_341_msgs/NavGoal: a PoseStamped-style arrow plus
/// the distance/yaw threshold region (see NavGoalVisual). The shared display flow
/// lives in SingleVisualDisplay; the rendering and property tree are shared with
/// NavGoalSequenceDisplay.
class NavGoalDisplay
    : public SingleVisualDisplay<avt_341_msgs::msg::NavGoal, NavGoalVisual>
{
    Q_OBJECT

public:
    NavGoalDisplay();
    ~NavGoalDisplay() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to the current visual.
    void updateStyle();

protected:
    bool validate( const avt_341_msgs::msg::NavGoal& msg ) const override;
    void setDomainFields( NavGoalVisual& visual,
                          const avt_341_msgs::msg::NavGoal& msg ) override;
    void applyStyle( NavGoalVisual& visual ) override;
    const char* invalidText() const override;

private:
    NavGoalProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_GOAL_DISPLAY_H
