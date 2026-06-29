#ifndef NAV_GOAL_SEQUENCE_DISPLAY_H
#define NAV_GOAL_SEQUENCE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <vector>

#include <avt_341_msgs/msg/nav_goal.hpp>
#include <avt_341_msgs/msg/nav_goal_sequence.hpp>

#include <avt_341_rviz_plugins/display_plugins/nav_goal_properties.h>
#include <avt_341_rviz_plugins/display_plugins/visual_display_base.h>
#include <avt_341_rviz_plugins/primitives/nav_goal_visual.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// RViz display for an avt_341_msgs/NavGoalSequence: one NavGoalVisual per goal.
/// The shared per-item flow lives in VisualArrayDisplay; the rendering and
/// property tree are shared with NavGoalDisplay.
class NavGoalSequenceDisplay
    : public VisualArrayDisplay<avt_341_msgs::msg::NavGoalSequence,
                                avt_341_msgs::msg::NavGoal, NavGoalVisual>
{
    Q_OBJECT

public:
    NavGoalSequenceDisplay();
    ~NavGoalSequenceDisplay() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to every visual.
    void updateStyle();

protected:
    const std::vector<avt_341_msgs::msg::NavGoal>& items(
        const avt_341_msgs::msg::NavGoalSequence& msg ) const override;
    bool validate( const avt_341_msgs::msg::NavGoal& item ) const override;
    void setDomainFields( NavGoalVisual& visual,
                          const avt_341_msgs::msg::NavGoal& item ) override;
    void applyStyle( NavGoalVisual& visual ) override;
    const char* invalidText() const override;

private:
    NavGoalProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_GOAL_SEQUENCE_DISPLAY_H
