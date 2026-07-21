#include <avt_341_rviz_plugins/display_plugins/nav_goal_sequence_display.h>

#include <rviz_common/validate_floats.hpp>

namespace avt_341::rviz_plugins
{

NavGoalSequenceDisplay::NavGoalSequenceDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

NavGoalSequenceDisplay::~NavGoalSequenceDisplay() = default;

void NavGoalSequenceDisplay::updateStyle()
{
    applyStyleAndRender();
}

const std::vector<avt_341_msgs::msg::NavGoal>& NavGoalSequenceDisplay::items(
    const avt_341_msgs::msg::NavGoalSequence& msg ) const
{
    return msg.goals;
}

bool NavGoalSequenceDisplay::validate( const avt_341_msgs::msg::NavGoal& item ) const
{
    return rviz_common::validateFloats( item.pose ) &&
           rviz_common::validateFloats( item.dist_threshold ) &&
           rviz_common::validateFloats( item.yaw_threshold );
}

void NavGoalSequenceDisplay::setDomainFields( NavGoalVisual& visual,
                                              const avt_341_msgs::msg::NavGoal& item )
{
    visual.setThresholds( item.dist_threshold, item.yaw_threshold );
}

void NavGoalSequenceDisplay::applyStyle( NavGoalVisual& visual )
{
    visual.setStyle( properties_.toStyle() );
}

const char* NavGoalSequenceDisplay::invalidText() const
{
    return "A goal contained invalid floating point values (nans or infs)";
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::NavGoalSequenceDisplay, rviz_common::Display )
