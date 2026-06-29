#include <avt_341_rviz_plugins/display_plugins/nav_goal_display.h>

#include <rviz_common/validate_floats.hpp>

namespace avt_341::rviz_plugins
{

NavGoalDisplay::NavGoalDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

NavGoalDisplay::~NavGoalDisplay() = default;

void NavGoalDisplay::updateStyle()
{
    applyStyleAndRender();
}

bool NavGoalDisplay::validate( const avt_341_msgs::msg::NavGoal& msg ) const
{
    return rviz_common::validateFloats( msg.pose ) &&
           rviz_common::validateFloats( msg.dist_threshold ) &&
           rviz_common::validateFloats( msg.yaw_threshold );
}

void NavGoalDisplay::setDomainFields( NavGoalVisual& visual,
                                      const avt_341_msgs::msg::NavGoal& msg )
{
    visual.setThresholds( msg.dist_threshold, msg.yaw_threshold );
}

void NavGoalDisplay::applyStyle( NavGoalVisual& visual )
{
    visual.setStyle( properties_.toStyle() );
}

const char* NavGoalDisplay::invalidText() const
{
    return "Message contained invalid floating point values (nans or infs)";
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::NavGoalDisplay, rviz_common::Display )
