#include <avt_341_rviz_plugins/display_plugins/nav_goal_display.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/validate_floats.hpp>

#include <avt_341_rviz_plugins/primitives/nav_goal_visual.h>

namespace avt_341::rviz_plugins
{

NavGoalDisplay::NavGoalDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

NavGoalDisplay::~NavGoalDisplay() = default;

void NavGoalDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void NavGoalDisplay::reset()
{
    MFDClass::reset();
    visual_.reset();
}

void NavGoalDisplay::updateStyle()
{
    if ( visual_ )
    {
        visual_->setStyle( properties_.toStyle() );
        context_->queueRender();
    }
}

void NavGoalDisplay::processMessage( avt_341_msgs::msg::NavGoal::ConstSharedPtr msg )
{
    if ( !rviz_common::validateFloats( msg->pose ) ||
         !rviz_common::validateFloats( msg->dist_threshold ) ||
         !rviz_common::validateFloats( msg->yaw_threshold ) )
    {
        setStatus( rviz_common::properties::StatusProperty::Error, "Topic",
                   "Message contained invalid floating point values (nans or infs)" );
        return;
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if ( !context_->getFrameManager()->transform( msg->header, msg->pose, position, orientation ) )
    {
        setMissingTransformToFixedFrame( msg->header.frame_id );
        return;
    }
    setTransformOk();

    if ( !visual_ )
    {
        visual_ = std::make_unique<NavGoalVisual>( scene_manager_, scene_node_ );
    }
    visual_->setPose( position, orientation );
    visual_->setThresholds( msg->dist_threshold, msg->yaw_threshold );
    visual_->setStyle( properties_.toStyle() );

    context_->queueRender();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::NavGoalDisplay, rviz_common::Display )
