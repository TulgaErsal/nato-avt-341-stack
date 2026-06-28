#include <avt_341_rviz_plugins/display_plugins/nav_goal_sequence_display.h>

#include <cstddef>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/validate_floats.hpp>

#include <avt_341_rviz_plugins/primitives/nav_goal_visual.h>

namespace avt_341::rviz_plugins
{

NavGoalSequenceDisplay::NavGoalSequenceDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

NavGoalSequenceDisplay::~NavGoalSequenceDisplay() = default;

void NavGoalSequenceDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void NavGoalSequenceDisplay::reset()
{
    MFDClass::reset();
    visuals_.clear();
}

void NavGoalSequenceDisplay::updateStyle()
{
    const NavGoalStyle style = properties_.toStyle();
    for ( const std::unique_ptr<NavGoalVisual>& visual : visuals_ )
    {
        if ( visual )
        {
            visual->setStyle( style );
        }
    }
    context_->queueRender();
}

void NavGoalSequenceDisplay::processMessage(
    avt_341_msgs::msg::NavGoalSequence::ConstSharedPtr msg )
{
    const NavGoalStyle style = properties_.toStyle();

    // Keep the pool of visuals sized to the number of goals so indices line up
    // across messages (a shorter sequence drops the trailing visuals).
    if ( visuals_.size() > msg->goals.size() )
    {
        visuals_.resize( msg->goals.size() );
    }

    bool transform_failed = false;
    for ( std::size_t i = 0; i < msg->goals.size(); ++i )
    {
        const avt_341_msgs::msg::NavGoal& goal = msg->goals[i];

        if ( !rviz_common::validateFloats( goal.pose ) ||
             !rviz_common::validateFloats( goal.dist_threshold ) ||
             !rviz_common::validateFloats( goal.yaw_threshold ) )
        {
            setStatus( rviz_common::properties::StatusProperty::Error, "Topic",
                       "A goal contained invalid floating point values (nans or infs)" );
            continue;
        }

        // Each goal has its own header/frame, so transform them individually
        // rather than assuming the sequence's frame.
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if ( !context_->getFrameManager()->transform(
                 goal.header, goal.pose, position, orientation ) )
        {
            transform_failed = true;
            continue;
        }

        if ( i >= visuals_.size() )
        {
            visuals_.push_back( std::make_unique<NavGoalVisual>( scene_manager_, scene_node_ ) );
        }
        else if ( !visuals_[i] )
        {
            visuals_[i] = std::make_unique<NavGoalVisual>( scene_manager_, scene_node_ );
        }

        visuals_[i]->setPose( position, orientation );
        visuals_[i]->setThresholds( goal.dist_threshold, goal.yaw_threshold );
        visuals_[i]->setStyle( style );
    }

    if ( transform_failed )
    {
        setMissingTransformToFixedFrame( msg->header.frame_id );
    }
    else
    {
        setTransformOk();
    }

    context_->queueRender();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::NavGoalSequenceDisplay, rviz_common::Display )
