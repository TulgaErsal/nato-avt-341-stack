#include <avt_341_rviz_plugins/display_plugins/map_marker_display.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/validate_floats.hpp>

#include <avt_341_rviz_plugins/primitives/map_marker_visual.h>

namespace avt_341::rviz_plugins
{

MapMarkerDisplay::MapMarkerDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

MapMarkerDisplay::~MapMarkerDisplay() = default;

void MapMarkerDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void MapMarkerDisplay::reset()
{
    MFDClass::reset();
    visual_.reset();
}

void MapMarkerDisplay::updateStyle()
{
    if ( visual_ )
    {
        visual_->setStyle( properties_.toStyle() );
        context_->queueRender();
    }
}

void MapMarkerDisplay::processMessage( avt_341_msgs::msg::MapMarker::ConstSharedPtr msg )
{
    if ( !rviz_common::validateFloats( msg->pose ) )
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
        visual_ = std::make_unique<MapMarkerVisual>( scene_manager_, scene_node_ );
    }
    visual_->setPose( position, orientation );
    visual_->setMarkerId( msg->marker_id );
    visual_->setStyle( properties_.toStyle() );

    context_->queueRender();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::MapMarkerDisplay, rviz_common::Display )
