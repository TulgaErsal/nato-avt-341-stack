#include <avt_341_rviz_plugins/display_plugins/map_marker_list_display.h>

#include <cstddef>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/validate_floats.hpp>

#include <avt_341_rviz_plugins/primitives/map_marker_visual.h>

namespace avt_341::rviz_plugins
{

MapMarkerListDisplay::MapMarkerListDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

MapMarkerListDisplay::~MapMarkerListDisplay() = default;

void MapMarkerListDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void MapMarkerListDisplay::reset()
{
    MFDClass::reset();
    visuals_.clear();
}

void MapMarkerListDisplay::updateStyle()
{
    const MapMarkerStyle style = properties_.toStyle();
    for ( const std::unique_ptr<MapMarkerVisual>& visual : visuals_ )
    {
        if ( visual )
        {
            visual->setStyle( style );
        }
    }
    context_->queueRender();
}

void MapMarkerListDisplay::processMessage( avt_341_msgs::msg::MapMarkerList::ConstSharedPtr msg )
{
    const MapMarkerStyle style = properties_.toStyle();

    // Keep the pool of visuals sized to the number of markers so indices line up
    // across messages (a shorter list drops the trailing visuals).
    if ( visuals_.size() > msg->markers.size() )
    {
        visuals_.resize( msg->markers.size() );
    }

    bool transform_failed = false;
    for ( std::size_t i = 0; i < msg->markers.size(); ++i )
    {
        const avt_341_msgs::msg::MapMarker& marker = msg->markers[i];

        if ( !rviz_common::validateFloats( marker.pose ) )
        {
            setStatus( rviz_common::properties::StatusProperty::Error, "Topic",
                       "A marker contained invalid floating point values (nans or infs)" );
            continue;
        }

        // Each marker has its own header/frame, so transform them individually
        // rather than assuming the list's frame.
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if ( !context_->getFrameManager()->transform(
                 marker.header, marker.pose, position, orientation ) )
        {
            transform_failed = true;
            continue;
        }

        if ( i >= visuals_.size() )
        {
            visuals_.push_back( std::make_unique<MapMarkerVisual>( scene_manager_, scene_node_ ) );
        }
        else if ( !visuals_[i] )
        {
            visuals_[i] = std::make_unique<MapMarkerVisual>( scene_manager_, scene_node_ );
        }

        visuals_[i]->setPose( position, orientation );
        visuals_[i]->setMarkerId( marker.marker_id );
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
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::MapMarkerListDisplay, rviz_common::Display )
