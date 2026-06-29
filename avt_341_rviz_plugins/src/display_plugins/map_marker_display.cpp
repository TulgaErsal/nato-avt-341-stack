#include <avt_341_rviz_plugins/display_plugins/map_marker_display.h>

#include <rviz_common/validate_floats.hpp>

namespace avt_341::rviz_plugins
{

MapMarkerDisplay::MapMarkerDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

MapMarkerDisplay::~MapMarkerDisplay() = default;

void MapMarkerDisplay::updateStyle()
{
    applyStyleAndRender();
}

bool MapMarkerDisplay::validate( const avt_341_msgs::msg::MapMarker& msg ) const
{
    return rviz_common::validateFloats( msg.pose );
}

void MapMarkerDisplay::setDomainFields( MapMarkerVisual& visual,
                                        const avt_341_msgs::msg::MapMarker& msg )
{
    visual.setMarkerId( msg.marker_id );
}

void MapMarkerDisplay::applyStyle( MapMarkerVisual& visual )
{
    visual.setStyle( properties_.toStyle() );
}

const char* MapMarkerDisplay::invalidText() const
{
    return "Message contained invalid floating point values (nans or infs)";
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::MapMarkerDisplay, rviz_common::Display )
