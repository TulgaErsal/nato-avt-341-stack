#include <avt_341_rviz_plugins/display_plugins/map_marker_list_display.h>

#include <rviz_common/validate_floats.hpp>

namespace avt_341::rviz_plugins
{

MapMarkerListDisplay::MapMarkerListDisplay()
    : properties_( this, this, SLOT( updateStyle() ) )
{
}

MapMarkerListDisplay::~MapMarkerListDisplay() = default;

void MapMarkerListDisplay::updateStyle()
{
    applyStyleAndRender();
}

const std::vector<avt_341_msgs::msg::MapMarker>& MapMarkerListDisplay::items(
    const avt_341_msgs::msg::MapMarkerList& msg ) const
{
    return msg.markers;
}

bool MapMarkerListDisplay::validate( const avt_341_msgs::msg::MapMarker& item ) const
{
    return rviz_common::validateFloats( item.pose );
}

void MapMarkerListDisplay::setDomainFields( MapMarkerVisual& visual,
                                            const avt_341_msgs::msg::MapMarker& item )
{
    visual.setMarkerId( item.marker_id );
}

void MapMarkerListDisplay::applyStyle( MapMarkerVisual& visual )
{
    visual.setStyle( properties_.toStyle() );
}

const char* MapMarkerListDisplay::invalidText() const
{
    return "A marker contained invalid floating point values (nans or infs)";
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::MapMarkerListDisplay, rviz_common::Display )
