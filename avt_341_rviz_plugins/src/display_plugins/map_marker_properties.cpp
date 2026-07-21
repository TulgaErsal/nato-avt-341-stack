#include <avt_341_rviz_plugins/display_plugins/map_marker_properties.h>

#include <QColor>
#include <QVariant>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::Property;

MapMarkerProperties::MapMarkerProperties( Property* parent, QObject* receiver,
                                          const char* changed_slot )
{
    color_ = new ColorProperty(
        "Color", QColor( 255, 255, 255 ),
        "Color of the ring, heading arrow and id text.",
        parent, changed_slot, receiver );
    alpha_ = new FloatProperty(
        "Alpha", 1.0f, "Opacity of the marker.",
        parent, changed_slot, receiver );
    alpha_->setMin( 0.0f );
    alpha_->setMax( 1.0f );

    radius_ = new FloatProperty(
        "Radius", 1.0f, "Radius of the marker ring (m).",
        parent, changed_slot, receiver );
    radius_->setMin( 0.0f );

    circle_thickness_ = new FloatProperty(
        "Circle Thickness", 0.15f, "Band width of the ring (m).",
        parent, changed_slot, receiver );
    circle_thickness_->setMin( 0.0f );

    text_scale_ = new FloatProperty(
        "Text Scale", 1.0f,
        "Multiplier on the id text size (which is otherwise sized to the radius).",
        parent, changed_slot, receiver );
    text_scale_->setMin( 0.0f );

    segments_ = new IntProperty(
        "Circle Segments", 64, "Number of segments used to tessellate the ring.",
        parent, changed_slot, receiver );
    segments_->setMin( 8 );
}

MapMarkerStyle MapMarkerProperties::toStyle() const
{
    MapMarkerStyle style;
    style.color = color_->getOgreColor();
    style.color.a = alpha_->getFloat();
    style.radius = radius_->getFloat();
    style.circle_thickness = circle_thickness_->getFloat();
    style.text_scale = text_scale_->getFloat();
    style.segments = segments_->getInt();
    return style;
}

}
