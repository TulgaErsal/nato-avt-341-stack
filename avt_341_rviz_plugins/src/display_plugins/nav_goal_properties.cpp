#include <avt_341_rviz_plugins/display_plugins/nav_goal_properties.h>

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

NavGoalProperties::NavGoalProperties( Property* parent, QObject* receiver,
                                      const char* changed_slot )
{
    // --- Pose arrow (mirrors the PoseStamped "Arrow" display) ---------------
    Property* arrow_group = new Property(
        "Pose Arrow", QVariant(), "Appearance of the goal-pose arrow.", parent );

    arrow_color_ = new ColorProperty(
        "Color", QColor( 255, 25, 0 ), "Color of the pose arrow.",
        arrow_group, changed_slot, receiver );
    arrow_alpha_ = new FloatProperty(
        "Alpha", 1.0f, "Opacity of the pose arrow.",
        arrow_group, changed_slot, receiver );
    arrow_alpha_->setMin( 0.0f );
    arrow_alpha_->setMax( 1.0f );
    shaft_length_ = new FloatProperty(
        "Shaft Length", 1.0f, "Length of the arrow shaft.",
        arrow_group, changed_slot, receiver );
    shaft_radius_ = new FloatProperty(
        "Shaft Radius", 0.05f, "Radius of the arrow shaft.",
        arrow_group, changed_slot, receiver );
    head_length_ = new FloatProperty(
        "Head Length", 0.3f, "Length of the arrow head.",
        arrow_group, changed_slot, receiver );
    head_radius_ = new FloatProperty(
        "Head Radius", 0.1f, "Radius of the arrow head.",
        arrow_group, changed_slot, receiver );

    // --- Threshold region ---------------------------------------------------
    Property* region_group = new Property(
        "Threshold Region", QVariant(),
        "Circle at the distance threshold; the filled wedge marks the poses "
        "(within the yaw threshold) where the goal counts as reached.",
        parent );

    circle_color_ = new ColorProperty(
        "Circle Color", QColor( 0, 170, 255 ),
        "Color of the distance-threshold circle outline.",
        region_group, changed_slot, receiver );
    circle_thickness_ = new FloatProperty(
        "Circle Thickness", 0.05f, "Line width of the circle outline (m).",
        region_group, changed_slot, receiver );
    circle_thickness_->setMin( 0.0f );

    fill_color_ = new ColorProperty(
        "Fill Color", QColor( 0, 204, 0 ), "Color of the filled viable region.",
        region_group, changed_slot, receiver );
    fill_alpha_ = new FloatProperty(
        "Fill Alpha", 0.3f, "Opacity of the filled viable region.",
        region_group, changed_slot, receiver );
    fill_alpha_->setMin( 0.0f );
    fill_alpha_->setMax( 1.0f );

    default_dist_threshold_ = new FloatProperty(
        "Default Distance Threshold", 1.0f,
        "Circle radius (m) drawn when a goal's dist_threshold is negative "
        "(i.e. requests the stack default).",
        region_group, changed_slot, receiver );
    default_dist_threshold_->setMin( 0.0f );
    default_yaw_threshold_ = new FloatProperty(
        "Default Yaw Threshold", 0.5f,
        "Half-wedge angle (rad) drawn when a goal's yaw_threshold is negative "
        "(i.e. requests the stack default).",
        region_group, changed_slot, receiver );
    default_yaw_threshold_->setMin( 0.0f );

    segments_ = new IntProperty(
        "Circle Segments", 64,
        "Number of segments used to tessellate the circle and wedge.",
        region_group, changed_slot, receiver );
    segments_->setMin( 3 );
}

NavGoalStyle NavGoalProperties::toStyle() const
{
    NavGoalStyle style;

    style.arrow_color = arrow_color_->getOgreColor();
    style.arrow_color.a = arrow_alpha_->getFloat();
    style.shaft_length = shaft_length_->getFloat();
    style.shaft_radius = shaft_radius_->getFloat();
    style.head_length = head_length_->getFloat();
    style.head_radius = head_radius_->getFloat();

    style.circle_color = circle_color_->getOgreColor();
    style.circle_color.a = 1.0f;
    style.circle_thickness = circle_thickness_->getFloat();

    style.fill_color = fill_color_->getOgreColor();
    style.fill_color.a = fill_alpha_->getFloat();

    style.default_dist_threshold = default_dist_threshold_->getFloat();
    style.default_yaw_threshold = default_yaw_threshold_->getFloat();
    style.segments = segments_->getInt();

    return style;
}

}
