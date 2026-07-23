#ifndef NAV_GOAL_PROPERTIES_H
#define NAV_GOAL_PROPERTIES_H

#ifndef Q_MOC_RUN
#include <avt_341_rviz_plugins/primitives/nav_goal_visual.h>
#endif

class QObject;

namespace rviz_common {
namespace properties {
class Property;
class ColorProperty;
class FloatProperty;
class IntProperty;
}
}

namespace avt_341 {
namespace rviz_plugins {

/// Builds and owns the RViz property tree shared by the NavGoal and
/// NavGoalSequence displays, and snapshots the current values into a
/// NavGoalStyle.
///
/// It is deliberately *not* a QObject. The properties are parented to (and their
/// change signals routed to a slot on) the owning display, so the display stays
/// the single place that reacts to edits, while both displays share one
/// definition of "what knobs a NavGoal has". MessageFilterDisplay is a class
/// template, which moc cannot give a Q_OBJECT, so a free-standing helper like
/// this is the natural way to share the slot-backed properties.
class NavGoalProperties
{
public:
    /// \param parent        The display (a Property) the knobs are added under.
    /// \param receiver      The object whose \p changed_slot fires on any edit.
    /// \param changed_slot  A SLOT(...) string invoked when any property changes.
    NavGoalProperties( rviz_common::properties::Property* parent,
                       QObject* receiver, const char* changed_slot );

    /// Snapshot the current property values as a render-ready style.
    NavGoalStyle toStyle() const;

private:
    rviz_common::properties::ColorProperty* arrow_color_;
    rviz_common::properties::FloatProperty* arrow_alpha_;
    rviz_common::properties::FloatProperty* shaft_length_;
    rviz_common::properties::FloatProperty* shaft_radius_;
    rviz_common::properties::FloatProperty* head_length_;
    rviz_common::properties::FloatProperty* head_radius_;

    rviz_common::properties::ColorProperty* circle_color_;
    rviz_common::properties::FloatProperty* circle_thickness_;

    rviz_common::properties::ColorProperty* fill_color_;
    rviz_common::properties::FloatProperty* fill_alpha_;

    rviz_common::properties::IntProperty* segments_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // NAV_GOAL_PROPERTIES_H
