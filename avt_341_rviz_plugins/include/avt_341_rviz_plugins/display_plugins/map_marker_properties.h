#ifndef MAP_MARKER_PROPERTIES_H
#define MAP_MARKER_PROPERTIES_H

#ifndef Q_MOC_RUN
#include <avt_341_rviz_plugins/primitives/map_marker_visual.h>
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

/// Builds and owns the RViz property tree shared by the MapMarker and
/// MapMarkerList displays, and snapshots the current values into a
/// MapMarkerStyle.
///
/// Like NavGoalProperties it is intentionally not a QObject: the properties are
/// parented to (and their change signals routed to a slot on) the owning
/// display, so both displays share one definition of a marker's knobs while the
/// display remains the single place that reacts to edits.
class MapMarkerProperties
{
public:
    /// \param parent        The display (a Property) the knobs are added under.
    /// \param receiver      The object whose \p changed_slot fires on any edit.
    /// \param changed_slot  A SLOT(...) string invoked when any property changes.
    MapMarkerProperties( rviz_common::properties::Property* parent,
                         QObject* receiver, const char* changed_slot );

    /// Snapshot the current property values as a render-ready style.
    MapMarkerStyle toStyle() const;

private:
    rviz_common::properties::ColorProperty* color_;
    rviz_common::properties::FloatProperty* alpha_;
    rviz_common::properties::FloatProperty* radius_;
    rviz_common::properties::FloatProperty* circle_thickness_;
    rviz_common::properties::FloatProperty* text_scale_;
    rviz_common::properties::IntProperty* segments_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKER_PROPERTIES_H
