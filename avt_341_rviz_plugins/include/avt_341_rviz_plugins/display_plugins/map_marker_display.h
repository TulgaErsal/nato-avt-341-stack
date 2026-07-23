#ifndef MAP_MARKER_DISPLAY_H
#define MAP_MARKER_DISPLAY_H

#ifndef Q_MOC_RUN
#include <avt_341_msgs/msg/map_marker.hpp>

#include <avt_341_rviz_plugins/display_plugins/map_marker_properties.h>
#include <avt_341_rviz_plugins/display_plugins/visual_display_base.h>
#include <avt_341_rviz_plugins/primitives/map_marker_visual.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// RViz display for a single avt_341_msgs/MapMarker: a flat ground decal with a
/// thick ring, a heading arrow and the marker id in the centre (see
/// MapMarkerVisual). The shared display flow lives in SingleVisualDisplay; the
/// rendering and property tree are shared with MapMarkerListDisplay.
class MapMarkerDisplay
    : public SingleVisualDisplay<avt_341_msgs::msg::MapMarker, MapMarkerVisual>
{
    Q_OBJECT

public:
    MapMarkerDisplay();
    ~MapMarkerDisplay() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to the current visual.
    void updateStyle();

protected:
    bool validate( const avt_341_msgs::msg::MapMarker& msg ) const override;
    void setDomainFields( MapMarkerVisual& visual,
                          const avt_341_msgs::msg::MapMarker& msg ) override;
    void applyStyle( MapMarkerVisual& visual ) override;
    const char* invalidText() const override;

private:
    MapMarkerProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKER_DISPLAY_H
