#ifndef MAP_MARKER_LIST_DISPLAY_H
#define MAP_MARKER_LIST_DISPLAY_H

#ifndef Q_MOC_RUN
#include <vector>

#include <avt_341_msgs/msg/map_marker.hpp>
#include <avt_341_msgs/msg/map_marker_list.hpp>

#include <avt_341_rviz_plugins/display_plugins/map_marker_properties.h>
#include <avt_341_rviz_plugins/display_plugins/visual_display_base.h>
#include <avt_341_rviz_plugins/primitives/map_marker_visual.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// RViz display for an avt_341_msgs/MapMarkerList: one MapMarkerVisual per
/// marker. The shared per-item flow lives in VisualArrayDisplay; the rendering
/// and property tree are shared with MapMarkerDisplay.
class MapMarkerListDisplay
    : public VisualArrayDisplay<avt_341_msgs::msg::MapMarkerList,
                                avt_341_msgs::msg::MapMarker, MapMarkerVisual>
{
    Q_OBJECT

public:
    MapMarkerListDisplay();
    ~MapMarkerListDisplay() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to every visual.
    void updateStyle();

protected:
    const std::vector<avt_341_msgs::msg::MapMarker>& items(
        const avt_341_msgs::msg::MapMarkerList& msg ) const override;
    bool validate( const avt_341_msgs::msg::MapMarker& item ) const override;
    void setDomainFields( MapMarkerVisual& visual,
                          const avt_341_msgs::msg::MapMarker& item ) override;
    void applyStyle( MapMarkerVisual& visual ) override;
    const char* invalidText() const override;

private:
    MapMarkerProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKER_LIST_DISPLAY_H
