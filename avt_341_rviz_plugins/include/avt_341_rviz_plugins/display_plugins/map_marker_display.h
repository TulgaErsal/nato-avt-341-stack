#ifndef MAP_MARKER_DISPLAY_H
#define MAP_MARKER_DISPLAY_H

#ifndef Q_MOC_RUN
#include <memory>

#include <avt_341_msgs/msg/map_marker.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <avt_341_rviz_plugins/display_plugins/map_marker_properties.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

class MapMarkerVisual;

/// RViz display for a single avt_341_msgs/MapMarker: a flat ground decal with a
/// thick ring, a heading arrow and the marker id in the centre (see
/// MapMarkerVisual). The rendering and property tree are shared with
/// MapMarkerListDisplay.
class MapMarkerDisplay : public rviz_common::MessageFilterDisplay<avt_341_msgs::msg::MapMarker>
{
    Q_OBJECT

public:
    MapMarkerDisplay();
    ~MapMarkerDisplay() override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to the current visual.
    void updateStyle();

private:
    void processMessage( avt_341_msgs::msg::MapMarker::ConstSharedPtr msg ) override;

    std::unique_ptr<MapMarkerVisual> visual_;
    MapMarkerProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKER_DISPLAY_H
