#ifndef MAP_MARKER_LIST_DISPLAY_H
#define MAP_MARKER_LIST_DISPLAY_H

#ifndef Q_MOC_RUN
#include <memory>
#include <vector>

#include <avt_341_msgs/msg/map_marker_list.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <avt_341_rviz_plugins/display_plugins/map_marker_properties.h>
#endif

namespace avt_341 {
namespace rviz_plugins {

class MapMarkerVisual;

/// RViz display for an avt_341_msgs/MapMarkerList.
///
/// A MapMarkerList is just a list of MapMarkers, so this reuses the same
/// rendering (MapMarkerVisual) and property tree (MapMarkerProperties) as
/// MapMarkerDisplay, drawing one visual per marker. Each marker carries its own
/// header and is transformed independently.
class MapMarkerListDisplay
    : public rviz_common::MessageFilterDisplay<avt_341_msgs::msg::MapMarkerList>
{
    Q_OBJECT

public:
    MapMarkerListDisplay();
    ~MapMarkerListDisplay() override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    /// Re-apply the (possibly edited) style to every visual.
    void updateStyle();

private:
    void processMessage( avt_341_msgs::msg::MapMarkerList::ConstSharedPtr msg ) override;

    std::vector<std::unique_ptr<MapMarkerVisual>> visuals_;
    MapMarkerProperties properties_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_MARKER_LIST_DISPLAY_H
