/**
* @file      geometry_dto.hpp
* @brief     Polygon zone value types shared between file parsing and consumers.
*/

#ifndef AVT_341_CORE_GEOMETRY_DTO_HPP
#define AVT_341_CORE_GEOMETRY_DTO_HPP

#include <string>
#include <vector>

#include <Eigen/Dense>
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace avt_341_nav::core
{

    struct PolygonZone
    {
        std::string label;
        std::vector<Eigen::Vector2d> vertices;
        double occ_value = 0.0;
        double max_speed = 0.0;
        int seg_value = -1;
    };

    struct PolygonZoneCollection
    {
        std::string frame;
        std::vector<PolygonZone> zones;
    };

    /**
     * @brief Builds outline and centroid-label markers for the zones. Label
     * markers use "<marker_namespace>_labels" as their namespace.
     */
    inline visualization_msgs::msg::MarkerArray CreateVisualization(
        const PolygonZoneCollection& collection,
        const builtin_interfaces::msg::Time& stamp,
        const std::string& frame_id,
        const std::string& marker_namespace,
        const float label_size = 20.0f)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        for (const PolygonZone& zone : collection.zones) {
            // Polygon outline.
            visualization_msgs::msg::Marker outline;
            outline.header.frame_id = frame_id;
            outline.header.stamp    = stamp;
            outline.ns              = marker_namespace;
            outline.id              = marker_id++;
            outline.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            outline.action          = visualization_msgs::msg::Marker::ADD;
            outline.scale.x         = 0.3; // line width in metres
            outline.color.r         = 1.0f;
            outline.color.g         = 0.0f;
            outline.color.b         = 0.0f;
            outline.color.a         = 1.0f;

            for (const auto& v : zone.vertices) {
                geometry_msgs::msg::Point p;
                p.x = v.x();
                p.y = v.y();
                p.z = 0.0;
                outline.points.push_back(p);
            }
            // Close the ring.
            if (!zone.vertices.empty()) {
                geometry_msgs::msg::Point p;
                p.x = zone.vertices.front().x();
                p.y = zone.vertices.front().y();
                p.z = 0.0;
                outline.points.push_back(p);
            }

            marker_array.markers.push_back(outline);

            // Text label at the polygon centroid.
            double cx = 0.0, cy = 0.0;
            for (const auto& v : zone.vertices) { cx += v.x(); cy += v.y(); }
            cx /= static_cast<double>(zone.vertices.size());
            cy /= static_cast<double>(zone.vertices.size());

            visualization_msgs::msg::Marker label;
            label.header          = outline.header;
            label.ns              = marker_namespace + "_labels";
            label.id              = marker_id++;
            label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action          = visualization_msgs::msg::Marker::ADD;
            label.scale.z         = label_size; // text height in metres
            label.color.r         = 1.0f;
            label.color.g         = 1.0f;
            label.color.b         = 0.0f;
            label.color.a         = 1.0f;
            label.pose.position.x = cx;
            label.pose.position.y = cy;
            label.pose.position.z = 1.0;
            label.text            = zone.label;

            marker_array.markers.push_back(label);
        }

        return marker_array;
    }

}

#endif // AVT_341_CORE_GEOMETRY_DTO_HPP
