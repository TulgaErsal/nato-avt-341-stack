
#ifndef AVT_341_RVIZ_SPLINE_PLOTTER_H
#define AVT_341_RVIZ_SPLINE_PLOTTER_H

#include <memory>
#include <string>
#include <vector>

#include "avt_341_nav/core/math_dto.hpp"
#include "avt_341_nav/planning/local/candidate.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <rclcpp/rclcpp.hpp>

namespace avt_341_nav {
  namespace planning{

    class RVIZPlotter {
    public:
      RVIZPlotter(const std::string & cost_vis,
                  rclcpp::Node::SharedPtr node, float w_c, float w_s, float w_r, float w_d, float w_t, float cost_vis_text_size);

      /**
       * Set the centerline to be plotted.
       * \param path List of points representing the centerline to be plotted.
       */
      void SetPath(std::vector<core::vec2> path);

      /**
       * Add the candidate paths to be plotted.
       * \param curves A list of candidate paths to be plotted.
       */
      void AddCurves(std::vector<Candidate> curves);

      /**
       * Add the occupancy grid that will be plotted
       * \param grid The occupancy grid to be plotted.
       */
      void AddMap(const nav_msgs::msg::OccupancyGrid & grid);

      /**
       * Publish the candidate path markers for rviz.
       */
      void Display();

    private:
      visualization_msgs::msg::Marker get_marker_msg(int type, int id, bool is_blocked = false) const;

      std::vector<core::vec2> path_;
      std::vector<Candidate> curves_;
      float pixdim_;
      bool map_set_;

      rclcpp::Node::SharedPtr node_;
      std::string cost_vis_;
      float cost_vis_text_size_;
      std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> candidate_paths_publisher;
      float w_c_;
      float w_d_;
      float w_r_;
      float w_s_;
      float w_t_;
    };
  } // namespace planning
} // namespace avt_341_nav

#endif //AVT_341_RVIZ_SPLINE_PLOTTER_H
