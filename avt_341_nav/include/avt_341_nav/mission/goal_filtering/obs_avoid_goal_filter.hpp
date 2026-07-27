#ifndef OBS_AVOIDANCE_GOAL_FILTER_HPP
#define OBS_AVOIDANCE_GOAL_FILTER_HPP

#include <Eigen/Dense>
#include <optional>
#include <string>

#include "goal_filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341/node/occupancy_grid_subscriber.h"
#include <avt_341/mission_manager_params_dto.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341::mission {

struct ObsAvoidGoalFilterParams {

    std::string vehicle_id;
    // costmap publish method of the grid producer; drives grid subscription
    // latching (empty = normal subscription)
    std::string publish_method;
    int occ_threshold = 0;
    double padding = 1.0;
    bool pub_unfiltered_goal = false;
    double patch_pad_width = 20.0;
    double min_obstacle_width = 5.0;
    double follower_divergence_threshold = 30.0;
    bool persist_state = true;
    bool reset_side_on_free_space = true;
    bool ignore_deadlock = false;

    explicit ObsAvoidGoalFilterParams(const std::string &vehicle_id)
        : vehicle_id(vehicle_id){
    }
};

class ObsAvoidGoalFilter : public GoalFilter {

public:
    explicit ObsAvoidGoalFilter(
        rclcpp::Node::SharedPtr node,
        const std::string& vehicle_id,
        const avt_341::params::mission_manager::Params::FgfObsAvoid& filter_params =
            avt_341::params::mission_manager::Params::FgfObsAvoid{},
        const std::string& publish_method = std::string());

    geometry_msgs::msg::Pose Filter(const geometry_msgs::msg::Pose &candidate_goal, const geometry_msgs::msg::Pose &leader_pose) override;

    void Reset() override;

    // Divergence check
    bool FollowerDiverges(const Eigen::Vector2d& leader_point,
                          const Eigen::Vector2d& follower_point) const;

    // Main reference‐point logic
    std::tuple<Eigen::MatrixXi, Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector2d>
    GetRefPoint(const Eigen::MatrixXi& grid,
                const std::tuple<Eigen::MatrixXi, Eigen::Vector2i, Eigen::Vector2i>& patch_data,
                const std::tuple<Eigen::Vector2d, Eigen::Vector2d, double>& follower_data);

private:

    void OccupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    Eigen::Vector2d ToGridCoords(const geometry_msgs::msg::Point& ros_point);
    Eigen::Vector2d ToRosCoords(const Eigen::Vector2d& grid_point);

    // process one (point, offset) sample and return corrected point
    Eigen::Vector2d ProcessSample(const Eigen::Vector2d& point,
                                  const Eigen::Vector2d& offset,
                                  double desired_yaw);

    rclcpp::Node::SharedPtr node_;
    ObsAvoidGoalFilterParams params_;

    std::shared_ptr<node::OccupancyGridSubscriber> grid_sub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> unfiltered_goal_pub_ = nullptr;

    Eigen::MatrixXi occupancy_grid_;

    // minimal state to preserve behavior across steps
    std::optional<Eigen::Vector2d> last_point_;    // last published (for intersection check)

    // cached patch / state used by the avoidance logic
    Eigen::Vector2d map_origin_;
    double map_resolution_;

    Eigen::MatrixXi patch_;
    Eigen::Vector2i patch_origin_;
    Eigen::Vector2i patch_padding_offset_;
    std::string     direction_;
    int             row_idx_;
    bool            deadlock_;
};

} // namespace avt_341::mission


#endif // OBS_AVOIDANCE_GOAL_FILTER_HPP
