#include "avt_341_nav/mission/goal_filtering/obs_avoid_goal_filter.hpp"

#include <avt_341_nav/core/ros_msg_utils.hpp>

#include "avt_341_nav/mission/goal_filtering/obs_avoid_goal_filter_utils.hpp"
#include "avt_341_nav/node/occupancy_grid_subscriber.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341_nav::mission {

#define OCCUPANCY_GRID_TOPIC_NAME "avt_341/occupancy_grid"
#define UNFILTERED_GOAL_TOPIC_NAME "avt_341/unfiltered_follower_pose"
#define WORLD_TF_FRAME "map"

ObsAvoidGoalFilter::ObsAvoidGoalFilter(
    rclcpp::Node::SharedPtr node,
    const std::string& vehicle_id,
    const avt_341_nav::params::mission_manager::Params::FgfObsAvoid& filter_params,
    const std::string& publish_method)
    : node_(node),
    params_(vehicle_id),
    last_point_(std::optional<Eigen::Vector2d>()),
    row_idx_(-1),
    deadlock_(false) {

    params_.publish_method = publish_method;
    params_.occ_threshold = static_cast<int>(filter_params.occ_threshold);
    params_.padding = filter_params.padding;
    params_.pub_unfiltered_goal = filter_params.pub_unfiltered_goal;
    params_.patch_pad_width = filter_params.patch_pad_width;
    params_.min_obstacle_width = filter_params.min_obstacle_width;
    params_.follower_divergence_threshold = filter_params.follower_divergence_threshold;
    params_.reset_side_on_free_space = filter_params.reset_side_on_free_space;
    params_.persist_state = filter_params.persist_state;
    params_.ignore_deadlock = filter_params.ignore_deadlock;

    RCLCPP_INFO(node_->get_logger(), "Formation goal filter parameters:"
                    "\n vehicle_id: %s"
                    "\n method: obs_avoid"
                    "\n occ_threshold: %d"
                    "\n padding: %.2f"
                    "\n pub_unfiltered_goal: %d"
                    "\n patch_pad_width: %.2f"
                    "\n min_obstacle_width: %.2f"
                    "\n follower_divergence_threshold: %.2f"
                    "\n reset_side_on_free_space: %d"
                    "\n persist_state: %d"
                    "\n ignore_deadlock: %d", params_.vehicle_id.c_str(), params_.occ_threshold, params_.padding, params_.pub_unfiltered_goal, params_.patch_pad_width, params_.min_obstacle_width, params_.follower_divergence_threshold, params_.reset_side_on_free_space, params_.persist_state, params_.ignore_deadlock);

    grid_sub_ = std::make_shared<node::OccupancyGridSubscriber>(
        node,
        OCCUPANCY_GRID_TOPIC_NAME,
        10,
        params_.publish_method,
        std::bind(&ObsAvoidGoalFilter::OccupancyGridCallback, this, std::placeholders::_1));

    if (params_.pub_unfiltered_goal) {
        unfiltered_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(UNFILTERED_GOAL_TOPIC_NAME, 1);
    }
}

void ObsAvoidGoalFilter::OccupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    const int W = msg->info.width;
    const int H = msg->info.height;

    if (occupancy_grid_.size() == 0 || occupancy_grid_.rows() != W || occupancy_grid_.cols() != H) {
        // Code currently expects rows to be in x position
        map_origin_ = Eigen::Vector2d(msg->info.origin.position.y, msg->info.origin.position.x);
        map_resolution_ = msg->info.resolution;
        occupancy_grid_ = Eigen::MatrixXi::Zero(H, W);
    }

    const core::GridRegion update_region = grid_sub_->GetAndResetLastUpdateBounds();

    // map incoming ROS values → internal OCC_* values
    for (int i = update_region.y_min; i < update_region.y_max; ++i) {
        for (int j = update_region.x_min; j < update_region.x_max; ++j) {
            const int v = msg->data[i * W + j];
            occupancy_grid_(i, j) = v > params_.occ_threshold ? OCC_OBSTACLE : OCC_FREE;
        }
    }

    // one-cell padding around obstacles
    const auto& p = static_cast<int>(params_.padding / map_resolution_);
    const int i_start = std::max(p, update_region.y_min - p);
    const int i_end = std::min(H-p, update_region.y_max + p);
    const int j_start = std::max(p, update_region.x_min - p);
    const int j_end = std::min(W-p, update_region.x_max + p);

    Eigen::MatrixXi region_to_pad = occupancy_grid_.block(i_start, j_start, i_end-i_start, j_end-j_start);
    for (int i = 0; i < region_to_pad.rows(); ++i) {
        for (int j = 0; j < region_to_pad.cols(); ++j) {
            const int i_world = i + i_start;
            const int j_world = j + j_start;

            if (occupancy_grid_(i_world, j_world) == OCC_FREE) {
                for (int di = -p; di <= p; ++di) {
                    for (int dj = -p; dj <= p; ++dj) {

                        if (occupancy_grid_(i_world + di, j_world + dj) != OCC_FREE) {
                            region_to_pad(i, j) = OCC_PADDING;
                            goto next_cell;
                        }

                    }
                }
            }

            next_cell: ;
        }
    }
    occupancy_grid_.block(i_start, j_start, i_end-i_start, j_end-j_start) = region_to_pad;
}

Eigen::Vector2d ObsAvoidGoalFilter::ToGridCoords(const geometry_msgs::msg::Point& ros_point) {
    // Code currently expects rows to be in x position
    return (Eigen::Vector2d(ros_point.y, ros_point.x) - map_origin_)/map_resolution_;
}

Eigen::Vector2d ObsAvoidGoalFilter::ToRosCoords(const Eigen::Vector2d& grid_point) {
    // Code currently expects rows to be in x position
    auto temp = (grid_point + map_origin_) * map_resolution_;
    return Eigen::Vector2d(temp.y(), temp.x());
}

geometry_msgs::msg::Pose ObsAvoidGoalFilter::Filter(const geometry_msgs::msg::Pose &candidate_goal, const geometry_msgs::msg::Pose &leader_pose) {

    if (occupancy_grid_.size() == 0) {
        RCLCPP_WARN(node_->get_logger(), "No occupancy grid received yet.");
        return candidate_goal;
    }

    Eigen::Vector2d pt_candidate = ToGridCoords(candidate_goal.position);
    const double lateral_angle = std::fmod(core::GetHeadingFromOrientation(candidate_goal.orientation) + M_PI_2, 2*M_PI);
    Eigen::Vector2d pt_leader = ToGridCoords(leader_pose.position);
    Eigen::Vector2d off = pt_candidate - pt_leader;

    Eigen::Vector2d new_pt = ProcessSample(pt_candidate, off, lateral_angle);
    new_pt = ToRosCoords(new_pt);

    if (params_.pub_unfiltered_goal) {
        geometry_msgs::msg::PoseStamped unfiltered_goal;
        unfiltered_goal.pose = candidate_goal;
        unfiltered_goal.header.frame_id = WORLD_TF_FRAME;
        unfiltered_goal.header.stamp = node_->now();
        unfiltered_goal_pub_->publish(unfiltered_goal);
    }

    geometry_msgs::msg::Pose return_msg = candidate_goal;
    return_msg.position.x = new_pt.x();
    return_msg.position.y = new_pt.y();
    return return_msg;
}

void ObsAvoidGoalFilter::Reset() {
    deadlock_ = false;
    direction_ = "";
    row_idx_ = -1;
    last_point_ = std::optional<Eigen::Vector2d>();
}

bool ObsAvoidGoalFilter::FollowerDiverges(const Eigen::Vector2d& leader_point, const Eigen::Vector2d& follower_point) const {
    return (leader_point - follower_point).norm() > params_.follower_divergence_threshold;
}


std::tuple<Eigen::MatrixXi, Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector2d>
ObsAvoidGoalFilter::GetRefPoint(const Eigen::MatrixXi& grid,
            const std::tuple<Eigen::MatrixXi, Eigen::Vector2i, Eigen::Vector2i>& pd,
            const std::tuple<Eigen::Vector2d, Eigen::Vector2d, double>& fd) {
    using Eigen::MatrixXi;
    using Eigen::Vector2d;
    using Eigen::Vector2i;
    using Eigen::VectorXi;
    using std::string;

    auto [po, oo, pad] = pd;        // patch (window), origin (window start), padding_offset (0,0)
    auto [pt, off, angle] = fd;         // candidate goal, offset to leader, heading angle of candidate goal

    Vector2d newpt = pt;
    MatrixXi patch = po;
    Vector2i orig  = oo;

    bool no_collision    = true;
    bool no_intersection = true;

    const auto deadlock_obs_cells_width = static_cast<int>(params_.min_obstacle_width/map_resolution_);
    const auto patch_path_cells_width = static_cast<int>(params_.patch_pad_width/map_resolution_);

    bool deadlock = deadlock_;
    std::string dir = direction_;
    int prev = row_idx_;

    // -----------------------------
    // PART 1: direct collision?
    // -----------------------------
    const bool is_in_collision = isInCollision(grid, pt);

    if (params_.reset_side_on_free_space && !is_in_collision) {
        dir = "";
        prev = -1;
    }

    if (!deadlock && is_in_collision) {
        no_collision = false;

        // window-based patch around current point
        std::tie(patch, orig, pad) = extractPatch(grid, pt, patch_path_cells_width);

        // patch center in (row, col) coords
        const Vector2d pc(patch.rows() / 2.0, patch.cols() / 2.0);

        // patch point in patch coords (window semantics: pad is (0,0))
        Vector2d ppt(pt[0] - orig[0] + pad[0],
                     pt[1] - orig[1] + pad[1]);

        // rotate patch and extract row aligned with motion direction
        auto [row, rot, ppt_adj] = getRowFromPatch(patch, ppt, pc, angle);
        std::tie(newpt, dir, prev, deadlock) =
            avoidCollision({rot, ppt_adj, pc, orig, pad}, angle, row, dir);
    }

    // -------------------------------------------------------------------------------------------------------
    // PART 2: last timestep filtered goal and current filtered goal segment intersection with obstacles?
    // -------------------------------------------------------------------------------------------------------
    Eigen::Vector2d last_filtered_goal = last_point_.value_or(newpt);
    if (!deadlock && pointsIntersect(grid, last_filtered_goal, newpt)) {
        no_intersection = false;

        // if we didn't already extract patch above, do it now w.r.t. newpt
        if (no_collision) {
            std::tie(patch, orig, pad) = extractPatch(grid, newpt, patch_path_cells_width);
        }

        const Vector2d pc2(patch.rows() / 2.0, patch.cols() / 2.0);
        Vector2d ppt2(pt[0] - orig[0] + pad[0],
                      pt[1] - orig[1] + pad[1]);

        auto [row2, rot2, ppt_adj2] = getRowFromPatch(patch, ppt2, pc2, angle);
        std::tie(newpt, dir, prev, deadlock) =
            avoidIntersection({rot2, ppt_adj2, pc2, orig, pad}, angle, row2, prev, dir, deadlock_obs_cells_width);
    }

    // --------------------------------------------
    // PART 3: deadlock handling
    // --------------------------------------------
    if (deadlock) {
        if (!deadlock_) {
            RCLCPP_WARN(node_->get_logger(), "ObsAvoidGoalFilter: Entering deadlock state");
        }
        newpt = last_filtered_goal;    // stay at previous point
        prev  = -1;     // no valid row index
    }

    // divergence diagnostic
    const Vector2d leader_pt = pt - off;
    if (!deadlock && FollowerDiverges(leader_pt, newpt)) {
        RCLCPP_WARN(node_->get_logger(), "Follower %s diverges from leader path at point [%.2f, %.2f]", params_.vehicle_id.c_str(), newpt[0], newpt[1]);
    }

    if (params_.persist_state)
    {
        deadlock_ = deadlock;
        direction_ = dir;
        row_idx_ = prev;
        last_point_ = newpt;
        // patch, orig, and pad also technically part of state but patch and orig always re-assigned and pad currently always returned as 0s
    }

    if (deadlock_ && params_.ignore_deadlock) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring detected deadlock and resetting internal state since ignore_deadlock=True");
        deadlock_ = false;
        Reset();
    }

    return {patch, orig, pad, newpt};
}




Eigen::Vector2d ObsAvoidGoalFilter::ProcessSample(
                                             const Eigen::Vector2d& point,
                                             const Eigen::Vector2d& offset,
                                             const double desired_yaw) {

    // pack patch/follower data
    auto patch_data = std::make_tuple(patch_, patch_origin_, patch_padding_offset_);
    auto flwr_data  = std::make_tuple(point, offset, desired_yaw);

    // run core logic
    auto result = GetRefPoint(occupancy_grid_, patch_data, flwr_data);
    Eigen::Vector2d point_new;
    std::tie(patch_, patch_origin_, patch_padding_offset_, point_new) = result;
    return point_new;
}

} // namespace avt_341_nav::mission
