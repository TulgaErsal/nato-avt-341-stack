#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <avt_341/perception/occupancy_grid_parser/obstacle.hpp>
#include <avt_341/perception/occupancy_grid_parser/occupancy_grid.hpp>
#include <avt_341/perception/occupancy_grid_parser/state.hpp>
#include <avt_341_msgs/msg/static_obstacle_array.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

enum HorizonType {
    STATIC=0,
    DYNAMIC=1,
    ADAPTIVE=2
};

ObstacleType GetObstacleTypeFromString(std::string obstacle_type);

HorizonType GetHorizonTypeFromString(std::string horizon_type);

class OccupancyGridParserNode : public rclcpp::Node {
  public:
    OccupancyGridParserNode();

  protected:
    void GetParameters();
    void Initialize();
    void CreateCallbackGroups();
    void CreateSubscriptions();
    void CreateTimers();
    void CreatePublishers();
    void Start();

  private:
    // Thread safety
    // -------------

    /** @brief Shared timed mutex for the odometry and occupancy grid data
     * updates. */
    std::shared_timed_mutex data_mutex_;

    /** @brief Callback group for the obstacles refresh timer. */
    rclcpp::CallbackGroup::SharedPtr refresh_callback_group_;

    /** @brief Shared pointer to the odometry subscription. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    /** @brief Callback group for the AGV odometry subscription. */
    rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
    // -------------

    void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry_message);

    /** @brief Whether or not the node stores valid AGV odometry. */
    bool has_odometry_;

    /** @brief AGV state. */
    State state_;

    // Occupancy grid
    // --------------

    /** @brief Shared pointer to the occuancy grid subscription. */
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        occupancy_grid_subscription_;

    /** @brief Callback group for the occupancy grid. */
    rclcpp::CallbackGroup::SharedPtr occupancy_grid_callback_group_;

    void OccupancyGridCallback(
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_message);

    /** @brief Occupancy grid. */
    OccupancyGrid occupancy_grid_;

    /** @brief Whether or not the node stores a valid occupancy grid. */
    bool has_occupancy_grid_ = false;

    /** @brief Threshold for occupancy grid cell values filtering. */
    int occupancy_threshold_;

    /** @brief Frame ID of the latest occupancy grid message. */
    std::string occupancy_grid_frame_id_;
    // --------------

    // Obstacles
    // ---------

    /** @brief Vector of parsed obstacles. */
    std::vector<Obstacle> obstacles_;

    /** @brief Maximum number of parsed obstacles (within the bounds of the
     * search region). */
    int maximum_obstacles_;

    /** @brief Inflation factor used to scale the obstacle extents. */
    double inflation_factor_;

    /** @brief Obstacle type. */
    ObstacleType obstacle_type_ = ObstacleType::BOX;

    /** @brief Obstacles refresh timer. */
    rclcpp::TimerBase::SharedPtr refresh_timer_;

    /** @brief Callback fro the obstacles refresh timer. */
    void RefreshTimerCallback();

    /** @brief Obstacles refresh timer rate. */
    double refresh_rate_;

    /** @brief Shared pointer to the static obstacles publisher. */
    rclcpp::Publisher<avt_341_msgs::msg::StaticObstacleArray>::SharedPtr
        static_obstacles_publisher_;

    void PublishObstacles(const OccupancyGrid& occupancy_grid);
    // ---------

    // Visualization
    // -------------

    /** @brief Whether or not to enable visualization callbacks and
     * publishers.
     */
    bool use_visualization_;

    /** @brief Shared pointer to the markers publisher. */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_publisher_;

    /**
     * @brief Publish the markers.
     */
    void PublishMarkers(const State& state,
                        const OccupancyGrid& occupancy_grid);

    /**
     * @brief Publish obstacle markers.
     */
    void PublishObstacleMarkers(const State& state,
                                const OccupancyGrid& occupancy_grid);

    /** @brief Number of obstacles at the previous refresh callback. */
    int previous_obstacles_ = 0;

    /** @brief Difference in obstacle count between two refresh callbacks.
     */
    int changed_obstacles_ = 0;

    /** @brief Visualization update rate. */
    double visualization_rate_;

    /** @brief Last visualization time stamp. */
    rclcpp::Time last_visualization_time_;

    /**
     * @brief Publish search region markers.
     */
    void PublishSearchRegionMarkers(const State& state,
                                    const OccupancyGrid& occupancy_grid);
    // -------------

    // Horizon
    // -------

    /** @brief Whether or not to use obstacle filtering based on a fixed or
     * dynamic horizon. */
    bool use_horizon_;

    /** @brief Whether or not to use the dynamic horizon. */
    HorizonType horizon_type_;

    /** @brief Length of the filtering horizon. */
    double horizon_radius_;

    /** @brief Time interval for the dynamic horizon (relative to vehicle speed). */
    double horizon_time_;

    /** @brief The maximum vehicle speed for the dynamic horizon calculation. */
    double horizon_max_speed_;

    // -------

    // Field of view
    // -------------

    /** @brief Whether or not to use obstacle filtering based on the field
     * of view. */
    bool use_fov_;

    /** @brief Field of view arc in degrees. */
    double fov_degrees_;

    /** @brief Vector defining the direction of the left edge of the field
     * of view sector. */
    Eigen::Vector3d fov_cone_left_ = Eigen::Vector3d::Zero();

    /** @brief Vector defining the direction of the right edge of the field
     * of view sector. */
    Eigen::Vector3d fov_cone_right_ = Eigen::Vector3d::Zero();
    // -------------
};

} // namespace occupancy
} // namespace perception
} // namespace avt_341