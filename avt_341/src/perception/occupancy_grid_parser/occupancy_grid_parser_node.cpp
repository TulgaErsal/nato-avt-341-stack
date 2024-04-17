#include <avt_341/perception/occupancy_grid_parser/occupancy_grid_parser_node.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

HorizonType GetHorizonTypeFromString(std::string horizon_type) {
    if(horizon_type == "static") {
        return HorizonType::STATIC;
    } else if(horizon_type == "dynamic") {
        return HorizonType::DYNAMIC;
    } else if(horizon_type == "adaptive") {
        return HorizonType::ADAPTIVE;
    }
    throw rclcpp::exceptions::InvalidParameterValueException(
        "Invalid horizon type.");
}

ObstacleType GetObstacleTypeFromString(std::string obstacle_type) {
    if(obstacle_type == "box") {
        return ObstacleType::BOX;
    } else if(obstacle_type == "cylinder") {
        return ObstacleType::CYLINDER;
    }
    throw rclcpp::exceptions::InvalidParameterValueException(
        "Invalid obstacle type.");
}

OccupancyGridParserNode::OccupancyGridParserNode()
    : rclcpp::Node("occupancy_grid_parser") {
    GetParameters();
    Initialize();
    CreateCallbackGroups();
    CreateSubscriptions();
    CreateTimers();
    CreatePublishers();
    Start();
}

void OccupancyGridParserNode::GetParameters() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Getting node parameters ...");

    declare_parameter("obstacles.rate", 50.0);
    refresh_rate_ = get_parameter("obstacles.rate").as_double();

    declare_parameter("obstacles.limit", -1);
    maximum_obstacles_ = get_parameter("obstacles.limit").as_int();

    declare_parameter("obstacles.type", "cylinder");
    obstacle_type_ =
        GetObstacleTypeFromString(get_parameter("obstacles.type").as_string());

    declare_parameter("obstacles.inflation", 1.0);
    inflation_factor_ = get_parameter("obstacles.inflation").as_double();

    declare_parameter("occupancy.threshold", 0);
    occupancy_threshold_ = get_parameter("occupancy.threshold").as_int();

    declare_parameter("horizon.enable", true);
    use_horizon_ = get_parameter("horizon.enable").as_bool();

    declare_parameter("horizon.type", "static");
    horizon_type_ =
        GetHorizonTypeFromString(get_parameter("horizon.type").as_string());

    declare_parameter("horizon.radius", 5.0);
    horizon_radius_ = get_parameter("horizon.radius").as_double();

    declare_parameter("horizon.time", 5.0);
    horizon_time_ = get_parameter("horizon.max_speed").as_double();

    declare_parameter("horizon.max_speed", 15.0);
    horizon_max_speed_ = get_parameter("horizon.max_speed").as_double();

    declare_parameter("fov.enable", true);
    use_fov_ = get_parameter("fov.enable").as_bool();

    declare_parameter("fov.degrees", 90.0);
    fov_degrees_ = get_parameter("fov.degrees").as_double();

    declare_parameter("visualization.enable", true);
    use_visualization_ = get_parameter("visualization.enable").as_bool();

    declare_parameter("visualization.rate", 5.0);
    visualization_rate_ = get_parameter("visualization.rate").as_double();
}

void OccupancyGridParserNode::Initialize() {
    last_visualization_time_ = get_clock()->now();
}

void OccupancyGridParserNode::CreateCallbackGroups() {
    RCLCPP_DEBUG(get_logger(), "Creating callback groups ...");

    odometry_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    occupancy_grid_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    refresh_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void OccupancyGridParserNode::CreateSubscriptions() {
    RCLCPP_DEBUG(get_logger(), "Creating node subscriptions ...");

    rclcpp::SubscriptionOptions odometry_subscription_options;
    odometry_subscription_options.callback_group = odometry_callback_group_;
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        1,
        std::bind(&OccupancyGridParserNode::OdometryCallback,
                  this,
                  std::placeholders::_1),
        odometry_subscription_options);

    rclcpp::SubscriptionOptions occupancy_grid_subscription_options;
    occupancy_grid_subscription_options.callback_group =
        occupancy_grid_callback_group_;
    occupancy_grid_subscription_ =
        create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid",
            1,
            std::bind(&OccupancyGridParserNode::OccupancyGridCallback,
                      this,
                      std::placeholders::_1),
            occupancy_grid_subscription_options);
}

void OccupancyGridParserNode::CreatePublishers() {
    RCLCPP_DEBUG(get_logger(), "Creating node publishers ...");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);

    static_obstacles_publisher_ =
        create_publisher<avt_341_msgs::msg::StaticObstacleArray>(
            "obstacles/static",
            qos);

    if(use_visualization_) {
        markers_publisher_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("markers",
                                                                   qos);
    }
}

void OccupancyGridParserNode::CreateTimers() {
    RCLCPP_DEBUG(get_logger(), "Creating node timers ...");

    // Register the planning timer (without starting it).
    refresh_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / refresh_rate_),
        std::bind(&OccupancyGridParserNode::RefreshTimerCallback, this),
        refresh_callback_group_);
    refresh_timer_->cancel();
}

void OccupancyGridParserNode::Start() { refresh_timer_->reset(); }

void OccupancyGridParserNode::OdometryCallback(
    nav_msgs::msg::Odometry::SharedPtr odometry_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Odometry callback triggered!");

    if(data_mutex_.try_lock_shared_for(
           std::chrono::duration<double>(1.0 / refresh_rate_))) {
        state_ = State(odometry_message);
        has_odometry_ = true;
        data_mutex_.unlock_shared();
    }
}

void OccupancyGridParserNode::OccupancyGridCallback(
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Occupancy grid callback triggered!");

    if(data_mutex_.try_lock_shared_for(
           std::chrono::duration<double>(1.0 / refresh_rate_))) {
        occupancy_grid_ = OccupancyGrid(occupancy_grid_message);
        has_occupancy_grid_ = true;
        data_mutex_.unlock_shared();
    }
}

void OccupancyGridParserNode::RefreshTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Refresh timer callback triggered!");

    obstacles_.clear();
    int processed_obstacles = 0;

    if(!has_occupancy_grid_) { return; }
    if((use_horizon_ || use_fov_) && !has_odometry_) { return; }

    State state;
    OccupancyGrid occupancy_grid;
    if(data_mutex_.try_lock_for(
           std::chrono::duration<double>(1.0 / refresh_rate_))) {
        occupancy_grid = occupancy_grid_;
        state = state_;
        data_mutex_.unlock();
    }

    // Scale the dynamic horizon
    switch(horizon_type_) {
    case HorizonType::STATIC: break;
    case HorizonType::DYNAMIC:
        // Consider the distance covered at maximum speed with +10% margin.
        horizon_radius_ = std::max(horizon_radius_,
                                   (horizon_time_ + 0.1) * horizon_max_speed_);
        break;
    case HorizonType::ADAPTIVE:
        horizon_radius_ =
            std::max(horizon_radius_, (horizon_time_ + 0.1) * state.GetSpeed());
    }

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < occupancy_grid.GetHeight(); ++i) {
        for(int j = 0; j < occupancy_grid.GetWidth(); ++j) {
            // Skip cells below the specified occupancy threshold (this includes
            // cells for which the occupancy value is unknown, with value -1).
            if(occupancy_grid(i, j) <= occupancy_threshold_) { continue; }

            // Check if the maximum number of obstacles has been exceeded and
            // notify the user.
            if(maximum_obstacles_ != -1 &&
               processed_obstacles >= maximum_obstacles_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000.0, // Throttle time in milliseconds.
                    "Number of processed obstacles exceeded! Consider "
                    "increasing the maximum number of obstacles "
                    "(obstacles.limit), enabling "
                    "the search horizon (horizon.enable) and/or limit the "
                    "search field of view (fov.enable).");
#ifndef USE_OPENMP
                return;
#endif
            };

            auto centroid =
                Eigen::Vector3d((j + 0.5) * occupancy_grid.GetResolution() +
                                    occupancy_grid.GetPosition().x(),
                                (i + 0.5) * occupancy_grid.GetResolution() +
                                    occupancy_grid.GetPosition().y(),
                                0.0);

            // Compute the distance between the AGV and the obstacle centroid.
            Eigen::Vector3d distance = centroid - state.GetPosition();

            bool is_within_search_region = true;
            if(use_horizon_) {
                if(distance.norm() > horizon_radius_) {
                    is_within_search_region = false;
                }
            }

            if(use_fov_) {
                fov_cone_right_ = state.GetTransform().rotate(
                                      -(fov_degrees_ / 2.0) * M_PI / 180.0) *
                    distance;
                fov_cone_left_ = state.GetTransform().rotate(
                                     (fov_degrees_ / 2.0) * M_PI / 180.0) *
                    distance;
                is_within_search_region =
                    ((fov_cone_left_).cross(distance).z()) <= 0 &&
                    ((fov_cone_right_).cross(distance)).z() >= 0;
            }
            if(is_within_search_region) {
                obstacles_.push_back(Obstacle(centroid.x(),
                                              centroid.y(),
                                              occupancy_grid.GetResolution(),
                                              obstacle_type_));
            }

            // Spin a separate thread to publish the obstacles.
            std::thread publish_obstacles_thread(
                &OccupancyGridParserNode::PublishObstacles,
                this,
                occupancy_grid);
            publish_obstacles_thread.detach();

            changed_obstacles_ = obstacles_.size() - previous_obstacles_;
            if(use_visualization_ &&
               (get_clock()->now() - last_visualization_time_).seconds() >
                   1.0 / visualization_rate_) {
                PublishMarkers(state, occupancy_grid);

                last_visualization_time_ = get_clock()->now();
            }
            previous_obstacles_ = obstacles_.size();
        }
    }
}

void OccupancyGridParserNode::PublishObstacles(
    const OccupancyGrid& occupancy_grid) {
    auto static_obstacles_array_message =
        std::make_shared<avt_341_msgs::msg::StaticObstacleArray>();
    static_obstacles_array_message->header.frame_id =
        occupancy_grid.GetFrameId();
    static_obstacles_array_message->header.stamp = get_clock()->now();
    for(const auto& obstacle : obstacles_) {
        auto static_obstacle_message =
            std::make_shared<avt_341_msgs::msg::StaticObstacle>();
        static_obstacle_message->type = obstacle.GetMarkerType();
        static_obstacle_message->pose.position.x = obstacle.GetPosition().x();
        static_obstacle_message->pose.position.y = obstacle.GetPosition().y();
        static_obstacle_message->pose.position.z = obstacle.GetPosition().z();
        static_obstacle_message->pose.orientation.x =
            obstacle.GetOrientation().x();
        static_obstacle_message->pose.orientation.y =
            obstacle.GetOrientation().y();
        static_obstacle_message->pose.orientation.z =
            obstacle.GetOrientation().z();
        static_obstacle_message->pose.orientation.w =
            obstacle.GetOrientation().w();
        static_obstacle_message->extents.x = obstacle.GetExtent().x();
        static_obstacle_message->extents.y = obstacle.GetExtent().y();
        static_obstacle_message->extents.z = obstacle.GetExtent().z();
        static_obstacles_array_message->obstacles.push_back(
            *static_obstacle_message);
    }
    static_obstacles_publisher_->publish(*static_obstacles_array_message);
}

void OccupancyGridParserNode::PublishMarkers(
    const State& state,
    const OccupancyGrid& occupancy_grid) {
    if(use_horizon_ || use_fov_) {
        std::thread publish_fov_thread(
            &OccupancyGridParserNode::PublishSearchRegionMarkers,
            this,
            state,
            occupancy_grid);
        publish_fov_thread.detach();
    }

    std::thread publish_obstacle_markers_thread(
        &OccupancyGridParserNode::PublishObstacleMarkers,
        this,
        state,
        occupancy_grid);
    publish_obstacle_markers_thread.detach();
}

void OccupancyGridParserNode::PublishObstacleMarkers(
    const State& state,
    const OccupancyGrid& occupancy_grid) {
    if(changed_obstacles_ <= 0) {
        int delete_obstacle_id = previous_obstacles_;
        visualization_msgs::msg::MarkerArray delete_obstacle_markers_array;
        visualization_msgs::msg::Marker delete_obstacle_markers;
        delete_obstacle_markers.header.stamp = get_clock()->now();
        delete_obstacle_markers.header.frame_id = occupancy_grid.GetFrameId();
        delete_obstacle_markers.id = delete_obstacle_id;
        delete_obstacle_markers.ns = "obstacles";
        delete_obstacle_markers.type =
            visualization_msgs::msg::Marker::DELETEALL;
        delete_obstacle_markers.scale.x = 1.0;
        delete_obstacle_markers.scale.y = 1.0;
        delete_obstacle_markers.scale.z = 1.0;
        delete_obstacle_markers_array.markers.push_back(
            delete_obstacle_markers);
        delete_obstacle_id--;
        markers_publisher_->publish(delete_obstacle_markers_array);
    }

    visualization_msgs::msg::MarkerArray obstacles_marker_array_message;
    int obstacle_id = 0;
    for(const auto& obstacle : obstacles_) {
        visualization_msgs::msg::Marker obstacle_marker_message;
        obstacle_marker_message.header.stamp = get_clock()->now();
        obstacle_marker_message.header.frame_id = occupancy_grid.GetFrameId();
        obstacle_marker_message.id = obstacle_id;
        obstacle_marker_message.ns = "obstacles";
        obstacle_marker_message.type = obstacle.GetMarkerType();
        obstacle_marker_message.action = visualization_msgs::msg::Marker::ADD;
        obstacle_marker_message.pose.position.x = obstacle.GetPosition().x();
        obstacle_marker_message.pose.position.y = obstacle.GetPosition().y();
        obstacle_marker_message.pose.position.z = 0.5;
        obstacle_marker_message.scale.x =
            obstacle.GetInflationFactor() * obstacle.GetExtent().x();
        obstacle_marker_message.scale.y =
            obstacle.GetInflationFactor() * obstacle.GetExtent().y();
        obstacle_marker_message.scale.z = 1.0;
        obstacle_marker_message.color.r = 1.0;
        obstacle_marker_message.color.a = 1.0;
        obstacles_marker_array_message.markers.push_back(
            obstacle_marker_message);
        obstacle_id++;
    }
    markers_publisher_->publish(obstacles_marker_array_message);
}

void OccupancyGridParserNode::PublishSearchRegionMarkers(
    const State& state,
    const OccupancyGrid& occupancy_grid) {
    visualization_msgs::msg::MarkerArray search_region_marker_array_message;

    // Horizon marker
    visualization_msgs::msg::Marker horizon_marker_message;
    horizon_marker_message.header.stamp = get_clock()->now();
    horizon_marker_message.header.frame_id = occupancy_grid.GetFrameId();
    horizon_marker_message.id = 0;
    horizon_marker_message.ns = "horizon";
    horizon_marker_message.type = visualization_msgs::msg::Marker::CYLINDER;
    horizon_marker_message.action = visualization_msgs::msg::Marker::ADD;
    horizon_marker_message.pose.position.x = state.GetPosition().x();
    horizon_marker_message.pose.position.y = state.GetPosition().y();
    horizon_marker_message.pose.position.z = 0.5;
    horizon_marker_message.scale.x = 2.0 * horizon_radius_;
    horizon_marker_message.scale.y = 2.0 * horizon_radius_;
    horizon_marker_message.scale.z = 1.0;
    horizon_marker_message.color.b = 1.0;
    horizon_marker_message.color.a = 0.25;
    search_region_marker_array_message.markers.push_back(
        horizon_marker_message);

    visualization_msgs::msg::Marker fov_marker_message;
    fov_marker_message.header.stamp = get_clock()->now();
    fov_marker_message.header.frame_id = occupancy_grid.GetFrameId();
    fov_marker_message.id = 0;
    fov_marker_message.ns = "fov";
    fov_marker_message.type = visualization_msgs::msg::Marker::LINE_LIST;
    fov_marker_message.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point agv_reference_point;
    // Pass the state into this function.
    agv_reference_point.x = state_.GetPosition().x();
    agv_reference_point.y = state_.GetPosition().y();

    geometry_msgs::msg::Point fov_cone_left_point;
    fov_cone_left_point.x = fov_cone_left_.normalized().x() * horizon_radius_;
    fov_cone_left_point.y = fov_cone_left_.normalized().y() * horizon_radius_;
    fov_marker_message.points.push_back(agv_reference_point);
    fov_marker_message.points.push_back(fov_cone_left_point);

    geometry_msgs::msg::Point fov_cone_right_point;
    fov_cone_right_point.x = fov_cone_right_.normalized().x() * horizon_radius_;
    fov_cone_right_point.y = fov_cone_right_.normalized().y() * horizon_radius_;
    fov_marker_message.points.push_back(agv_reference_point);
    fov_marker_message.points.push_back(fov_cone_right_point);

    fov_marker_message.scale.x = 0.05;
    fov_marker_message.scale.y = 1.0;
    fov_marker_message.scale.z = 1.0;
    fov_marker_message.color.b = 0.5;
    fov_marker_message.color.a = 1.0;
    search_region_marker_array_message.markers.push_back(fov_marker_message);
    markers_publisher_->publish(search_region_marker_array_message);
}

} // namespace occupancy
} // namespace perception
} // namespace avt_341