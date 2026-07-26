#ifndef AVT_341_RAYTRACE_CLEARING_METHOD_H
#define AVT_341_RAYTRACE_CLEARING_METHOD_H

#include "costmap_clearing_method.h"
#include "avt_341/node/tf_interface.h"
#include <bitset>
#include "avt_341/avt_341_utils.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <rclcpp/rclcpp.hpp>

namespace avt_341::perception {

// RAYTRACE CLEARING SETTINGS
// ==================================================================================================================
// ==================================================================================================================

using RaytraceSettings = ClearMethodSettings;



// RAYTRACE CLEARING
// ==================================================================================================================
// ==================================================================================================================

class RaytraceClearingMethod : public OccupancyClearingMethod {
public:
    static constexpr int N_VOXELS_PER_CELL = 1024;

    RaytraceClearingMethod(const rclcpp::Node::SharedPtr & node_ref,
                           const std::shared_ptr<node::TfInterface> & tf,
                           std::vector<std::vector<Cell>> & cells,
                           const PerceptionSettings & settings,
                           const RaytraceSettings & rt_config,
                           CellObstacleCalculator *obs_calculator,
                           bool handle_dilation = true
                           );

    RaytraceClearingMethod(const rclcpp::Node::SharedPtr& node_ref,
                           const std::shared_ptr<node::TfInterface> & tf,
                           std::vector<std::vector<Cell>> & cells,
                           int Ny,
                           int Nx,
                           const PerceptionSettings & settings,
                           const RaytraceSettings & rt_config,
                           CellObstacleCalculator *obs_calculatorr,
                           bool handle_dilation = true
                           );

    virtual ~RaytraceClearingMethod() override;

    void ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) override;

    void Visualize() const override;

    virtual void ResetInternalCellState(int x, int y) override;

    std::string GetDescription() const override;

protected:
    geometry_msgs::msg::Point TfTransformToPoint(const geometry_msgs::msg::TransformStamped &transform) const;

    geometry_msgs::msg::Point GetSensorOrigin(const rclcpp::Time &stamp) const;

    geometry_msgs::msg::Point GetSensorOrigin() const;

    void GetGridBounds(
        const geometry_msgs::msg::Point &origin,
        float range,
        int &x_0,
        int &y_0,
        int &x_N,
        int &y_N) const;

    visualization_msgs::msg::Marker GetMarkerMsg(int type, int id, utils::vec3 color, float alpha = 1.0, double z_scale = 1.0) const;

    virtual void RaytraceLine(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point32 &end);

    void CleanupUnattachedDilation(const geometry_msgs::msg::Point &origin, std::vector<std::vector<Cell> > &cells);

    void ClearVoxelAt(int x, int y, int z);

    void Reset() override;

    void SetLidarFrame();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<node::TfInterface> tf_;
    RaytraceSettings rt_config_;
    bool handle_dilation_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray> > minmax_vis_publisher_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray> > voxel_vis_publisher_;
    std::string lidar_frame_;
    std::bitset<N_VOXELS_PER_CELL> *voxel_grid;
};



// RAYTRACE CLEARING WITH OBSTACLE DISTANCE FILTERING
// ==================================================================================================================
// ==================================================================================================================

class RaytraceWithFilteringClearingMethod : public RaytraceClearingMethod {

public:

    RaytraceWithFilteringClearingMethod(const rclcpp::Node::SharedPtr& node_ref,
                                        const std::shared_ptr<node::TfInterface> & tf,
                                        std::vector<std::vector<Cell>> & cells,
                                        const PerceptionSettings & settings,
                                        const RaytraceSettings & rt_config,
                                        CellObstacleCalculator *obs_calculator
                                        );

    void ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) override;

    void OnOccupancyAdded(const sensor_msgs::msg::PointCloud &point_cloud, const geometry_msgs::msg::Point &veh_pos) override;

    void Visualize() const override;

    void Reset() override;

    virtual void ResetInternalCellState(int x, int y) override;

    std::string GetDescription() const override;

protected:
    std::vector<std::vector<Cell>> &cells_without_clearing_;
    std::vector<std::vector<Cell>> cells_with_clearing_;
    std::vector<std::vector<bool>> occupancy_delta_;
    utils::vec2 last_position_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> occupancy_delta_publisher_;
};

}

#endif //AVT_341_RAYTRACE_CLEARING_METHOD_H
