#ifndef AVT_341_RAYTRACE_CLEARING_METHOD_H
#define AVT_341_RAYTRACE_CLEARING_METHOD_H

#include "costmap_clearing_method.h"
#include <bitset>
#include "avt_341/avt_341_utils.h"

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

    RaytraceClearingMethod(const std::shared_ptr<node::NodeProxy> & node_ref,
                           std::vector<std::vector<Cell>> & cells,
                           const PerceptionSettings & settings,
                           const RaytraceSettings & rt_config,
                           CellObstacleCalculator *obs_calculator,
                           bool handle_dilation = true
                           );

    RaytraceClearingMethod(const std::shared_ptr<node::NodeProxy>& node_ref,
                           std::vector<std::vector<Cell>> & cells,
                           int Ny,
                           int Nx,
                           const PerceptionSettings & settings,
                           const RaytraceSettings & rt_config,
                           CellObstacleCalculator *obs_calculatorr,
                           bool handle_dilation = true
                           );

    virtual ~RaytraceClearingMethod() override;

    void ClearOccupancy(const msg::PointCloud &point_cloud) override;

    void Visualize() const override;

    virtual void ResetInternalCellState(int x, int y) override;

    std::string GetDescription() const override;

protected:
    msg::Point TfTransformToPoint(const msg::TransformStamped &transform) const;

    msg::Point GetSensorOrigin(const msg::Time &stamp) const;

    msg::Point GetSensorOrigin() const;

    void GetGridBounds(
        const msg::Point &origin,
        float range,
        int &x_0,
        int &y_0,
        int &x_N,
        int &y_N) const;

    msg::Marker GetMarkerMsg(int type, int id, utils::vec3 color, float alpha = 1.0, double z_scale = 1.0) const;

    virtual void RaytraceLine(const msg::Point &start, const msg::Point32 &end);

    void CleanupUnattachedDilation(const msg::Point &origin, std::vector<std::vector<Cell> > &cells);

    void ClearVoxelAt(int x, int y, int z);

    void Reset() override;

    void SetLidarFrame();

    std::shared_ptr<node::NodeProxy> node_;
    RaytraceSettings rt_config_;
    bool handle_dilation_;

    std::shared_ptr<node::Publisher<msg::MarkerArray> > minmax_vis_publisher_;
    std::shared_ptr<node::Publisher<msg::MarkerArray> > voxel_vis_publisher_;
    std::string lidar_frame_;
    std::bitset<N_VOXELS_PER_CELL> *voxel_grid;
};



// RAYTRACE CLEARING WITH OBSTACLE DISTANCE FILTERING
// ==================================================================================================================
// ==================================================================================================================

class RaytraceWithFilteringClearingMethod : public RaytraceClearingMethod {

public:

    RaytraceWithFilteringClearingMethod(const std::shared_ptr<node::NodeProxy>& node_ref,
                                        std::vector<std::vector<Cell>> & cells,
                                        const PerceptionSettings & settings,
                                        const RaytraceSettings & rt_config,
                                        CellObstacleCalculator *obs_calculator
                                        );

    void ClearOccupancy(const msg::PointCloud &point_cloud) override;

    void OnOccupancyAdded(const msg::PointCloud &point_cloud, const msg::Point &veh_pos) override;

    void Visualize() const override;

    void Reset() override;

    virtual void ResetInternalCellState(int x, int y) override;

    std::string GetDescription() const override;

protected:
    std::vector<std::vector<Cell>> &cells_without_clearing_;
    std::vector<std::vector<Cell>> cells_with_clearing_;
    std::vector<std::vector<bool>> occupancy_delta_;
    utils::vec2 last_position_;
    std::shared_ptr<node::Publisher<msg::OccupancyGrid>> occupancy_delta_publisher_;
};

}

#endif //AVT_341_RAYTRACE_CLEARING_METHOD_H
