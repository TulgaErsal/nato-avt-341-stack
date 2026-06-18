#ifndef AVT_341_COSTMAP_CLEARING_METHOD_H
#define AVT_341_COSTMAP_CLEARING_METHOD_H

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/costmap_dtos.h"

namespace avt_341::perception {

struct ClearMethodRosParameters {
    std::string clear_methods_str;

    // Visualization
    float visualization_range;
    bool visualize;

    // Raytracing
    float raytrace_range;
    bool use_voxels;
    float voxel_height_min;
    float voxel_height_res;
    bool immediate_clr_dilation;
    bool clr_on_scan_below_only;
    std::string lidar_frame;

    // Raytracing with object filter
    float obj_range_filter;

    // Timed no-obs clearing
    int sampled_threshold;
    float max_point_age;
    float no_obs_dist_threshold;

    // Clear by channel
    std::string channel_to_clear;
    float channel_threshold;
};

struct CostmapClearMethodType {
public:
    const static std::string None;
    const static std::string Time;
    const static std::string Raytrace;
    const static std::string RaytraceWithFiltering;
    const static std::string NoObsTime;
    const static std::string ChannelThreshold;
};

struct BaseClearingSettings {
    float llx;
    float lly;
    float res;
    int grid_dilate_x;
    int grid_dilate_y;
    float thresh;
    bool immediate_clear_dilation;
    float visualization_range;
    bool visualize;
};

class OccupancyClearingMethod {

public:

    OccupancyClearingMethod(
        std::vector<std::vector<Cell>> & cells,
        const BaseClearingSettings &config,
        CellObstacleCalculator *obs_calculator
        );

    OccupancyClearingMethod(
        std::vector<std::vector<Cell>> & costmap_cells,
        int Ny,
        int Nx,
        const BaseClearingSettings &config,
        CellObstacleCalculator *obs_calculator
        );

    virtual ~OccupancyClearingMethod() = default;

    /**
    * Clear occupancy for the given point cloud.
    * @param point_cloud Point cloud being processed.
    */
    virtual void ClearOccupancy(const msg::PointCloud &point_cloud) = 0;

    /**
    * Invokes any visualization that clearing method may have.
    */
    virtual void Visualize() const;

    /**
     * Called after occupancy has beed added to the costmap for the input point cloud.
     * @param point_cloud Point cloud that has been processed.
     */
    virtual void OnOccupancyAdded(const msg::PointCloud &point_cloud, const msg::Point &veh_pos);

    /**
     * Resets the clearing method's local state.
     */
    virtual void Reset();

    /**
     * Set sibling clearing methods that may also clear cells in the same costmap.
     */
    void SetSiblingClearingMethods(const std::vector<std::shared_ptr<OccupancyClearingMethod>> &sibling_cms);

    /**
     * Called when a cell is cleared externally (typically by another clearing method).
     */
    virtual void ResetInternalCellState(int x, int y);

    /**
     * String description of clearing method.
     */
    virtual std::string GetDescription() const = 0;

protected:

    void RemoveDilationAtCell(int x, int y) const;
    void RemoveDilationAtCell(int x, int y, std::vector<std::vector<Cell>> &cells) const;

    void BroadcastClearToSiblings(int x, int y);

    std::vector<std::shared_ptr<OccupancyClearingMethod>> sibling_cms_;
    std::vector<std::vector<Cell>> &cells_;
    int Ny_;
    int Nx_;
    BaseClearingSettings config_;
    CellObstacleCalculator *cell_obstacle_calculator_;
};

class NullClearingMethod : public OccupancyClearingMethod {

public:

    NullClearingMethod(
        std::vector<std::vector<Cell> > &cells,
        const BaseClearingSettings &config,
        CellObstacleCalculator *obs_calculator
        );

    void ClearOccupancy(const msg::PointCloud &point_cloud) override;

    std::string GetDescription() const override;
};

}

#endif //AVT_341_COSTMAP_CLEARING_METHOD_H
