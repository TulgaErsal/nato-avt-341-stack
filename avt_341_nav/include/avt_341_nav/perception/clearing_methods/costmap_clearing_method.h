#ifndef AVT_341_COSTMAP_CLEARING_METHOD_H
#define AVT_341_COSTMAP_CLEARING_METHOD_H

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "avt_341_nav/perception/costmap_dtos.h"
#include "avt_341_nav/perception/perception_settings.hpp"

namespace avt_341_nav::perception {

struct CostmapClearMethodType {
public:
    const static std::string None;
    const static std::string Time;
    const static std::string Raytrace;
    const static std::string RaytraceWithFiltering;
    const static std::string NoObsTime;
    const static std::string ChannelThreshold;
};

class OccupancyClearingMethod {

public:

    OccupancyClearingMethod(
        std::vector<std::vector<Cell>> & cells,
        const PerceptionSettings &settings,
        CellObstacleCalculator *obs_calculator
        );

    OccupancyClearingMethod(
        std::vector<std::vector<Cell>> & costmap_cells,
        int Ny,
        int Nx,
        const PerceptionSettings &settings,
        CellObstacleCalculator *obs_calculator
        );

    virtual ~OccupancyClearingMethod() = default;

    /**
    * Clear occupancy for the given point cloud.
    * @param point_cloud Point cloud being processed.
    */
    virtual void ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) = 0;

    /**
    * Invokes any visualization that clearing method may have.
    */
    virtual void Visualize() const;

    /**
     * Called after occupancy has beed added to the costmap for the input point cloud.
     * @param point_cloud Point cloud that has been processed.
     */
    virtual void OnOccupancyAdded(const sensor_msgs::msg::PointCloud &point_cloud, const geometry_msgs::msg::Point &veh_pos);

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
    PerceptionSettings settings_;
    CellObstacleCalculator *cell_obstacle_calculator_;
};

class NullClearingMethod : public OccupancyClearingMethod {

public:

    NullClearingMethod(
        std::vector<std::vector<Cell> > &cells,
        const PerceptionSettings &settings,
        CellObstacleCalculator *obs_calculator
        );

    void ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) override;

    std::string GetDescription() const override;
};

}

#endif //AVT_341_COSTMAP_CLEARING_METHOD_H
