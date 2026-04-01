
#ifndef AVT_341_POINT_CLOUD_LAYER_H
#define AVT_341_POINT_CLOUD_LAYER_H
#include "costmap_layer.h"
#include "avt_341/perception/point_cloud_filter.hpp"
#include "avt_341/perception/clearing_methods/costmap_clearing_method.h"

namespace avt_341::perception
{

    class PointCloudLayer : public CostmapLayer
    {

    public:
        /**
        * Set point cloud filtering configuration
        * \param filter_pc_config Configuration for normal occupancy addition point cloud.
        * \param filter_pc_cm_config Configuration for additional point cloud filtering of costmap clearing methods.
        */
        void SetPointCloudFilterConfig(
            const PointCloudFilterConfig& filter_pc_config,
            const PointCloudFilterConfig& filter_pc_cm_config);

        /**
        * Add points to be processed
        * \param point_cloud PointCloud message
        */
        void ProcessPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose, bool clear_only = false);

        /**
        * Clear points in point cloud
        * \param point_cloud PointCloud message
        */
        void ClearPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose);

        void AddOccupancy(const msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) override;

        void SetGridClearingMethod(const ClearMethodRosParameters & params);

        void VisualizeClearMethods() const {
            for (auto& cm : clear_methods_) {
                cm->Visualize();
            }
        }

    private:
        PointCloudFilter pc_filter;						// Filter for input point clouds
        PointCloudFilter pc_cm_filter;					// Additional filter for clearing methods applied after regular filter
	    std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;
    };

}

#endif //AVT_341_POINT_CLOUD_LAYER_H
