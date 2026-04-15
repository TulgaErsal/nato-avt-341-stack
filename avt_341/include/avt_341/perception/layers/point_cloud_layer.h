
#ifndef AVT_341_POINT_CLOUD_LAYER_H
#define AVT_341_POINT_CLOUD_LAYER_H
#include "costmap_layer.h"
#include "avt_341/core/monitoring.hpp"
#include "avt_341/perception/point_cloud_filter.hpp"
#include "avt_341/perception/clearing_methods/costmap_clearing_method.h"

namespace avt_341::perception
{

    class PointCloudLayer : public CostmapLayer, public CellObstacleCalculator
    {

    public:

        PointCloudLayer(
            const std::shared_ptr<node::NodeProxy>& node_ref,
            const CostmapSettings& cm_settings,
            const std::string & label
            );

        void SetupPointCloudFilter();

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

        void SetupGridClearingMethod();

        void Visualize() override;

        void Reset() override;

        bool PastSlopeThreshold(const Cell& cell) const override { return CostmapLayer::PastSlopeThreshold(cell); }
        float Slope(const Cell& cell) const override { return CostmapLayer::Slope(cell); }

        void SetupPcSubscriptions();

        std::string ToString() const override;

    protected:
        std::string pc_seg_channel_;
        void PointCloudCallback(msg::PointCloud2Ptr rcv_cloud);

    private:
        double pc_callback_warn_dur_ = 0.0; // in seconds
        void ClearOnlyPointsCallback(msg::PointCloud2Ptr rcv_cloud);
        std::shared_ptr<msg::PointCloud> RegisterPc2Msg(const msg::PointCloud2Ptr & rcv_cloud);

        PointCloudFilter pc_filter;						// Filter for input point clouds
        PointCloudFilter pc_cm_filter;					// Additional filter for clearing methods applied after regular filter
	    std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;
        core::WindowedMean pc_callback_time_;
        std::shared_ptr<msg::PointCloud> clr_only_pc_ = nullptr;
        std::string pc_topic_id_;

        node::Subscriber<msg::PointCloud2>::SharedPtr pc_sub_;
        node::Subscriber<msg::PointCloud2>::SharedPtr pc_ground_sub_;

        ClearMethodRosParameters ParseClearMethodsConfig() const;
        PointCloudFilterConfig ParseFilterConfig(const std::string &param_prefix = "") const;
    };

}

#endif //AVT_341_POINT_CLOUD_LAYER_H
