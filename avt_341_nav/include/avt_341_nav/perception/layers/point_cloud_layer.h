
#ifndef AVT_341_POINT_CLOUD_LAYER_H
#define AVT_341_POINT_CLOUD_LAYER_H
#include "costmap_layer.h"
#include "avt_341_nav/node/tf_interface.h"
#include "avt_341_nav/perception/point_cloud_filter.hpp"
#include "avt_341_nav/perception/clearing_methods/costmap_clearing_method.h"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/rclcpp.hpp>

namespace avt_341_nav::perception
{

    class PointCloudLayer : public CostmapLayer, public CellObstacleCalculator
    {

    public:

        PointCloudLayer(
            const rclcpp::Node::SharedPtr& node_ref,
            const std::shared_ptr<node::TfInterface>& tf,
            const PerceptionSettings& settings,
            const std::string & label,
            const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
            const std::string& point_cloud_topic,
            const std::string& clear_only_points_topic,
            bool contribute_occupancy,
            bool contribute_segmentation,
            bool setup_point_cloud_subscriptions = true
            );

        void SetupPointCloudFilter(
            const PointCloudFilterConfig& point_cloud_config,
            const PointCloudFilterConfig& clearing_config);

        /**
        * Add points to be processed
        * \param point_cloud PointCloud message
        */
        void ProcessPoints(const std::shared_ptr<sensor_msgs::msg::PointCloud>& pc_ptr, const geometry_msgs::msg::Pose& vehicle_pose, bool clear_only = false);

        /**
        * Clear points in point cloud
        * \param point_cloud PointCloud message
        */
        void ClearPoints(const std::shared_ptr<sensor_msgs::msg::PointCloud>& pc_ptr, const geometry_msgs::msg::Pose& vehicle_pose);

        void AddOccupancy(const sensor_msgs::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) override;

        void SetupGridClearingMethod(
            const ClearMethodSettings& clear_method_params);

        void Visualize() override;

        void Reset() override;

        bool PastSlopeThreshold(const Cell& cell) const override { return CostmapLayer::PastSlopeThreshold(cell); }
        float Slope(const Cell& cell) const override { return CostmapLayer::Slope(cell); }

        void SetupPcSubscriptions(
            const std::string& point_cloud_topic,
            const std::string& clear_only_points_topic);

        std::string ToString() const override;

    protected:
        std::string pc_seg_channel_;
        void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr rcv_cloud);

    private:
        void ClearOnlyPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr rcv_cloud);
        std::shared_ptr<sensor_msgs::msg::PointCloud> RegisterPc2Msg(const sensor_msgs::msg::PointCloud2::SharedPtr & rcv_cloud);

        PointCloudFilter pc_filter;						// Filter for input point clouds
        PointCloudFilter pc_cm_filter;					// Additional filter for clearing methods applied after regular filter
	    std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;
        std::string pc_section_id_;
        std::shared_ptr<sensor_msgs::msg::PointCloud> clr_only_pc_ = nullptr;
        std::string pc_topic_id_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_ground_sub_;
        std::shared_ptr<node::TfInterface> tf_;

        static PointCloudFilterConfig ParseFilterConfig(
            const GeneratedPerceptionParams::PointCloudLayer::Filter& params);
        static PointCloudFilterConfig ParseFilterConfig(
            const GeneratedPerceptionParams::ClearMethod::Filter& params);
    };

}

#endif //AVT_341_POINT_CLOUD_LAYER_H
