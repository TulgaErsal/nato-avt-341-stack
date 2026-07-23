#ifndef AVT_341_TIME_CLEARING_METHOD_H
#define AVT_341_TIME_CLEARING_METHOD_H

#include "costmap_clearing_method.h"

namespace avt_341::perception{

struct TimedNoObsData {
    double obs_time;
    int num_samples;
    float last_scan_pos_x;
    float last_scan_pos_y;

    TimedNoObsData() {
        Reset();
    }

    inline void Reset() {
        Reset(std::numeric_limits<float>::max());
    }

    inline void Reset(const double obs_time_in) {
        obs_time = obs_time_in;
        num_samples = 0;
        last_scan_pos_x = 0.0;
        last_scan_pos_y = 0.0;
    }
};

using TimedNoObsClearingSettings = ClearMethodSettings;

class TimedClearingMethod: public OccupancyClearingMethod {

public:

    TimedClearingMethod(
        float max_point_age,
        std::vector< std::vector<Cell>> & cells,
        const PerceptionSettings & settings,
        CellObstacleCalculator* obs_calculator
        );

    void ClearOccupancy(const msg::PointCloud &point_cloud) override;
    void AgeCells(const float dt) const;
    std::string GetDescription() const override;

private:
    float max_point_age_;
    double last_timestamp_ = -1.0;
};


class TimedNoObsClearingMethod: public OccupancyClearingMethod {

public:

    TimedNoObsClearingMethod(
        std::vector< std::vector<Cell>> & cells,
        const PerceptionSettings & settings,
        const TimedNoObsClearingSettings & time_config,
        CellObstacleCalculator* obs_calculator
        );

    void ClearOccupancy(const msg::PointCloud &point_cloud) override;
    void OnOccupancyAdded(const msg::PointCloud &point_cloud, const msg::Point & veh_pos) override;
    void Reset() override;
    std::string GetDescription() const override;
    virtual void ResetInternalCellState(int xi, int yi) override;

private:
    std::vector< std::vector<Cell>> timed_cells_;
    std::vector< std::vector<TimedNoObsData>> timed_cells_data;
    TimedNoObsClearingSettings time_config_;
};

}

#endif //AVT_341_TIME_CLEARING_METHOD_H
