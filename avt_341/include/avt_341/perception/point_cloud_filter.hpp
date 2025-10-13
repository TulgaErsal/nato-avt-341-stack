#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include "avt_341/node/ros_types.h"

namespace avt_341::perception {

class PointCloudFilter {

public:

    void SetOrigin(const msg::Pose & origin) { origin_ = origin; }
    void Filter(const std::vector<msg::Point32> & points_in, std::vector<msg::Point32> & points_out);
    bool IsValid(const msg::Point32 & point) const;

private:

    msg::Pose origin_;

};

}

#endif //POINT_CLOUD_FILTER_H
