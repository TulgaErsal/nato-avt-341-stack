#ifndef AVT_341_CORE_FRAME_ID_COLLECTION_HPP
#define AVT_341_CORE_FRAME_ID_COLLECTION_HPP

#include <string>

#include <avt_341_nav/coordinate_frames_mixin_params_dto.hpp>

#include "avt_341_nav/core/string_utils.hpp"

namespace avt_341_nav::core
{

/**
 * @brief Resolved tf frame ids from the coordinate frames parameters. The
 * camera, lidar, base_link and cg frames are prefixed with the vehicle id
 * when frames.use_vehicle_prefix is set. Frame ids are computed once at
 * construction.
 */
class FrameIdCollection
{
public:
    FrameIdCollection(const params::core::Frames& frames, const std::string& vehicle_id)
        : camera_(Resolve(frames, vehicle_id, frames.camera)),
          lidar_(Resolve(frames, vehicle_id, frames.lidar)),
          base_link_(Resolve(frames, vehicle_id, frames.base_link)),
          cg_(Resolve(frames, vehicle_id, frames.cg)),
          map_(frames.map),
          odom_(frames.odom),
          gis_frame_(CrsToFrameId(frames.gis.crs))
    {}

    const std::string& Camera() const { return camera_; }
    const std::string& Lidar() const { return lidar_; }
    const std::string& BaseLink() const { return base_link_; }
    const std::string& Cg() const { return cg_; }
    const std::string& Map() const { return map_; }
    const std::string& Odom() const { return odom_; }
    /// Frame id derived from the GIS crs, e.g. "EPSG:6495" -> "epsg_6495".
    const std::string& GisFrame() const { return gis_frame_; }

private:
    static std::string Resolve(const params::core::Frames& frames,
                               const std::string& vehicle_id,
                               const std::string& frame)
    {
        return frames.use_vehicle_prefix ? CombineTfParts(vehicle_id, frame) : frame;
    }

    std::string camera_;
    std::string lidar_;
    std::string base_link_;
    std::string cg_;
    std::string map_;
    std::string odom_;
    std::string gis_frame_;
};

}

#endif // AVT_341_CORE_FRAME_ID_COLLECTION_HPP
