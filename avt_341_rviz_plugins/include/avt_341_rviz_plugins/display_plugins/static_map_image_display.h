#ifndef STATIC_MAP_IMAGE_DISPLAY_H
#define STATIC_MAP_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <memory>

#include <QImage>

#include <rviz_common/display.hpp>

#include <avt_341_rviz_plugins/primitives/map_image.h>
#endif

namespace rviz_common {
namespace properties {
class BoolProperty;
class EnumProperty;
class FilePickerProperty;
class FloatProperty;
class IntProperty;
class TfFrameProperty;
} // namespace properties
} // namespace rviz_common

namespace avt_341 {
namespace rviz_plugins {

/// RViz display that renders a static map image (.png/.pgm) read directly from a file
/// path as a full-colour, flat ground decal - no topic, no publishing. The image is
/// placed and scaled in the world using a configurable resolution (m/pixel) and origin
/// pose (x, y, yaw) in a configurable TF frame, the same placement semantics as a
/// nav_msgs/OccupancyGrid map. Width and height are taken from the image.
///
/// Rendering is a plain Ogre textured quad (see MapImage), so the image's RGB is
/// shown as-is - unlike the built-in Map display, which maps occupancy values through a
/// grayscale palette. The quad's node is driven each frame to the origin pose
/// transformed into the fixed frame.
class StaticMapImageDisplay : public rviz_common::Display
{
    Q_OBJECT

public:
    StaticMapImageDisplay();
    ~StaticMapImageDisplay() override;

    void onInitialize() override;
    void update( float wall_dt, float ros_dt ) override;
    void reset() override;

protected:
    void onEnable() override;
    void onDisable() override;

private Q_SLOTS:
    /// The image path changed: re-decode the file and rebuild the quad.
    void reloadImage();
    /// Resolution changed: rebuild the quad from the cached image at the new scale.
    void updateGeometry();
    /// Cheap appearance updates (alpha / draw-behind / interpolation).
    void updateAppearance();

private:
    /// Decode image_file_ into image_. Returns false (setting an "Image" status) when
    /// the path is empty or the file cannot be read.
    bool decodeImage();
    /// Push image_ + resolution into the MapImage and refresh the read-only dims.
    void rebuild();
    /// Transform the origin pose from the chosen frame to the fixed frame and drive
    /// the image node; hide it (with a status) when the transform is unavailable.
    void updateTransform();

    std::unique_ptr<MapImage> map_image_;
    QImage image_; ///< Cached decoded image; re-read only when the path changes.

    // Editable properties.
    rviz_common::properties::FilePickerProperty* image_file_ = nullptr;
    rviz_common::properties::FloatProperty*      resolution_ = nullptr;
    rviz_common::properties::FloatProperty*      origin_x_ = nullptr;
    rviz_common::properties::FloatProperty*      origin_y_ = nullptr;
    rviz_common::properties::FloatProperty*      origin_yaw_ = nullptr;
    rviz_common::properties::TfFrameProperty*    frame_property_ = nullptr;
    rviz_common::properties::FloatProperty*      alpha_ = nullptr;
    rviz_common::properties::BoolProperty*       draw_under_ = nullptr;
    rviz_common::properties::EnumProperty*       interpolation_ = nullptr;

    // Read-only image dimensions (pixels).
    rviz_common::properties::IntProperty* width_ = nullptr;
    rviz_common::properties::IntProperty* height_ = nullptr;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // STATIC_MAP_IMAGE_DISPLAY_H
