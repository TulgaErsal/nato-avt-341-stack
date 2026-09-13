#ifndef AUGMENTED_MAP_DISPLAY_H
#define AUGMENTED_MAP_DISPLAY_H

#ifndef Q_MOC_RUN
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include <OgreCommon.h>

#include <nav_msgs/msg/map_meta_data.hpp>

#include <rviz_default_plugins/displays/map/map_display.hpp>

#include <avt_341_rviz_plugins/display_plugins/map_color_scheme.h>
#endif

namespace rviz_common {
namespace properties {
class BoolProperty;
class ColorProperty;
class EnumProperty;
class FilePickerProperty;
class FloatProperty;
class IntProperty;
} // namespace properties
} // namespace rviz_common

namespace avt_341 {
namespace rviz_plugins {

class MapChangeOverlay;

/// The built-in Map display (rviz_default_plugins/Map) extended with custom color
/// schemes for nav_msgs/OccupancyGrid, loaded from a YAML definition file (see
/// resources/color_schemes.yaml for the format), and with a change highlight that
/// flashes cells whose value changed in an incremental {topic}_updates message.
/// Everything else - swatching of huge maps, the updates subscription, alpha /
/// draw-behind - is inherited unchanged, and the built-in map/costmap/raw schemes
/// stay available.
///
/// The base display keeps its schemes as parallel palette-texture vectors indexed
/// by the "Color Scheme" enum's option int; custom schemes are appended to those
/// (protected) vectors after the three built-ins. One base quirk is compensated:
/// the base decides alpha blending purely from the global Alpha property, so a
/// translucent scheme (any LUT entry with alpha < 255) forces blending on the
/// swatches even at Alpha = 1.
///
/// The change highlight keeps a shadow copy of the map bytes and, on every
/// mapUpdated, diffs the base's current_map_ against it (the base does not expose
/// which cells an update touched). Changed cells get a timestamp and their
/// bounding box is queued as a "burst"; each frame the cells inside live bursts
/// are re-shaded from their age (hold at peak, then ease-out) into a
/// MapChangeOverlay quad drawn just above the swatches. With the highlight
/// disabled nothing is diffed, allocated or updated per frame.
///
/// The base class's Binary view is intentionally not supported (a two-stop
/// gradient scheme expresses the same thresholded look): where the feature exists
/// (rviz >= 14, i.e. Jazzy; guarded by AVT341_RVIZ_HAS_BINARY_MAP_VIEW from
/// CMake), its properties are hidden and each custom scheme's normal palette is
/// aliased into the binary texture vector, keeping the base updatePalette()
/// indexing in bounds should a config still enable it. Pre-binary rviz (Humble's
/// 11.x) compiles with the guarded code omitted.
class AugmentedMapDisplay : public rviz_default_plugins::displays::MapDisplay
{
    Q_OBJECT

public:
    AugmentedMapDisplay();
    ~AugmentedMapDisplay() override;

    void onInitialize() override;
    void reset() override;

protected:
    void update( float wall_dt, float ros_dt ) override;

private Q_SLOTS:
    /// (Re)load the scheme file: drop custom palettes, parse, rebuild the dropdown.
    void reloadSchemes();
    /// Force alpha blending on the swatches when the selected scheme is translucent
    /// and the global Alpha is 1 (where the base leaves the material opaque).
    void applySchemeTransparency();
    /// Diff the new map against the shadow copy and queue a highlight burst.
    void onMapUpdated();
    /// Highlight checkbox toggled: start tracking from the current map, or tear
    /// the highlight state down entirely.
    void updateHighlightEnabled();
    /// Color / alpha / draw-behind changed: push to the overlay.
    void updateHighlightAppearance();

private:
    struct Burst
    {
        Ogre::Box box;
        std::uint32_t start_ms;
    };

    /// The scheme file to load: the property's path (plain, file:// or package://)
    /// or, when empty, the package's default resources/color_schemes.yaml. Empty on
    /// resolution failure.
    QString resolveSchemeFilePath() const;
    /// Release the custom palette textures and truncate the palette/flag vectors
    /// back to the three built-in schemes.
    void removeCustomPaletteTextures();

    /// True when the base holds a valid, swatched map.
    bool mapIsValid() const;
    /// Take a fresh shadow of current_map_ (no highlight) and size the overlay.
    void resyncHighlightState();
    /// Drop all highlight state and free the overlay texture.
    void clearHighlightState();
    /// Re-shade the live bursts from their cell ages and drop expired ones.
    void advanceHighlight();
    /// Shade one box of cells into the staging buffer and upload it.
    void uploadBox( const Ogre::Box& box, std::uint32_t now_ms );
    std::uint32_t nowMs() const;

    rviz_common::properties::FilePickerProperty* scheme_file_ = nullptr;

    std::vector<MapColorScheme> custom_schemes_;

    /// Per-scheme "LUT contains alpha < 255" flags, index-aligned with
    /// palette_textures_ (built-ins at 0-2).
    std::vector<bool> scheme_translucent_;

    bool initialized_ = false; ///< Palettes need Ogre; gate slots until onInitialize.

    rviz_common::properties::BoolProperty* highlight_enabled_ = nullptr;
    rviz_common::properties::ColorProperty* highlight_color_ = nullptr;
    rviz_common::properties::FloatProperty* highlight_alpha_ = nullptr;
    rviz_common::properties::FloatProperty* highlight_hold_ = nullptr;
    rviz_common::properties::FloatProperty* highlight_fade_ = nullptr;
    rviz_common::properties::EnumProperty* highlight_source_ = nullptr;
    rviz_common::properties::IntProperty* highlight_min_change_ = nullptr;

    std::unique_ptr<MapChangeOverlay> overlay_;
    nav_msgs::msg::MapMetaData shadow_info_;
    std::vector<std::int8_t> shadow_data_;
    std::vector<std::uint32_t> changed_at_ms_; ///< 0 = never changed.
    std::vector<std::uint8_t> staging_;
    std::deque<Burst> bursts_;
    std::chrono::steady_clock::time_point epoch_;
    std::uint32_t last_update_count_ = 0;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // AUGMENTED_MAP_DISPLAY_H
