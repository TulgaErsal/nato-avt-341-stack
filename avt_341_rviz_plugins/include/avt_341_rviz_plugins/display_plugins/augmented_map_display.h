#ifndef AUGMENTED_MAP_DISPLAY_H
#define AUGMENTED_MAP_DISPLAY_H

#ifndef Q_MOC_RUN
#include <vector>

#include <rviz_default_plugins/displays/map/map_display.hpp>

#include <avt_341_rviz_plugins/display_plugins/map_color_scheme.h>
#endif

namespace rviz_common {
namespace properties {
class FilePickerProperty;
} // namespace properties
} // namespace rviz_common

namespace avt_341 {
namespace rviz_plugins {

/// The built-in Map display (rviz_default_plugins/Map) extended with custom color
/// schemes for nav_msgs/OccupancyGrid, loaded from a YAML definition file (see
/// resources/color_schemes.yaml for the format). Everything else - swatching of
/// huge maps, the {topic}_updates subscription, alpha / draw-behind - is inherited
/// unchanged, and the built-in map/costmap/raw schemes stay available.
///
/// The base display keeps its schemes as parallel palette-texture vectors indexed
/// by the "Color Scheme" enum's option int; custom schemes are appended to those
/// (protected) vectors after the three built-ins. One base quirk is compensated:
/// the base decides alpha blending purely from the global Alpha property, so a
/// translucent scheme (any LUT entry with alpha < 255) forces blending on the
/// swatches even at Alpha = 1.
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

private Q_SLOTS:
    /// (Re)load the scheme file: drop custom palettes, parse, rebuild the dropdown.
    void reloadSchemes();
    /// Force alpha blending on the swatches when the selected scheme is translucent
    /// and the global Alpha is 1 (where the base leaves the material opaque).
    void applySchemeTransparency();

private:
    /// The scheme file to load: the property's path (plain, file:// or package://)
    /// or, when empty, the package's default resources/color_schemes.yaml. Empty on
    /// resolution failure.
    QString resolveSchemeFilePath() const;
    /// Release the custom palette textures and truncate the palette/flag vectors
    /// back to the three built-in schemes.
    void removeCustomPaletteTextures();

    rviz_common::properties::FilePickerProperty* scheme_file_ = nullptr;

    std::vector<MapColorScheme> custom_schemes_;

    /// Per-scheme "LUT contains alpha < 255" flags, index-aligned with
    /// palette_textures_ (built-ins at 0-2).
    std::vector<bool> scheme_translucent_;

    bool initialized_ = false; ///< Palettes need Ogre; gate slots until onInitialize.
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // AUGMENTED_MAP_DISPLAY_H
