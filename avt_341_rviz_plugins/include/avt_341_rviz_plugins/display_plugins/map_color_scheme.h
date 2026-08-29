#ifndef MAP_COLOR_SCHEME_H
#define MAP_COLOR_SCHEME_H

#include <vector>

#include <QString>
#include <QStringList>

namespace avt_341 {
namespace rviz_plugins {

/// One RGBA color, 0-255 per channel.
struct SchemeColor
{
    unsigned char r = 0;
    unsigned char g = 0;
    unsigned char b = 0;
    unsigned char a = 0;
};

/// A named occupancy-value -> color mapping for the AugmentedMap display, loaded
/// from a YAML definition file (see resources/color_schemes.yaml for the format).
///
/// The mapping is stored as the same 256-entry RGBA lookup table the built-in Map
/// display uploads as a palette texture, indexed by the occupancy byte reinterpreted
/// as unsigned: 0-100 are the legal occupancy/cost/class values, 101-127 illegal
/// positive values, 128-254 illegal negatives (-128..-2) and 255 the legal
/// "unknown" value -1.
struct MapColorScheme
{
    QString name;

    /// The 256-entry RGBA lookup table (256 * 4 bytes).
    std::vector<unsigned char> palette;

    /// Binary-view colors for legal values below / at-or-above the display's
    /// "Binary Threshold" property (indices 101-255 mirror `palette`).
    SchemeColor binary_below; ///< Default: fully transparent.
    SchemeColor binary_above; ///< Default: the palette color at value 100.

    /// True when the corresponding lookup table contains an entry with alpha < 255;
    /// the display must then keep alpha blending on even at global Alpha = 1.
    bool translucent = false;
    bool binary_translucent = false;

    /// Build the 256-entry binary-view table for `threshold` in [0, 100]: legal
    /// values below the threshold get binary_below, values at or above it get
    /// binary_above (the same >= convention as the built-in schemes), and the
    /// unknown/illegal entries are copied from `palette`.
    std::vector<unsigned char> BuildBinaryPalette( int threshold ) const;
};

/// Result of LoadMapColorSchemes. On a fatal error (unreadable / structurally
/// invalid file) `schemes` is empty and `error` is set. Individually invalid
/// schemes are skipped with a message appended to `warnings`.
struct MapColorSchemeLoadResult
{
    std::vector<MapColorScheme> schemes;
    QStringList warnings;
    QString error;
};

/// Parse a color-scheme YAML file. Scheme names matching `reserved_names` (the
/// built-in schemes) or repeated within the file are rejected, case-insensitively.
MapColorSchemeLoadResult LoadMapColorSchemes(
    const QString& path, const QStringList& reserved_names );

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MAP_COLOR_SCHEME_H
