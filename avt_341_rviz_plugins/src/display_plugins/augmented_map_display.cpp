#include <avt_341_rviz_plugins/display_plugins/augmented_map_display.h>

#include <atomic>
#include <cstdint>
#include <string>

#include <QString>
#include <QUrl>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <OgreBlendMode.h>
#include <OgreDataStream.h>
#include <OgrePixelFormat.h>
#include <OgreResourceGroupManager.h>
#include <OgreTextureManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/file_picker_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_default_plugins/displays/map/swatch.hpp>
#include <rviz_rendering/material_manager.hpp>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::FilePickerProperty;
using rviz_common::properties::Property;
using rviz_common::properties::StatusProperty;

namespace
{
/// The base display always registers map / costmap / raw at option ints 0-2;
/// custom schemes are appended from this index on.
constexpr int kBuiltinSchemeCount = 3;

const QStringList kBuiltinSchemeNames = { "map", "costmap", "raw" };

/// Monotonic counter for globally unique Ogre texture names (same convention as
/// the other avt_341 primitives).
std::atomic<std::uint64_t> g_palette_counter{ 0 };

/// Upload a 256-entry RGBA lookup table as the 256x1 1D palette texture the map
/// material samples with the occupancy byte (mirrors the base display's private
/// makePaletteTexture, which is not exported).
Ogre::TexturePtr makePaletteTexture( const std::vector<unsigned char>& bytes )
{
    // loadRawData() copies the bytes into the GPU texture synchronously, so the
    // stream must not own or free the caller's buffer.
    Ogre::DataStreamPtr stream(
        new Ogre::MemoryDataStream(
            const_cast<unsigned char*>( bytes.data() ), 256 * 4, false, true ) );
    const std::string id = std::to_string( g_palette_counter++ );
    return Ogre::TextureManager::getSingleton().loadRawData(
        "avt_341_augmented_map_palette_" + id,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, stream,
        256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0 );
}

/// Resolve a user-entered scheme file location to an absolute filesystem path.
/// Accepts a plain path, a file:// URL, or a package://<pkg>/<path> resource URL
/// (the same forms StaticMapImageDisplay accepts for its image file). Returns an
/// empty string when a package:// URL is malformed or its package is unknown.
QString resolveResourcePath( const QString& raw )
{
    const QString path = raw.trimmed();

    const QString kPackage = "package://";
    if ( path.startsWith( kPackage ) )
    {
        const QString rest = path.mid( kPackage.length() );
        const int slash = rest.indexOf( '/' );
        if ( slash <= 0 )
        {
            return QString(); // missing package name or relative path
        }
        const std::string pkg = rest.left( slash ).toStdString();
        const QString rel = rest.mid( slash + 1 );
        try
        {
            const std::string share = ament_index_cpp::get_package_share_directory( pkg );
            return QString::fromStdString( share ) + "/" + rel;
        }
        catch ( const std::exception& )
        {
            return QString(); // package not found
        }
    }

    if ( path.startsWith( "file:" ) )
    {
        return QUrl( path ).toLocalFile();
    }

    return path;
}
} // namespace

AugmentedMapDisplay::AugmentedMapDisplay()
{
    scheme_file_ = new FilePickerProperty(
        "Color Scheme File", "",
        "YAML file defining extra color schemes for the Color Scheme dropdown. "
        "Accepts a filesystem path, a file:// URL, or a package://<pkg>/<path> "
        "resource URL. Leave empty to use this package's default "
        "resources/color_schemes.yaml (which also documents the format).",
        this, SLOT( reloadSchemes() ), this );

    // Whether the built-in schemes' palettes contain translucent entries (costmap
    // and raw do), and likewise for their binary variants (all opaque).
    scheme_translucent_ = { false, true, true };
    scheme_translucent_binary_ = { false, false, false };

    // Base handlers for these signals were connected first (in the base
    // constructor) and therefore run first; the hooks below then compensate on
    // top of the state they left behind.
    connect(
        binary_threshold_property_, &Property::changed,
        this, &AugmentedMapDisplay::rebuildCustomBinaryPalettes );
    connect(
        color_scheme_property_, &Property::changed,
        this, &AugmentedMapDisplay::applySchemeTransparency );
    connect(
        alpha_property_, &Property::changed,
        this, &AugmentedMapDisplay::applySchemeTransparency );
    connect(
        draw_under_property_, &Property::changed,
        this, &AugmentedMapDisplay::applySchemeTransparency );
    connect(
        binary_view_property_, &Property::changed,
        this, &AugmentedMapDisplay::applySchemeTransparency );
    // Fires after the base's showMap() slot, i.e. after swatches are (re)created
    // and the base has applied palette / alpha / draw-under state to them.
    connect(
        this, &AugmentedMapDisplay::mapUpdated,
        this, &AugmentedMapDisplay::applySchemeTransparency );
}

AugmentedMapDisplay::~AugmentedMapDisplay()
{
    if ( initialized_ )
    {
        removeCustomPaletteTextures();
    }
}

void AugmentedMapDisplay::onInitialize()
{
    MapDisplay::onInitialize(); // creates the built-in palette textures (0-2)
    initialized_ = true;
    reloadSchemes();
}

void AugmentedMapDisplay::reloadSchemes()
{
    if ( !initialized_ )
    {
        return; // onInitialize() will load
    }

    removeCustomPaletteTextures();
    custom_schemes_.clear();

    const QString path = resolveSchemeFilePath();
    MapColorSchemeLoadResult result;
    if ( path.isEmpty() )
    {
        result.error =
            "Could not resolve path (malformed URL or unknown package): " +
            scheme_file_->getString();
    }
    else
    {
        result = LoadMapColorSchemes( path, kBuiltinSchemeNames );
    }

    const int threshold = binary_threshold_property_->getInt();
    for ( auto& scheme : result.schemes )
    {
        palette_textures_.push_back( makePaletteTexture( scheme.palette ) );
        palette_textures_binary_.push_back(
            makePaletteTexture( scheme.BuildBinaryPalette( threshold ) ) );
        // Vestigial in Jazzy (written, never read) but kept index-aligned in case
        // a future rviz release reads it again.
        color_scheme_transparency_.push_back( scheme.translucent );
        scheme_translucent_.push_back( scheme.translucent );
        scheme_translucent_binary_.push_back( scheme.binary_translucent );
        custom_schemes_.push_back( std::move( scheme ) );
    }

    // Rebuild the dropdown: built-ins first (option ints must match the palette
    // vector indices), then the customs, preserving the current selection when it
    // still exists.
    const QString selected = color_scheme_property_->getString();
    color_scheme_property_->clearOptions();
    for ( int i = 0; i < kBuiltinSchemeCount; i++ )
    {
        color_scheme_property_->addOption( kBuiltinSchemeNames[i], i );
    }
    bool selection_exists = kBuiltinSchemeNames.contains( selected );
    for ( std::size_t i = 0; i < custom_schemes_.size(); i++ )
    {
        color_scheme_property_->addOption(
            custom_schemes_[i].name, kBuiltinSchemeCount + static_cast<int>( i ) );
        selection_exists |= custom_schemes_[i].name == selected;
    }

    if ( !selection_exists )
    {
        color_scheme_property_->setString( "map" ); // triggers updatePalette()
    }
    else
    {
        updatePalette(); // re-point the swatches at the rebuilt textures
    }
    applySchemeTransparency();

    if ( !result.error.isEmpty() )
    {
        setStatus(
            StatusProperty::Error, "Color Schemes",
            result.error + " (only the built-in schemes are available)" );
    }
    else if ( !result.warnings.isEmpty() )
    {
        setStatus(
            StatusProperty::Warn, "Color Schemes",
            QString( "Loaded %1 scheme(s) from %2; %3" )
                .arg( custom_schemes_.size() ).arg( path, result.warnings.join( "; " ) ) );
    }
    else
    {
        setStatus(
            StatusProperty::Ok, "Color Schemes",
            QString( "Loaded %1 scheme(s) from %2" ).arg( custom_schemes_.size() ).arg( path ) );
    }
}

void AugmentedMapDisplay::rebuildCustomBinaryPalettes()
{
    if ( !initialized_ )
    {
        return;
    }

    const int threshold = binary_threshold_property_->getInt();
    auto& texture_manager = Ogre::TextureManager::getSingleton();
    for ( std::size_t i = 0; i < custom_schemes_.size(); i++ )
    {
        auto& slot = palette_textures_binary_[kBuiltinSchemeCount + i];
        texture_manager.remove( slot );
        slot = makePaletteTexture( custom_schemes_[i].BuildBinaryPalette( threshold ) );
    }
    // The base slot (which ran first) only swapped texture objects inside the
    // vectors; re-point the swatches at the current ones.
    updatePalette();
    applySchemeTransparency();
}

void AugmentedMapDisplay::applySchemeTransparency()
{
    if ( !initialized_ || swatches_.empty() )
    {
        return;
    }
    const float alpha = alpha_property_->getFloat();
    if ( alpha < rviz_rendering::unit_alpha_threshold )
    {
        return; // base already chose alpha blending; per-texel alpha works
    }
    const int index = color_scheme_property_->getOptionInt();
    const auto& flags =
        binary_view_property_->getBool() ? scheme_translucent_binary_ : scheme_translucent_;
    if ( index < 0 || index >= static_cast<int>( flags.size() ) || !flags[index] )
    {
        return; // opaque scheme: the base's opaque material state is correct
    }
    for ( const auto& swatch : swatches_ )
    {
        swatch->updateAlpha( Ogre::SBT_TRANSPARENT_ALPHA, false, alpha );
    }
    context_->queueRender();
}

QString AugmentedMapDisplay::resolveSchemeFilePath() const
{
    const QString raw = scheme_file_->getString().trimmed();
    if ( !raw.isEmpty() )
    {
        return resolveResourcePath( raw );
    }
    try
    {
        const std::string share =
            ament_index_cpp::get_package_share_directory( "avt_341_rviz_plugins" );
        return QString::fromStdString( share ) + "/resources/color_schemes.yaml";
    }
    catch ( const std::exception& )
    {
        return QString();
    }
}

void AugmentedMapDisplay::removeCustomPaletteTextures()
{
    // Remove by pointer, not by name: the name-based overload defaults to the
    // "General" group and asserts if the resource isn't found there. Swatches
    // still referencing a removed texture keep it alive via their shared pointer
    // until updatePalette() re-points them.
    auto& texture_manager = Ogre::TextureManager::getSingleton();
    for ( std::size_t i = kBuiltinSchemeCount; i < palette_textures_.size(); i++ )
    {
        texture_manager.remove( palette_textures_[i] );
    }
    for ( std::size_t i = kBuiltinSchemeCount; i < palette_textures_binary_.size(); i++ )
    {
        texture_manager.remove( palette_textures_binary_[i] );
    }
    palette_textures_.resize( kBuiltinSchemeCount );
    palette_textures_binary_.resize( kBuiltinSchemeCount );
    color_scheme_transparency_.resize( kBuiltinSchemeCount );
    scheme_translucent_.resize( kBuiltinSchemeCount );
    scheme_translucent_binary_.resize( kBuiltinSchemeCount );
}

} // namespace avt_341::rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::AugmentedMapDisplay, rviz_common::Display )
