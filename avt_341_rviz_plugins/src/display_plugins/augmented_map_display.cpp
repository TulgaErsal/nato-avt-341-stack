#include <avt_341_rviz_plugins/display_plugins/augmented_map_display.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include <QColor>
#include <QString>
#include <QUrl>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <OgreBlendMode.h>
#include <OgreColourValue.h>
#include <OgreDataStream.h>
#include <OgrePixelFormat.h>
#include <OgreResourceGroupManager.h>
#include <OgreTextureManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/file_picker_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_default_plugins/displays/map/swatch.hpp>
#include <rviz_rendering/material_manager.hpp>

#include <avt_341_rviz_plugins/primitives/map_change_overlay.h>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::FilePickerProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
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

constexpr const char* kHighlightStatus = "Change Highlight";

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

bool sameGeometry( const nav_msgs::msg::MapMetaData& a, const nav_msgs::msg::MapMetaData& b )
{
    return a.width == b.width && a.height == b.height && a.resolution == b.resolution &&
        a.origin.position.x == b.origin.position.x &&
        a.origin.position.y == b.origin.position.y &&
        a.origin.position.z == b.origin.position.z &&
        a.origin.orientation.x == b.origin.orientation.x &&
        a.origin.orientation.y == b.origin.orientation.y &&
        a.origin.orientation.z == b.origin.orientation.z &&
        a.origin.orientation.w == b.origin.orientation.w;
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

    highlight_enabled_ = new BoolProperty(
        "Change Highlight", true,
        "Flash the cells whose value changed in an incoming map update, then fade "
        "them out. When unchecked no change tracking or per-frame work is done.",
        this, SLOT( updateHighlightEnabled() ), this );
    highlight_enabled_->setDisableChildrenIfFalse( true );
    highlight_color_ = new ColorProperty(
        "Color", QColor( 255, 255, 255 ), "Color of the highlight.",
        highlight_enabled_, SLOT( updateHighlightAppearance() ), this );
    highlight_alpha_ = new FloatProperty(
        "Peak Alpha", 0.8f, "Opacity of a freshly changed cell.",
        highlight_enabled_, SLOT( updateHighlightAppearance() ), this );
    highlight_alpha_->setMin( 0.0f );
    highlight_alpha_->setMax( 1.0f );
    highlight_hold_ = new FloatProperty(
        "Hold", 0.05f, "Seconds a changed cell stays at peak opacity before fading.",
        highlight_enabled_ );
    highlight_hold_->setMin( 0.0f );
    highlight_fade_ = new FloatProperty(
        "Fade", 0.35f, "Seconds the highlight takes to fade out after the hold.",
        highlight_enabled_ );
    highlight_fade_->setMin( 0.0f );
    highlight_source_ = new EnumProperty(
        "Source", "Updates only",
        "Which messages trigger the highlight: incremental updates on the Update "
        "Topic only, or full maps on the main topic as well.",
        highlight_enabled_ );
    highlight_source_->addOption( "Updates only", 0 );
    highlight_source_->addOption( "Updates and full maps", 1 );
    highlight_min_change_ = new IntProperty(
        "Min Change", 1,
        "Smallest change in a cell's value that counts as a change. A cell "
        "switching to or from unknown (-1) always counts.",
        highlight_enabled_ );
    highlight_min_change_->setMin( 1 );
    highlight_min_change_->setMax( 100 );

    // Whether the built-in schemes' palettes contain translucent entries (costmap
    // and raw do).
    scheme_translucent_ = { false, true, true };

#ifdef AVT341_RVIZ_HAS_BINARY_MAP_VIEW
    // Binary view is intentionally unsupported (a two-stop gradient scheme
    // expresses the same thresholded look); hide its properties so the display
    // does not advertise a toggle that ignores custom schemes.
    binary_view_property_->hide();
    binary_threshold_property_->hide();
#endif

    // Base handlers for these signals were connected first (in the base
    // constructor) and therefore run first; the hooks below then compensate on
    // top of the state they left behind.
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
        draw_under_property_, &Property::changed,
        this, &AugmentedMapDisplay::updateHighlightAppearance );
    // Fires after the base's showMap() slot, i.e. after swatches are (re)created
    // and the base has applied palette / alpha / draw-under state to them.
    connect(
        this, &AugmentedMapDisplay::mapUpdated,
        this, &AugmentedMapDisplay::applySchemeTransparency );
    connect(
        this, &AugmentedMapDisplay::mapUpdated,
        this, &AugmentedMapDisplay::onMapUpdated );
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
    epoch_ = std::chrono::steady_clock::now();
    overlay_ = std::make_unique<MapChangeOverlay>( scene_manager_, scene_node_ );
    updateHighlightAppearance();
    reloadSchemes();
}

void AugmentedMapDisplay::reset()
{
    MapDisplay::reset();
    clearHighlightState();
}

void AugmentedMapDisplay::update( float wall_dt, float ros_dt )
{
    MapDisplay::update( wall_dt, ros_dt );
    if ( !bursts_.empty() )
    {
        advanceHighlight();
    }
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

    for ( auto& scheme : result.schemes )
    {
        palette_textures_.push_back( makePaletteTexture( scheme.palette ) );
#ifdef AVT341_RVIZ_HAS_BINARY_MAP_VIEW
        // Alias the normal palette as the (unsupported) binary variant so the
        // base updatePalette() stays in bounds if a saved config still enables
        // Binary view: the scheme then just renders with its normal colors.
        palette_textures_binary_.push_back( palette_textures_.back() );
#endif
        // Vestigial in current rviz (written, never read) but kept index-aligned
        // in case a future release reads it again.
        color_scheme_transparency_.push_back( scheme.translucent );
        scheme_translucent_.push_back( scheme.translucent );
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
    if ( index < 0 || index >= static_cast<int>( scheme_translucent_.size() ) ||
         !scheme_translucent_[index] )
    {
        return; // opaque scheme: the base's opaque material state is correct
    }
    for ( const auto& swatch : swatches_ )
    {
        swatch->updateAlpha( Ogre::SBT_TRANSPARENT_ALPHA, false, alpha );
    }
    context_->queueRender();
}

void AugmentedMapDisplay::onMapUpdated()
{
    if ( !overlay_ || !highlight_enabled_->getBool() || !mapIsValid() )
    {
        return;
    }

    const bool from_update = update_messages_received_ != last_update_count_;
    last_update_count_ = update_messages_received_;

    if ( !sameGeometry( current_map_.info, shadow_info_ ) )
    {
        resyncHighlightState();
        return;
    }
    if ( !overlay_->valid() )
    {
        return;
    }
    if ( !from_update && highlight_source_->getOptionInt() == 0 )
    {
        shadow_data_ = current_map_.data;
        return;
    }

    const int min_change = highlight_min_change_->getInt();
    const std::uint32_t now = nowMs();
    const std::uint32_t width = shadow_info_.width;
    const std::uint32_t height = shadow_info_.height;
    const std::int8_t* current = current_map_.data.data();
    std::int8_t* shadow = shadow_data_.data();

    std::uint32_t min_x = width, min_y = height, max_x = 0, max_y = 0;
    bool any = false;
    for ( std::uint32_t y = 0; y < height; y++ )
    {
        const std::size_t row = static_cast<std::size_t>( y ) * width;
        if ( std::memcmp( current + row, shadow + row, width ) == 0 )
        {
            continue;
        }
        for ( std::uint32_t x = 0; x < width; x++ )
        {
            const std::int8_t before = shadow[row + x];
            const std::int8_t after = current[row + x];
            if ( before == after )
            {
                continue;
            }
            const bool counts = before < 0 || after < 0 ||
                std::abs( static_cast<int>( after ) - static_cast<int>( before ) ) >= min_change;
            if ( !counts )
            {
                continue;
            }
            changed_at_ms_[row + x] = now;
            min_x = std::min( min_x, x );
            max_x = std::max( max_x, x );
            min_y = std::min( min_y, y );
            max_y = std::max( max_y, y );
            any = true;
        }
        std::memcpy( shadow + row, current + row, width );
    }

    if ( !any )
    {
        return;
    }
    bursts_.push_back( { Ogre::Box( min_x, min_y, max_x + 1, max_y + 1 ), now } );
    overlay_->setVisible( true );
    context_->queueRender();
}

void AugmentedMapDisplay::updateHighlightEnabled()
{
    if ( !overlay_ )
    {
        return;
    }
    if ( highlight_enabled_->getBool() )
    {
        resyncHighlightState();
    }
    else
    {
        clearHighlightState();
    }
    context_->queueRender();
}

void AugmentedMapDisplay::updateHighlightAppearance()
{
    if ( !overlay_ )
    {
        return;
    }
    overlay_->setAlpha( highlight_alpha_->getFloat() );
    overlay_->setDrawUnder( draw_under_property_->getValue().toBool() );
    context_->queueRender();
}

bool AugmentedMapDisplay::mapIsValid() const
{
    return loaded_ && !swatches_.empty() && width_ != 0 && height_ != 0 &&
        current_map_.info.width == width_ && current_map_.info.height == height_ &&
        current_map_.data.size() == static_cast<std::size_t>( width_ ) * height_;
}

void AugmentedMapDisplay::resyncHighlightState()
{
    bursts_.clear();
    last_update_count_ = update_messages_received_;
    if ( !overlay_ || !mapIsValid() )
    {
        clearHighlightState();
        return;
    }

    shadow_info_ = current_map_.info;
    overlay_->setVisible( false );
    if ( overlay_->resize( shadow_info_.width, shadow_info_.height, shadow_info_.resolution ) )
    {
        shadow_data_ = current_map_.data;
        changed_at_ms_.assign( shadow_data_.size(), 0 );
        deleteStatus( kHighlightStatus );
    }
    else
    {
        shadow_data_.clear();
        changed_at_ms_.clear();
        setStatus(
            StatusProperty::Warn, kHighlightStatus,
            "Could not create an overlay texture for this map size; highlighting "
            "is off for this map" );
    }
}

void AugmentedMapDisplay::clearHighlightState()
{
    bursts_.clear();
    shadow_info_ = nav_msgs::msg::MapMetaData();
    shadow_data_.clear();
    shadow_data_.shrink_to_fit();
    changed_at_ms_.clear();
    changed_at_ms_.shrink_to_fit();
    staging_.clear();
    staging_.shrink_to_fit();
    if ( overlay_ )
    {
        overlay_->clear();
    }
    deleteStatus( kHighlightStatus );
}

void AugmentedMapDisplay::advanceHighlight()
{
    if ( !overlay_ || !overlay_->valid() )
    {
        bursts_.clear();
        return;
    }

    const std::uint32_t now = nowMs();
    const double lifetime_ms =
        ( highlight_hold_->getFloat() + highlight_fade_->getFloat() ) * 1000.0;

    for ( const Burst& burst : bursts_ )
    {
        uploadBox( burst.box, now );
    }
    while ( !bursts_.empty() &&
            static_cast<double>( now - bursts_.front().start_ms ) > lifetime_ms )
    {
        bursts_.pop_front();
    }
    if ( bursts_.empty() )
    {
        overlay_->setVisible( false );
    }
    context_->queueRender();
}

void AugmentedMapDisplay::uploadBox( const Ogre::Box& box, std::uint32_t now_ms )
{
    const float hold_ms = highlight_hold_->getFloat() * 1000.0f;
    const float fade_ms = highlight_fade_->getFloat() * 1000.0f;
    const Ogre::ColourValue color = highlight_color_->getOgreColor();
    const auto r = static_cast<std::uint8_t>( color.r * 255.0f + 0.5f );
    const auto g = static_cast<std::uint8_t>( color.g * 255.0f + 0.5f );
    const auto b = static_cast<std::uint8_t>( color.b * 255.0f + 0.5f );
    const std::uint32_t width = shadow_info_.width;

    staging_.resize( static_cast<std::size_t>( box.getWidth() ) * box.getHeight() * 4u );
    std::uint8_t* out = staging_.data();
    for ( std::uint32_t y = box.top; y < box.bottom; y++ )
    {
        const std::size_t row = static_cast<std::size_t>( y ) * width;
        for ( std::uint32_t x = box.left; x < box.right; x++ )
        {
            const std::uint32_t stamp = changed_at_ms_[row + x];
            float intensity = 0.0f;
            if ( stamp != 0 )
            {
                const float age_ms = static_cast<float>( now_ms - stamp );
                if ( age_ms <= hold_ms )
                {
                    intensity = 1.0f;
                }
                else if ( fade_ms > 0.0f )
                {
                    const float u = ( age_ms - hold_ms ) / fade_ms;
                    if ( u < 1.0f )
                    {
                        intensity = ( 1.0f - u ) * ( 1.0f - u );
                    }
                }
            }
            *out++ = r;
            *out++ = g;
            *out++ = b;
            *out++ = static_cast<std::uint8_t>( intensity * 255.0f + 0.5f );
        }
    }
    overlay_->upload( box, staging_.data() );
}

std::uint32_t AugmentedMapDisplay::nowMs() const
{
    const auto elapsed = std::chrono::steady_clock::now() - epoch_;
    return static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>( elapsed ).count() ) + 1u;
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
    // until updatePalette() re-points them. The binary vector holds aliases of
    // the same textures, so it is only truncated, never removed from.
    auto& texture_manager = Ogre::TextureManager::getSingleton();
    for ( std::size_t i = kBuiltinSchemeCount; i < palette_textures_.size(); i++ )
    {
        texture_manager.remove( palette_textures_[i] );
    }
    palette_textures_.resize( kBuiltinSchemeCount );
#ifdef AVT341_RVIZ_HAS_BINARY_MAP_VIEW
    palette_textures_binary_.resize( kBuiltinSchemeCount );
#endif
    color_scheme_transparency_.resize( kBuiltinSchemeCount );
    scheme_translucent_.resize( kBuiltinSchemeCount );
}

} // namespace avt_341::rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::AugmentedMapDisplay, rviz_common::Display )
