#include <avt_341_rviz_plugins/display_plugins/map_color_scheme.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>

#include <yaml-cpp/yaml.h>

namespace avt_341::rviz_plugins
{

namespace
{
constexpr int kMaxLegalValue = 100;    // legal occupancy values are 0..100
constexpr int kUnknownIndex = 255;     // the value -1 as an unsigned byte
constexpr int kIllegalPositiveFirst = 101, kIllegalPositiveLast = 127;
constexpr int kIllegalNegativeFirst = 128, kIllegalNegativeLast = 254;

void setEntry( std::vector<unsigned char>& palette, int index, const SchemeColor& c )
{
    palette[4 * index + 0] = c.r;
    palette[4 * index + 1] = c.g;
    palette[4 * index + 2] = c.b;
    palette[4 * index + 3] = c.a;
}

SchemeColor lerp( const SchemeColor& lo, const SchemeColor& hi, double t )
{
    auto mix = [t]( unsigned char a, unsigned char b ) {
        return static_cast<unsigned char>( std::lround( a + t * ( b - a ) ) );
    };
    return SchemeColor{ mix( lo.r, hi.r ), mix( lo.g, hi.g ),
                        mix( lo.b, hi.b ), mix( lo.a, hi.a ) };
}

/// Parse "#RRGGBB" / "#RRGGBBAA" (this order, unlike Qt's #AARRGGBB convention).
bool parseHexColor( const QString& text, SchemeColor& out )
{
    if ( !text.startsWith( '#' ) || ( text.length() != 7 && text.length() != 9 ) )
    {
        return false;
    }
    auto channel = [&text]( int pos, bool& ok ) {
        return static_cast<unsigned char>( text.mid( pos, 2 ).toUInt( &ok, 16 ) );
    };
    bool ok = true, all_ok = true;
    out.r = channel( 1, ok ); all_ok &= ok;
    out.g = channel( 3, ok ); all_ok &= ok;
    out.b = channel( 5, ok ); all_ok &= ok;
    out.a = 255;
    if ( text.length() == 9 )
    {
        out.a = channel( 7, ok );
        all_ok &= ok;
    }
    return all_ok;
}

/// Parse a color node: [r,g,b] (opaque), [r,g,b,a], or a hex string.
bool parseColor( const YAML::Node& node, SchemeColor& out, QString& err )
{
    if ( node.IsScalar() )
    {
        if ( parseHexColor( QString::fromStdString( node.as<std::string>() ), out ) )
        {
            return true;
        }
        err = QString( "invalid color \"%1\" (expected #RRGGBB or #RRGGBBAA)" )
                  .arg( QString::fromStdString( node.as<std::string>() ) );
        return false;
    }
    if ( !node.IsSequence() || ( node.size() != 3 && node.size() != 4 ) )
    {
        err = "invalid color (expected [r, g, b], [r, g, b, a] or a #hex string)";
        return false;
    }
    unsigned char rgba[4] = { 0, 0, 0, 255 };
    for ( std::size_t i = 0; i < node.size(); i++ )
    {
        int channel = -1;
        try
        {
            channel = node[i].as<int>();
        }
        catch ( const YAML::Exception& )
        {
        }
        if ( channel < 0 || channel > 255 )
        {
            err = "invalid color component (expected an integer in 0-255)";
            return false;
        }
        rgba[i] = static_cast<unsigned char>( channel );
    }
    out = SchemeColor{ rgba[0], rgba[1], rgba[2], rgba[3] };
    return true;
}

/// Parse a `gradient:` / `values:` style map of occupancy value -> color, with
/// range and duplicate-key validation.
bool parseValueColorMap(
    const YAML::Node& node, const QString& key, std::map<int, SchemeColor>& out,
    QString& err )
{
    if ( !node.IsMap() || node.size() == 0 )
    {
        err = key + " must be a non-empty map of value: color";
        return false;
    }
    for ( const auto& kv : node )
    {
        int value = -1;
        try
        {
            value = kv.first.as<int>();
        }
        catch ( const YAML::Exception& )
        {
        }
        if ( value < 0 || value > kMaxLegalValue )
        {
            err = key + " keys must be integers in 0-100";
            return false;
        }
        SchemeColor color;
        if ( !parseColor( kv.second, color, err ) )
        {
            err = QString( "%1[%2]: %3" ).arg( key ).arg( value ).arg( err );
            return false;
        }
        if ( !out.emplace( value, color ).second )
        {
            err = QString( "%1 has a duplicate entry for value %2" ).arg( key ).arg( value );
            return false;
        }
    }
    return true;
}

/// Evaluate a gradient at `v`: linear per-channel RGBA interpolation between the
/// bracketing stops, clamping to the end stops outside their range.
SchemeColor evalGradient( const std::map<int, SchemeColor>& stops, int v )
{
    if ( v <= stops.begin()->first )
    {
        return stops.begin()->second;
    }
    const auto last = std::prev( stops.end() );
    if ( v >= last->first )
    {
        return last->second;
    }
    const auto hi = stops.upper_bound( v );
    const auto lo = std::prev( hi );
    const double t =
        static_cast<double>( v - lo->first ) / static_cast<double>( hi->first - lo->first );
    return lerp( lo->second, hi->second, t );
}

/// Parse one scheme entry. Returns false with `err` set on any validation error
/// (the caller skips the scheme). Unrecognized keys are errors so typos surface.
bool parseScheme( const YAML::Node& node, MapColorScheme& out, QString& err )
{
    if ( !node.IsMap() )
    {
        err = "scheme entry is not a map";
        return false;
    }

    static const std::set<std::string> kKnownKeys = {
        "name", "gradient", "values", "default", "unknown", "illegal" };
    for ( const auto& kv : node )
    {
        const std::string key = kv.first.as<std::string>();
        if ( kKnownKeys.find( key ) == kKnownKeys.end() )
        {
            err = QString( "unrecognized key \"%1\"" ).arg( QString::fromStdString( key ) );
            return false;
        }
    }

    if ( !node["name"] || !node["name"].IsScalar() )
    {
        err = "missing required key \"name\"";
        return false;
    }
    out.name = QString::fromStdString( node["name"].as<std::string>() ).trimmed();
    if ( out.name.isEmpty() )
    {
        err = "\"name\" must not be empty";
        return false;
    }

    if ( !node["gradient"] && !node["values"] && !node["default"] )
    {
        err = "scheme defines no colors (need at least one of gradient / values / default)";
        return false;
    }

    // Legal values 0-100: default color first, then the gradient (which covers the
    // whole range via clamping when present), then exact `values` overrides.
    SchemeColor default_color; // fully transparent
    if ( node["default"] && !parseColor( node["default"], default_color, err ) )
    {
        err = "default: " + err;
        return false;
    }

    out.palette.assign( 256 * 4, 0 );
    for ( int v = 0; v <= kMaxLegalValue; v++ )
    {
        setEntry( out.palette, v, default_color );
    }

    if ( node["gradient"] )
    {
        std::map<int, SchemeColor> stops;
        if ( !parseValueColorMap( node["gradient"], "gradient", stops, err ) )
        {
            return false;
        }
        for ( int v = 0; v <= kMaxLegalValue; v++ )
        {
            setEntry( out.palette, v, evalGradient( stops, v ) );
        }
    }

    if ( node["values"] )
    {
        std::map<int, SchemeColor> values;
        if ( !parseValueColorMap( node["values"], "values", values, err ) )
        {
            return false;
        }
        for ( const auto& [value, color] : values )
        {
            setEntry( out.palette, value, color );
        }
    }

    // Out-of-contract values: a single override color, or the built-in Map
    // display's loud debug colors (green band, red-to-yellow ramp) so misbehaving
    // publishers stay visible.
    if ( node["illegal"] )
    {
        SchemeColor illegal;
        if ( !parseColor( node["illegal"], illegal, err ) )
        {
            err = "illegal: " + err;
            return false;
        }
        for ( int i = kIllegalPositiveFirst; i <= kIllegalNegativeLast; i++ )
        {
            setEntry( out.palette, i, illegal );
        }
    }
    else
    {
        for ( int i = kIllegalPositiveFirst; i <= kIllegalPositiveLast; i++ )
        {
            setEntry( out.palette, i, SchemeColor{ 0, 255, 0, 255 } );
        }
        for ( int i = kIllegalNegativeFirst; i <= kIllegalNegativeLast; i++ )
        {
            const auto yellow = static_cast<unsigned char>(
                ( 255 * ( i - kIllegalNegativeFirst ) ) /
                ( kIllegalNegativeLast - kIllegalNegativeFirst ) );
            setEntry( out.palette, i, SchemeColor{ 255, yellow, 0, 255 } );
        }
    }

    SchemeColor unknown; // fully transparent
    if ( node["unknown"] && !parseColor( node["unknown"], unknown, err ) )
    {
        err = "unknown: " + err;
        return false;
    }
    setEntry( out.palette, kUnknownIndex, unknown );

    out.translucent = false;
    for ( int i = 0; i < 256; i++ )
    {
        out.translucent |= out.palette[4 * i + 3] < 255;
    }

    return true;
}
} // namespace

MapColorSchemeLoadResult LoadMapColorSchemes(
    const QString& path, const QStringList& reserved_names )
{
    MapColorSchemeLoadResult result;

    YAML::Node root;
    try
    {
        root = YAML::LoadFile( path.toStdString() );
    }
    catch ( const YAML::Exception& e )
    {
        result.error =
            QString( "Cannot read %1: %2" ).arg( path, QString::fromStdString( e.what() ) );
        return result;
    }

    if ( !root.IsMap() || !root["schemes"] || !root["schemes"].IsSequence() )
    {
        result.error = path + ": missing top-level \"schemes\" list";
        return result;
    }

    QStringList seen = reserved_names;
    const YAML::Node schemes = root["schemes"];
    for ( std::size_t i = 0; i < schemes.size(); i++ )
    {
        MapColorScheme scheme;
        QString err;
        bool ok = false;
        try
        {
            ok = parseScheme( schemes[i], scheme, err );
        }
        catch ( const YAML::Exception& e )
        {
            err = QString::fromStdString( e.what() );
        }
        if ( !ok )
        {
            result.warnings << QString( "scheme %1 skipped: %2" ).arg( i + 1 ).arg( err );
            continue;
        }
        if ( seen.contains( scheme.name, Qt::CaseInsensitive ) )
        {
            result.warnings << QString( "scheme \"%1\" skipped: name already in use" )
                                   .arg( scheme.name );
            continue;
        }
        seen << scheme.name;
        result.schemes.push_back( std::move( scheme ) );
    }

    return result;
}

} // namespace avt_341::rviz_plugins
