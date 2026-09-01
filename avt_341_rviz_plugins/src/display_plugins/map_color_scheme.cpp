#include <avt_341_rviz_plugins/display_plugins/map_color_scheme.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <QRegularExpression>

#include <yaml-cpp/yaml.h>

namespace avt_341::rviz_plugins
{

namespace
{
constexpr int kMinCellValue = -128; // OccupancyGrid data is signed int8
constexpr int kMaxCellValue = 127;

/// The palette texture is indexed by the occupancy byte reinterpreted as
/// unsigned, so a signed cell value maps to its two's-complement byte
/// (0..127 identity, -1 -> 255, -128 -> 128).
int paletteIndex( int value )
{
    return static_cast<unsigned char>( value );
}

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

/// Parse a single-color node: [r,g,b] (opaque), [r,g,b,a], or a hex string.
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

/// Parse a `values:` key: a single cell value "V" or an inclusive range
/// "LO..HI", whitespace around the ".." tolerated. yaml-cpp hands every key
/// over as a string, so integer-looking and range-looking keys arrive
/// uniformly; the strict pattern rejects float look-alikes ("0.100", ".5")
/// and stray extra dots ("0...100") instead of misreading them.
bool parseValueKey( const QString& text, int& lo, int& hi, QString& err )
{
    static const QRegularExpression kPattern(
        R"(^\s*(-?\d+)\s*(?:\.\.\s*(-?\d+)\s*)?$)" );
    const QRegularExpressionMatch match = kPattern.match( text );
    bool ok = match.hasMatch();
    if ( ok )
    {
        lo = match.captured( 1 ).toInt( &ok );
    }
    hi = lo;
    if ( ok && !match.captured( 2 ).isNull() )
    {
        hi = match.captured( 2 ).toInt( &ok );
    }
    if ( !ok || lo < kMinCellValue || lo > kMaxCellValue ||
         hi < kMinCellValue || hi > kMaxCellValue )
    {
        err = QString( "invalid cell value or range \"%1\" "
                       "(expected V or LO..HI, integers in -128..127)" )
                  .arg( text.trimmed() );
        return false;
    }
    if ( lo > hi )
    {
        err = QString( "invalid range \"%1\" (LO must be <= HI)" ).arg( text.trimmed() );
        return false;
    }
    return true;
}

/// Parse a `values:` entry into gradient stops: a single color (one stop), a
/// ".."-joined string of hex stops, or a list of two or more colors. A
/// sequence whose elements are all integers is one [r,g,b](,a) color; any
/// other sequence is a stop list.
bool parseStops( const YAML::Node& node, std::vector<SchemeColor>& out, QString& err )
{
    out.clear();

    if ( node.IsScalar() )
    {
        const QString text = QString::fromStdString( node.as<std::string>() );
        if ( !text.contains( ".." ) )
        {
            SchemeColor color;
            if ( !parseColor( node, color, err ) )
            {
                return false;
            }
            out.push_back( color );
            return true;
        }
        for ( const QString& piece : text.split( ".." ) ) // ".." implies >= 2 pieces
        {
            SchemeColor color;
            if ( !parseHexColor( piece.trimmed(), color ) )
            {
                err = QString( "invalid gradient stop \"%1\" "
                               "(expected #RRGGBB or #RRGGBBAA)" )
                          .arg( piece.trimmed() );
                return false;
            }
            out.push_back( color );
        }
        return true;
    }

    if ( node.IsSequence() )
    {
        bool all_ints = node.size() > 0;
        for ( const auto& element : node )
        {
            if ( !element.IsScalar() )
            {
                all_ints = false;
                break;
            }
            try
            {
                element.as<int>();
            }
            catch ( const YAML::Exception& )
            {
                all_ints = false;
                break;
            }
        }
        if ( all_ints )
        {
            SchemeColor color;
            if ( !parseColor( node, color, err ) )
            {
                return false;
            }
            out.push_back( color );
            return true;
        }
        for ( const auto& element : node )
        {
            SchemeColor color;
            if ( !parseColor( element, color, err ) )
            {
                return false;
            }
            out.push_back( color );
        }
        if ( out.size() < 2 )
        {
            err = "a gradient list needs at least 2 colors";
            return false;
        }
        return true;
    }

    err = "invalid color (expected [r, g, b], [r, g, b, a], a \"#hex\" string, "
          "a \"#hex..#hex\" gradient or a list of colors)";
    return false;
}

/// Paint one values: entry onto the palette. A single stop fills the whole
/// key range; two or more stops are spread evenly across it with linear
/// per-channel RGBA interpolation between adjacent stops.
void applyStops(
    std::vector<unsigned char>& palette, int lo, int hi,
    const std::vector<SchemeColor>& stops )
{
    if ( stops.size() == 1 )
    {
        for ( int v = lo; v <= hi; v++ )
        {
            setEntry( palette, paletteIndex( v ), stops.front() );
        }
        return;
    }
    const int segments = static_cast<int>( stops.size() ) - 1; // lo < hi checked by caller
    for ( int v = lo; v <= hi; v++ )
    {
        const double t = static_cast<double>( v - lo ) / ( hi - lo ) * segments;
        const int seg = std::min( static_cast<int>( t ), segments - 1 );
        setEntry( palette, paletteIndex( v ), lerp( stops[seg], stops[seg + 1], t - seg ) );
    }
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

    static const std::set<std::string> kKnownKeys = { "name", "default", "values" };
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

    if ( !node["values"] && !node["default"] )
    {
        err = "scheme defines no colors (need at least one of values / default)";
        return false;
    }

    // `default` covers every cell value not painted by a values: entry,
    // including -1 and the out-of-contract values.
    SchemeColor default_color; // fully transparent
    if ( node["default"] && !parseColor( node["default"], default_color, err ) )
    {
        err = "default: " + err;
        return false;
    }

    out.palette.assign( 256 * 4, 0 );
    for ( int i = 0; i < 256; i++ )
    {
        setEntry( out.palette, i, default_color );
    }

    if ( node["values"] )
    {
        const YAML::Node values = node["values"];
        if ( !values.IsMap() || values.size() == 0 )
        {
            err = "values must be a non-empty map of cell value/range: color";
            return false;
        }
        // yaml-cpp iterates in document order, so later entries override earlier
        // ones where they overlap; only an exact repeat of the same value/range
        // (a likely copy-paste slip YAML itself does not flag) is rejected.
        std::set<std::pair<int, int>> seen;
        for ( const auto& kv : values )
        {
            const QString key_text = QString::fromStdString( kv.first.as<std::string>() );
            int lo = 0, hi = 0;
            if ( !parseValueKey( key_text, lo, hi, err ) )
            {
                return false;
            }
            if ( !seen.emplace( lo, hi ).second )
            {
                err = QString( "values has a duplicate entry for %1" ).arg( key_text );
                return false;
            }
            std::vector<SchemeColor> stops;
            if ( !parseStops( kv.second, stops, err ) )
            {
                err = QString( "values[%1]: %2" ).arg( key_text, err );
                return false;
            }
            if ( stops.size() > 1 && lo == hi )
            {
                err = QString( "values[%1]: a gradient needs a range of at "
                               "least two cells" )
                          .arg( key_text );
                return false;
            }
            applyStops( out.palette, lo, hi, stops );
        }
    }

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
