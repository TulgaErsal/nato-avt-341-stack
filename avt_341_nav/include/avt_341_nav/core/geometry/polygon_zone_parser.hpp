#ifndef AVT_341_CORE_GEOMETRY_POLYGON_ZONE_PARSER_HPP
#define AVT_341_CORE_GEOMETRY_POLYGON_ZONE_PARSER_HPP
#include <string>
#include <vector>

#include "avt_341_nav/core/geometry/geometry_dto.hpp"

namespace avt_341_nav::core
{

class PolygonZoneParser
{
public:
    explicit PolygonZoneParser(const std::string& text);

    /// Load and parse a polygon zone json file. Throws if the file cannot be
    /// opened or parsed.
    static PolygonZoneCollection ParseFile(const std::string& file_path);

    /// Parse the document and return the extracted zones.
    PolygonZoneCollection Parse();

private:
    // ---- low-level helpers -----------------------------------------------

    void SkipWs();

    char Peek();

    void Expect(char c);

    void SkipComma();

    // ---- value parsers ---------------------------------------------------

    std::string ParseString();

    double ParseNumber();

    /// Skip over any JSON value (string, number, object, or array).
    void SkipValue();

    // ---- structure parsers -----------------------------------------------

    /// Parse [[x, y], [x, y], ...].
    std::vector<Eigen::Vector2d> ParseVertexArray();

    /// Parse one zone object: { "name": "...", "vertices": [[...], ...] }.
    PolygonZone ParseZoneObject();

    /// Parse the top-level array of zone objects.
    std::vector<PolygonZone> ParseZoneArray();

    const std::string& s_;
    std::size_t        pos_;
};

}

#endif // AVT_341_CORE_GEOMETRY_POLYGON_ZONE_PARSER_HPP
