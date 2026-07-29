#ifndef AVT_341_POLYGON_ZONE_PARSER_H
#define AVT_341_POLYGON_ZONE_PARSER_H
#include <string>
#include <vector>

#include "avt_341_nav/perception/costmap_dtos.h"

namespace avt_341_nav::perception
{

class PolygonZoneParser
{
public:
    explicit PolygonZoneParser(const std::string& text);

    /// Parse the document and return the extracted zones.
    std::vector<PolygonZone> Parse();

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
    std::vector<core::vec2> ParseVertexArray();

    /// Parse one zone object: { "name": "...", "vertices": [[...], ...] }.
    PolygonZone ParseZoneObject();

    /// Parse the top-level array of zone objects.
    std::vector<PolygonZone> ParseZoneArray();

    const std::string& s_;
    std::size_t        pos_;
};

}

#endif //AVT_341_POLYGON_ZONE_PARSER_H
