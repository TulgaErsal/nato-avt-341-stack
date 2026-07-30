#include "avt_341_nav/core/geometry/polygon_zone_parser.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace avt_341_nav::core
{

PolygonZoneParser::PolygonZoneParser(const std::string& text)
    : s_(text), pos_(0)
{}

PolygonZoneCollection PolygonZoneParser::ParseFile(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::invalid_argument("Cannot open input file " + file_path);
    }

    const std::string text(
        (std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());

    PolygonZoneParser parser(text);
    PolygonZoneCollection collection = parser.Parse();

    for (const PolygonZone& zone : collection.zones) {
        if (zone.vertices.size() < 3) {
            throw std::runtime_error("Zone '" + zone.label + "' has fewer than 3 vertices.");
        }
    }

    return collection;
}

PolygonZoneCollection PolygonZoneParser::Parse()
{
    PolygonZoneCollection collection;
    Expect('{');
    while (Peek() != '}') {
        const std::string key = ParseString();
        Expect(':');
        if (key == "zones") {
            collection.zones = ParseZoneArray();
        } else if (key == "frame") {
            collection.frame = ParseString();
        } else {
            SkipValue();
        }
        SkipComma();
    }
    Expect('}');
    return collection;
}

// ---- low-level helpers -----------------------------------------------

void PolygonZoneParser::SkipWs()
{
    while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_]))) {
        ++pos_;
    }
}

char PolygonZoneParser::Peek()
{
    SkipWs();
    if (pos_ >= s_.size()) {
        throw std::runtime_error("Unexpected end of JSON input.");
    }
    return s_[pos_];
}

void PolygonZoneParser::Expect(char c)
{
    if (Peek() != c) {
        std::ostringstream msg;
        msg << "Expected '" << c << "' at position " << pos_
            << " but found '" << Peek() << "'.";
        throw std::runtime_error(msg.str());
    }
    ++pos_;
}

void PolygonZoneParser::SkipComma()
{
    SkipWs();
    if (pos_ < s_.size() && s_[pos_] == ',') {
        ++pos_;
    }
}

void PolygonZoneParser::SkipValue()
{
    const char c = Peek();
    if (c == '"') {
        ParseString();
    } else if (c == '{') {
        ++pos_;
        while (Peek() != '}') {
            ParseString(); // key
            Expect(':');
            SkipValue();   // value
            SkipComma();
        }
        Expect('}');
    } else if (c == '[') {
        ++pos_;
        while (Peek() != ']') {
            SkipValue();
            SkipComma();
        }
        Expect(']');
    } else if (c == 't') { pos_ += 4; } // true
    else if  (c == 'f') { pos_ += 5; } // false
    else if  (c == 'n') { pos_ += 4; } // null
    else {
        ParseNumber();
    }
}

// ---- value parsers ---------------------------------------------------

std::string PolygonZoneParser::ParseString()
{
    Expect('"');
    std::string result;
    while (pos_ < s_.size() && s_[pos_] != '"') {
        if (s_[pos_] == '\\') {
            ++pos_; // skip escape character
        }
        if (pos_ < s_.size()) {
            result += s_[pos_++];
        }
    }
    Expect('"');
    return result;
}

double PolygonZoneParser::ParseNumber()
{
    SkipWs();
    const std::size_t start = pos_;
    if (pos_ < s_.size() && (s_[pos_] == '-' || s_[pos_] == '+')) {
        ++pos_;
    }
    while (pos_ < s_.size() &&
           (std::isdigit(static_cast<unsigned char>(s_[pos_])) ||
            s_[pos_] == '.' || s_[pos_] == 'e' || s_[pos_] == 'E' ||
            s_[pos_] == '+' || s_[pos_] == '-'))
    {
        ++pos_;
    }
    if (pos_ == start) {
        throw std::runtime_error("Expected a number at position " + std::to_string(pos_));
    }
    return std::stod(s_.substr(start, pos_ - start));
}

// ---- structure parsers -----------------------------------------------

std::vector<Eigen::Vector2d> PolygonZoneParser::ParseVertexArray()
{
    std::vector<Eigen::Vector2d> result;
    Expect('[');
    while (Peek() != ']') {
        Expect('[');
        const double x = ParseNumber();
        Expect(',');
        const double y = ParseNumber();
        Expect(']');
        result.emplace_back(x, y);
        SkipComma();
    }
    Expect(']');
    return result;
}

PolygonZone PolygonZoneParser::ParseZoneObject()
{
    PolygonZone zone;

    Expect('{');
    while (Peek() != '}') {
        const std::string key = ParseString();
        Expect(':');
        if (key == "label") {
            zone.label = ParseString();
        } else if (key == "vertices") {
            zone.vertices = ParseVertexArray();
        } else if (key == "occupancy") {
            zone.occ_value = ParseNumber();
        } else if (key == "max_speed") {
            zone.max_speed = ParseNumber();
        } else if (key == "segmentation") {
            zone.seg_value = static_cast<int>(ParseNumber());
        } else {
            SkipValue(); // skip "comment" and any other unknown keys
        }
        SkipComma();
    }
    Expect('}');
    return zone;
}

std::vector<PolygonZone> PolygonZoneParser::ParseZoneArray()
{
    std::vector<PolygonZone> zones;
    Expect('[');
    while (Peek() != ']') {
        zones.push_back(ParseZoneObject());
        SkipComma();
    }
    Expect(']');
    return zones;
}

}
