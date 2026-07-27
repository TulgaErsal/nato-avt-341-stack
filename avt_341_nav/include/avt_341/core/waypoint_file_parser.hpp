#ifndef AVT_341_WAYPOINT_FILE_PARSER_HPP
#define AVT_341_WAYPOINT_FILE_PARSER_HPP

#include <string>
#include <vector>

namespace avt_341::core
{

    /// Waypoint coordinate lists read from a waypoint yaml file.
    struct WaypointLists
    {
        std::vector<double> x;
        std::vector<double> y;
    };

    /**
     * Minimal parser for waypoint yaml files of the form:
     *
     *   waypoints_x: [ -50.0, 0.0, 50.0 ]
     *   waypoints_y: [ 0.0, 0.0, 0.0 ]
     *
     * Not a general yaml parser: each list must be a single-line flow sequence
     * keyed by waypoints_x/waypoints_y at the document root.
     */
    class WaypointFileParser
    {
    public:
        /// Parse the waypoint lists from the given file. Missing keys yield
        /// empty lists. Throws std::runtime_error when the file cannot be
        /// opened or a list cannot be parsed.
        static WaypointLists Parse(const std::string & file_path);
    };

}

#endif // AVT_341_WAYPOINT_FILE_PARSER_HPP
