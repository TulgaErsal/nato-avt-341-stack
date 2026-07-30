#ifndef AVT_341_WAYPOINT_FILE_PARSER_HPP
#define AVT_341_WAYPOINT_FILE_PARSER_HPP

#include <string>

#include "nav_msgs/msg/path.hpp"

namespace avt_341_nav::core
{

    /**
     * Minimal parser for waypoint yaml files of the form:
     *
     *   frame: epsg_6495   # optional coordinate frame of the waypoints
     *   waypoints_x: [ -50.0, 0.0, 50.0 ]
     *   waypoints_y: [ 0.0, 0.0, 0.0 ]
     *
     * Not a general yaml parser: each list must be a single-line flow sequence
     * keyed by waypoints_x/waypoints_y at the document root.
     */
    class WaypointFileParser
    {
    public:
        /// Parse the waypoints from the given file into a path message. The
        /// path frame_id is the parsed frame ("map" when the frame key is
        /// absent). Throws when the file cannot be opened or parsed.
        static nav_msgs::msg::Path Parse(const std::string & file_path);
    };

}

#endif // AVT_341_WAYPOINT_FILE_PARSER_HPP
