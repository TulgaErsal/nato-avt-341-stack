#include "avt_341_nav/core/waypoint_file_parser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace avt_341_nav::core
{

namespace
{

    std::string Trim(const std::string & text)
    {
        const auto begin = text.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos)
        {
            return "";
        }
        const auto end = text.find_last_not_of(" \t\r\n");
        return text.substr(begin, end - begin + 1);
    }

    std::vector<double> ParseList(const std::string & line,
                                  const std::string & key,
                                  const std::string & file_path)
    {
        const auto open = line.find('[');
        const auto close = line.find(']');
        if (open == std::string::npos || close == std::string::npos || close < open)
        {
            throw std::runtime_error(
                "Waypoint file '" + file_path + "': expected a bracketed list for '" + key + "'");
        }
        std::vector<double> values;
        std::stringstream items(line.substr(open + 1, close - open - 1));
        std::string item;
        while (std::getline(items, item, ','))
        {
            item = Trim(item);
            if (item.empty())
            {
                continue; // tolerate trailing commas and empty lists
            }
            std::size_t parsed = 0;
            try
            {
                values.push_back(std::stod(item, &parsed));
            }
            catch (const std::exception &)
            {
                parsed = std::string::npos;
            }
            if (parsed != item.size())
            {
                throw std::runtime_error(
                    "Waypoint file '" + file_path + "': invalid number '" + item
                    + "' in '" + key + "'");
            }
        }
        return values;
    }

} // namespace

nav_msgs::msg::Path WaypointFileParser::Parse(const std::string & file_path)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    if (file_path.empty())
    {
        return path;
    }

    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open waypoint file '" + file_path + "'");
    }
    std::vector<double> xs, ys;
    std::string line;
    while (std::getline(file, line))
    {
        const auto comment = line.find('#');
        if (comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }
        line = Trim(line);
        if (line.rfind("waypoints_x:", 0) == 0)
        {
            xs = ParseList(line, "waypoints_x", file_path);
        }
        else if (line.rfind("waypoints_y:", 0) == 0)
        {
            ys = ParseList(line, "waypoints_y", file_path);
        }
        else if (line.rfind("frame:", 0) == 0)
        {
            const std::string frame = Trim(line.substr(std::string("frame:").size()));
            if (!frame.empty())
            {
                path.header.frame_id = frame;
            }
        }
    }

    if (xs.size() != ys.size())
    {
        throw std::runtime_error(
            "Waypoint file '" + file_path + "': waypoints_x size ("
            + std::to_string(xs.size()) + ") does not match waypoints_y size ("
            + std::to_string(ys.size()) + ")");
    }

    path.poses.reserve(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.pose.position.x = xs[i];
        pose.pose.position.y = ys[i];
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return path;
}

}
