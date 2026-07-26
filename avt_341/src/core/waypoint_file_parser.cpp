#include "avt_341/core/waypoint_file_parser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace avt_341::core
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

WaypointLists WaypointFileParser::Parse(const std::string & file_path)
{
    if (file_path.empty())
    {
        return WaypointLists{};
    }

    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open waypoint file '" + file_path + "'");
    }
    WaypointLists waypoints;
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
            waypoints.x = ParseList(line, "waypoints_x", file_path);
        }
        else if (line.rfind("waypoints_y:", 0) == 0)
        {
            waypoints.y = ParseList(line, "waypoints_y", file_path);
        }
    }
    return waypoints;
}

}
