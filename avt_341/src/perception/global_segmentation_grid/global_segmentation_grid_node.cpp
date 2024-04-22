#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include <vector>
#include <fstream>

//number of rows in grid
const int ROWS = 3696;
//number of columns in grid
const int COLS = 1971;
//resolution
const float RES = 0.5;

void BuildOccupancyGrid(avt_341::msg::OccupancyGrid &grid, std::vector<double> data, float grid_llx, float grid_lly)
{
    grid.header.frame_id = "map";
    grid.info.resolution = RES;
    //rows and columns are swapped since the map is transposed
    grid.info.height = ROWS;
    grid.info.width = COLS;
    grid.info.origin.position.x = grid_llx;
    grid.info.origin.position.y = grid_lly;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.data.resize(ROWS*COLS);

    int c = 0;
    for (double val : data)
    {
        grid.data[c++] = 100 - val;
    }
}

std::vector<double> GetCostMapFromTif(std::string path)
{
    std::vector<double> costmap;
    std::ifstream file(path);
    if (!file.good())
    {
        printf("Failed to open global segmentation grid CSV.\n");
        return {};
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream stream(line);
        std::string val;
        
        while (std::getline(stream, val, ','))
        {
            double d = stod(val);
            costmap.push_back(d);
        }
    }

    file.close();
    
    return costmap;
}

int main(int argc, char *argv[])
{
    auto node = avt_341::node::init_node(argc, argv, "avt_341_global_grid_publisher_node");
    auto seg_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);

    std::string globalGridCSVPath = "";
    std::string defaultPath = "";
    node->get_parameter("~global_grid_csv_path", globalGridCSVPath, defaultPath);
    float grid_llx;
    node->get_parameter("~grid_llx", grid_llx, 0.0f);
    float grid_lly;
    node->get_parameter("~grid_lly", grid_lly, 0.0f);
    float res;
    node->get_parameter("~grid_res", res, 1.0f);

    std::vector<double> costmap = GetCostMapFromTif(globalGridCSVPath);
    if (costmap.size() > 0)
    {
        //wait 2s to allow time for the rest of the stack to
        //start up since this message is only published once
        rclcpp::Rate rate(std::chrono::seconds(2));
        rate.sleep();

        avt_341::msg::OccupancyGrid grid;
        BuildOccupancyGrid(grid, costmap, grid_llx, grid_lly);
        seg_grid_pub->publish(grid);

        node->spin_some();
        rate.sleep();
    }
    return 0;
}