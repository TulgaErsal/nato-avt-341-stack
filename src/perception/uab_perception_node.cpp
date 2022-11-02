#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/perception_wrapper.h"
#include "mclcppclass.h"
#include "mclmcrrt.h"
#include <vector>
#include <array>

avt_341::msg::Odometry current_pose;
bool odom_received = false;

avt_341::msg::PointCloud2 pc;
bool pc_received = false;

avt_341::msg::Image img;
bool img_received = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom)
{
    current_pose = *rcv_odom;
    odom_received = true;
}

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_pc)
{
    pc = *rcv_pc;
    pc_received = true;
}

void ImageCallback(avt_341::msg::ImagePtr rcv_img)
{
    img = *rcv_img;
    img_received = true;
}

bool allMsgsReceived()
{
    return odom_received && pc_received && img_received;
}

/*
* GetCostmapFromMatlab packages up all the data from ROS (odometry, pointcloud, and image), calls
* the semantic segmentation model in Matlab, and populates an array of cost values based on traversability
*/
std::vector<double> GetCostmapFromMatlab(std::vector<double> costs)
{
    //odometry
    mwArray x(current_pose.pose.pose.position.x);
    mwArray y(current_pose.pose.pose.position.y);
    mwArray z(current_pose.pose.pose.position.z);
    mwArray qw(current_pose.pose.pose.orientation.w);
    mwArray qx(current_pose.pose.pose.orientation.x);
    mwArray qy(current_pose.pose.pose.orientation.y);
    mwArray qz(current_pose.pose.pose.orientation.z);

    //raw pointcloud
    mwArray pcData(1, std::size(pc.data), mxUINT8_CLASS);
    pcData.SetData(&pc.data[0], std::size(pc.data));

    //raw image
    mwArray imgData(1, std::size(img.data), mxUINT8_CLASS);
    imgData.SetData(&img.data[0], std::size(img.data));

    try
    {
        std::cout << "all messages received, calling matlab" << std::endl;
        mwArray costmap;
        perception_wrapper(1, costmap, imgData, pcData, x, y, z, qw, qx, qy, qz);

        costmap.GetData(costs.data(), costs.size());
        std::vector<double> costVec(std::begin(costs), std::end(costs));
        return costVec;
    }
    catch(const mwException& e)
    {
        std::cerr << "Failed to execute Matlab perception_wrapper. " << e.what() << std::endl;
    }
    return {};
}

void BuildOccupancyGrid(avt_341::msg::OccupancyGrid &grid,
                        float width,
                        float height,
                        float startX,
                        float startY,
                        float res,
                        std::vector<std::vector<double>> data,
                        bool forVisualization = false)
{
    grid.header.frame_id = "map";
    grid.info.resolution = res;
    grid.info.height = height;
    grid.info.width = width;
    grid.info.origin.position.x = startX;
    grid.info.origin.position.y = startY;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.data.resize(width*height);

    int c = 0;
    if (forVisualization)
    {
        //RVIZ expects row major order
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                //scale matlab costs up
                grid.data[c++] = data[i][j] * 100;
            }
        }
    }
    else
    {
        //planners expect column major order
        for (int j = 0; j < width; j++)
        {
            for (int i = 0; i < height; i++)
            {
                //scale matlab costs up
                grid.data[c++] = data[i][j] * 100;
            }
        }
    }
}

std::vector<std::vector<double>> to2D(std::vector<double> vec, float width, float height)
{
    //1d -> 2d
    std::vector<std::vector<double>> vec2D(height, std::vector<double>(width));
    int c = 0;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            vec2D[i][j] = vec[c++];
        }
    }

    return vec2D;
}

int main(int argc, char *argv[])
{
    auto node = avt_341::node::init_node(argc, argv, "avt_341_perception_node");

    auto odom_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
    auto pc_sub = node->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 2, PointCloudCallback);
    auto img_sub = node->create_subscription<avt_341::msg::Image>("camera/rgb/image_raw", 10, ImageCallback);

    auto seg_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);
    auto seg_grid_vis_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid_vis", 1);

    float width;
    node->get_parameter("~grid_width", width, 100.0f);
    float height;
    node->get_parameter("~grid_height", height, 100.0f);
    float startX;
    node->get_parameter("~grid_llx", startX, 0.0f);
    float startY;
    node->get_parameter("~grid_lly", startY, 0.0f);
    float res;
    node->get_parameter("~grid_res", res, 1.0f);
    width = width/res;
    height = height/res;

    //initialize matlab runtime
    if (!mclInitializeApplication(NULL, 0))
    {
        std::cerr << "Failed to initialize Matlab Runtime" << std::endl;
        return -1;
    }

    //initialize uab matlab perception model
    if(!perception_wrapperInitialize())
    {
        std::cerr << "Failed to initialize perception_wrapper package" << std::endl;
        return -1;
    }

    avt_341::node::Rate rate(100.0);
    while (avt_341::node::ok())
    {
        if (!allMsgsReceived())
        {
            std::string waitingOn = "waiting on ";
            if (!odom_received) waitingOn += "odom ";
            if (!pc_received) waitingOn += "pointcloud ";
            if (!img_received) waitingOn += "image";
            std::cout << waitingOn << std::endl;
        }
        else
        {
            std::vector<double> costs(width * height);
            std::vector<double> costmap = GetCostmapFromMatlab(costs);
            std::vector<std::vector<double>> costmap2D = to2D(costmap, width, height);
            
            //grid for planners
            avt_341::msg::OccupancyGrid grid;
            BuildOccupancyGrid(grid, width, height, startX, startY, res, costmap2D);
            seg_grid_pub->publish(grid);

            //grid for RVIZ
            avt_341::msg::OccupancyGrid visGrid;
            bool forVisualization = true;
            BuildOccupancyGrid(visGrid, width, height, startX, startY, res, costmap2D, forVisualization);
            seg_grid_vis_pub->publish(visGrid);
        }
        node->spin_some();
        rate.sleep();
    }

    mclTerminateApplication();
    return 0;
}