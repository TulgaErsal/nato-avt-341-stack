#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/perception_wrapper.h"
#include "mclcppclass.h"
#include "mclmcrrt.h"
#include <vector>
#include <array>
#include <math.h>

const float MATLAB_COSTMAP_DEFAULT_VAL = 0.5;

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
std::vector<int8_t> GetCostmapFromMatlab(std::vector<int8_t> costs, float width, float height, float res, float grid_llx, float grid_lly)
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

    //pointcloud properties
    mwArray pcWidth(pc.width);
    mwArray pcHeight(pc.height);
    mwArray pcPointStep(pc.point_step);
    mwArray pcRowStep(pc.row_step);

    //raw image
    mwArray imgData(1, std::size(img.data), mxUINT8_CLASS);
    imgData.SetData(&img.data[0], std::size(img.data));

    //grid width
    mwArray gridWidth(width);

    //grid height
    mwArray gridHeight(height);

    //grid resolution
    mwArray gridRes(res);

    //lower left corner grid offset in meters (x/east direction)
    mwArray llx(grid_llx);

    //lower right corner grid offset in meters (y/north direction)
    mwArray lly(grid_lly);

    //disable Matlab debug windows
    mwArray debug(false);

    //1, 1 -> original
    //0, 0 -> lidar only esn
    //1, 0 -> mixed
    mwArray C_model(1); mwArray CL_model(0);

    //use probabilistic grid
    mwArray useProbabilisticGrid(true);

    try
    {
        std::cout << "all messages received, calling matlab" << std::endl;
        mwArray costmap;
        perception_wrapper(1,
                            costmap,
                            imgData, //camera data
                            pcData, pcWidth, pcHeight, pcPointStep, pcRowStep, //pointcloud data
                            x, y, z, qw, qx, qy, qz, //odometry
                            gridWidth, gridHeight, gridRes, llx, lly, //grid params
                            debug, //debug flag
                            CL_model, C_model, //model selection
                            useProbabilisticGrid //which type of grid to use
                        ); 
        
        costmap.GetData(costs.data(), costs.size());
        std::vector<int8_t> costVec(std::begin(costs), std::end(costs));
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
                        float grid_llx,
                        float grid_lly,
                        float res,
                        std::vector<int8_t> data)
{
    grid.header.frame_id = "map";
    grid.info.resolution = res;
    grid.info.height = height;
    grid.info.width = width;
    grid.info.origin.position.x = grid_llx;
    grid.info.origin.position.y = grid_lly;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.data = data;
}

int main(int argc, char *argv[])
{
    auto node = avt_341::node::init_node(argc, argv, "avt_341_perception_node");

    auto odom_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
    auto pc_sub = node->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 2, PointCloudCallback);
    auto img_sub = node->create_subscription<avt_341::msg::Image>("camera/rgb/image_raw", 10, ImageCallback);

    auto seg_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);

    float width;
    node->get_parameter("~grid_width", width, 100.0f);
    float height;
    node->get_parameter("~grid_height", height, 100.0f);
    float grid_llx;
    node->get_parameter("~grid_llx", grid_llx, 0.0f);
    float grid_lly;
    node->get_parameter("~grid_lly", grid_lly, 0.0f);
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
            
            avt_341::node::Rate wait(1.0);
            wait.sleep();
        }
        else
        {
            std::vector<int8_t> costs(width * height);
            std::vector<int8_t> costmap = GetCostmapFromMatlab(costs, width, height, res, grid_llx, grid_lly);
            
            avt_341::msg::OccupancyGrid grid;
            BuildOccupancyGrid(grid, width, height, grid_llx, grid_lly, res, costmap);
            seg_grid_pub->publish(grid);
        }
        node->spin_some();
        rate.sleep();
    }

    mclTerminateApplication();
    return 0;
}