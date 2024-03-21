#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/perception_wrapper.h"
#include "mclcppclass.h"
#include "mclmcrrt.h"
#include <vector>
#include <array>
#include <math.h>

const uint8_t TERRAIN_GRID_DEFAULT_VAL = 50;
const uint8_t OBSTACLE_GRID_DEFAULT_VAL = 0;

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
void GetCostmapFromMatlab(float width,
                            float height,
                            float res,
                            float grid_llx,
                            float grid_lly,
                            std::vector<int8_t> &terrainGrid,
                            std::vector<double> &terrainGridModifiedIdxs,
                            std::vector<int8_t> &obstacleGrid,
                            std::vector<double> &obstacleGridModifiedIdxs)
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

    //image width
    mwArray imgWidth(img.width);

    //image height
    mwArray imgHeight(img.height);

    //grid width
    mwArray mwGridWidth(width);

    //grid height
    mwArray mwGridHeight(height);

    //grid resolution
    mwArray mwGridRes(res);

    //lower left corner grid offset in meters (x/east direction)
    mwArray llx(grid_llx);

    //lower right corner grid offset in meters (y/north direction)
    mwArray lly(grid_lly);

    //1, 1 -> original
    //0, 0 -> lidar only esn
    //1, 0 -> mixed
    mwArray C_model(1); mwArray CL_model(0);

    //use DeepLab?
    mwArray DL_model(0);

    try
    {
        //output variables
        mwArray mwTerrainCostmap;
        mwArray mwTerrainSubGridSize;
        mwArray mwTerrainGridModifiedIdxs;
        mwArray mwObstacleCostmap;
        mwArray mwObstacleSubGridSize;
        mwArray mwObstacleGridModifiedIdxs;
        perception_wrapper(
                            6, //number of output arguments
                            mwTerrainCostmap,
                            mwTerrainSubGridSize,
                            mwTerrainGridModifiedIdxs,
                            mwObstacleCostmap,
                            mwObstacleSubGridSize,
                            mwObstacleGridModifiedIdxs,
                            imgData, //camera data
                            imgWidth, //image width
                            imgHeight, //image height
                            pcData, pcWidth, pcHeight, pcPointStep, pcRowStep, //pointcloud data
                            x, y, z, qw, qx, qy, qz, //odometry
                            mwGridWidth, mwGridHeight, mwGridRes, llx, lly, //grid params
                            CL_model, C_model, DL_model //model selection
                        );

        // parse terrain sub grid
        uint32_t terrainSubGridSize = mwTerrainSubGridSize;
        terrainGridModifiedIdxs.resize(terrainSubGridSize);
        mwTerrainGridModifiedIdxs.GetData(terrainGridModifiedIdxs.data(), terrainGridModifiedIdxs.size());

        terrainGrid.resize(terrainSubGridSize);
        mwTerrainCostmap.GetData(terrainGrid.data(), terrainGrid.size());

        // parse obstacle sub grid
        uint32_t obstacleSubGridSize = mwObstacleSubGridSize;
        obstacleGridModifiedIdxs.resize(obstacleSubGridSize);
        mwObstacleGridModifiedIdxs.GetData(obstacleGridModifiedIdxs.data(), obstacleGridModifiedIdxs.size());

        obstacleGrid.resize(obstacleSubGridSize);
        mwObstacleCostmap.GetData(obstacleGrid.data(), obstacleGrid.size());
    }
    catch(const mwException& e)
    {
        std::cerr << "Failed to execute Matlab perception_wrapper. " << e.what() << std::endl;
    }
}

void BuildOccupancyGrid(avt_341::msg::OccupancyGrid &grid,
                        float width,
                        float height,
                        float grid_llx,
                        float grid_lly,
                        float res,
                        float defaultCellVal)
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
    std::vector<int8_t> initVals(width * height, defaultCellVal);
    grid.data = initVals;
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos){
    reset_called = true;
  }
}

int main(int argc, char *argv[])
{
    auto node = avt_341::node::init_node(argc, argv, "uab_perception_node");

    auto odom_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
    auto pc_sub = node->create_subscription<avt_341::msg::PointCloud2>("avt_341/points", 2, PointCloudCallback);
    auto img_sub = node->create_subscription<avt_341::msg::Image>("camera/rgb/image_raw", 10, ImageCallback);
    auto reset_sub = node->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
    
    auto seg_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);
    auto occ_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);
    auto reset_ack_pub = node->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);

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
    bool publishUabOccupancyGrid;
    node->get_parameter("~publish_uab_occupancy_grid", publishUabOccupancyGrid, false);

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

    avt_341::msg::OccupancyGrid terrainGrid;
    BuildOccupancyGrid(terrainGrid, width, height, grid_llx, grid_lly, res, TERRAIN_GRID_DEFAULT_VAL);

    avt_341::msg::OccupancyGrid obstacleGrid;
    BuildOccupancyGrid(obstacleGrid, width, height, grid_llx, grid_lly, res, OBSTACLE_GRID_DEFAULT_VAL);
    
    avt_341::node::Rate rate(100.0);
    uint16_t timeout = 20; //exit if messages not received within 20s
    while (avt_341::node::ok())
    {
        if(reset_called){
            avt_341::msg::String reset_ack_msg;
            reset_ack_msg.data = avt_341::node::NodeType::Perception;
            reset_ack_pub->publish(reset_ack_msg);
            reset_called = false;
        }
            
        if (!allMsgsReceived())
        {
            std::string waitingOn = "waiting on ";
            if (!odom_received) waitingOn += "odom ";
            if (!pc_received) waitingOn += "pointcloud ";
            if (!img_received) waitingOn += "image";
            std::cout << waitingOn << std::endl;
            
            avt_341::node::Rate wait(1.0);
            wait.sleep();

            if (--timeout == 0) break;
        }
        else
        {
            std::vector<int8_t> terrainSubGrid;
            std::vector<double> terrainSubGridIdxs;
            std::vector<int8_t> obstacleSubGrid;
            std::vector<double> obstacleSubGridIdxs;
            GetCostmapFromMatlab(width, height, res, grid_llx, grid_lly, terrainSubGrid, terrainSubGridIdxs, obstacleSubGrid, obstacleSubGridIdxs);

            int c = 0;
            for (auto i : terrainSubGridIdxs)
            {
                terrainGrid.data[i] = terrainSubGrid[c++];
            }

            seg_grid_pub->publish(terrainGrid);

            if (publishUabOccupancyGrid)
            {
                c = 0;
                for (auto i : obstacleSubGridIdxs)
                {
                    double obstacleThreshold = 95.0;
                    double val = obstacleSubGrid[c++];
                    if (val >= obstacleThreshold)
                    {
                        obstacleGrid.data[i] = val;
                    }
                }

                occ_grid_pub->publish(obstacleGrid);
            }
        }
        node->spin_some();
        rate.sleep();
    }

    mclTerminateApplication();
    return 0;
}