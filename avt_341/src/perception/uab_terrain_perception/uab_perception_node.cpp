#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/lib_uab_perception_wrapper.h"
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

class Grid {
public:
    Grid() {
        width = 200.0f;
        height = 200.0f;
        llx = -100.0f;
        lly = -100.0f;
        res = 0.5f;
        default_val = 0;
        ResizeGrid();
    }

    Grid(double w, double h, double x, double y, double resolution, int8_t init_val) : 
        width(w),
        height(h),
        llx(x),
        lly(y),
        res(resolution),
        default_val(init_val)
    {
        ResizeGrid();
    }

    void SetSize(float w,float h){
        width = w;
        height = h;
        ResizeGrid();
    }

    void SetRes(float r){
        res = r;
        ResizeGrid();
    }

    void ResizeGrid(){
        nx = (int)ceil(width/res);
        ny = (int)ceil(height/res);
        std::vector<int8_t> row;
        row.resize(nx,default_val);
        cells.resize(ny,row);
    }

    void ClearGrid(){
        for (int i=0;i<(ny);i++){
            for (int j=0;j<(nx);j++){ 
                cells[i][j] = default_val;
            }
        }
    }

    void AddData(std::vector<int8_t> dataVals, std::vector<double> dataIndices, double nx_in, double ny_in, double llx_in, double lly_in, double res_in) {
        int c = 0;
        for (auto i : dataIndices) {
            // Determine data cell coordinates in "map" space [meters]
            int yi_in = int(i/nx_in);
            int xi_in = i-yi_in*nx_in;
            double x_in = xi_in*res_in + llx_in;
            double y_in = yi_in*res_in + lly_in;

            // Determine if data is contained within grid
            if (x_in < llx || x_in > llx + width || y_in < lly || y_in > lly + height) {
                c++;
                continue;
            }

            // Copy data
            int xi = (x_in-llx)/res;
            int yi = (y_in-lly)/res;
            cells[yi][xi] = dataVals[c++];
        }
    }

    avt_341::msg::OccupancyGrid GetGrid(){
        avt_341::msg::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.info.resolution = res;
        grid.info.width = nx;
        grid.info.height = ny;
        grid.info.origin.position.x = llx;
        grid.info.origin.position.y = lly;
        grid.info.origin.orientation.w = 1.0;
        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;
        
        for (auto& row : cells) {
            grid.data.insert(std::end(grid.data), std::begin(row), std::end(row));
        }
        
        return grid;
    }

    avt_341::msg::OccupancyGrid GetGrid(double w, double h, double x, double y) {
        int xi_min = std::max(0,(int)((x-llx)/res));
        int yi_min = std::max(0,(int)((y-lly)/res));
        int xi_max = std::min(nx,xi_min+(int)(w/res));
        int yi_max = std::min(ny,yi_min+(int)(h/res));
        int local_nx = xi_max-xi_min;
        int local_ny = yi_max-yi_min;

        avt_341::msg::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.info.resolution = res;
        grid.info.width = local_nx;
        grid.info.height = local_ny;
        grid.info.origin.position.x = xi_min*res+llx;
        grid.info.origin.position.y = yi_min*res+lly;
        grid.info.origin.orientation.w = 1.0;
        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;

        grid.data.resize(local_nx*local_ny);

        int c = 0;
        for (int j = yi_min; j < yi_max; j++) {
            for (int i = xi_min; i < xi_max; i++) {
                grid.data[c++] = cells[j][i];
            }
        }

        return grid;
    }

private:
    // Grid Params
    double width, height;                       // Overall grid dimensions [m]
    double res;                                 // Grid resolution [m/cell]
    double llx, lly;                            // Origin in "map" frame [m]
    int8_t default_val;                         // Default cell value
    // Vars
    std::vector<std::vector<int8_t>> cells;     // Map cells container
    int nx, ny;                                 // Overall grid dimensions [cells]
};

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
* the semantic segmentation model in Matlab, and populates an array of cost values based on
* terrain traversability
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
    std::vector<uint8_t> rawPcData(std::begin(pc.data), std::end(pc.data));
    mwArray pcData(1, rawPcData.size(), mxUINT8_CLASS);
    pcData.SetData(&rawPcData.front(), rawPcData.size());

    //pointcloud properties
    mwArray pcWidth(pc.width);
    mwArray pcHeight(pc.height);
    mwArray pcPointStep(pc.point_step);
    mwArray pcRowStep(pc.row_step);

    //raw image
    std::vector<uint8_t> rawImgData(std::begin(img.data), std::end(img.data));
    mwArray imgData(1, rawImgData.size(), mxUINT8_CLASS);
    imgData.SetData(&rawImgData.front(), rawImgData.size());

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
                            mwGridWidth, mwGridHeight, mwGridRes, llx, lly //grid params
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

    float grid_width, grid_height;
    node->get_parameter("/grid_width", grid_width, 200.0f);
    node->get_parameter("/grid_height", grid_height, 200.0f);
    float width;
    node->get_parameter("~max_width", width, 100.0f);
    float height;
    node->get_parameter("~max_height", height, 100.0f);
    float res;
    node->get_parameter("~grid_res", res, 1.0f);
    bool publishUabOccupancyGrid;
    node->get_parameter("~publish_uab_occupancy_grid", publishUabOccupancyGrid, false);

    double nx = width/res;
    double ny = height/res;
    double grid_llx = 0.0;
    double grid_lly = 0.0;

    //initialize matlab runtime
    if (!mclInitializeApplication(NULL, 0))
    {
        std::cerr << "Failed to initialize Matlab Runtime" << std::endl;
        return -1;
    }

    //initialize uab matlab perception model
    if(!lib_uab_perception_wrapperInitialize())
    {
        std::cerr << "Failed to initialize perception_wrapper package" << std::endl;
        return -1;
    }

    // Initialize grids
    Grid terrainGrid(grid_width, grid_height, 0.0, 0.0, res, TERRAIN_GRID_DEFAULT_VAL);
    Grid obstacleGrid(grid_width, grid_height, 0.0, 0.0, res, OBSTACLE_GRID_DEFAULT_VAL);
    
    avt_341::node::Rate rate(100.0);
    //number of seconds to wait for messages before exiting
    uint16_t timeout = 20;
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
            // Determine local origin
            grid_llx = current_pose.pose.pose.position.x - width/2.0;
            grid_lly = current_pose.pose.pose.position.y - height/2.0;

            std::vector<int8_t> terrainSubGrid;
            std::vector<double> terrainSubGridIdxs;
            std::vector<int8_t> obstacleSubGrid;
            std::vector<double> obstacleSubGridIdxs;
            GetCostmapFromMatlab(nx, ny, res, grid_llx, grid_lly, terrainSubGrid, terrainSubGridIdxs, obstacleSubGrid, obstacleSubGridIdxs);

            // Update terrain grid
            terrainGrid.AddData(terrainSubGrid, terrainSubGridIdxs, nx, ny, grid_llx, grid_lly, res);
            seg_grid_pub->publish(terrainGrid.GetGrid(width, height, grid_llx, grid_lly));

            if (publishUabOccupancyGrid)
            {
                // Apply threshold to obstacles
                int c = 0;
                std::vector<int8_t> obstacleSubGridThres;
                std::vector<double> obstacleSubGridIdxsThres;
                for (auto i : obstacleSubGridIdxs)
                {
                    double obstacleThreshold = 95.0;
                    double val = obstacleSubGrid[c++];
                    if (val >= obstacleThreshold)
                    {
                        obstacleSubGridThres.push_back(val);
                        obstacleSubGridIdxsThres.push_back(i);
                    }
                }
                // Update obstacle grid
                obstacleGrid.AddData(obstacleSubGridThres, obstacleSubGridIdxsThres, nx, ny, grid_llx, grid_lly, res);
                occ_grid_pub->publish(obstacleGrid.GetGrid(width, height, grid_llx, grid_lly));
            }
        }
        node->spin_some();
        rate.sleep();
    }

    mclTerminateApplication();
    return 0;
}
