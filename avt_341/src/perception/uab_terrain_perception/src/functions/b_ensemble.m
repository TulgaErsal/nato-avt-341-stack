%% UAB AV Perception Ensemble
% Dr. S. D. Gardner
% 
% UAB - EITD - ARC
% 
% Updated: January 2024

setenv('ROS_MASTER_URI', 'http://localhost:11311'); % local computer
%setenv('ROS_MASTER_URI', 'http://xavier-a:11311'); % xavier-a
rosinit;
%%
cameraSub = rossubscriber('/left_camera/image_raw/compressed', 'sensor_msgs/CompressedImage', ...
                          @cameraCallback, 'DataFormat', 'struct');

lidarSub = rossubscriber('/os_cloud_node_front/points', 'sensor_msgs/PointCloud2', ...
                          @lidarCallback, 'DataFormat', 'struct');

% lidarSub = rossubscriber('/filtered_points', 'sensor_msgs/PointCloud2', ...
%                           @lidarCallback, 'DataFormat', 'struct');

odomSub = rossubscriber('/nav/odom', 'nav_msgs/Odometry', ...
                          @odomCallback, 'DataFormat', 'struct');

% odomSub = rossubscriber('/lvi_sam/lidar/mapping/odometry_incremental', 'nav_msgs/Odometry', ...
%                           @odomCallback, 'DataFormat', 'struct');

% odomSub = rossubscriber('/avt_341/odometry', 'nav_msgs/Odometry', ...
%                           @odomCallback, 'DataFormat', 'struct');

% Output occupancy grid publishers
obstacle_pub = rospublisher('/agv1/avt_341/occupancy_grid', 'nav_msgs/OccupancyGrid');
terrain_pub = rospublisher('/agv1/avt_341/segmentation_grid', 'nav_msgs/OccupancyGrid');

global latestCameraMsg latestLidarMsg latestOdomMsg

latestCameraMsg = [];
latestLidarMsg = [];
latestOdomMsg = [];

% Settings
%% Model Selection
% Semantic segmentation model
% AECESN or DeepLabV3
modelParams = 'ae_net_RUGD_RELLIS_VS.mat';

% AECESN (trained on MSU virtual and real-world data)
% modelParams = 'ae_net_REL_RUG_VS_MSU_NorthFarm.mat';

% AECESN (trained on MSU virtual data)
% modelParams = 'ae_net_REL_RUG_VS_MSUVS.mat';
segmentationModel = AECESN(modelParams);

% Lidar processing model
% KNN or LESN
pcPostProcessingEnabled = true;
lidarProcessingModel = KNN(pcPostProcessingEnabled);

% Map Setttings
grid_width = 1971; % Should add "auto-crop"
grid_height = 3696;
grid_res = 1; % Resolution 0.5 in other scripts?
grid_llx = -400;
grid_lly = -1000;

% Initialize Maps
gridSize = [grid_width,grid_height];
defaultTerrainCellVal = 0.6;
defaultObstacleCellVal = 0.5;
terrainMatrix = defaultTerrainCellVal*ones(gridSize);
obstacleMatrix = defaultObstacleCellVal*ones(gridSize);

%% Initialize Occupancy Grid
gridBuilder = GridBuilder(grid_width, ...
                            grid_height, ...
                            grid_res, ...
                            grid_llx, ...
                            grid_lly, ...
                            defaultTerrainCellVal, ...
                            defaultObstacleCellVal);

debug = true;
lidarDebug = false;

if debug
    runCount = 1;
end
%%
latestCameraMsg = [];
latestLidarMsg = [];
latestOdomMsg = [];
runCount = 1;
imgMsgReceived = false;

while true
    if ~isempty(latestCameraMsg) && ~isempty(latestLidarMsg) && ~isempty(latestOdomMsg)

        rosImgMsg = latestCameraMsg;
        rosLidarMsg = latestLidarMsg;
        rosOdomMsg = latestOdomMsg;

        % todo: there has to be a better way to do this
        % Get camera intrinsics and camera-lidar sensor transform
        if ~imgMsgReceived % we only want to do this once
            rotation = [0.548967924386481,-0.496937965808561,0.477152914019004,-0.473299031032202];
            %translation = [1.46091,-0.3205873,-1.00284]; % top lidar to left camera
            %translation = [1.46091,0.2704127,-1.00284]; % top lidar to right camera
            translation = [-0.0391,-0.3005,0.0218]; % front lidar to left camera

            rosImg = rosReadImage(rosImgMsg);
            imgSize = size(rosImg, 1:2);
            imgWidth = imgSize(2);
            imgHeight = imgSize(1);
            sensorInfo = SensorInfo(imgWidth, imgHeight, rotation, translation);
            imgMsgReceived = true;
        end
        
        [terrainSubGrid,terrainSubgridSize,terrainModifiedCellIdxs,...
            obstacleSubGrid,obstacleSubgridSize,obstacleModifiedCellIdxs,...
            cam,segmentedCam,lidar,segmentedPC] = main(rosImgMsg, rosLidarMsg, rosOdomMsg, ...
                                                        sensorInfo, ...
                                                        gridBuilder, ...
                                                        segmentationModel, ...
                                                        lidarProcessingModel);

        if terrainSubGrid ~= 0
            % insert subgrids into main terrain grid
            c = 1;
            for x = 1:size(terrainSubGrid,1)
                idx = terrainModifiedCellIdxs(x);
                % update value at this index from incoming subgrid
                [row, col] = ind2sub(gridSize, idx);
                val = terrainSubGrid(c)/100;
                terrainMatrix(row, col) = val;
                c = c + 1;
            end

            % insert subgrids into main obstacle grid
            c = 1;
            for x = 1:size(obstacleSubGrid,1)
                idx = obstacleModifiedCellIdxs(x);
                % update value at this index from incoming subgrid
                [row, col] = ind2sub(gridSize, idx);
                val = obstacleSubGrid(c)/100;

                % only include this cell in the main grid if we're fairly
                % confident this is an obstacle
                if val >= 95
                    obstacleMatrix(row, col) = val;
                end
                c = c + 1;
            end

            if debug
                camSmall = imresize(cam,[300,480]);
                figure(1)
                subplot(1,3,1);
                imshow(camSmall);
                hold on
                h = imshow(segmentedCam);
                alpha(h,0.2);
                hold off
                title(['Run Count: ', num2str(runCount)]);

                % subplot(4, 1, 2);
                % pcshow(lframe,ViewPlane="XY");

                subplot(1,3,2);
                terrainGrid = occupancyMap(terrainMatrix);
                show(terrainGrid);
                xlim([900,1150])
                ylim([1500,1620])
                title('Terrain Map');

                subplot(1,3,3);
                obstacleGrid = occupancyMap(obstacleMatrix);
                show(obstacleGrid);
                xlim([900,1150])
                ylim([1500,1620])
                title('Obstacle Map');

                if lidarDebug
                    figure(2)
                    subplot(1,2,1)
                    pcshow(lidar);
                    title('LIDAR Cloud');

                    subplot(1,2,2)
                    pcshow(segmentedPC);
                    title('Segmented LIDAR Cloud');
                end

                drawnow;
            end
        end

        %% Build and publish the maps %%
        obstacleGridMsg = rosmessage(obstacle_pub);
        terrainGridMsg = rosmessage(terrain_pub);

        obstacleGridMsg.Data = obstacleMatrix * 100;
        obstacleGridMsg.Info.Resolution = grid_res;
        obstacleGridMsg.Info.Width = grid_width; 
        obstacleGridMsg.Info.Height = grid_height;
        obstacleGridMsg.Info.Origin.Position.X = grid_llx;
        obstacleGridMsg.Info.Origin.Position.Y = grid_lly;
        obstacleGridMsg.Info.Origin.Position.Z = 0;
        obstacleGridMsg.Info.Origin.Orientation.W = 1; % No rotation

        terrainGridMsg.Data = terrainMatrix * 100;
        terrainGridMsg.Info.Resolution = grid_res;
        terrainGridMsg.Info.Width = grid_width; 
        terrainGridMsg.Info.Height = grid_height;
        terrainGridMsg.Info.Origin.Position.X = grid_llx;
        terrainGridMsg.Info.Origin.Position.Y = grid_lly;
        terrainGridMsg.Info.Origin.Position.Z = 0;
        terrainGridMsg.Info.Origin.Orientation.W = 1; % No rotation

        send(obstacle_pub, obstacleGridMsg);
        send(terrain_pub, terrainGridMsg);

        runCount = runCount + 1;

        latestCameraMsg = [];
        latestLidarMsg = [];
        latestOdomMsg = [];
    end

    pause(0.0001);
end
%%
function lidarCallback(~, message)
    global latestLidarMsg
    latestLidarMsg = message;
end

function cameraCallback(~, message)
    global latestCameraMsg
    latestCameraMsg = message;
end

function odomCallback(~, message)
    global latestOdomMsg
    latestOdomMsg = message;
end