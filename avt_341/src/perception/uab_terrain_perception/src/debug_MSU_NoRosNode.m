%% AV Perception without ROS
% S. D. Gardner; N. Bowen; A. Morales
% ARC - EITD - UAB
clear all % yes, this is necessary
%bag = rosbag('Bags\MSU\May6-2.bag');
bag = rosbag('D:\MSU_Integration_Tests\ridge_2_loam.bag');
%%
% Select Topics. May streamline if using MessageType instead?
%topics = {'/os_cloud_node_front/points','/left_camera/image_raw/compressed','/robot/dlo/odom_node/odom'};
topics = {'/os_cloud_node_front/points','/left_camera/image_raw/compressed','/lvi_sam/lidar/mapping/odometry'};
bagSelect = select(bag,'Topic',topics);
% There's no need to read all messages. FIX?
msgs = readMessages(bagSelect);
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
grid_width = 350; % Should add "auto-crop"
grid_height = 300;
grid_res = 1; % Resolution 0.5 in other scripts?
grid_llx = -20;
grid_lly = -50;

% Set Debug Options
debug = true;
video = false;
lidarDebug = false;

% Initialize Maps
gridSize = [grid_width,grid_height];
defaultTerrainCellVal = 0.6;
defaultObstacleCellVal = 0.5;
terrainMatrix = defaultTerrainCellVal*ones(gridSize);
obstacleMatrix = defaultObstacleCellVal*ones(gridSize);

latestCameraMsg = [];
latestLidarMsg = [];
latestOdomMsg = [];
runCount = 1;

%% Initialize Occupancy Grid
gridBuilder = GridBuilder(grid_width, ...
                            grid_height, ...
                            grid_res, ...
                            grid_llx, ...
                            grid_lly, ...
                            defaultTerrainCellVal, ...
                            defaultObstacleCellVal);
%% Step Over Messages
if debug
    figure('units','normalized','outerposition',[0 0 1 1])
end
if video
    vid = VideoWriter("MSU-ridge.avi"); 
    vid.FrameRate = 10;
    open(vid);
end

imgMsgReceived = false;
for m = 1:length(msgs)
    disp([num2str(100*m/length(msgs)),'%'])
    currentMsg = msgs{m};
    switch currentMsg.MessageType
        case 'sensor_msgs/CompressedImage'
            latestCameraMsg = currentMsg;
            % Need to build?
            rosImgMsg = struct();
            rosImgMsg.MessageType = latestCameraMsg.MessageType;
            rosImgMsg.Header = latestCameraMsg.Header;
            rosImgMsg.Format = latestCameraMsg.Format;
            rosImgMsg.Data = latestCameraMsg.Data;

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
        case 'sensor_msgs/PointCloud2'
            latestLidarMsg = currentMsg;
            rosLidarMsg = struct();
            rosLidarMsg.PreserveStructureOnRead = latestLidarMsg.PreserveStructureOnRead;
            rosLidarMsg.MessageType = latestLidarMsg.MessageType;
            rosLidarMsg.Header = latestLidarMsg.Header;
            rosLidarMsg.Fields = latestLidarMsg.Fields;
            rosLidarMsg.Height = latestLidarMsg.Height;
            rosLidarMsg.Width = latestLidarMsg.Width;
            rosLidarMsg.IsBigendian = latestLidarMsg.IsBigendian;
            rosLidarMsg.PointStep = latestLidarMsg.PointStep;
            rosLidarMsg.RowStep = latestLidarMsg.RowStep;
            rosLidarMsg.Data = latestLidarMsg.Data;
            rosLidarMsg.IsDense = latestLidarMsg.IsDense;
        case 'nav_msgs/Odometry'
            latestOdomMsg = currentMsg;
            rosOdomMsg = struct();
            rosOdomMsg.MessageType = latestOdomMsg.MessageType;
            rosOdomMsg.Header = latestOdomMsg.Header;
            rosOdomMsg.Pose = latestOdomMsg.Pose;
            rosOdomMsg.Twist = latestOdomMsg.Twist;
            rosOdomMsg.ChildFrameId = latestOdomMsg.ChildFrameId;
    end

    if ~isempty(latestCameraMsg) && ~isempty(latestLidarMsg) && ~isempty(latestOdomMsg)
        % cam and vis are temporary outputs! Needed for visuals.
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
                obstacleMatrix(row, col) = val;
                c = c + 1;
            end

            if debug
                camSmall = imresize(cam,[300,480]);
                subplot(1,3,1);
                imshow(camSmall);
                title(['Run Count: ', num2str(runCount)]);

                subplot(1,3,2);
                imshow(camSmall);
                hold on
                h = imshow(segmentedCam);
                alpha(h,0.2);
                hold off
                title('Segmentation');

                % merge grids together and display
                subplot(1,3,3);
                h = 1/6+(1-terrainMatrix)/6;
                s = ones(size(h));
                v = (1-obstacleMatrix);
                hsv = cat(3,h,s,v);
                rgb = hsv2rgb(hsv);
                imshow(rgb)
                title('Merged Map')
                
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
                if video
                    img = getframe(gcf);
                    writeVideo(vid,img)
                end
            end
        end
        runCount = runCount + 1;
        latestCameraMsg = [];
        latestLidarMsg = [];
        latestOdomMsg = [];
    end
end

if video
    close(vid)
end