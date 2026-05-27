%% Load Bag
clear all %#ok<CLALL> This is necessary.
% Bag with Odometry
bagPath = "Bags/KRC/Real/250717_terrain_segmentation_validation/FullFrame";
bag = ros2bagreader(bagPath);
selectTime = [bag.StartTime + 65, bag.EndTime];
odom_select = select(bag,"Topic","mrzr/avt_341/odometry");
odom_msgs = readMessages(odom_select);
lidar_select = select(bag,"Topic","ouster/points", "Time", selectTime);
lidar_msgs = readMessages(lidar_select);
camera_select = select(bag,"Topic","/flir_camera/image_raw", "Time", selectTime);
camera_msgs = readMessages(camera_select);
% Sync Messages
[xyz, quats, camera_msgs, lidar_msgs] = SyncOdom2Sensors(odom_msgs, camera_msgs, lidar_msgs);
%% Loop Through Frames
% Settings
step = 1;
grid_width = 2955;
grid_height = 2290;
grid_res = 1.0;
grid_llx = 0;
grid_lly = 0;

%% Model Selection
% Semantic segmentation model
% AECESN, CAE, or DeepLabV3
modelParams = 'sae_net_5class_RUGD_GOD_544_688_package.mat';

% AECESN (trained on MSU virtual and real-world data)
% modelParams = 'ae_net_REL_RUG_VS_MSU_NorthFarm.mat';

% AECESN (trained on MSU virtual data)
% modelParams = 'ae_net_REL_RUG_VS_MSUVS.mat';
% segmentationModel = AECESN(modelParams);

segmentationModel = CAE(modelParams);

% Lidar processing model
% KNN or LESN
pcPostProcessingEnabled = true;
lidarProcessingModel = KNN(pcPostProcessingEnabled);

if isempty(camera_msgs)
    fprintf("No camera messages found.");
    return
end

% Get camera intrinsics and camera-lidar sensor transform
% translation = [0,0,0];
% rotation = [0.5, -0.5, 0.5, -0.5]; % Need explanation for this
% imgWidth = camera_msgs{1}.width;
% imgHeight = camera_msgs{1}.height;
% sensorInfo = SensorInfo(imgWidth, imgHeight);
cameraInfo = struct();
cameraInfo.width = 540;
cameraInfo.height = 960;
cameraInfo.k = [479.99993896484375, 0.0, 480.0, 0.0, 479.99993896484375, 270.0, 0.0, 0.0, 1.0];

% translation = [-0.018065188231544,-0.088574917569181,-0.232218571180658];
% R = [[0.000371587597833468,	0.999891931230839,	0.0146965227901230]
%     [-0.0780352323915932,	0.0146807020407715,	-0.996842504858807]
%     [-0.996950532588376,	-0.000776432259476824,	0.0780322543868833]];
% cameraToLidarTform = rigidtform3d(R, translation);
R = [0 -1 0
     0 0 -1
     1 0 0];
translation = [0.0, -0.3513, -1.3];

cameraToLidarTform = rigidtform3d(R, translation);

% lidar -> vbox tform
% lidarToVboxTform = [
%     [0.955131033020617,	0.00497294897875304,	0.296141823353988,	0.238900000000000]
%     [-0.00207811969980224,	0.999946944194438,	-0.0100891136736027,	0.0147574000000000]
%     [-0.296176283958529,	0.00902100740828325,	0.955090692157481,	1.57416100000000]
%     [0	0	0	1]
% ];
lidarToVboxTform = [
    [1 0 0 0]
    [0 1 0 0]
    [0 0 1 2.2513]
    [0 0 0 1]
];

debug = true;

%% Initialize Occupancy Grid
gridSize = [grid_width, grid_height] / grid_res;
defaultTerrainCellVal = 75; % unknown areas - less trafficable than low trafficable
defaultObstacleCellVal = 50;
costmap = defaultTerrainCellVal * ones(gridSize);
obstacleMatrix = defaultObstacleCellVal * ones(gridSize);

% confidence values
lowConfidenceVal = 0.3;
highConfidenceVal = 0.7;

% trafficability costs
lowTraffCost = 50;
mediumTraffCost = 33;
highTraffCost = 1;

gridBuilder = GridBuilder(grid_width, ...
                            grid_height, ...
                            grid_res, ...
                            grid_llx, ...
                            grid_lly, ...
                            defaultTerrainCellVal / 100, ... % scale down for occupancyMap
                            defaultObstacleCellVal / 100, ... %          ^^
                            highConfidenceVal, ...
                            lowConfidenceVal, ...
                            lowTraffCost, ...
                            mediumTraffCost, ...
                            highTraffCost);

numMsgs = min(length(camera_msgs),length(lidar_msgs));

for i = 1:step:numMsgs
    pc = lidar_msgs{i};
    img = camera_msgs{i};
    odom = BuildRosOdometryMsg(xyz(i,1), xyz(i,2), xyz(i,3), quats(i,1), quats(i,2), quats(i,3), quats(i,4));
    [costmapSubGrid, costmapSubGridSize, costmapModifiedIdxs, ...
        obstacleSubGrid, obstacleSubgridSize, obstacleModifiedCellIdxs, cam, seg] = main(img, pc, ...
                                                                                        cameraInfo,...
                                                                                        cameraToLidarTform,...
                                                                                        lidarToVboxTform,...
                                                                                        1, ...
                                                                                        odom, ...
                                                                                        gridBuilder, ...
                                                                                        segmentationModel, ...
                                                                                        lidarProcessingModel);

    if (costmapSubGrid == 0)
        continue;
    end
    
    % insert subgrids into respective main grid
    c = 1;
    for x = 1:costmapSubGridSize
        idx = costmapModifiedIdxs(x);
        % update value at this index from incoming subgrid
        [row, col] = ind2sub(gridSize, idx);
        val = costmapSubGrid(c);
        costmap(row, col) = val;
        c = c + 1;
    end

    c = 1;
    for x = 1:obstacleSubgridSize
        idx = obstacleModifiedCellIdxs(x);
        % update value at this index from incoming subgrid
        [row, col] = ind2sub(gridSize, idx);
        val = obstacleSubGrid(c);
        obstacleMatrix(row, col) = val;
        c = c + 1;
    end

    if debug
        % Find the bounds for filled cells in terrainMatrix
        filledCells = obstacleMatrix ~= defaultObstacleCellVal;
        [rowIndices, colIndices] = find(filledCells);

        % Get the bounding box for non-default cells
        maxRow = grid_width - (min(rowIndices)-1);
        minRow = grid_width - (max(rowIndices)-1);
        minCol = min(colIndices);
        maxCol = max(colIndices);

        xrange = [minCol, maxCol];
        yrange = [minRow, maxRow];

        % Visualizations
        figure(1)
        tiledlayout(2, 4, "TileSpacing", "tight", "Padding", "compact");
        
        nexttile(1);
        title('Camera Segmentation')
        smallCam = imresize(cam,[size(seg,1),size(seg,2)]);
        imshow(smallCam);

        nexttile(5);
        title('Camera Segmentation')
        imshow(smallCam);
        hold on;
        hSeg = imshow(seg);
        set(hSeg, 'AlphaData', 0.3);
        hold off;

        nexttile(2);
        % display obstacle grid
        obstacleGrid = occupancyMap(obstacleMatrix / 100);
        show(obstacleGrid);
        title('Obstacle Map');
        xlim([xrange(1) xrange(2)]);
        ylim([yrange(1) yrange(2)]);
        
        nexttile(6);
        % display terrain grid
        terrainGrid = occupancyMap(costmap / 100);
        show(terrainGrid);
        xlim([xrange(1) xrange(2)]);
        ylim([yrange(1) yrange(2)]);
        title('Terrain Map');

        nexttile(3, [2, 2]);
        % merge grids together and display (MIGHT NOW WORK WITH RES =! 1)
        hue = 1/6 + (1 - costmap/100)/6;
        sat = ones(size(hue));
        value = (1 - obstacleMatrix/100);
        hsv = cat(3,hue,sat,value);
        rgb = hsv2rgb(hsv);
        % Crop in X-direction using xrange
        rgb = rgb(:, xrange(1):xrange(2), :);
        % Convert Cartesian yrange to image coordinates
        % grid_width is the image height in pixels (CONFUSING)
        y_top = grid_width - yrange(2);  % Top row (because image y=0 is top)
        y_bottom = grid_width - yrange(1);  % Bottom row
        rgb = rgb(y_top:y_bottom, :, :);
        imshow(rgb);
        title('Merged Map');

        drawnow;

    end
end