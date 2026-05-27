%% Load Bag
clear all % yes, this is necessary
%bagPath = 'D:\Datasets\240717 LW Sensor Data\240717 LW Sensor Data\240717_mrzr_lw_sensors/240717_mrzr_lw_sensors_0.db3';
bagPath = '../Bags\KRC\Real\240717 LW Sensor Data\240717_mrzr_lw_sensors';
bag = ros2bag(bagPath);
lidar_select = select(bag,"Topic","ouster/points");
lidar_msgs_in = readMessages(lidar_select);
camera_select = select(bag,"Topic","flir_camera/image_raw");
camera_msgs_in = readMessages(camera_select);

%% Format VBOX data
%filename = 'C:\Users\stevendg\Desktop\UAB_EITD_ARC\Projects\2024_ARC\PerceptionDemo\PerceptionDemo\Bags\KRC\vbox.csv'; % LWS
filename = '../Bags/KRC/Real/vbox.csv'; % Leader Follower
vboxTable = readtable(filename);
lat = vboxTable.position_latitude_48bit;
lon = vboxTable.position_longitude_48bit;
alt = 0;
origin = [lat(1), lon(1), alt];
[xVBOX,yVBOX] = latlon2local(lat,lon,alt,origin);
vboxTable.x = xVBOX;
vboxTable.y = yVBOX;

% VBOX Yaw Rotations in NED
yawVboxNED = vboxTable.heading_imu;
rollVboxNED = vboxTable.roll_ang_imu;   
pitchVboxNED = vboxTable.pitch_ang_imu; 

% Transform Rotations to ENU
yawVbox = deg2rad(90-yawVboxNED);
pitchVbox = deg2rad(pitchVboxNED);
rollVbox = deg2rad(-rollVboxNED);

eul = [yawVbox, pitchVbox, rollVbox];

quaternion = eul2quat(eul, 'ZYX'); % euler to quaternion
vboxTable.q_w = quaternion(:,1);
vboxTable.q_x = quaternion(:,2);
vboxTable.q_y = quaternion(:,3);
vboxTable.q_z = quaternion(:,4);

%% 
[xyz,quats,camera_msgs,lidar_msgs] = SyncOdom2Sensors_KRC(vboxTable,camera_msgs_in,lidar_msgs_in,alt);

%% Loop Through Frames
% Settings
step = 4;
grid_width = 1971;
grid_height = 3696;
grid_res = 1;
grid_llx = -1200;
grid_lly = -2200;

%% Model Selection
% Semantic segmentation model
% AECESN or DeepLabV3
%modelParams = 'ae_net_RUGD_RELLIS_VS.mat';

% AECESN (trained on MSU virtual and real-world data)
% modelParams = 'ae_net_REL_RUG_VS_MSU_NorthFarm.mat';

% AECESN (trained on MSU virtual data)
% modelParams = 'ae_net_REL_RUG_VS_MSUVS.mat';
%segmentationModel = AECESN(modelParams);

% CAE
modelParams = 'sae_net_5class_RUGD_GOD_544_688_package.mat';
segmentationModel = CAE(modelParams);

% DeepLab
% modelParams = 'deepLab_RUGD_5class_10_17_24_package.mat';
% segmentationModel = DeepLabV3(modelParams);

% Lidar processing model
% KNN or LESN
pcPostProcessingEnabled = true;
lidarProcessingModel = KNN(pcPostProcessingEnabled);

if isempty(camera_msgs)
    fprintf("No camera messages found.");
    return
end

% Get camera intrinsics and camera-lidar sensor transform
translation = [0,0,0];
rotation = [0.5, -0.5, 0.5, -0.5]; % Need explanation for this

KRC_flg = 1; % set to 0 if not using KRC data
imgWidth = camera_msgs{1}.width;
imgHeight = camera_msgs{1}.height;
sensorInfo = SensorInfo(imgWidth, imgHeight);

debug = true;

%% Initialize Occupancy Grid
gridSize = [grid_width, grid_height] / grid_res;
defaultTerrainCellVal = 75;
defaultObstacleCellVal = 50;
terrainMatrix = defaultTerrainCellVal * ones(gridSize);
obstacleMatrix = defaultObstacleCellVal * ones(gridSize);

% confidence values
lowConfidenceVal = 0.3;
highConfidenceVal = 0.7;

% trafficability costs (pretty arbitrary)
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
                            highTraffCost, ...
                            mediumTraffCost, ...
                            lowTraffCost);

numMsgs = min(length(camera_msgs),length(lidar_msgs));

for i = 100:step:2967
    if any(isnan(xyz(i,:))) || any(isnan(quats(i,:)))
        continue
    end

    disp(i)

    % camera info
    cameraInfo = struct();
    cameraInfo.focalLength = [364.5151 364.6252];
    cameraInfo.principalPoint = [309.2227 258.8534];
    cameraInfo.width = 512;
    cameraInfo.height = 608;
    cameraInfo.radialDistortion = [-0.0296; 0.0638];
    cameraInfo.tangentialDistortion = [0; 0];
    cameraInfo.skew = 0;
    cameraInfo.k = [364.5151, 0, 309.2227, 0, 364.6252, 258.8534, 0, 0, 1];

    % camera -> lidar tform
    translation = [-0.018065188231544,-0.088574917569181,-0.232218571180658];
    R = [[0.000371587597833468,	0.999891931230839,	0.0146965227901230]
        [-0.0780352323915932,	0.0146807020407715,	-0.996842504858807]
        [-0.996950532588376,	-0.000776432259476824,	0.0780322543868833]];
    cameraToLidarTform = rigidtform3d(R, translation);

    % lidar -> vbox tform
    lidarToVboxTform = [
        [0.955131033020617,	0.00497294897875304,	0.296141823353988,	0.238900000000000]
        [-0.00207811969980224,	0.999946944194438,	-0.0100891136736027,	0.0147574000000000]
        [-0.296176283958529,	0.00902100740828325,	0.955090692157481,	1.57416100000000]
        [0	0	0	1]
    ];

    pc = lidar_msgs{i};
    img = camera_msgs{i};
    odom = BuildRosOdometryMsg(xyz(i,1), xyz(i,2), xyz(i,3), quats(i,1), quats(i,2), quats(i,3), quats(i,4));
    [terrainSubGrid, terrainSubgridSize, terrainModifiedCellIdxs, ...
        obstacleSubGrid, obstacleSubgridSize, obstacleModifiedCellIdxs, cam, seg] = main(img, pc, ...
                                                                                        cameraInfo,...
                                                                                        cameraToLidarTform,...
                                                                                        lidarToVboxTform,...
                                                                                        1, ...
                                                                                        odom, ...
                                                                                        gridBuilder, ...
                                                                                        segmentationModel, ...
                                                                                        lidarProcessingModel);

    % insert subgrids into respective main grid
    c = 1;
    for x = 1:terrainSubgridSize
        idx = terrainModifiedCellIdxs(x);
        % update value at this index from incoming subgrid
        [row, col] = ind2sub(gridSize, idx);
        val = terrainSubGrid(c);
        terrainMatrix(row, col) = val;
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
        tiledlayout(2,2);
        nexttile;

        title('Camera Image')
        smallCam = imresize(cam,[size(seg,1),size(seg,2)]);
        imshow(smallCam);

        nexttile;

        % display terrain grid
        terrainGrid = occupancyMap(terrainMatrix / 100);
        show(terrainGrid);
        xlim([xrange(1) xrange(2)]);
        ylim([yrange(1) yrange(2)]);
        title('Terrain Map');

        nexttile;
    
        title('Camera Segmentation')
        smallCam = imresize(cam,[size(seg,1),size(seg,2)]);
        imshow(smallCam);
        hold on;
        h = imshow(seg);
        set(h, 'AlphaData', 0.5);
        hold off;

        nexttile;

        % display obstacle grid
        obstacleGrid = occupancyMap(obstacleMatrix / 100);
        show(obstacleGrid);
        title('Obstacle Map');
        xlim([xrange(1) xrange(2)]);
        ylim([yrange(1) yrange(2)]);

        % nexttile;

        % merge grids together and display
        % h = 1/6+(1-terrainMatrix)/6;
        % s = ones(size(h));
        % v = (1-obstacleMatrix);
        % hsv = cat(3,h,s,v);
        % rgb = hsv2rgb(hsv);
        % imshow(rgb)
        % title('Merged Map')
        % xlim([xrange(1) xrange(2)]);
        % ylim([yrange(1)-100 yrange(2)-100]);

        drawnow;
    end
end