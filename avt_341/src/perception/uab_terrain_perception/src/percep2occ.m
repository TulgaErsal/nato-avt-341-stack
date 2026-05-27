function [costmap, terrainModifiedCellIdxs, obstacleMatrix, obstacleModifiedCellIdxs] = ...
    percep2occ(ptCloudOut, ...
                    lidarToVboxTform,...
                    invertLidarZRot, ...
                    position, ...
                    orientation, ...
                    gridBuilder, ...
                    convertNEDToENU)
persistent lidarPoseVbox
if convertNEDToENU
    %% Transform Point Cloud from LIDAR Frame to Global Frame
    % Write VBOX as Homogeneous Matrix in ENU
    poseVbox = eye(4);
    rotm = quat2rotm(quaternion(orientation));
    poseVbox(1:3, 1:3) = rotm;
    poseVbox(1:3, 4) = position';
    
    % Obtain LIDAR Pose in Global Coordinates
    % Get LIDAR Pose with respect to VBOX
    if isempty(lidarPoseVbox)
        if (invertLidarZRot)
            % LIDAR is pointing backwards
            tCorrection =  eye(4);
            tCorrection(1:3, 1:3) = [-1, 0, 0;
                0, -1, 0;
                0, 0, 1];
            lidarPoseVbox = lidarToVboxTform * tCorrection;
        else
            lidarPoseVbox = lidarToVboxTform;
        end
        disp('Loaded LIDAR to VBOX TFORM')
    end
    lidarPose = poseVbox * lidarPoseVbox;
    lidarTFORM = rigidtform3d(lidarPose);
    ptCloudOut = pctransform(ptCloudOut, lidarTFORM);
else
    % no NED -> ENU conversion
    pose_quaternion = quaternion(orientation);
    rot2global = rotmat(pose_quaternion, 'frame');
    tform2Global = rigid3d(rot2global, position);
    ptCloudOut = pctransform(ptCloudOut, tform2Global);
end

% % Fit plane to filter overhangs
% [~,inplaneidx,~] = pcfitplane(ptCloudOut,2.5);
% ptCloudOut = select(ptCloudOut,inplaneidx);
% Divide Point Cloud by Class
% Would be easier with pure RGB or Gray Channel
redidx = ptCloudOut.Color(:, 1) == 255;
greenidx = ptCloudOut.Color(:, 2) == 255;
blueidx = ptCloudOut.Color(:, 3) == 255;

% Trail
trailIdx = find(~redidx & greenidx);
trailPC = select(ptCloudOut, trailIdx);

% Grass
grassIdx = find(redidx & greenidx);
grassPC = select(ptCloudOut, grassIdx);

% Vegetation
orangeIdx = find(redidx & ptCloudOut.Color(:, 2) == 165 & ptCloudOut.Color(:, 3) == 0);
vegetationPC = select(ptCloudOut, orangeIdx);

% Obstruction (Obstruction or Sky)
% May need to add sky if decided to have different value.
obstructionidx = find((redidx & ~greenidx) | (greenidx & blueidx));
obstructionPC = select(ptCloudOut, obstructionidx);

%% Update Obstacle & Terrain Maps
[lowTraff_x, lowTraff_y] = gridBuilder.UpdateLowTraffCells(vegetationPC.Location);
[medTraff_x, medTraff_y] = gridBuilder.UpdateMediumTraffCells(grassPC.Location);
[highTraff_x, highTraff_y] = gridBuilder.UpdateHighTraffCells(trailPC.Location);
[obstruction_x, obstruction_y] = gridBuilder.UpdateObstructionCells(obstructionPC.Location);

% Retrieve all trafficability cells that were updated
highTraffCroppedArea = [highTraff_x, highTraff_y];
medTraffCroppedArea = [medTraff_x, medTraff_y];
lowTraffCroppedArea = [lowTraff_x, lowTraff_y];
croppedArea = unique([highTraffCroppedArea; medTraffCroppedArea; lowTraffCroppedArea], 'rows');
[costmap, terrainModifiedCellIdxs] = gridBuilder.GetCostmap(croppedArea);

% Retrieve all cells (terrain and obstruction) that were updated
% A cell classified as "high", "medium", or "low" trafficability tells us that there's no
% obstruction at that location
obstacleCroppedArea = unique([obstruction_x, obstruction_y], 'rows');
obstacleCroppedArea = unique([lowTraffCroppedArea; medTraffCroppedArea; highTraffCroppedArea; obstacleCroppedArea], 'rows');
[obstacleMatrix, obstacleModifiedCellIdxs] = gridBuilder.GetCroppedObstacleGrid(obstacleCroppedArea);
end