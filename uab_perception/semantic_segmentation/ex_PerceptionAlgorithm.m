function [costNow] = ex_PerceptionAlgorithm(net, cmap, odom, pc, img)
    fprintf('entered  ex_PerceptionAlgorithm\n');

    sprintf("create rawImage");
    readImage = rosReadImage(img);
    rawImage = imresize(readImage,'OutputSize',[1200 1920]); % resize to match RELLIS
    
    %% Semantic Segmentation
    C = semanticseg(rawImage, net);
    B = labeloverlay(rawImage,C,'Colormap',cmap,'Transparency',0);
    
    %% Fuse camera and lidar
    lidar = rosReadXYZ(pc);
    pcl = pointCloud(lidar);
    
    %% PCL transform:   
    focalLength    = [2813.64322, 2808.326079]; 
    principalPoint = [969.285772, 624.049972];
    imageSize = [double(size(rawImage,1)) double(size(rawImage,2))];
    intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
    
    pose_position = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z];
    pose_quaternion = quaternion(odom.pose.pose.orientation.z, odom.pose.pose.orientation.y, odom.pose.pose.orientation.x, odom.pose.pose.orientation.w);
    
    % true west = [270 -90 0]
    q = quaternion([270 -90 0],'eulerd','ZYX','frame');
    rot = rotmat(q, 'point');
    tform = rigid3d(rot,[0 0 0]);
    
    % fuse the camera to lidar
    ptCloudOut = fuseCameraToLidar(B,pcl,intrinsics,tform);
    
    %% Occupancy Grid
    sensorHeight = pose_position(3);
    vehicleRadius = 1;
    cellSize = 2; % scaling of global grid
    gridSize = [510 160];
    grid_res = 1/cellSize;
    
    % create empty OG:
    costmap = occupancyMap(gridSize(1),gridSize(2),grid_res,'grid');
    % default cell value for occupancyMap is 0.5, set all cells to 0
    for w=1:costmap.GridSize(1)
        for h=1:costmap.GridSize(2)
            setOccupancy(costmap, [w, h], 0.0, 'grid');
        end
    end

    % segment and remove ego vehicle
    egoIndices = findNeighborsInRadius(ptCloudOut,[0 0 0],vehicleRadius);
    egoFixed = false(ptCloudOut.Count,1);
    egoFixed(egoIndices) = true;
    ptCloudOut = select(ptCloudOut,~egoFixed);

    %% Location within the global grid:
   
    %now we need to transform the entire ptCloud to the global reference
    %frame
    %first find the translation vector based on ODM
    trans2global = [-pose_position(1) (gridSize(2)/(2*cellSize))-pose_position(2) pose_position(3)+sensorHeight];%[-pose_position(1) (gridSize(2)/(2*cellSize)-pose_position(2)) sensorHeight];

    %next find the roation vector based on ODM
    rot2global = rotmat(pose_quaternion, 'point');

    %then create the rigid 3d transformation matrix
    tform2Global = rigid3d(rot2global,trans2global);

    %now use the nifty pointclound transform built into Matlab
    ptCloudOut = pctransform(ptCloudOut,tform2Global);
    
    
    costValues = [0;    % sky
                  0.01; % concrete
                  0.02; % dirt
                  0.08; % grass
                  0.1;  % gravel
                  0.2;  % water, mud
                  0.7;  % bush
                  1];   % tree, barrier
    
    for i=1:ptCloudOut.Count
        if ptCloudOut.Location(i,1) ~= 0 && ptCloudOut.Location(i,2) ~= 0
            %determine which cell the lidar point should be added to
            x = floor(ptCloudOut.Location(i,1)/grid_res);
            y = floor(ptCloudOut.Location(i,2)/grid_res);

            if x < gridSize(1) && y < gridSize(2)
                 switch num2str(ptCloudOut.Color(i,:))
                     case num2str([000 000 255]) % sky
                        setOccupancy(costmap,[x y],costValues(1),'grid');
                     case num2str([170 170 170]) % concrete
                        setOccupancy(costmap,[x y],costValues(2),'grid');
                     case num2str([108 064 020]) % dirt
                        setOccupancy(costmap,[x y],costValues(3),'grid');
                     case num2str([000 102 000]) % grass
                        setOccupancy(costmap,[x y],costValues(4),'grid');
                     case num2str([255 128 000]) % gravel
                        setOccupancy(costmap,[x y],costValues(5),'grid');
                     case num2str([000 128 255]) % water
                        setOccupancy(costmap,[x y],costValues(6),'grid'); 
                     case num2str([099 066 034]) % mud
                        setOccupancy(costmap,[x y],costValues(6),'grid'); 
                     case num2str([255 153 204]) % bush7
                        setOccupancy(costmap,[x y],costValues(7),'grid');
                     case num2str([000 255 000]) % tree
                        setOccupancy(costmap,[x y],costValues(8),'grid');
                     case num2str([041 121 255]) % barrier
                        setOccupancy(costmap,[x y],costValues(8),'grid');
                 end
            end
        end
    end
    
    costNow = getOccupancy(costmap);
    costNow = rot90(costNow, 2);
end