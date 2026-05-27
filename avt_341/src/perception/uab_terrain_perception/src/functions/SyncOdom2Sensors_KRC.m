function [xyz,quats,matched_camera_msgs,matched_lidar_msgs] = SyncOdom2Sensors_KRC(odom_msgs,camera_msgs,lidar_msgs,alt)
% Synchronize ROS odometry messages to Camera/LIDAR.
% Returns Synchronized odometry as XYZ and Quaternion.
%   Interpolates odometry to match Camera/LIDAR.
%   This function assumes that Camera/LIDAR are synchronous.
%   Function will crop Camera/LIDAR messages to avoid extrapolation.

% Extract Odometry
% xyz = zeros(height(odom_msgs),3);
% quats = zeros(height(odom_msgs),4);
% for i = 1:height(odom_msgs)
%     % Testing KRC LWS data:
%     xyz(i,:) = [odom_msgs.x(i),odom_msgs.y(i),alt];
%     quats(i,:) = [odom_msgs.q_w(i),odom_msgs.q_x(i),odom_msgs.q_y(i),odom_msgs.q_z(i)];
% end

% Extract Odometry Timestamps
odomTime = zeros(height(odom_msgs),1);
for i = 1:height(odom_msgs)
    nanosec = double(odom_msgs.ts(i));
    odomTime(i) = (10^-9)*nanosec;
end

% Check Odometry Includes Unique Timestamps
if height(odomTime) ~= height(unique(odomTime))
    disp('Warning: Repeated Timestamps in Odometry')
    [odomTime,idx,~] = unique(odomTime);
    %xyz = xyz(idx,:);
    odom_msgs = odom_msgs{idx, 1};
    disp('Fixed: Repeated Timestamps in Odometry')
end

% Extract Sensors' Timestamps. Camera/LIDAR in sync: same index = same pose.
numMsgs = min(length(camera_msgs),length(lidar_msgs));
sensorTime = zeros(numMsgs,1);
for i = 1:numMsgs
    sec = double(camera_msgs{i}.header.stamp.sec);
    nanosec = double(camera_msgs{i}.header.stamp.nanosec);
    sensorTime(i) = sec+(10^-9)*nanosec;
end

% Avoid Extrapolation Programatically
if sensorTime(1) < odomTime(1)
    lowidx = find(sensorTime>odomTime(1),1);
    sensorTime = sensorTime(lowidx:end);
    camera_msgs = camera_msgs(lowidx:end);
    lidar_msgs = lidar_msgs(lowidx:end);
end
if sensorTime(end) > odomTime(end)
    highidx = find(sensorTime>odomTime(end),1)-1;
    sensorTime = sensorTime(1:highidx);
    camera_msgs = camera_msgs(1:highidx);
    lidar_msgs = lidar_msgs(1:highidx);
end

% Ensure # of LiDAR = # of camera
if size(camera_msgs,1) < size(lidar_msgs,1)
    matched_lidar_msgs = cell(size(camera_msgs));
    matched_odom_msgs = odom_msgs([],:);
    
    % Convert camera timestamps to double precision for comparison
    camera_times = zeros(length(camera_msgs), 1);
    for i = 1:length(camera_msgs)
        % Combine seconds and nanoseconds into a single timestamp in seconds
        camera_times(i) = double(camera_msgs{i, 1}.header.stamp.sec) + ...
                          double(camera_msgs{i, 1}.header.stamp.nanosec) * 1e-9;
    end
    
    % Convert lidar timestamps to double precision for comparison
    lidar_times = zeros(length(lidar_msgs), 1);
    for i = 1:length(lidar_msgs)
        lidar_times(i) = double(lidar_msgs{i, 1}.header.stamp.sec) + ...
                         double(lidar_msgs{i, 1}.header.stamp.nanosec) * 1e-9;
    end

    % Convert odom timestamps to double precision for comparison
    odom_times = zeros(height(odom_msgs), 1);
    for i = 1:height(odom_msgs)
        odom_times(i) = double(odom_msgs.ts(i)) * 1e-9;
    end
    
    % Loop through each camera timestamp and find the closest lidar timestamp
    for i = 1:length(camera_times)
        [~, closest_idx_lidar] = min(abs(lidar_times - camera_times(i)));
        [~, closest_idx_odom] = min(abs(odom_times - camera_times(i)));

        matched_lidar_msgs{i, 1} = lidar_msgs{closest_idx_lidar, 1};
        matched_odom_msgs(i, :) = odom_msgs(closest_idx_odom, :);
    end

    matched_camera_msgs = camera_msgs;

else
    matched_camera_msgs = cell(size(lidar_msgs));
    matched_odom_msgs = odom_msgs([],:);

    % Convert lidar timestamps to double precision for comparison
    lidar_times = zeros(length(lidar_msgs), 1);
    for i = 1:length(lidar_msgs)
        % Combine seconds and nanoseconds into a single timestamp in seconds
        lidar_times(i) = double(lidar_msgs{i, 1}.header.stamp.sec) + ...
                          double(lidar_msgs{i, 1}.header.stamp.nanosec) * 1e-9;
    end
    
    % Convert camera timestamps to double precision for comparison
    camera_times = zeros(length(camera_msgs), 1);
    for i = 1:length(camera_msgs)
        camera_times(i) = double(camera_msgs{i, 1}.header.stamp.sec) + ...
                         double(camera_msgs{i, 1}.header.stamp.nanosec) * 1e-9;
    end
    
    % Convert odom timestamps to double precision for comparison
    odom_times = zeros(height(odom_msgs), 1);
    for i = 1:height(odom_msgs)
        odom_times(i) = double(odom_msgs.ts(i)) * 1e-9;
    end

    % Loop through each lidar timestamp and find the closest camera timestamp
    for i = 1:length(lidar_times)
        [~, closest_idx_cam] = min(abs(camera_times - lidar_times(i)));
        [~, closest_idx_odom] = min(abs(odom_times - lidar_times(i)));

        matched_camera_msgs{i, 1} = camera_msgs{closest_idx_cam, 1};
        matched_odom_msgs(i, :) = odom_msgs(closest_idx_odom, :);

    end

    matched_lidar_msgs = lidar_msgs;

end

% Extract Odometry
xyz = zeros(height(matched_odom_msgs),3);
quats = zeros(height(matched_odom_msgs),4);
for i = 1:height(matched_odom_msgs)
    % Testing KRC LWS data:
    xyz(i,:) = [matched_odom_msgs.x(i),matched_odom_msgs.y(i),alt];
    quats(i,:) = [matched_odom_msgs.q_w(i),matched_odom_msgs.q_x(i),matched_odom_msgs.q_y(i),matched_odom_msgs.q_z(i)];
end

% Interpolate Odometry to Sensors' Timestamps
% xyz = interp1(odomTime,xyz,sensorTime);
% quats = slerparray(odomTime,quats,sensorTime);
% quats = compact(quats);
end