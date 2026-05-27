function [xyz,quats,camera_msgs,lidar_msgs] = SyncOdom2Sensors(odom_msgs,camera_msgs,lidar_msgs)
% Synchronize ROS odometry messages to Camera/LIDAR.
% Returns Synchronized odometry as XYZ and Quaternion.
%   Interpolates odometry to match Camera/LIDAR.
%   This function assumes that Camera/LIDAR are synchronous.
%   Function will crop Camera/LIDAR messages to avoid extrapolation.

% Extract Odometry
xyz = zeros(length(odom_msgs),3);
quats = zeros(length(odom_msgs),4);
for i = 1:length(odom_msgs)
    position = odom_msgs{i}.pose.pose.position;
    xyz(i,:) = [position.x,position.y,position.z];
    orientation = odom_msgs{i}.pose.pose.orientation;
    quats(i,:) = [orientation.w,orientation.x,orientation.y,orientation.z];
end

% Extract Odometry Timestamps
odomTime = zeros(length(odom_msgs),1);
for i = 1:length(odom_msgs)
    sec = double(odom_msgs{i}.header.stamp.sec);
    nanosec = double(odom_msgs{i}.header.stamp.nanosec);
    odomTime(i) = sec+(10^-9)*nanosec;
end
% Check Odometry Includes Unique Timestamps
if length(odomTime) ~= length(unique(odomTime))
    disp('Warning: Repeated Timestamps in Odometry')
    [odomTime,idx,~] = unique(odomTime);
    xyz = xyz(idx,:);
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

% Interpolate Odometry to Sensors' Timestamps
xyz = interp1(odomTime,xyz,sensorTime);
quats = slerparray(odomTime,quats,sensorTime);
quats = compact(quats);
end