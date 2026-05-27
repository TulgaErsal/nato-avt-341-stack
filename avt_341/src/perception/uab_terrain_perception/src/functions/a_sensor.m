%% MSU AV Sensors
% Dr. S. D. Gardner
% 
% UAB - EITD - ARC
% 
% Updated: January 2024

setenv('ROS_MASTER_URI', 'http://localhost:11311'); % local computer
%setenv('ROS_MASTER_URI', 'http://xavier-a:11311'); % xavier-a
rosinit;
%%
%bag = rosbag(['C:\Users\stevendg\Desktop\UAB_EITD_ARC\Projects\2024_ARC' ...
%              '\vehicle_perception\MSU\ridge_2_loam.bag']);
bag = rosbag('Bags/MSU/UAB_sam_walking.bag');
%%
%topics = {'/os_cloud_node_front/points', '/left_camera/image_raw/compressed', '/lvi_sam/lidar/mapping/odometry_incremental'};                          
topics = {'/os_cloud_node_front/points', '/left_camera/image_raw/compressed', '/nav/odom'};
% topics = {'/os_cloud_node_front/points', '/left_camera/image_raw/compressed', '/avt_341/odometry'};
publishers = {};

% Create publishers for each topic
for i = 1:size(topics, 2)
    topic = topics{i};
    bagSelect = select(bag, 'Topic', topic);
    msgType = char(bagSelect.MessageList.MessageType(1));
    publishers{end+1} = rospublisher(topic, msgType);
end

messageData = struct('Time', [], 'Message', [], 'Topic', []);

% Read messages from each topic
for i = 1:length(topics)
    topic = topics{i};
    bagSelect = select(bag, 'Topic', topic);
    msgs = readMessages(bagSelect);
    times = bagSelect.MessageList.Time;
    
    for j = 1:length(msgs)
        messageData(end+1) = struct('Time', times(j), 'Message', msgs{j}, 'Topic', topic);
    end
end

messageData(1) = [];

% Sort the structure by timestamps
[~, idx] = sort([messageData.Time]);
messageDataSorted = messageData(idx);
%%
% Iterate through the sorted messages and publish them
for i = 1:length(messageDataSorted)
    pubIndex = find(strcmp(topics, messageDataSorted(i).Topic));

    if ~isempty(pubIndex)
        send(publishers{pubIndex}, messageDataSorted(i).Message);
    end
    pause(0.02); % odom publish rate of 10Hz, with 5 sensor messages between = 0.1/5 = 0.02

    %disp(i)
end