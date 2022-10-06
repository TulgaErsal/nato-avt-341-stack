function [costNow] = perception_wrapper(flgLoadNet, rawImage, rawLidar, pose_point_x, pose_point_y, pose_point_z, pose_quat_w, pose_quat_x, pose_quat_y, pose_quat_z)
    if flgLoadNet
        load('dlnet10.mat');
    end

    % build ROS2 Odometry message
    odom = ros2message("nav_msgs/Odometry");
    odom.pose.pose.position.x = pose_point_x;
    odom.pose.pose.position.y = pose_point_y;
    odom.pose.pose.position.z = pose_point_z;
    odom.pose.pose.orientation.w = pose_quat_w;
    odom.pose.pose.orientation.x = pose_quat_x;
    odom.pose.pose.orientation.y = pose_quat_y;
    odom.pose.pose.orientation.z = pose_quat_z;

    % build ROS2 Image message
    img = ros2message("sensor_msgs/Image");
    img.height = 540;
    img.width = 960;
    img.encoding = 'rgb8';
    img.is_bigendian = 0;
    img.step = 2880;
    img.data = uint8(rawImage);

    % build ROS PointCloud2 message
    sprintf("create rawLidar");
    
    xFields = ros2message("sensor_msgs/PointField");
    yFields = ros2message("sensor_msgs/PointField");
    zFields = ros2message("sensor_msgs/PointField");
    
    xFields.name = 'x';
    xFields.offset = 0;
    xFields.datatype = 7;
    xFields.count = 1;
    
    yFields.name = 'y';
    yFields.offset = 4;
    yFields.datatype = 7;
    yFields.count = 1;
    
    zFields.name = 'z';
    zFields.offset = 8;
    zFields.datatype = 7;
    zFields.count = 1;
    
    pc = ros2message("sensor_msgs/PointCloud2");
    pc.height = 64;
    pc.width = 2048;
    pc.point_step = 48;
    pc.row_step = 98304;
    pc.is_dense = 1;
    pc.is_bigendian = 0;
    pc.data = uint8(rawLidar);
    pc.fields = [xFields; yFields; zFields];
    pc.PreserveStructureOnRead = false;

    % net and cmap are set from 'dlnet10.mat'
    costNow = ex_PerceptionAlgorithm(net, cmap, odom, pc, img);
end