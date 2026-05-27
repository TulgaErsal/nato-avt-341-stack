function odomMsg = BuildRosOdometryMsg(x, y, z, qw, qx, qy, qz)
    odomMsg = ros2message("nav_msgs/Odometry");
    odomMsg.Pose.Pose.Position.X = x;
    odomMsg.Pose.Pose.Position.Y = y;
    odomMsg.Pose.Pose.Position.Z = z;
    odomMsg.Pose.Pose.Orientation.W = qw;
    odomMsg.Pose.Pose.Orientation.X = qx;
    odomMsg.Pose.Pose.Orientation.Y = qy;
    odomMsg.Pose.Pose.Orientation.Z = qz;
end