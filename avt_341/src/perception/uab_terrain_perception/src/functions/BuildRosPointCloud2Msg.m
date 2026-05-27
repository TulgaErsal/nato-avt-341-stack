function pcMsg = BuildRosPointCloud2Msg(pcWidth, pcHeight, pcPointStep, pcRowStep, rawLidar)
    xFields = ros2message("sensor_msgs/PointField");
    yFields = ros2message("sensor_msgs/PointField");
    zFields = ros2message("sensor_msgs/PointField");
    intensity = ros2message("sensor_msgs/PointField");
    ring = ros2message("sensor_msgs/PointField");
    
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
    
    intensity.name = 'intensity';
    intensity.offset = 12;
    intensity.datatype = 7;
    intensity.count = 1;
    
    ring.name = 'ring';
    ring.offset = 16;
    ring.datatype = 4;
    ring.count = 1;
    
    pcMsg = ros2message("sensor_msgs/PointCloud2");
    pcMsg.height = pcHeight;
    pcMsg.width = pcWidth;
    pcMsg.point_step = pcPointStep;
    pcMsg.row_step = pcRowStep;
    pcMsg.is_dense = 0;
    pcMsg.is_bigendian = 0;
    pcMsg.data = rawLidar;
    pcMsg.fields = [xFields;yFields;zFields;intensity;ring];
    pcMsg.PreserveStructureOnRead = false;
end