function imgMsg = BuildRosImageMsg(imgWidth, imgHeight, encoding, rawImg, step)
    imgMsg = ros2message("sensor_msgs/Image");
    imgMsg.height = imgHeight;
    imgMsg.width = imgWidth;
    imgMsg.encoding = encoding;
    imgMsg.is_bigendian = 0;
    imgMsg.step = step;
    imgMsg.data = rawImg;
end