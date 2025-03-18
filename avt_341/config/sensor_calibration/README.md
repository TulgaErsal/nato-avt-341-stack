# Camera-lidar calibration
The camera calibration file can be read directly by ROS camera drivers such as the FLIR driver by setting a parameter that points to the calibration file. e.g.:
'camerainfo_url': 'package://avt_341/config/sensor_calibration/2025-03-17-08-12-39-camera-info-matlab-ffi.yaml'

## FIle format
https://wiki.ros.org/camera_info_manager
