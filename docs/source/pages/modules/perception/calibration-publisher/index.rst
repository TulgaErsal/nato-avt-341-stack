Static Transforms and Calibration Publisher
===========================================

In ROS, the typical way to publish static transforms is by defining all the frames in a URDF file, and using a static transform publisher to publish these frames.
In the NATO stack, a URDF file is used to define the frames as well, with one notable exception, the transform between the camera and lidar.

The camera-lidar is particularly important for the perception, it needs to be accurate for some perception algorithms to work properly.
To obtain the transform, we perform a special calibration procedure to both determine the transform between them.
The same calibration procedure also yields the intrinsic camera parameters needed by other parts of the perception stack.
The output of the calibration procedure is a yaml file that contains the intrinsic camera parameters and the transform between the camera and lidar frames.
To avoid having to modify the URDF file each time a new calibration is performed, we instead use a separate static transform publisher that reads the transform directly from the yaml file and publishes it.

Camera-Lidar Calibration
------------------------
The calibration procedure is described here: `MATLAB: Lidar and Camera Calibration <https://mathworks.com/help/lidar/ug/lidar-and-camera-calibration.html>`_

In the NATO stack, we have provided a MATLAB function that exports the calibration results to a yaml file, located at:

``nato-avt-341-stack/tools/camera_lidar_calibration.m.``

An example of the yaml file is shown below:
::

    ---
    image_width: 2048
    image_height: 1536
    camera_name: FLIR Oryx
    camera_matrix:
      rows: 3
      cols: 3
      data: [1059.26155375821, 0, 1027.25091600072, 0, 1060.46731890928, 811.439626067897, 0, 0, 1]
    distortion_model: plumb_bob
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [-0.0136392629621658, 0.0323813548026934, 0, 0, 0]
    rectification_matrix:
      rows: 3
      cols: 3
      data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    projection_matrix:
      rows: 3
      cols: 4
      data: [1059.26155375821, 0, 1027.25091600072, 0, 0, 1060.46731890928, 811.439626067897, 0, 0, 0, 1, 0]
    transformation_lidar_cam_matrix:
      rows: 4
      cols: 4
      data: [-0.00175886451518136, -0.999998447071948, -0.00011067659735554, 0.000874998668178296, 0.180960259879553, -0.000209435535480701, -0.983490386572782, -0.100257306151693, 0.98348883610336, -0.00174985440778491, 0.180960347230021, -0.00937402944409714, 0, 0, 0, 1]

The yaml format is based on the standard `camera_info` message type, which is used in ROS to represent camera intrinsic parameters
This means that typical ROS camera drivers such as the `flir_camera_driver <https://github.com/ros-drivers/flir_camera_driver>`_ can read the yaml file and publish the camera intrinsic parameters as a `sensor_msgs/CameraInfo` message, it just need the correct path to the yaml file.

For the FLIR driver, the following parameter must be set

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1

For more info, see:

* `camera_calibration_parsers <https://wiki.ros.org/camera_calibration_parsers>`_.
* `camera_info_manager <https://wiki.ros.org/camera_info_manager>`_ with the syntax for camera info file URL.
* `camera_info_manager::CameraInfoManager Class Reference <https://docs.ros.org/en/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html>`_


calibration_tf_publisher
------------------------
The calibration_tf_publisher node is a node that can read the yaml calibration file and publish the static transform between the camera and lidar frames.

A suggested location where it can be placed is under ``nato-avt-341-stack/avt_341/config/sensor_calibration/``

To run the node, use the launch file ``calibration_tf_publisher.launch`` and refer to the correct calibration file.
