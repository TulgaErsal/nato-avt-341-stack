# Attribution

## Overview

The package [nato-avt-341-stack](https://github.com/TulgaErsal/nato-avt-341-stack) depends on open source software
released under permissive licenses. Below is a list of the license source files and the license under which these files
are released.

Further below, the original license texts for all licenses are included.

## Merged source code

* Modules from [JuliaInterop/libcxxwrap-julia](https://github.com/JuliaInterop/libcxxwrap-julia) are
  released under the [MIT License](#mit-license), copyright holder Bart Janssens.
  * [FindJulia.cmake](avt_341/cmake/FindJulia.cmake)

* Modules from [github.com/SS47816/lidar_obstacle_detector](https://github.comSS47816/lidar_obstacle_detector/) are
  released under the [MIT License](#mit-license), copyright holder Shuo Sun.
  * [lidar_obstacle_detector_node.hpp](avt_341/include/avt_341/perception/lidar_obstacle_detector.hpp)
  * [lidar_obstacle_detector_node.cpp](avt_341/src/perception/lidar_obstacle_detector/lidar_obstacle_detector_node.cpp)

* Modules from [github.com/aarhus-robotics/navi](https://github.com/aarhus-robotics/navi) are released under the [MIT
  License](#mit-license), copyright holder Dario Sirangelo ([dsi@aarhusrobotics.com](mailto:dsi@aarhusrobotics.com)):
  * [object_detection_node_executor.cpp](avt_341/src/perception/detection/object_detector/object_detection_node_executor.cpp)
  * [object_detection_node.cpp](avt_341/src/perception/detection/object_detector/object_detection_node.cpp)
  * [object_detector.cpp](avt_341/src/perception/detection/object_detector/object_detector.cpp)
  * [gym_executor.py](avt_341/src/perception/detection/training/gym_executor.py)
  * [bounding_box_2d.hpp](avt_341/include/avt_341/perception/detection/common/bounding_box_2d.hpp)
  * [detection_2d.hpp](avt_341/include/avt_341/perception/detection/common/detection_2d.hpp)
  * [hypothesis.hpp](avt_341/include/avt_341/perception/detection/common/hypothesis.hpp)
  * [object_visualizer.hpp](avt_341/include/avt_341/perception/detection/common/object_visualizer.hpp)
  * [cv_filter.hpp](avt_341/include/avt_341/perception/filtering/cv_filter.hpp)
  * [kalman_filter.hpp](avt_341/include/avt_341/perception/filtering/kalman_filter.hpp)
  * [kinematic_kalman_filter.hpp](avt_341/include/avt_341/perception/filtering/kinematic_kalman_filter.hpp)
  * [process_covariance.hpp](avt_341/include/avt_341/perception/filtering/process_covariance.hpp)
  * [pixel_coordinates.hpp](avt_341/include/avt_341/perception/tracking/pixel_coordinates.hpp)
  * [object_tracking_node.hpp](avt_341/include/avt_341/perception/tracking/object_tracking_node.hpp)
  * [object_tracking_node_executor.cpp](avt_341/src/perception/tracking/object_tracking_node_executor.cpp)
  * [object_tracking_node.cpp](avt_341/src/perception/tracking/object_tracking_node.cpp)

## Licenses

### MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.