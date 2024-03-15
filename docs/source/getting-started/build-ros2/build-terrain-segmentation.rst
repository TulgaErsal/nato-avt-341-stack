Build terrain segmentation node
===============================

Prerequisites
-------------

MATLAB Runtime
^^^^^^^^^^^^^^

- Install MATLAB Runtime 2023a

- Update `Matlab_MCLMCRRT_LIB` path in `CMakeLists.txt` to MATLAB Runtime
  install location

Segmentation DLL
^^^^^^^^^^^^^^^^

The terrain segmentation node depends on a precompiled dynamically linekd
library (DLL). The file is downloaded automatically through CMake if a `curl` is
available on PATH. If that is not the case, the DLL is available for manual
:download:`here
<https://www.dropbox.com/s/7m0fy3fx3uitxj5/perception_wrapper.dll?dl=1>`.

After the download is complete, move the `perception_wrapper.dll` to the
`uab_perception` folder under the root of the `avt_341` ROS package.