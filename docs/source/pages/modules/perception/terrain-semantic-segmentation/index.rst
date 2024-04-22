Semantic terrain segmentation
=============================

Overview
--------

Performs semantic segmentation on camera images using the DeepLab V3+
architecture and fuses with point cloud data from a time-of-flight sensor to
create a costmap based on vehicle traversability.

The `uab_perception_node` is a C++ wrapper for the machine learning model
written in MATLAB. This is easier to build/run due to issues with ROS2 support
in MATLAB, but decreases performance.

ROS Interface
-------------

Subscriptions
^^^^^^^^^^^^^

.. csv-table:: Subscribed topics
   :file: subscriptions.csv
   :header-rows: 1

Publishers
^^^^^^^^^^

.. csv-table:: Published topics
   :file: publishers.csv
   :header-rows: 1

Parameters
^^^^^^^^^^

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1

Troubleshooting
---------------

* **Issue**: "Fatal error C1083: Cannot open include file: 'mclmcrrt.h': No such
  file or directory"
* **Solution**: Set the MATLAB include directory manually under the
  `Matlab_MCLMRRT_LIB` line in the CMakeLists file.
  
  .. code-block:: cmake

    set(Matlab_INCLUDE_DIRS "<path-to-MATLAB-Runtime>/v912/extern/include")

* **Issue**: The occupancy grid published by the perception node is not being
  drawn or is out of sync.
* **Solution**: There is an outstanding issue with the timestamps between the
  odometry and point cloud. You can solve this by temporarily replacing the
  `GetPoseToUse` function in `avt_341_perception_node.cpp` with
    
    .. code-block:: cpp
    
      double GetPoseToUse(avt_341::msg::Odometry & pose_to_use,
                          avt_341::msg::PointCloud2Ptr rcv_cloud){
          double dt = 1.0;
        
          for (int i = 0; i < current_pose_list.size(); i++) {
              pose_to_use = current_pose_list[i];
          }

          return dt;
      }

Further information
-------------------

For more information on the terrain semantic segmentation node algorithms and
implementation, refer to the following documents:

* :download:`Dataset preparation for DeepLab V3+ Training <dataset_preparation_for_deeplab_v3+_training.pptx>`
* :download:`Machine Learning Overview: Modified Echo State Networks for LiDAR Data Processing <ml_esn_overview.pptx>`

.. raw:: latex

  If you are browsing the documentation in PDF format, you may need to use these
  links instead:

  \begin{itemize}
    \item \href{https://www.dropbox.com/scl/fi/rejwf0q3hba38lkljfme0/dataset\_preparation\_for\_deeplab\_v3-\_training.pptx?rlkey=tr0si3ctbdtjjw80q522wjiv3\&st=o7eswjd8\&dl=0}{Dataset preparation for DeepLab V3+ Training}
    \item \href{https://www.dropbox.com/scl/fi/f0evthrbaxeg6hn5rsjul/ml\_esn\_overview.pptx?rlkey=dun0bxe1yq0sbtmxqoydtxa83\&st=9idqq87i\&dl=0}{Machine Learning Overview: Modified Echo State Networks for LiDAR Data Processing}
  \end{itemize}