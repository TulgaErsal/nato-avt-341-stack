# TRACER ROS Segmentation

This ROS package provides segmentation functionality for the TRACER robot. The package processes data from the robot's camera and uses the Bisenet model to segment frames.

## Installation

To install the TRACER ROS Segmentation package, follow these steps:

1. Navigate to your ROS workspace and clone the package repository:
    ```
    cd ~/catkin_ws/src
    git clone https://gitlab.com/innervycslabs/perception_lab/tracer_x/tracer_simulacion/ros_stack_innervycs/tracer_ros_segmentation.git
    ```

2. Install Python dependencies:
    ```
    cd tracer_ros_segmentation
    pip install -r requirements.txt
    ```

3. Install pytorch:
    ```
    pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113
    ```

4. Build the package:
    ```
    cd ~/catkin_ws
    catkin_make
    ```

5. Source the package:
    ```
    cd ~/catkin_ws
    source devel/setup.bash
    ```

## Usage

To use the TRACER ROS Segmentation package, launch the tracer_segmentation_krc.launch file:
```
    roslaunch tracer_ros_segmentation tracer_segmentation_krc.launch
```

This will launch the segmentation node and start processing data from the robot's camera. The segmentation map will be published to the `/segmented_image` topic.


### Note
The previous roslaunch assumes that the airsim_ros_pkgs package is installed and that the robot is running in AirSim. The segmentation model used was trained on the KRC images dataset.


### Parameters

- `input_image_topic` (default: `/airsim_node/base_link/front_center/Scene`): The topic where the robot's camera publishes image data.
- `segmented_image_topic` (default: `/segmented_image`): The topic where the segmentation map will be published.


