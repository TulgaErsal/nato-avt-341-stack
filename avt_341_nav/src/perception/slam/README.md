### Compilation

- Before compilation, install dependencies 
```
sudo apt-get update \
    && sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0 \
    && sudo apt-get update \
    && sudo apt install -y libgtsam-dev libgtsam-unstable-dev \
    && sudo apt install -y ros-noetic-navigation \
    && sudo apt install -y ros-noetic-robot-localization \
    && sudo apt install -y ros-noetic-robot-state-publisher
```
- Alternatively, gtsam can be compiled from sources, see [https://github.com/borglab/gtsam]( https://github.com/borglab/gtsam).
- Set `BUILD_SLAM` to `ON` in the (catkin) root `CMakeLists.txt` — the current ament (ROS2) root no longer defines it. `BUILD_SLAM ON` should also set the build type to `"Release"`, since otherwise, the SLAM performance might drop in some environments.
- Compilation links to the pre-built `avt_341/src/perception/slam/lib/libCommonLib.so`. The ROS1 (catkin) build wiring for the SLAM nodes is kept in `avt_341/src/perception/slam/CMakeLists.txt`; a catkin root CMakeLists only needs to `add_subdirectory(src/perception/slam)` with `BUILD_SLAM` set to `ON`.
- Currently, only ROS1 is supported. We need to figure out additional details, such as message synchronization within the avt_341 node proxies, to support ROS2. 

### Startup

- Use  `avt_341/launch/slam_ouster.launch` to run the SLAM. This will launch verbose version of the SLAM and an associated rviz file, which can be commented out in launch file. 
- Running on low-performance hardware might decrease the localization accuracy. If this is an issue, run bag files with a rate of 1.0 or lower. Besides, we recommend to test the benchmark dataset on the target hardware before the deployment. 
- The provided config uses Ouster LiDAR and Ouster 6-axis IMU. If you would like us to give you a config for a different setup, let us know.

### Perfomance

- The system is a LiDAR-based SLAM. Therefore, it works consistently only when enough features are around. Currently, there are no guarantees in scan alignment degenerate areas such as open fields. 
- Currently, 6-axis IMU is used, which may result in yaw drifting away after some time. 9-axis IMU should improve this, and can be integrated.
- Tilt (roll, pitch) drift is bounded through corrections from absolute measurements.
- Heading (yaw) drift is unbounded for 6-axis IMU due to absence of magnetometer.

### Runtime Examples

During the runtime, the SLAM uses LiDAR scan alignment w.r.t. the continuously updated global map. 
<details>
<summary>Images - SLAM operation </summary>

White point clouds: current scan; colored point clouds: map.

![image](https://github.com/user-attachments/assets/7e47b712-aef6-46b6-b8a5-84c4ab1c3677)
![image](https://github.com/user-attachments/assets/f08308bd-4652-4217-b751-08da42a509c5)
![image](https://github.com/user-attachments/assets/379f879d-a6c4-4119-a616-41be9471a32e)
![image](https://github.com/user-attachments/assets/25500b4d-b582-4ead-acd4-9bb6204bf05b)
![image](https://github.com/user-attachments/assets/f84237e1-8cf6-4a72-8d43-e19f23a4296f)

</details>

Scan alignment degeneracy occurs in nearly planar areas. This may result in localization drift. Also to be expected in corridors/other scan alignment-degenerate areas.
<details>
<summary>Images - degenerate areas </summary>

![image](https://github.com/user-attachments/assets/4de1cf33-c3b9-46d8-850f-5e3315299336)
![image](https://github.com/user-attachments/assets/7812c586-f28d-4b5b-81bd-06740f75eba5)

</details>

After the degenerate areas, it takes some time to recover, and unexpected drift is possible.
<details>
<summary>Images - trajectory discontinuity areas </summary>

![image](https://github.com/user-attachments/assets/77097014-3083-4987-b8cd-322beb62b5c0)

</details>

The system works in some of the planar-like areas, but no guarantees can be provided.
<details>
<summary>Images - operation in planar areas </summary>

![image](https://github.com/user-attachments/assets/5a4cac42-8e27-4509-ad94-d80fcf395d12)
![image](https://github.com/user-attachments/assets/e09f4959-2253-4a50-9e6f-15cf7e07170a)

</details>

Angular drift may occur - bounded for tilt, unbounded for heading.
<details>
<summary>Images - angular drift </summary>

- As a result of the bounded tilt (roll and pitch) drift (up to 5 degrees, in this case), the elevation offset is accumulated.
![image](https://github.com/user-attachments/assets/dfadb568-e53d-45f5-a6ef-673e6c7a5f2b)

- Heading (yaw) drift; unbounded while using Ouster IMU which has no magnetometer due to absence of the heading absolute measurement corrections. 
![image](https://github.com/user-attachments/assets/bddc197c-ea31-4264-8f99-4ffbf6985595)

</details>

Resulting trajectory and map.

<details>
<summary>Images - resulting trajectory and map </summary>

- Top view.
![image](https://github.com/user-attachments/assets/f7170dfa-f985-407a-8eed-150054609da5)
- Side view.
![image](https://github.com/user-attachments/assets/839f871e-a9a7-42f8-b17d-564a9f084477)

</details>

### Main Changes Compared to LIO-SAM
The system is based on the LIO-SAM modification [liorf](https://github.com/YJZLuckyBoy/liorf), with the following improvements.
- Linear motion model for the translation component of the initial guess for scan alignment. (IMU is used for the rotation component of the initial guess.)
- IMU fusion for the tilt drift corrections.

### Possible Improvements
 - Integrate 9-axis IMU. **Impact**: bounded heading drift. @EvanVandermate Does the IMU output depend on the presence of GPS?
- Add velocity estimation. **Impact**: application-specific, better initial guess for scan alignment.
-  Improve the tilt correction algorithm. **Impact**: map consistency improvements; possible localization improvements in near-degenerate areas.
- Improve detection of scan alignment degeneracy. **Impact**: detect when scan alignment does not work; lessen drift, and make the system safer.
- (long-term) Add Visual features to deal with degeneracy. Alternatively, wheeled odometry can be added. However, for this, the whole optimization process needs to be changed. **Impact**: improvement in LiDAR alignment-degenerate areas. 





