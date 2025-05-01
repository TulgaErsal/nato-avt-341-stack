FROM ros:humble-ros-base-jammy

RUN apt update && apt install -y vim tree git ros-humble-sensor-msgs-py ros-humble-rviz2 python3-easydict 
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN mkdir -p /ros2ws/src
RUN cd /ros2ws && colcon build
COPY ekf3d /ros2ws/src/ekf3d 
RUN cd /ros2ws && colcon build
RUN echo "source /ros2ws/install/setup.bash" >> /root/.bashrc
RUN rm -rf /ros2ws/src/ekf3d
RUN cd /ros2ws/src && ln -s /data/repos/crl-cdt/ekf3d 

WORKDIR /ros2ws
