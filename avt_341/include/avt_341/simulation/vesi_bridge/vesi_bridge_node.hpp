#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <avt_341/simulation/vesi_bridge/api/AtomicSensorInformation.h>
#include <avt_341/simulation/vesi_bridge/api/CameraDataDeserializerDefault.h>
#include <avt_341/simulation/vesi_bridge/api/CameraDataDeserializerTypes.h>
#include <avt_341/simulation/vesi_bridge/api/VESIAPI.h>
#include <avt_341/simulation/vesi_bridge/iac_qos.hpp>

namespace bridge {
class SensorBridgeNode : public rclcpp::Node {
  public:
    SensorBridgeNode();
    ~SensorBridgeNode() = default;

  private:
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        lidarDataPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        radarDataPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cameraDataPublisher_;

    // Sensors publishing functions
    VESIAPI sensorApi;
    void connectToSensorApi(int16_t max_retries);
    void publishLidarData(uint8_t sensorId);
    void publishRadarData(uint8_t sensorId);
    void publishCameraData(
        std::unique_ptr<CameraDataDeserializerDefault>& deserializer,
        uint8_t sensorId,
        int cameraImageType);

    // Parameters
    int MAX_SENSOR_RESULT_PRINTOUTS = 3;
    bool verbosePrinting = false;
    std::string sensorType;
    uint8_t sensorId;
    std::string rosTopic;
    int cameraImageType;
};

} // namespace bridge