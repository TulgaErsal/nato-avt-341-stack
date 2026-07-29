/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                      _    _    _    _    _    _    _                      +
 +                     / \  / \  / \  / \  / \  / \  / \                     +
 +                    ( A )( V )( T )( - )( 3 )( 4 )( 1 )                    +
 +                     \_/  \_/  \_/  \_/  \_/  \_/  \_/                     +
 +       _    _    _    _    _    _    _    _     _    _    _    _    _      +
 +      / \  / \  / \  / \  / \  / \  / \  / \   / \  / \  / \  / \  / \     +
 +     ( A )( U )( T )( O )( N )( O )( M )( Y ) ( S )( T )( A )( C )( K )    +
 +      \_/  \_/  \_/  \_/  \_/  \_/  \_/  \_/   \_/  \_/  \_/  \_/  \_/     +
 +                                                                           +
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +                                                                           +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      object_detection_node.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for the YOLOv8 object detector rclcpp ROS node.
* @copyright MIT License

             NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
             Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).

             NOTE: The above copyright only applies to the contents of this file. The source code contained in this file
             is a direct port from the GitHub repository aarhus-robotics/navi, released by the copyright holder under
             the MIT license.

             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
             associated documentation files (the "Software"), to deal in the Software without restriction, including
             without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
             copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
             following conditions:

             The above copyright notice and this permission notice shall be included in all copies or substantial
             portions of the Software.

             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
             LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
             EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
             IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
             THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <random>

#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>
#include <torch/script.h>

// get_package_prefix.hpp declares PackageNotFoundError, thrown by
// get_package_share_directory().
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#ifdef GTE_ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/label_info.hpp>
#include <vision_msgs/msg/vision_info.hpp>
#include <image_transport/image_transport.hpp>
#include <avt_341_nav/perception/detection/common/bounding_box_2d.hpp>
#include <avt_341_nav/perception/detection/common/detection_2d.hpp>
#include <avt_341_nav/perception/detection/common/hypothesis.hpp>
#include <avt_341_nav/perception/detection/common/object_visualizer.hpp>
#include <avt_341_nav/perception/detection/object_detector/object_detector.hpp>
#include <avt_341_nav/object_detector_params_dto.hpp>

namespace avt_341_nav::params::object_detector {
class ParamsListener;
}  // namespace avt_341_nav::params::object_detector

namespace avt_341_nav {
namespace perception {

class ObjectDetectorNode : public rclcpp::Node {
  public:
    ObjectDetectorNode();

  protected:
    /**
     * @brief Get the object detection node parameters.
     */
    void GetParameters();

    /**
     * @brief Initialize the object detection node class members.
     */
    void Initialize();

    /**
     * @brief Create the object detection node subscriptions.
     */
    void CreateSubscriptions();

    /**
     * @brief Create the object detection node timers.
     */
    void CreateTimers();

    /**
     * @brief Create the object detection node publisher.
     */
    void CreatePublishers();

  private:
    /**
     * @brief Publish the vision pipeline information messages from the ROS
     * vision_msgs package.
     */
    void PublishInfoMessages();

    /**
     * @brief Input image callback function.
     *
     * @param image_message Constant shared pointer to an uncompressed images
     * message to perform detection on.
     */

    void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_message);

    /**
     * @brief Callback function for the detection timer.
     */
    void DetectionCallback();

    std::shared_ptr<avt_341_nav::params::object_detector::ParamsListener> param_listener_;
    avt_341_nav::params::object_detector::Params params_;

    // ROS package management
    // ----------------------
    /** @brief Share directory of the package deploying the detection models. */
    std::string model_package_share_dir_;
    // ----------------------

    // Interfaces for vision_msgs
    // --------------------------
    /** @brief Shared pointer to the detections publisher for use with the
     * vision_msgs interface. */
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_vision_publisher_;

    /** @brief Shared pointer to the bounding boxes publisher for use with the
     * vision_msgs_layers interface. */
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr bounding_boxes_publisher_;

    /** @brief Shared pointer to the vision info publisher for use with the
     * vision_msgs interface. */
    rclcpp::Publisher<vision_msgs::msg::VisionInfo>::SharedPtr vision_info_publisher_;

    /** @brief Shared pointer to the label info publisher for use with the
     * vision_msgs interface. */
    rclcpp::Publisher<vision_msgs::msg::LabelInfo>::SharedPtr label_info_publisher_;
    // --------------------------

    // Visualization
    // -------------
    /** @brief Unique pointer to the object visualizer. */
    std::unique_ptr<ObjectVisualizer> visualizer_;

    /** @brief Handle for the visualization thread.
     * @details This thread runs in the background after a detection is
     * completed to publish an overlay image without blocking the processing of
     * the next image.
     */
    std::thread visualization_thread_;

    /**
     * @brief Publish an image with an overlay containing the detection
     * hypotheses bounding boxes and class labels.
     *
     * @param image OpenCV image to overlay the detections on.
     * @param detections Vector of image detections.
     * @param frame_id Frame ID used to stamp the overlay image message header.
     */
    void PublishDetectionImage(cv::Mat image, std::vector<Detection2D> detections, std::string frame_id);

    /** @brief Shared pointer to the object detections overlay image publisher.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detections_image_publisher_;
    // -------------

    // Object detection model
    // ----------------------
    /** @brief Unique pointer to the object detector. */
    std::unique_ptr<ObjectDetector> detector_;

    /** @brief Fully qualified path to the names file. Used when loading class
     * labels and publishing vision pipeline information. */
    std::string names_path_;

    /** @brief Vector of class labels. */
    std::vector<std::string> classes_;

    /** @brief Shared pointer to the detection timer.  */
    rclcpp::TimerBase::SharedPtr detection_timer_;

    /** @brief Circular buffer storing the latest detection tasks execution
     * time. */
    boost::circular_buffer<double> time_buffer_;

    /** @brief Size of the detection tasks execution time circular buffer. */
    int time_buffer_size_ = 10;

    /**
     * @brief Get a new mean detection time from the detection tasks execution
     * times stored in the circular buffer.
     *
     * @param last_detection_time Last detection task execution time in
     * milliseconds.
     * @return double Mean detection task execution time in milliseconds.
     */
    double GetMeanDetectionTime(double last_detection_time);
    // ----------------------

    // Image parsing
    // -------------
    /** @brief Shared pointer to the image subscription. */
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    image_transport::Subscriber image_subscription_;

    /** @brief Image transport instance */
    image_transport::ImageTransport image_transport_;

    /** @brief Whether or not a valid image has been received since the latest
     * object detector reset. */
    bool has_image_ = false;

    /** @brief Mutex for accessing the latest received image. */
    std::mutex image_mutex_;

    /** @brief Constant shared pointer to the latest received image in a
     * cv_bridge wrapper. */
    cv_bridge::CvImageConstPtr latest_image_;
    // -------------
};

} // namespace perception
} // namespace avt_341_nav
