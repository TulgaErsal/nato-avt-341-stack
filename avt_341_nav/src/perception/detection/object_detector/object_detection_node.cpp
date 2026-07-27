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

* @file      object_detection_node.cpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Source file for the YOLOv8 object detector rclcpp ROS node.
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

#include <avt_341_nav/perception/detection/object_detector/object_detection_node.hpp>
#include <avt_341_nav/object_detector_params_service.hpp>

namespace avt_341_nav {
namespace perception {

ObjectDetectorNode::ObjectDetectorNode() : image_transport_(std::shared_ptr<ObjectDetectorNode>(this)), rclcpp::Node("object_detector") {
    param_listener_ =
        std::make_shared<avt_341_nav::params::object_detector::ParamsListener>(
            get_node_parameters_interface(), get_logger());
    GetParameters();
    Initialize();

    CreateSubscriptions();
    CreateTimers();
    CreatePublishers();
    PublishInfoMessages();

    if(params_.rate > 0.0) { detection_timer_->reset(); }
}

void ObjectDetectorNode::GetParameters() {
    params_ = param_listener_->get_params();
}

void ObjectDetectorNode::Initialize() {
    std::string model_path;
    if(!params_.model.external.empty()) {
        model_path = params_.model.external + params_.model.name;
    } else {
        try {
            model_package_share_dir_ = ament_index_cpp::get_package_share_directory("avt_341_nav");
        } catch(const ament_index_cpp::PackageNotFoundError& exception) {
            RCLCPP_ERROR(get_logger(), "Package not found - is your environment sourced correctly?");
            rclcpp::shutdown();
        }

        model_path = model_package_share_dir_ +
                     "/ml_weights/detection/models/" +
                     params_.model.name;
    }

    names_path_ = model_path + ".names";
    auto torchscript_path = model_path + ".torchscript";

    // Check if the TorchScript file is available at the specified path.
    RCLCPP_INFO_STREAM(get_logger(), "TorchScript path: " << torchscript_path);
    auto script_stream = std::ifstream(torchscript_path);
    if(script_stream.fail()) {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "TorchScript file is not inaccessible or not existing.");
    }

    // Check if the classes names file is available at the specified path.
    auto names_stream = std::ifstream(names_path_);
    if(names_stream.fail()) {
        throw rclcpp::exceptions::InvalidParameterValueException("Names file is not inaccessible or not existing.");
    }

    // Initialize the object detector.
    RCLCPP_DEBUG(get_logger(), "Initialized object detector.");
    detector_ = std::make_unique<ObjectDetector>();
    detector_->SetScoreThreshold(params_.thresholds.score);
    detector_->SetIOUThreshold(params_.thresholds.iou);
    detector_->SetCountThreshold(
        static_cast<int>(params_.thresholds.count));
    detector_->Load(model_path);

    // Load the object detection model classes.
    classes_ = detector_->ReadClassNames(names_stream);
    std::string classes;
    for(size_t i = 0; i < classes_.size(); ++i) { classes += " <[" + std::to_string(i) + "] " + classes_[i] + "> "; }
    RCLCPP_DEBUG(get_logger(), "Loaded %i object detection model classes > %s", int(classes_.size()), classes.c_str());
    detector_->SetValidClasses(params_.model.classes);

    if(params_.model.warmup > 0) {
        RCLCPP_DEBUG(get_logger(), "Warming up model ...");
        for(int64_t i = 0; i < params_.model.warmup; ++i) {
            auto start = get_clock()->now().nanoseconds();
            detector_->Warmup();
            auto stop = get_clock()->now().nanoseconds();

            if(i == 0) {
                RCLCPP_DEBUG(get_logger(), "├ Inference time (i=%lli) -> %0.1f ms",
                             static_cast<long long>(i),
                             (stop - start) / 1.0e6);
            } else if(i == params_.model.warmup - 1) {
                RCLCPP_DEBUG(get_logger(), "└ Inference time (i=%lli) -> %0.1f ms",
                             static_cast<long long>(i),
                             (stop - start) / 1.0e6);
            }
        }
        RCLCPP_DEBUG(get_logger(), "Completed %lli warmup iterations!",
                     static_cast<long long>(params_.model.warmup));
    } else {
        RCLCPP_WARN(get_logger(),
                    "Skipping model warmup (null or negative iterations) - "
                    "first inference callback "
                    "will be significantly slower!");
    }

    // Initialize the detection time buffer.
    time_buffer_ = boost::circular_buffer<double>(time_buffer_size_);

    // Initialize the object visualizer
    visualizer_ = std::make_unique<ObjectVisualizer>();
    visualizer_->SetClasses(classes_);
    visualizer_->UseTextbox(params_.visualizer.textbox);
    visualizer_->SetFontScale(params_.visualizer.size_font);
    visualizer_->SetBorderSize(
        static_cast<int>(params_.visualizer.size_border));
    if(params_.visualizer.seed != 0) {
        visualizer_->ShuffleColors(
            static_cast<int>(params_.visualizer.seed));
    }
}


void ObjectDetectorNode::CreateSubscriptions() {
    image_subscription_ = image_transport_.subscribe("image", 1, std::bind(&ObjectDetectorNode::ImageCallback, this, std::placeholders::_1));
    // image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    //     "image",
    //     RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
    //     std::bind(&ObjectDetectorNode::ImageCallback, this, std::placeholders::_1));
}

void ObjectDetectorNode::CreateTimers() {
    if(params_.rate <= 0.0) {
        RCLCPP_INFO(get_logger(),
                    "Detection rate is null or negative - detection will try "
                    "to keep up with the image stream.");
        return;
    }

    detection_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / params_.rate),
                                         std::bind(&ObjectDetectorNode::DetectionCallback, this));
    detection_timer_->cancel();
}

void ObjectDetectorNode::CreatePublishers() {
    detections_vision_publisher_ = create_publisher<vision_msgs::msg::Detection2DArray>("detections/vision", 1);

    bounding_boxes_publisher_ = create_publisher<vision_msgs::msg::BoundingBox2D>("detections/bounding_boxes", 1);

    if(params_.visualizer.enabled) {
        detections_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("detections/image/raw", 1);
    } else {
        RCLCPP_DEBUG(get_logger(),
                     "Visualizer not enabled, skipping object detections "
                     "overlay image publisher creation ...");
    }

    // Define a QoS profile with transient local durability policy to use with
    // publishers that require latching behavior.
    rclcpp::QoS qos_profile(1);
    qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // This publisher must be latching, as we are only publishing the vision
    // pipeline information once, on model load.
    vision_info_publisher_ = create_publisher<vision_msgs::msg::VisionInfo>("info/vision", qos_profile);

    // This publisher must be latching, as we are only publishing the classes
    // information once, on model load.
    label_info_publisher_ = create_publisher<vision_msgs::msg::LabelInfo>("info/label", qos_profile);
}

void ObjectDetectorNode::PublishInfoMessages() {
    // Publish the vision_msgs/msg/VisionInfo message.
    RCLCPP_DEBUG(get_logger(), "Publishing vision_msgs/msg/VisionInfo message ...");
    auto vision_info_message = std::make_shared<vision_msgs::msg::VisionInfo>();
    vision_info_message->header.frame_id = "avt_341";
    vision_info_message->header.stamp = get_clock()->now();
    vision_info_message->method = params_.model.name;
    vision_info_message->database_location = names_path_;
    vision_info_message->database_version = 0;
    vision_info_publisher_->publish(*vision_info_message);

    // Publish the vision_msgs/msg/LabelInfo message.
    RCLCPP_DEBUG(get_logger(), "Publishing vision_msgs/msg/LabelInfo message ...");
    auto label_info_message = std::make_shared<vision_msgs::msg::LabelInfo>();
    label_info_message->header.frame_id = "";
    label_info_message->header.stamp = get_clock()->now();
    for(size_t i = 0; i < classes_.size(); ++i) {
        auto vision_class_message = std::make_shared<vision_msgs::msg::VisionClass>();
        vision_class_message->class_id = i;
        vision_class_message->class_name = classes_[i];
        label_info_message->class_map.push_back(*vision_class_message);
    }
    label_info_message->threshold = params_.thresholds.score;
    label_info_publisher_->publish(*label_info_message);
}

void ObjectDetectorNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Image callback triggered.");

    if(image_mutex_.try_lock()) {
        try {
            latest_image_ = cv_bridge::toCvShare(image_message, "bgr8");
            image_mutex_.unlock();
            has_image_ = true;
        } catch(const cv_bridge::Exception& exception) {
            RCLCPP_ERROR(get_logger(), "CVBridge exception: %s", exception.what());
            image_mutex_.unlock();
            return;
        }
    }

    if(params_.rate <= 0.0) { DetectionCallback(); }
}

void ObjectDetectorNode::DetectionCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Detection callback triggered!");

    if(!has_image_) { return; }

    auto start_time = get_clock()->now().nanoseconds();

    std::shared_ptr<cv_bridge::CvImage> image_copy;
    if(image_mutex_.try_lock()) {
        image_copy = std::make_shared<cv_bridge::CvImage>(*latest_image_);
        image_mutex_.unlock();
    }

    auto detections = detector_->Detect(image_copy->image);

    vision_msgs::msg::Detection2DArray detections_message;
    detections_message.detections.reserve(detections.size());
    for(auto& detection : detections) {
        const auto detection_message = detection.ToROSVisionMessage();
        detections_message.detections.push_back(detection_message);
    }
    detections_message.header.stamp = get_clock()->now();
    detections_message.header.frame_id = "?";
    detections_vision_publisher_->publish(detections_message);

    if(params_.visualizer.enabled) {
        visualization_thread_ = std::thread(&ObjectDetectorNode::PublishDetectionImage,
                                            this,
                                            image_copy->image.clone(),
                                            detections,
                                            image_copy->header.frame_id);
        visualization_thread_.detach();
    }

    auto end_time = get_clock()->now().nanoseconds();
    auto last_detection_time = (end_time - start_time) / 1.0e6;

    auto mean_detection_time = GetMeanDetectionTime(last_detection_time);

    if(params_.rate > 0.0 &&
       mean_detection_time / 1.0e3 > 1.0 / params_.rate) {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             time_buffer_size_ * (1.0 / params_.rate),
                             "Detection running too slow: %s ms",
                             std::to_string(mean_detection_time).c_str());
    }
}

void ObjectDetectorNode::PublishDetectionImage(cv::Mat image,
                                               std::vector<Detection2D> detections,
                                               std::string frame_id) {
    visualizer_->Overlay(image, detections);

    // Publish the detection image
    cv_bridge::CvImage detection_image;
    detection_image.header.stamp = get_clock()->now();
    detection_image.header.frame_id = frame_id;
    detection_image.encoding = "bgr8";
    detection_image.image = image;
    detections_image_publisher_->publish(*detection_image.toImageMsg());
}

double ObjectDetectorNode::GetMeanDetectionTime(double last_detection_time) {
    // Add the latest detection time to the buffer.
    time_buffer_.push_back(last_detection_time);

    // Compute the mean detection time in seconds.
    double mean_detection_time = 0.0;
    for(boost::circular_buffer<double>::const_iterator buffer_iterator = time_buffer_.begin();
        buffer_iterator != time_buffer_.end();
        ++buffer_iterator) {
        mean_detection_time += *(buffer_iterator);
    }
    mean_detection_time /= time_buffer_size_;

    return mean_detection_time;
}

} // namespace perception
} // namespace avt_341_nav
