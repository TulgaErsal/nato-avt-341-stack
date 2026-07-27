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

* @file      object_detector.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for the object detector.
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

#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

#include <avt_341/perception/detection/common/detection_2d.hpp>

namespace avt_341 {
namespace perception {

class ObjectDetector {
  public:
    /**
     * @brief Construct a new object detector.
     */
    ObjectDetector();

    /**
     * @brief Read object classes from a plain-text file with one class per row into a vector of strings.
     *
     * @param file
     * @return std::vector<std::string>
     */
    std::vector<std::string> ReadClassNames(std::ifstream& file);

    /**
     * @brief Load a TorchScript model and class names files from a specified path.
     *
     * @param path Path to an object detector model (including the model filename, excluding the extension).
     */
    void Load(std::string path);

    /**
     * @brief Warm up the neural network by performing inference with an empty tensor.
     */
    void Warmup();

    /**
     * @brief Run the object detection model on an OpenCV image.
     *
     * @param image Input OpenCV image.
     * @return std::vector<Detection2D> Vecotr of image detections.
     */
    std::vector<Detection2D> Detect(cv::Mat& image);

    /**
     * @brief Set the confidence score threshold.
     *
     * @param score_threshold Confidence score threshold.
     */
    void SetScoreThreshold(double score_threshold);

    /**
     * @brief Set the Intersection-Over-Union metric threshold during Non-Maximum Suppression (NMS) filtering.
     *
     * @param iou_threshold Intersection-Over-Union (IOU) metric threshold.
     */
    void SetIOUThreshold(double iou_threshold);

    /**
     * @brief Set the maximum number of detections in a single frame detection task.
     *
     * @param count_threshold Maximum number of detections.
     */
    void SetCountThreshold(int count_threshold);

    /**
     * @brief Set the subset of valid detection classes in the detection model.
     *
     * @param classes Vector of valid classes.
     */
    void SetValidClasses(std::vector<std::string> classes);

  private:
    /**
     * @brief Split a string with a specified delimiter.
     *
     * @param string String to be split.
     * @param delimiter Delimiter character used to split the string.
     * @return std::vector<std::string> Vector of split substrings (excluding the delimiter).
     */
    std::vector<std::string> SplitByDelimiter(const std::string& string, const char delimiter = '-');

    /**
     * @brief Get the model name token from a filename.
     *
     * @param filename Model filename to be parsed.
     * @return std::string Model name token.
     */
    inline std::string GetModelNameToken(const std::string& filename);

    /**
     * @brief Get the model device token from a filename.
     *
     * @param filename Model filename to be parsed.
     * @return std::string Model device token (can be either "cpu" or "cuda").
     */
    inline std::string GetModelDeviceToken(const std::string& filename);

    /**
     * @brief Get the model Torch device type from a model device token.
     *
     * @param device_name Model device token.
     * @return torch::DeviceType Torch device type.
     */
    torch::DeviceType GetModelDevice(const std::string& device_name);

    /**
     * @brief Get the model size token from a filename.
     *
     * @param filename Model filename to be parsed.
     * @return std::string Model size token (with format WxH)
     */
    inline std::string GetModelSizeToken(const std::string& filename);

    /**
     * @brief Get the the model size from a model size token.
     *
     * @param size Model size token.
     * @return cv::Size Model input size (as OpenCV image size).
     */
    cv::Size GetModelSize(const std::string& size);

    /**
     * @brief Create a letterbox version of an image.
     *
     * @param image Input image to be letterboxed.
     * @param letterboxed_image Empty letterboxed image.
     * @param size Size of the letterboxed image.
     * @param value Background color used for the letterbox.
     */
    void GetLetterbox(cv::Mat& image, cv::Mat& letterboxed_image, cv::Size size, cv::Scalar value = cv::Scalar(139.0));

    /**
     * @brief Convert a tensor of XYWH bounding box coordinates to a tensor of XYXY bounding box coordinates.
     *
     * @param xywh_tensor Input tensor of bounding boxes XYWH coordinates.
     * @return torch::Tensor Output tensor of bounding boxes in XYXY coordinates.
     */
    torch::Tensor FromXYWH2XYXY(const torch::Tensor& xywh_tensor);

    /**
     * @brief Scale a tensor of bounding box coordinates referenced to an input image shape to an output image shape.
     *
     * @param bounding_boxes
     * @param input_shape
     * @param output_shape
     * @return torch::Tensor
     */
    torch::Tensor
    ScaleBoundingBoxes(torch::Tensor& bounding_boxes, const cv::Size& input_shape, const cv::Size& output_shape);

    /**
     * @brief Apply Non-Maximum Suppression (NMS) to a TorchScript tensor.
     *
     * @param bounding_boxes Tensor of bounding boxes in YOLOv8 format.
     * @param scores Tensor of confidence scores.
     * @return torch::Tensor Indexes of the filtered detections.
     */
    torch::Tensor NonMaximumSuppressionKernel(const torch::Tensor& bounding_boxes, const torch::Tensor& scores);

    /**
     * @brief Run a YOLOv8 Torch tensor through the Non-Maximum Suppression (NMS) kernel.
     *
     * @param predictions Tensor of predictions.
     * @return torch::Tensor Tensor of filtered predictions.
     */
    torch::Tensor NonMaximumSuppression(torch::Tensor& predictions);

    /** @brief Loaded TorchScript module. */
    torch::jit::Module module_;

    /** @brief Loaded TorchScript module device type.  */
    c10::DeviceType device_type_;

    /** @brief Loaded TorchScript module model size. */
    cv::Size model_size_;

    /** @brief Confidence score threshold. */
    double score_threshold_ = 0.25;

    /** @brief Intersection-Over-Union (IOU) metric threshold. */
    double iou_threshold_ = 0.5;

    /** @brief Maximum number of detections for a single frame detection task.
     */
    int count_threshold_ = 100;

    /** @brief Vector of loaded detection classes. */
    std::vector<std::string> classes_;

    /** @brief Vector of valid detection classes. */
    std::vector<std::string> valid_classes_ = std::vector<std::string>();
};

} // namespace perception
} // namespace avt_341