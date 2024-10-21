#include <avt_341/perception/detection/object_detector/object_detector.hpp>

namespace avt_341 {
namespace perception {

ObjectDetector::ObjectDetector() {}

std::vector<std::string> ObjectDetector::ReadClassNames(std::ifstream& file) {
    std::string line;
    std::vector<std::string> names;
    while(std::getline(file, line)) { names.push_back(line); }
    return names;
}

void ObjectDetector::GetLetterbox(cv::Mat& image,
                                  cv::Mat& letterboxed_image,
                                  cv::Size size,
                                  cv::Scalar value) {
    if(image.cols == size.width && image.rows == size.height) {
        // Nothing to resize.
        if(image.data == letterboxed_image.data) {
            // Nothing to copy.
            return;
        } else {
            letterboxed_image = image.clone();
            return;
        }
    }

    // Get the smallest ratio.
    auto scale_factor = std::min(
        static_cast<float>(size.width) / static_cast<float>(image.rows),
        static_cast<float>(size.height) / static_cast<float>(image.cols));

    int width = std::round(image.cols * scale_factor);
    int height = std::round(image.rows * scale_factor);

    float pad_horizontal = (size.width - width) / 2.0f;
    float pad_vertical = (size.height - height) / 2.0f;

    cv::resize(/* Input image */ image,
               /* Output image */ letterboxed_image,
               /* Target size */ cv::Size(width, height),
               /* Width scale factor */ 0,
               /* Height scale factor */ 0,
               /* Interpolation */ cv::INTER_AREA);

    // Add letterbox.
    cv::copyMakeBorder(letterboxed_image,
                       letterboxed_image,
                       std::round(pad_vertical - 0.1),
                       std::round(pad_vertical + 0.1),
                       std::round(pad_horizontal - 0.1),
                       std::round(pad_horizontal + 0.1),
                       cv::BORDER_CONSTANT,
                       value);
}

torch::Tensor ObjectDetector::FromXYWH2XYXY(const torch::Tensor& xywh_tensor) {
    // Pre-allocate the output tensor.
    auto xyxy_tensor = torch::empty_like(xywh_tensor);

    // Define the pixel shifts in each direction.
    auto horizontal_shift = xywh_tensor.index({"...", 2}).div(2);
    auto vertical_shift = xywh_tensor.index({"...", 3}).div(2);

    // Apply the shifts to each coordinate.
    xyxy_tensor.index_put_({"...", 0},
                           xywh_tensor.index({"...", 0}) - horizontal_shift);
    xyxy_tensor.index_put_({"...", 1},
                           xywh_tensor.index({"...", 1}) - vertical_shift);
    xyxy_tensor.index_put_({"...", 2},
                           xywh_tensor.index({"...", 0}) + horizontal_shift);
    xyxy_tensor.index_put_({"...", 3},
                           xywh_tensor.index({"...", 1}) + vertical_shift);

    return xyxy_tensor;
}

torch::Tensor ObjectDetector::ScaleBoundingBoxes(torch::Tensor& bounding_boxes,
                                                 const cv::Size& input_shape,
                                                 const cv::Size& output_shape) {
    // Apply gain.
    auto gain = std::min(float(input_shape.height) / output_shape.height,
                         float(input_shape.width) / output_shape.width);

    // Define the padding.
    auto width_pad = std::round(
        float(input_shape.width - output_shape.width * gain) / 2.0 - 0.1);
    auto height_pad = std::round(
        float(input_shape.height - output_shape.height * gain) / 2.0 - 0.1);

    // Apply the padding.
    bounding_boxes.index_put_({"...", 0},
                              bounding_boxes.index({"...", 0}) - width_pad);
    bounding_boxes.index_put_({"...", 2},
                              bounding_boxes.index({"...", 2}) - width_pad);
    bounding_boxes.index_put_({"...", 1},
                              bounding_boxes.index({"...", 1}) - height_pad);
    bounding_boxes.index_put_({"...", 3},
                              bounding_boxes.index({"...", 3}) - height_pad);

    // Scale by the gain.
    bounding_boxes.index_put_(
        {"...", torch::indexing::Slice(torch::indexing::None, 4)},
        bounding_boxes
            .index({"...", torch::indexing::Slice(torch::indexing::None, 4)})
            .div(gain));

    return bounding_boxes;
}

torch::Tensor
ObjectDetector::NonMaximumSuppressionKernel(const torch::Tensor& bounding_boxes,
                                            const torch::Tensor& scores) {
    // If no bounding boxes are present, return an empty tensor of the matching
    // type.
    if(bounding_boxes.numel() == 0)
        return torch::empty({0}, bounding_boxes.options().dtype(torch::kLong));

    // Get the bounding boxes coordinates and areas.
    auto x_min = bounding_boxes.select(1, 0).contiguous();
    auto y_min = bounding_boxes.select(1, 1).contiguous();
    auto x_max = bounding_boxes.select(1, 2).contiguous();
    auto y_max = bounding_boxes.select(1, 3).contiguous();

    torch::Tensor areas_t = (x_max - x_min) * (y_max - y_min);

    auto order_t = std::get<1>(scores.sort(/*  Use stable sort. */ true,
                                           /* Sort along this dimension. */ 0,
                                           /* Use descending order. */ true));

    auto number_of_detections = bounding_boxes.size(0);
    torch::Tensor suppressed_t =
        torch::zeros({number_of_detections},
                     bounding_boxes.options().dtype(torch::kByte));
    torch::Tensor keep_t =
        torch::zeros({number_of_detections},
                     bounding_boxes.options().dtype(torch::kLong));

    auto suppressed = suppressed_t.data_ptr<uint8_t>();
    auto keep = keep_t.data_ptr<int64_t>();
    auto order = order_t.data_ptr<int64_t>();
    auto x_min_ptr = x_min.data_ptr<float>();
    auto y_min_ptr = y_min.data_ptr<float>();
    auto x_max_ptr = x_max.data_ptr<float>();
    auto y_max_ptr = y_max.data_ptr<float>();
    auto areas_ptr = areas_t.data_ptr<float>();

    int64_t number_of_filtered_detections = 0;

    for(int64_t k = 0; k < number_of_detections; k++) {
        auto i = order[k];
        if(suppressed[i] == 1) continue;
        keep[number_of_filtered_detections++] = i;
        auto x_min_1 = x_min_ptr[i];
        auto y_min_1 = y_min_ptr[i];
        auto x_max_1 = x_max_ptr[i];
        auto y_max_1 = y_max_ptr[i];
        auto iarea = areas_ptr[i];

        for(int64_t l = k + 1; l < number_of_detections; l++) {
            auto j = order[l];
            if(suppressed[j] == 1) continue;
            auto x_min_2 = std::max(x_min_1, x_min_ptr[j]);
            auto y_min_2 = std::max(y_min_1, y_min_ptr[j]);
            auto x_max_2 = std::min(x_max_1, x_max_ptr[j]);
            auto y_max_2 = std::min(y_max_1, y_max_ptr[j]);

            auto intersection_width =
                std::max(static_cast<float>(0), x_max_2 - x_min_2);
            auto intersection_height =
                std::max(static_cast<float>(0), y_max_2 - y_min_2);
            auto intersection = intersection_width * intersection_height;

            // Compute the Intersection-Over_union (IOU) metric for this pair.
            auto iou = intersection / (iarea + areas_ptr[j] - intersection);
            if(iou > iou_threshold_) suppressed[j] = 1;
        }
    }
    return keep_t.narrow(0, 0, number_of_filtered_detections);
}

torch::Tensor
ObjectDetector::NonMaximumSuppression(torch::Tensor& predictions) {
    auto batch_size = predictions.size(0);
    auto nc = predictions.size(1) - 4;
    auto nm = predictions.size(1) - nc - 4;
    auto mi = 4 + nc;

    // Select the prediction above the score threshold.
    auto filtered_predictions =
        predictions
            .index({torch::indexing::Slice(), torch::indexing::Slice(4, mi)})
            .amax(1) > score_threshold_;

    // Convert the bounding boxes from XYWH format to XYXY format.
    predictions = predictions.transpose(-1, -2);
    predictions.index_put_(
        {"...", torch::indexing::Slice({torch::indexing::None, 4})},
        FromXYWH2XYXY(predictions.index(
            {"...", torch::indexing::Slice(torch::indexing::None, 4)})));

    // Pre-allocate and initialize the vector of output (filtered) tensors.
    std::vector<torch::Tensor> output;
    for(int i = 0; i < batch_size; i++) {
        output.push_back(torch::zeros({0, 6 + nm}, predictions.device()));
    }

    for(int i = 0; i < predictions.size(0); i++) {
        auto prediction = predictions[i];
        prediction = prediction.index({filtered_predictions[i]});
        auto x_split = prediction.split({4, nc, nm}, 1);
        auto box = x_split[0], cls = x_split[1], mask = x_split[2];
        auto [conf, j] = cls.max(1, true);
        prediction = torch::cat({box, conf, j.toType(torch::kFloat), mask}, 1);
        prediction = prediction.index({conf.view(-1) > score_threshold_});
        int n = prediction.size(0);
        if(!n) { continue; }

        // Non-Maximum Suppression (NMS)
        // -----------------------------
        auto c = prediction.index(
                     {torch::indexing::Slice(), torch::indexing::Slice{5, 6}}) *
            7680;

        // Collect the bounding boxes and confidence scores.
        auto bounding_boxes =
            prediction.index(
                {torch::indexing::Slice(),
                 torch::indexing::Slice(torch::indexing::None, 4)}) +
            c;
        auto scores = prediction.index({torch::indexing::Slice(), 4});

        // Apply the Non-Maximum Suppression (NMS) kernel.
        auto filtered_indices =
            NonMaximumSuppressionKernel(bounding_boxes, scores);
        // -----------------------------

        // Ensure there are no more detections than the user-specific threshold.
        filtered_indices = filtered_indices.index(
            {torch::indexing::Slice(torch::indexing::None, count_threshold_)});

        // Add the filtered prediction to the output vector.
        output[i] = prediction.index({filtered_indices});
    }

    return torch::stack(output);
}

void ObjectDetector::Load(std::string path) {
    device_type_ = GetModelDevice(GetModelDeviceToken(path));

    auto classes_file = std::ifstream(path + ".names");
    classes_ = ReadClassNames(classes_file);

    model_size_ = GetModelSize(GetModelSizeToken(path));

    module_ = torch::jit::load(path + ".torchscript", device_type_);
}

void ObjectDetector::Warmup() {
    module_.forward({torch::empty({1, 3, model_size_.width, model_size_.height},
                                  device_type_)});
}

std::vector<Detection2D> ObjectDetector::Detect(cv::Mat& image) {
    // Enable inference mode, disable dropout layer and adjust batch
    // normalization for this block.
    c10::InferenceMode inference_guard(true);
    module_.eval();

    cv::Mat letterboxed_image;
    GetLetterbox(image,
                 letterboxed_image,
                 cv::Size(model_size_.width, model_size_.height));

    auto image_tensor =
        torch::from_blob(letterboxed_image.data,
                         {letterboxed_image.rows, letterboxed_image.cols, 3},
                         torch::kByte)
            .toType(torch::kFloat32) /* Convert to floating point */
            .div(255) /* Map values from [0 - 255] to [0.0, 1.0] */
            .permute({2, 0, 1}) /* Permute to (channel, row, column)  */
            .unsqueeze(0) /* Add a dimension for the batch size. */
            .to(device_type_); /* Move to the selected device. */

    // Run the inference.
    auto output = module_.forward({image_tensor}).toTensor().cpu();

    // Perform the Non-Maximum-Suppression (NMS) step.
    auto keep = NonMaximumSuppression(output)[0];

    // Extract the tensor dimensions for the bounding boxes coordinates.
    auto bounding_boxes =
        keep.index({torch::indexing::Slice(),
                    torch::indexing::Slice(torch::indexing::None, 4)});

    keep.index_put_({torch::indexing::Slice(),
                     torch::indexing::Slice(torch::indexing::None, 4)},
                    ScaleBoundingBoxes(bounding_boxes,
                                       letterboxed_image.size(),
                                       image.size()));

    std::vector<Detection2D> detections;
    for(int i = 0; i < keep.size(0); i++) {
        int x_min = keep[i][0].item().toFloat();
        int y_min = keep[i][1].item().toFloat();
        int x_max = keep[i][2].item().toFloat();
        int y_max = keep[i][3].item().toFloat();
        float score = keep[i][4].item().toFloat();
        int class_id = keep[i][5].item().toInt();

        // Check if the class is in the list of valid classes.
        if(valid_classes_.size() > 0 &&
           !(std::find(std::begin(valid_classes_),
                       std::end(valid_classes_),
                       classes_[class_id]) != std::end(valid_classes_))) {
            continue;
        }

        auto detection = Detection2D(BoundingBox2D(x_min, x_max, y_min, y_max),
                                     Hypothesis(class_id, score));
        detections.push_back(detection);
    }

    return detections;
}

void ObjectDetector::SetScoreThreshold(double score_threshold) {
    if(score_threshold < 0.0 || score_threshold > 1.0) {
        throw std::invalid_argument(
            "Score threshold must be a positive number lower than 1.0.");
        return;
    }

    score_threshold_ = score_threshold;
}

void ObjectDetector::SetIOUThreshold(double iou_threshold) {
    if(iou_threshold < 0.0 || iou_threshold > 1.0) {
        throw std::invalid_argument(
            "Intersection-Over-Union (IOU) threshold must be a positive number "
            "lower than 1.0.");
        return;
    }

    iou_threshold_ = iou_threshold;
}

void ObjectDetector::SetCountThreshold(int count_threshold) {
    if(count_threshold < 0) {
        throw std::invalid_argument(
            "Count threshold must be a strictly positive number.");
        return;
    }

    count_threshold_ = count_threshold;
}

void ObjectDetector::SetValidClasses(std::vector<std::string> classes) {
    valid_classes_ = classes;
}

std::vector<std::string>
ObjectDetector::SplitByDelimiter(const std::string& string,
                                 const char delimiter) {
    std::stringstream stream(string);
    std::vector<std::string> tokens;
    std::string token;
    while(std::getline(stream, token, delimiter)) { tokens.push_back(token); }
    return tokens;
}

inline std::string
ObjectDetector::GetModelNameToken(const std::string& filename) {
    return SplitByDelimiter(filename)[0];
}

inline std::string
ObjectDetector::GetModelDeviceToken(const std::string& filename) {
    return SplitByDelimiter(filename)[1];
}

torch::DeviceType
ObjectDetector::GetModelDevice(const std::string& device_name) {
    if(device_name == "cpu") {
        return torch::kCPU;
    } else if(device_name == "cuda") {
        return torch::kCUDA;
    } else {
        throw std::invalid_argument("Invalid device name.");
    }
}

inline std::string
ObjectDetector::GetModelSizeToken(const std::string& filename) {
    return SplitByDelimiter(filename)[2];
}

cv::Size ObjectDetector::GetModelSize(const std::string& size) {
    std::vector<std::string> shape = SplitByDelimiter(size, 'x');
    return cv::Size(std::stoi(shape[0]), std::stoi(shape[1]));
}

} // namespace perception
} // namespace avt_341