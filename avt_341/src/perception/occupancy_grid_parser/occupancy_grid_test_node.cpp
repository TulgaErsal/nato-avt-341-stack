#include <avt_341/perception/occupancy_grid_parser/occupancy_grid_test_node.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<avt_341::testing::OccupancyGridNode>());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

namespace avt_341 {
namespace testing {

OccupancyGridNode::OccupancyGridNode() : Node("occupancy_grid_test") {
    declare_parameter("use_image", false);
    bool use_image = get_parameter("use_image").as_bool();
    declare_parameter("image", "");
    auto path = get_parameter("image").as_string();

    occupancy_grid_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
    timer_ =
        create_wall_timer(500ms,
                          std::bind(&OccupancyGridNode::TimerCallback, this));

    if(use_image) {
        auto image = cv::imread(path);

        // Threshold the values to the valid occupancy range.
        cv::threshold(image, image_, 0, 100, cv::THRESH_BINARY);
    } else {
        // Add some occupancy.
        image_ = cv::Mat(5, 5, CV_8UC1, cv::Scalar(0));
        image_.at<cv::Vec3b>(1, 1) = 100;
    }
}

void OccupancyGridNode::TimerCallback() {
    auto occupancy_grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    occupancy_grid_msg->header.stamp = get_clock()->now();
    occupancy_grid_msg->header.frame_id = "map";
    occupancy_grid_msg->info.width = image_.cols;
    occupancy_grid_msg->info.height = image_.rows;
    occupancy_grid_msg->info.origin.position.x = 0;
    occupancy_grid_msg->info.origin.position.y = -1.0 * image_.cols;
    occupancy_grid_msg->info.resolution = 1.0;
    for(int i = image_.size[0] * image_.size[1] - 1; i >= 0; --i) {
        occupancy_grid_msg->data.push_back(int8_t(image_.data[i]));
    }

    occupancy_grid_pub_->publish(*occupancy_grid_msg);
}

} // namespace testing
} // namespace avt_341
