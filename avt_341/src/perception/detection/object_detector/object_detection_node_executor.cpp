#include <avt_341/perception/detection/object_detector/object_detection_node.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<avt_341::perception::ObjectDetectorNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    return EXIT_SUCCESS;
}