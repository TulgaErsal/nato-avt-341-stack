#include <rclcpp/rclcpp.hpp>

#include <avt_341/perception/occupancy_grid_parser/occupancy_grid_parser_node.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<
        avt_341::perception::occupancy::OccupancyGridParserNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    return EXIT_SUCCESS;
}