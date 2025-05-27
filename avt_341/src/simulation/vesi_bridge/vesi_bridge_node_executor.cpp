#include <avt_341/simulation/vesi_bridge/vesi_bridge_node.hpp>

int main(int argc, char* argv[]) {
    try {
        rclcpp::init(argc, argv);

        rclcpp::Node::SharedPtr SensorBridgeNodePtr =
            std::make_shared<bridge::SensorBridgeNode>();
        rclcpp::executors::StaticSingleThreadedExecutor executor;
        executor.add_node(SensorBridgeNodePtr);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "Failed to initialize sensor-Bridge node: " << e.what()
                  << '\n';
    }
}
