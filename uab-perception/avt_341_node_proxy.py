from typing import Callable
import rclpy

class NodeProxy:
    def __init__(self, node_name):
        rclpy.init()
        self.node_name = node_name
        self.node = rclpy.create_node(node_name)
        
    def spin(self):
        rclpy.spin_once(self.node, executor=None, timeout_sec=0.0)
        
    def get_logger(self):
        return self.node.get_logger()
    
    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    def create_subscription(self, type, topic: str, callback: Callable, queue_size: int):
        return self.node.create_subscription(type, topic, callback, queue_size)
    
    def create_publisher(self, type, topic: str, queue_size: int):
        return self.node.create_publisher(type, topic, queue_size)