import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)
class Base(Node):
    def p(self, name, default):
        return self.declare_parameter(name, default).value
    def stamp(self):
        return self.get_clock().now().to_msg()
    def age(self, header):
        return (self.get_clock().now().nanoseconds -
                (header.stamp.sec * 1000000000 + header.stamp.nanosec))/1e9

def run(cls):
    rclpy.init()
    node = None
    try:
        node = cls()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
