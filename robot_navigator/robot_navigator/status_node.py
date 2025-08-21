#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus, NavData
from central_logger.log_publisher import LogPublisher


class StatusNode(Node):

    def __init__(self):
        super().__init__("status_node")
        self.get_logger().info("Starting simple Status Node")

        self.pub = self.create_publisher(RobotStatus, "/robot_status", 10)
        self.sub = self.create_subscription(NavData, "/nav_data", self.__nav_data_callback, 10)
        self.log_pub = LogPublisher(self)
        self.log_pub.publish(1, "I-300", bot_id=0)

    def __nav_data_callback(self, msg: NavData):
        bot_id = getattr(msg, "bot_id", 0)
        try:
            current_status = msg.status
            self.get_logger().info(f"Current Robot Status is {current_status}")
            self.log_pub.publish(1, "I-311", bot_id)
        except Exception:
            self.log_pub.publish(3, "E-301", bot_id)
            self.get_logger().warning("Failed to get Data from Nav")

        try:
            status_msg = RobotStatus()
            status_msg.bot_id = bot_id
            status_msg.robot_status = current_status
            self.pub.publish(status_msg)
        except Exception as e:
            self.log_pub.publish(3, "E-302", bot_id)
            self.get_logger().error(f"Failed to publish RobotStatus: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
