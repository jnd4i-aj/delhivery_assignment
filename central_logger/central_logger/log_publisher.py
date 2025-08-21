from interfaces.msg import AllLogs

class LogPublisher:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(AllLogs, "/log", 200)

    def publish(self, level:int, code:str, bot_id:int=0):
        msg = AllLogs()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.bot_id = int(bot_id) & 0xFF
        msg.node_name = self.node.get_name()
        msg.log_level = int(level) & 0xFF
        msg.log_code = str(code)
        try:
            self.pub.publish(msg)
        except Exception as ex:
            self.node.get_logger().warning(f"Failed to publish log: {ex}")
