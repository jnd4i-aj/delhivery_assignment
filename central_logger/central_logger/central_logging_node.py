#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import AllLogs
import queue, threading
from utils.service_logger import ServiceLogger
from utils.file_sys_utils import FileSysUtility
from utils.yaml_editor import YamlEditor


class CentralLogger(Node):
    def __init__(self, out_queue_size=5000):
        super().__init__("central_logger")
        self.slogger = ServiceLogger.get()

        codes_yaml_path = FileSysUtility.log_code_file_path()
        self.codes = self.__load_codes(codes_yaml_path)

        self.sub = self.create_subscription(AllLogs, "/log", self._cb, 200)

        self._q = queue.Queue(maxsize=out_queue_size)
        self._stop = threading.Event()
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()

        self.get_logger().info("CentralLogger started, listening on /log")

    def __load_codes(self, path):
        try:
            editor = YamlEditor()
            return editor.read(path) or {}
        except Exception as e:
            self.get_logger().warn(f"Failed to load codes yaml {path}: {e}")
            return {}

    def _cb(self, msg: AllLogs):
        try:
            tpl = self.codes.get(msg.log_code, "{code}")
            try:
                decoded = tpl.format(code=msg.log_code)
            except Exception:
                decoded = msg.log_code

            timestamp = f"{msg.stamp.sec}.{msg.stamp.nanosec:09d}"
            line = f"{timestamp} bot_id={msg.bot_id} {decoded}"

            try:
                self._q.put_nowait((msg.node_name, line, msg.log_level))
            except queue.Full:
                _ = self._q.get_nowait()
                self._q.put_nowait((msg.node_name, line, msg.log_level))
        except Exception as e:
            self.get_logger().error(f"Central logger callback failed: {e}")

    def _writer_loop(self):
        while not self._stop.is_set():
            try:
                node_name, line, level = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                level_map = {
                    0: "debug",
                    1: "info",
                    2: "warn",
                    3: "error",
                    4: "fatal",
                }
                level_str = level_map.get(int(level), "info")

                getattr(self.slogger, f"central_data_{level_str}")(node_name, line)
            except Exception as e:
                self.get_logger().error(f"Failed to write log: {e}")
            self._q.task_done()

    def destroy_node(self):
        try:
            self._stop.set()
            self._writer.join(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = CentralLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Central Logger interrupted, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
