#!/usr/bin/env python3

import time
import threading
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import NavData, TaskQueue, TargetPose
from central_logger.log_publisher import LogPublisher


class L:
    DEBUG = 0
    INFO  = 1
    WARN  = 2
    ERROR = 3
    FATAL = 4


class NavigationNode(Node):
    def __init__(self):
        super().__init__("Navigation_node")
        self.get_logger().info("Starting Navigation Node")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("start_node", 1)
        self.declare_parameter("step_duration", 1.0)
        self.declare_parameter("pickup_dwell", 2.0)
        self.declare_parameter("status_publish_period", 1.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.current_node = int(self.get_parameter("start_node").value)
        self.step_duration = float(self.get_parameter("step_duration").value)
        self.pickup_dwell = float(self.get_parameter("pickup_dwell").value)
        self.status_period = float(self.get_parameter("status_publish_period").value)

        self.status_pub = self.create_publisher(NavData, "/nav_data", 10)
        self.goal_sub = self.create_subscription(TargetPose, "/target_pose", self.__goal_sub_callback, 10)
        self.task_queue_sub = self.create_subscription(TaskQueue, "/task_queue", self.__task_queue_callback, 10)

        self.log_pub = LogPublisher(self)

        self.task_id: str = ""
        self.pickup_tag: int = 0
        self.drop_tag: int = 0

        self.left_turn_nodes: List[int] = []
        self.right_turn_nodes: List[int] = []
        self.left_turns_to_pickup: List[int] = []
        self.right_turns_to_pickup: List[int] = []
        self.left_turns_to_drop: List[int] = []
        self.right_turns_to_drop: List[int] = []

        self.path_to_goal: List[int] = []

        self.pickup_flag: bool = False
        self.drop_flag: bool = False
        self.executing: bool = False

        self._lock = threading.Lock()

        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None

        self._status_timer = self.create_timer(self.status_period, self.__publish_status_periodic)

        self.__publish_status("idle")
        self.get_logger().info("Navigation Node ready and publishing status periodically.")
        self._safe_log(L.INFO,  "I-100", self.robot_id)

    def _safe_publish(self, pub, msg):
        try:
            if rclpy.ok() and not self._stop_event.is_set():
                pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"safe_publish suppressed: {e}")

    def _sleep_with_cancel(self, seconds: float):
        end = time.time() + seconds
        while not self._stop_event.is_set():
            remaining = end - time.time()
            if remaining <= 0:
                break
            time.sleep(min(0.1, remaining))

    def __task_queue_callback(self, msg: TaskQueue):
        try:
            with self._lock:
                self.task_id = str(msg.task_id)
                self.pickup_tag = int(msg.pickup_node)
                self.drop_tag = int(msg.drop_node)
            self.get_logger().info(f"TaskQueue received: {self.task_id} Pickup Node is {self.pickup_tag} Drop Node Location {self.drop_tag}")
            self._safe_log(L.INFO,  "I-110", self.robot_id)
        except Exception as e:
            self._safe_log(L.ERROR, "E-111", self.robot_id)
            self.get_logger().error(f"Malformed TaskQueue msg: {e}")

    def __goal_sub_callback(self, msg: TargetPose):
        # If already executing, ignore new target (Task Manager should check idle)
        with self._lock:
            if self.executing:
                self.get_logger().warn("Received TargetPose while already executing -> ignoring this TargetPose.")
                self._safe_log(L.WARN,  "W-120", self.robot_id)
                return

            try:
                pickup_path = [int(x) for x in msg.path_to_pickup]
                drop_path = [int(x) for x in msg.path_to_drop]
                left_p = [int(x) for x in msg.pickup_left_nodes]
                right_p = [int(x) for x in msg.pickup_right_nodes]
                left_d = [int(x) for x in msg.drop_left_nodes]
                right_d = [int(x) for x in msg.drop_right_nodes]
            except Exception as e:
                self._safe_log(L.ERROR, "E-121", self.robot_id)
                self.get_logger().error(f"Malformed TargetPose arrays: {e}")
                return

            if len(drop_path) > 0:
                composed = pickup_path + drop_path[1:]
            else:
                composed = pickup_path[:]

            if not self.task_id:
                self.get_logger().error("No TaskQueue info available (task_id missing). Aborting execution.")
                self._safe_log(L.ERROR, "E-122", self.robot_id)
                return
            if not composed:
                self.get_logger().error("Empty composed path -> aborting execution.")
                self._safe_log(L.ERROR, "E-123", self.robot_id)
                return
            if self.pickup_tag not in pickup_path or self.drop_tag not in drop_path:
                self.get_logger().error("Pickup/Drop tags not present in provided TargetPose -> aborting execution.")
                self._safe_log(L.ERROR, "E-124", self.robot_id)
                return

            self.path_to_goal = composed
            self.left_turns_to_pickup = left_p[:]
            self.right_turns_to_pickup = right_p[:]
            self.left_turns_to_drop = left_d[:]
            self.right_turns_to_drop = right_d[:]

            self.left_turn_nodes = self.left_turns_to_pickup[:]
            self.right_turn_nodes = self.right_turns_to_pickup[:]
            self.pickup_flag = False
            self.drop_flag = False
            self.executing = True

            task_snapshot = self.task_id

        self._worker_thread = threading.Thread(target=self._execute_task_worker, args=(task_snapshot,), daemon=True)
        self._worker_thread.start()
        self.get_logger().info(f"Started execution worker for task {task_snapshot}")
        self._safe_log(L.INFO,  "I-125", self.robot_id)

    def _safe_log(self, level, code, robot_id):
        if not self._stop_event.is_set() and rclpy.ok():
            try:
                self._safe_log(level, code, robot_id)
            except Exception:
                pass

    def _execute_task_worker(self, task_snapshot: str):
        try:
            path = []
            with self._lock:
                path = self.path_to_goal[:]

            self.get_logger().info(f"Worker executing task {task_snapshot} path={path}")
            self._safe_log(L.INFO,  "I-130", self.robot_id)

            for idx, node_id in enumerate(path):
                if self._stop_event.is_set():
                    break
                with self._lock:
                    self.current_node = int(node_id)

                self.get_logger().info(f"[worker] Moved to node {self.current_node}")
                self._safe_log(L.DEBUG, "D-131", self.robot_id)

                self.__check_goal()
                self.__publish_status(self.__get_status_string())

                if self._stop_event.is_set():
                    break

                with self._lock:
                    at_pickup = (self.pickup_flag and (self.current_node == self.pickup_tag))
                if at_pickup:
                    self.get_logger().info(f"[worker] At pickup {self.pickup_tag} — dwelling {self.pickup_dwell}s")
                    self._safe_log(L.INFO,  "I-132", self.robot_id)
                    self._safe_log(L.INFO,  "I-133", self.robot_id)
                    time.sleep(self.pickup_dwell)
                    with self._lock:
                        self.left_turn_nodes = self.left_turns_to_drop[:]
                        self.right_turn_nodes = self.right_turns_to_drop[:]
                    self.__publish_status(self.__get_status_string())
                    self._safe_log(L.INFO,  "I-134", self.robot_id)

                with self._lock:
                    if self.drop_flag:
                        self.get_logger().info(f"[worker] Drop completed at {self.current_node} for task {task_snapshot}")
                        self._safe_log(L.INFO,  "I-135", self.robot_id)
                        break

                if idx < (len(path) - 1):
                    time.sleep(self.step_duration)

            with self._lock:
                finished_ok = bool(self.drop_flag)
            if finished_ok:
                self.__publish_status("COMPLETED")
                self.get_logger().info(f"Task {task_snapshot} completed successfully.")
                self._safe_log(L.INFO,  "I-136", self.robot_id)
            else:
                self.__publish_status("failed")
                self.get_logger().warn(f"Task {task_snapshot} ended without drop_flag; marking failed.")
                self._safe_log(L.WARN,  "W-137", self.robot_id)

        except Exception as e:
            self._safe_log(L.ERROR, "E-141", self.robot_id)
            self.get_logger().error(f"Error during execution worker: {e}")
            self.__publish_status("failed")
        finally:
            with self._lock:
                self.executing = False
                self.pickup_tag = 0
                self.drop_tag = 0
                self.task_id = ""
                self.left_turn_nodes = []
                self.right_turn_nodes = []
                self.path_to_goal = []
                self.pickup_flag = False
                self.drop_flag = False

            self.__publish_status("idle")

    def __check_goal(self):

        try:
            with self._lock:
                curr = int(self.current_node)
                if (curr == self.pickup_tag) and (self.pickup_tag != 0) and (not self.pickup_flag):
                    self.get_logger().info(f"Reached Pickup location: {self.pickup_tag}. Setting pickup_flag=True")
                    self.pickup_flag = True
                    self.left_turn_nodes.clear()
                    self.right_turn_nodes.clear()
                    return

                if self.pickup_flag and (curr == self.drop_tag) and (self.drop_tag != 0):
                    self.get_logger().info(f"Reached Drop location: {self.drop_tag}. Performing drop and setting drop_flag=True")
                    self.drop_flag = True
                    self.left_turn_nodes.clear()
                    self.right_turn_nodes.clear()
                    return
        except Exception as e:
            self._safe_log(L.ERROR, "E-142", self.robot_id)
            self.get_logger().error(f"Error in __check_goal: {e}")
            raise

    def __get_status_string(self) -> str:

        try:
            with self._lock:
                executing = self.executing
                pickup_flag = self.pickup_flag
                drop_flag = self.drop_flag
                curr = int(self.current_node)
                pick = int(self.pickup_tag)
                drop = int(self.drop_tag)

            if executing and (not pickup_flag):
                if curr == pick:
                    return "AT_PICKUP"
                return "NAVIGATING_TO_PICKUP"
            if executing and pickup_flag and (not drop_flag):
                if curr == drop:
                    return "AT_DROPOFF"
                return "NAVIGATING_TO_DROPOFF"
            if drop_flag:
                return "COMPLETED"
            return "idle"
        except Exception:
            return "idle"

    def __publish_status(self, status: str = None):
        if status is None:
            status = self.__get_status_string()
        msg = NavData()

        try:
            msg.bot_id = self.robot_id
            msg.current_node = self.current_node
            msg.status = status
            self.status_pub.publish(msg)
            self.get_logger().info(f"Published Robot Current status={status}, Current Location Node={self.current_node}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish robot status: {e}")

    def __publish_status_periodic(self):
        self.__publish_status(None)


    def destroy_node(self):
        self._stop_event.set()
        try:
            self._status_timer.cancel()
        except Exception:
            pass
        try:
            if self._worker_thread and self._worker_thread.is_alive():
                self._worker_thread.join(timeout=2.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested—cleaning up!")
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
