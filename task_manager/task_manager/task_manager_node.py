#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import re
from collections import deque

from interfaces.msg import NavData, TaskQueue, TargetPose
from interfaces.srv import AllocateTask
from utils.file_sys_utils import FileSysUtility
from path_planning.graph import Graph
from central_logger.log_publisher import LogPublisher


class L:
    DEBUG = 0
    INFO  = 1
    WARN  = 2
    ERROR = 3
    FATAL = 4


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")

        self.__path_planner = Graph.get()
        self.__path_planner.from_yaml(FileSysUtility.trajectory_map_path())

        self.get_logger().info("Starting Task Manager Node")

        self.allocate_srv = self.create_service(
            AllocateTask,
            '/start_delivery',
            self.allocate_task_service_callback,
        )

        self.__task_pub = self.create_publisher(TaskQueue, '/task_queue', 10)
        self.__goal_pub = self.create_publisher(TargetPose, '/target_pose', 10)

        self.create_subscription(
            NavData, '/nav_data', self.__robot_status_callback, 10)

        self.log_pub = LogPublisher(self)
        self.__robot_status = 'idle'
        self.__robot_current_node = 0
        self.robot_id: int = 0

        self.__task_queue = deque()

    def __robot_status_callback(self, msg: NavData):
        self.robot_id = msg.bot_id
        self.__robot_status = msg.status
        self.__robot_current_node = msg.current_node
        self.get_logger().info(f"ROBOT STATUS AND NODE {self.__robot_status} and {self.__robot_current_node}")

        # When robot becomes idle → assign next task if queue is not empty
        if self.__robot_status == "idle" and self.__task_queue:
            next_task_id = self.__task_queue.popleft()
            self.get_logger().info(f"Robot is idle. Assigning next task from the queue {next_task_id}")
            self.log_pub.publish(L.INFO, "I-220", self.robot_id)
            self.__assign_task(next_task_id)

    def allocate_task_service_callback(self, request, response):
        received_task_id = request.task_id
        self.log_pub.publish(L.INFO, "I-200", self.robot_id)

        if self.__robot_current_node == 0:
            self.get_logger().error(f"Invalid Current Node {self.__robot_current_node}, Check the Current Node.")
            self.log_pub.publish(L.ERROR, "E-201", self.robot_id)
            response.success = False
            response.message = f"Invalid Current Node {self.__robot_current_node}, Check the Current Node."
            return response

        # If robot busy → enqueue the task
        if self.__robot_status != "idle":
            self.get_logger().warn(
                f"Robot busy (status={self.__robot_status}). Adding task {received_task_id} to queue."
            )
            self.__task_queue.append(received_task_id)
            self.log_pub.publish(L.INFO, "I-202", self.robot_id)
            response.success = True
            response.message = f"Robot busy, task {received_task_id} queued."
            return response

        # If robot idle → assign immediately
        self.__assign_task(received_task_id)
        response.success = True
        response.message = f"Task {received_task_id} assigned immediately."
        return response

    def __assign_task(self, task_id: str):
        self.get_logger().info(f"Assigning task {task_id}")

        match_task_id = re.match(r"P(\d+)D(\d+)", task_id)
        if not match_task_id:
            self.get_logger().error(f"Invalid task_id format: {task_id}")
            self.log_pub.publish(L.ERROR, "E-204", self.robot_id)
            return

        pickup_node = int(match_task_id.group(1))
        drop_node = int(match_task_id.group(2))

        self.get_logger().info(f"Bot current Node: {self.__robot_current_node}, Pickup: {pickup_node}, Drop: {drop_node}")

        task_msg = TaskQueue()
        task_msg.task_id = task_id
        task_msg.pickup_node = pickup_node
        task_msg.drop_node = drop_node
        self.__task_pub.publish(task_msg)

        try:
            path_to_pickup, left_turns_to_pickup, right_turns_to_pickup = self.__path_planner.plan_path(
                self.__robot_current_node, pickup_node
            )

            if path_to_pickup is None:
                self.get_logger().error(
                    f"No path found from {self.__robot_current_node} to {pickup_node} for Task {task_id}"
                )
                self.log_pub.publish(L.ERROR, "E-208", self.robot_id)
                self.get_logger().error(f"Aborting Task {task_id}")
                return

            self.get_logger().info(
                f"Generated path to Pickup {pickup_node} from Current Node {self.__robot_current_node} for Task {task_id}: {path_to_pickup}"
            )
            self.get_logger().info(
                f"Left Tags to Pickup {pickup_node} for Task {task_id}: {left_turns_to_pickup}"
            )
            self.get_logger().info(
                f"Right Tags to Pickup {pickup_node} for Task {task_id}: {right_turns_to_pickup}"
            )
            path_to_drop, left_turns_to_drop, right_turns_to_drop = self.__path_planner.plan_path(
                pickup_node, drop_node
            )

            if path_to_drop is None:
                self.get_logger().error(
                    f"No path found from {pickup_node} to {drop_node} for Task {task_id}"
                )
                self.log_pub.publish(L.ERROR, "E-211", self.robot_id)
                self.get_logger().error(f"Aborting Task {task_id}")
                return 

            self.get_logger().info(
                f"Generated path to Drop {drop_node} from Pickup {pickup_node} for Task {task_id}: {path_to_drop}"
            )
            self.get_logger().info(
                f"Left Tags to Drop {drop_node} for Task {task_id}: {left_turns_to_drop}"
            )
            self.get_logger().info(
                f"Right Tags to Drop {drop_node} for Task {task_id}: {right_turns_to_drop}"
            )

            target_pose_msg = TargetPose()
            target_pose_msg.pickup_left_nodes = left_turns_to_pickup
            target_pose_msg.pickup_right_nodes = right_turns_to_pickup
            target_pose_msg.drop_left_nodes = left_turns_to_drop
            target_pose_msg.drop_right_nodes = right_turns_to_drop
            target_pose_msg.path_to_drop = path_to_drop
            target_pose_msg.path_to_pickup = path_to_pickup

            self.__goal_pub.publish(target_pose_msg)
            self.get_logger().info(f"Task {task_id} published to /target_pose")
            self.log_pub.publish(L.INFO, "I-213", self.robot_id)

        except KeyError:
            self.get_logger().error("The current Node doesn't exist in the map.")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
