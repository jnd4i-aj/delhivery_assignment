import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import pytest

from interfaces.msg import NavData, TaskQueue, TargetPose
from interfaces.srv import AllocateTask

from task_manager.task_manager_node import TaskManagerNode


def spin_wait(executor, cond_fn, timeout=5.0, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        executor.spin_once(timeout_sec=step)
        if cond_fn():
            return True
    return False


def make_manager_and_helper():
    rclpy.init()
    manager = TaskManagerNode()
    helper = Node("test_helper_mgr")
    exec = MultiThreadedExecutor()
    exec.add_node(manager)
    exec.add_node(helper)
    return manager, helper, exec


def cleanup(manager, helper, exec):
    exec.shutdown()
    helper.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()


# -------------------- TESTS --------------------

def test_allocate_task_idle():
    """When robot idle, AllocateTask assigns immediately and publishes TaskQueue + TargetPose."""
    manager, helper, exec = make_manager_and_helper()

    received_task = []
    received_pose = []

    helper.create_subscription(TaskQueue, "/task_queue", lambda m: received_task.append(m), 10)
    helper.create_subscription(TargetPose, "/target_pose", lambda m: received_pose.append(m), 10)

    # Fake robot idle
    nav_msg = NavData()
    nav_msg.bot_id = 1
    nav_msg.current_node = 1
    nav_msg.status = "idle"
    manager._TaskManagerNode__robot_status_callback(nav_msg)

    cli = helper.create_client(AllocateTask, "/start_delivery")
    assert cli.wait_for_service(timeout_sec=2.0)

    req = AllocateTask.Request()
    req.task_id = "P1D2"

    fut = cli.call_async(req)
    spin_wait(exec, lambda: fut.done(), timeout=3.0)

    resp = fut.result()
    assert resp.success
    assert "assigned immediately" in resp.message

    ok_task = spin_wait(exec, lambda: len(received_task) > 0, timeout=2.0)
    ok_pose = spin_wait(exec, lambda: len(received_pose) > 0, timeout=2.0)

    cleanup(manager, helper, exec)

    assert ok_task, "TaskQueue not published"
    assert ok_pose, "TargetPose not published"


def test_allocate_task_busy_queue_then_idle():
    """When robot busy, AllocateTask enqueues task; once idle, it gets assigned."""
    manager, helper, exec = make_manager_and_helper()

    received_task = []

    helper.create_subscription(TaskQueue, "/task_queue", lambda m: received_task.append(m), 10)

    # Fake robot busy
    nav_msg = NavData()
    nav_msg.bot_id = 1
    nav_msg.current_node = 5
    nav_msg.status = "NAVIGATING_TO_PICKUP"
    manager._TaskManagerNode__robot_status_callback(nav_msg)

    cli = helper.create_client(AllocateTask, "/start_delivery")
    assert cli.wait_for_service(timeout_sec=2.0)

    req = AllocateTask.Request()
    req.task_id = "P2D3"

    fut = cli.call_async(req)
    spin_wait(exec, lambda: fut.done(), timeout=3.0)
    resp = fut.result()

    assert resp.success
    assert "queued" in resp.message

    # Now simulate robot idle
    nav_idle = NavData()
    nav_idle.bot_id = 1
    nav_idle.current_node = 5
    nav_idle.status = "idle"
    manager._TaskManagerNode__robot_status_callback(nav_idle)

    ok_task = spin_wait(exec, lambda: len(received_task) > 0, timeout=2.0)

    cleanup(manager, helper, exec)

    assert ok_task, "Task was not published after robot became idle"


def test_allocate_task_invalid_node():
    """If robot current_node=0, allocation fails."""
    manager, helper, exec = make_manager_and_helper()

    # Fake robot status
    nav_msg = NavData()
    nav_msg.bot_id = 1
    nav_msg.current_node = 0
    nav_msg.status = "idle"
    manager._TaskManagerNode__robot_status_callback(nav_msg)

    cli = helper.create_client(AllocateTask, "/start_delivery")
    assert cli.wait_for_service(timeout_sec=2.0)

    req = AllocateTask.Request()
    req.task_id = "P1D2"

    fut = cli.call_async(req)
    spin_wait(exec, lambda: fut.done(), timeout=3.0)
    resp = fut.result()

    cleanup(manager, helper, exec)

    assert not resp.success
    assert "Invalid Current Node" in resp.message


def test_allocate_task_invalid_format():
    """If task_id format invalid, no TaskQueue published."""
    manager, helper, exec = make_manager_and_helper()

    received_task = []
    helper.create_subscription(TaskQueue, "/task_queue", lambda m: received_task.append(m), 10)

    # Fake robot idle
    nav_msg = NavData()
    nav_msg.bot_id = 1
    nav_msg.current_node = 1
    nav_msg.status = "idle"
    manager._TaskManagerNode__robot_status_callback(nav_msg)

    cli = helper.create_client(AllocateTask, "/start_delivery")
    assert cli.wait_for_service(timeout_sec=2.0)

    req = AllocateTask.Request()
    req.task_id = "INVALID"

    fut = cli.call_async(req)
    spin_wait(exec, lambda: fut.done(), timeout=3.0)

    cleanup(manager, helper, exec)

    assert fut.result().success
    assert len(received_task) == 0, "TaskQueue should not be published for invalid task_id"
