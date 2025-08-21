import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import pytest

from interfaces.msg import AllLogs
from central_logger.central_logging_node import CentralLogger


class DummyServiceLogger:
    """Mock ServiceLogger that records calls instead of writing to disk."""
    def __init__(self):
        self.calls = []

    def central_data_debug(self, node_name, line):
        self.calls.append(("debug", node_name, line))

    def central_data_info(self, node_name, line):
        self.calls.append(("info", node_name, line))

    def central_data_warn(self, node_name, line):
        self.calls.append(("warn", node_name, line))

    def central_data_error(self, node_name, line):
        self.calls.append(("error", node_name, line))

    def central_data_fatal(self, node_name, line):
        self.calls.append(("fatal", node_name, line))


def spin_for(executor, timeout=1.0, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        executor.spin_once(timeout_sec=step)


def make_logger_and_helper():
    rclpy.init()
    logger = CentralLogger()
    helper = Node("test_helper_logger")
    exec = MultiThreadedExecutor()
    exec.add_node(logger)
    exec.add_node(helper)

    # Patch in dummy logger
    dummy = DummyServiceLogger()
    logger.slogger = dummy
    return logger, helper, exec, dummy


def cleanup(logger, helper, exec):
    exec.shutdown()
    helper.destroy_node()
    logger.destroy_node()
    rclpy.shutdown()


# ---------------- TESTS ------------------

def wait_for_writer(dummy, timeout=2.0):
    """Wait until ServiceLogger dummy has at least one call or timeout."""
    end = time.time() + timeout
    while time.time() < end:
        if dummy.calls:
            return True
        time.sleep(0.05)
    return False


def test_log_message_info():
    """CentralLogger receives /log msg and routes it to ServiceLogger.info."""
    logger, helper, exec, dummy = make_logger_and_helper()

    pub = helper.create_publisher(AllLogs, "/log", 10)
    msg = AllLogs()
    msg.node_name = "test_node"
    msg.bot_id = 1
    msg.log_code = "I-100"
    msg.log_level = 1  # INFO
    msg.stamp.sec = 123
    msg.stamp.nanosec = 456

    pub.publish(msg)
    spin_for(exec, timeout=0.5)
    assert wait_for_writer(dummy), "Writer thread did not flush in time"

    cleanup(logger, helper, exec)

    assert any(c[0] == "info" for c in dummy.calls)


def test_log_message_error():
    """CentralLogger routes ERROR log to ServiceLogger.error."""
    logger, helper, exec, dummy = make_logger_and_helper()

    pub = helper.create_publisher(AllLogs, "/log", 10)
    msg = AllLogs()
    msg.node_name = "test_node"
    msg.bot_id = 42
    msg.log_code = "E-201"
    msg.log_level = 3  # ERROR
    msg.stamp.sec = 5
    msg.stamp.nanosec = 99

    pub.publish(msg)
    spin_for(exec, timeout=0.5)
    assert wait_for_writer(dummy), "Writer thread did not flush in time"

    cleanup(logger, helper, exec)

    assert any(c[0] == "error" for c in dummy.calls)



def test_queue_overflow_drops_oldest():
    """When queue full, CentralLogger drops oldest and accepts new message."""
    logger, helper, exec, dummy = make_logger_and_helper()
    logger._q = logger._q.__class__(maxsize=2)  # shrink for test

    pub = helper.create_publisher(AllLogs, "/log", 10)

    for i in range(5):  # publish > capacity
        msg = AllLogs()
        msg.node_name = "node"
        msg.bot_id = i
        msg.log_code = f"C{i}"
        msg.log_level = 1
        msg.stamp.sec = i
        msg.stamp.nanosec = 0
        pub.publish(msg)
        spin_for(exec, timeout=0.2)

    cleanup(logger, helper, exec)

    # Should have logged only last 2, not all 5
    seen_codes = [c[2] for c in dummy.calls]
    assert any("C3" in s for s in seen_codes)
    assert any("C4" in s for s in seen_codes)


def test_destroy_node_stops_writer():
    """destroy_node should cleanly stop writer thread."""
    logger, helper, exec, dummy = make_logger_and_helper()
    assert logger._writer.is_alive()

    cleanup(logger, helper, exec)

    assert not logger._writer.is_alive()
