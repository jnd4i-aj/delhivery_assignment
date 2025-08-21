import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from interfaces.msg import NavData, RobotStatus


# Dummy LogPublisher to intercept logs
class DummyLogPublisher:
    def __init__(self, node):
        self.calls = []

    def publish(self, level, code, bot_id=0):
        self.calls.append((level, code, bot_id))


@pytest.fixture
def setup_status_node(monkeypatch):
    rclpy.init()

    # Patch LogPublisher before importing StatusNode
    monkeypatch.setattr("central_logger.log_publisher.LogPublisher", DummyLogPublisher)

    # Import here so patched LogPublisher gets used
    from robot_navigator.status_node import StatusNode

    node = StatusNode()
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    dummy = node.log_pub  # already DummyLogPublisher

    yield node, exec, dummy

    exec.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_for(exec, timeout=1.0):
    start = time.time()
    while time.time() - start < timeout:
        exec.spin_once(timeout_sec=0.1)


def test_startup_log(setup_status_node):
    node, exec, dummy = setup_status_node
    # On init, should publish startup log "I-300"
    assert any(code == "I-300" for _, code, _ in dummy.calls)


def test_republishes_navdata_as_robotstatus(setup_status_node):
    node, exec, dummy = setup_status_node

    received = []
    sub = node.create_subscription(RobotStatus, "/robot_status", lambda m: received.append(m), 10)
    pub = node.create_publisher(NavData, "/nav_data", 10)

    msg = NavData()
    msg.bot_id = 7
    msg.status = "NAVIGATING"
    pub.publish(msg)

    spin_for(exec, timeout=1.0)

    assert received, "No RobotStatus received"
    assert received[0].bot_id == 7
    assert received[0].robot_status == "NAVIGATING"


def test_malformed_navdata_triggers_error(setup_status_node):
    node, exec, dummy = setup_status_node

    # Call the callback directly with a bad object
    try:
        node._StatusNode__nav_data_callback(None)
    except Exception:
        pass  # We allow exception here, focus is on log code

    # Should log E-301 or E-302
    assert any(code in ("E-301", "E-302") for _, code, _ in dummy.calls)
