import time
import threading
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# IMPORT: change this if your NavigationNode module path differs
from robot_navigator.navigation_node import NavigationNode

from interfaces.msg import NavData, TaskQueue, TargetPose


def spin_wait(executor, nodes, cond_fn, timeout=10.0, step=0.05):
    """Spin executor until cond_fn() True or timeout."""
    end = time.time() + timeout
    while time.time() < end:
        executor.spin_once(timeout_sec=step)
        if cond_fn():
            return True
    return False


@pytest.fixture(autouse=True)
def rclpy_init_shutdown():
    """Initialize ROS for each test and ensure shutdown afterwards."""
    rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except Exception:
        # defensive: if rclpy already shutdown, ignore
        pass


def make_nav_and_helper(node_params=None):
    nav = NavigationNode()
    # set faster params to speed tests if provided
    if node_params:
        params = [Parameter(k, value=v) for k, v in node_params.items()]
        nav.set_parameters(params)
    helper = Node("test_helper_" + str(int(time.time() * 1000) % 100000))
    executor = MultiThreadedExecutor()
    executor.add_node(nav)
    executor.add_node(helper)
    return nav, helper, executor


def teardown_nodes(executor, helper, nav):
    try:
        executor.shutdown()
    except Exception:
        pass
    try:
        helper.destroy_node()
    except Exception:
        pass
    try:
        nav.destroy_node()
    except Exception:
        pass


def test_navigation_ignores_pose_while_executing():
    # make nav run faster to make test quicker
    nav, helper, executor = make_nav_and_helper({
        "step_duration": 0.01,
        "pickup_dwell": 0.02,
        "status_publish_period": 0.05
    })

    # subscribe to nav_data just to have the executor handle messages
    statuses = []
    helper.create_subscription(NavData, "/nav_data", lambda m: statuses.append(str(m.status)), 10)

    pub_task = helper.create_publisher(TaskQueue, "/task_queue", 10)
    pub_pose = helper.create_publisher(TargetPose, "/target_pose", 10)

    # first task: longer path to ensure executing remains True for a moment
    t1 = TaskQueue(); t1.task_id = "P1D4"; t1.pickup_node = 1; t1.drop_node = 4
    tp1 = TargetPose()
    tp1.path_to_pickup = [1]
    tp1.path_to_drop = [1, 2, 3, 4]
    tp1.pickup_left_nodes = []; tp1.pickup_right_nodes = []
    tp1.drop_left_nodes = []; tp1.drop_right_nodes = []

    pub_task.publish(t1)
    assert spin_wait(executor, [nav, helper], lambda: getattr(nav, "task_id", "") == t1.task_id, timeout=2.0)

    pub_pose.publish(tp1)
    assert spin_wait(executor, [nav, helper], lambda: nav.executing, timeout=3.0)

    # publish a new pose while executing — this should be ignored by nav
    t2 = TargetPose()
    t2.path_to_pickup = [2]
    t2.path_to_drop = [2, 5]
    t2.pickup_left_nodes = []; t2.pickup_right_nodes = []
    t2.drop_left_nodes = []; t2.drop_right_nodes = []
    pub_pose.publish(t2)

    # give a short time for callbacks, nav should remain executing the first task
    assert nav.executing

    teardown_nodes(executor, helper, nav)


def test_navigation_handles_malformed_targetpose():
    nav, helper, executor = make_nav_and_helper({
        "step_duration": 0.01,
        "pickup_dwell": 0.02,
        "status_publish_period": 0.05
    })

    msgs = []
    helper.create_subscription(NavData, "/nav_data", lambda m: msgs.append(m), 10)
    pub_task = helper.create_publisher(TaskQueue, "/task_queue", 10)
    pub_pose = helper.create_publisher(TargetPose, "/target_pose", 10)

    t = TaskQueue(); t.task_id = "P2D3"; t.pickup_node = 2; t.drop_node = 3
    pub_task.publish(t)
    assert spin_wait(executor, [nav, helper], lambda: getattr(nav, "task_id", "") == t.task_id, timeout=2.0)

    # Use type-correct but logically invalid / empty arrays to trigger node error handling
    tp = TargetPose()
    tp.path_to_pickup = []         # empty -> node should abort execution gracefully
    tp.path_to_drop = []
    tp.pickup_left_nodes = []
    tp.pickup_right_nodes = []
    tp.drop_left_nodes = []
    tp.drop_right_nodes = []

    pub_pose.publish(tp)

    # node should not crash and still publish periodic statuses (idle or error)
    ok = spin_wait(executor, [nav, helper], lambda: len(msgs) > 0, timeout=2.0)
    assert ok

    teardown_nodes(executor, helper, nav)


def test_status_periodic_publish():
    nav, helper, executor = make_nav_and_helper({
        "status_publish_period": 0.2  # faster than default
    })
    received = []
    helper.create_subscription(NavData, "/nav_data", lambda m: received.append((m.status, m.current_node)), 10)

    # allow up to 5s, should easily get 3+
    ok = spin_wait(executor, [nav, helper], lambda: len(received) >= 3, timeout=5.0)
    assert ok, f"Expected periodic publishes, got {len(received)}"

    teardown_nodes(executor, helper, nav)



def test_status_transitions_pickup_to_drop():
    nav, helper, executor = make_nav_and_helper({
        "step_duration": 0.01,
        "pickup_dwell": 0.02,
        "status_publish_period": 0.05
    })

    received = []
    helper.create_subscription(NavData, "/nav_data", lambda m: received.append(str(m.status)), 10)

    pub_task = helper.create_publisher(TaskQueue, "/task_queue", 10)
    pub_pose = helper.create_publisher(TargetPose, "/target_pose", 10)

    t = TaskQueue(); t.task_id = "P2D3"; t.pickup_node = 2; t.drop_node = 3
    tp = TargetPose()
    tp.path_to_pickup = [1, 2]
    tp.path_to_drop = [2, 3]
    tp.pickup_left_nodes = []; tp.pickup_right_nodes = []
    tp.drop_left_nodes = []; tp.drop_right_nodes = []

    pub_task.publish(t)
    assert spin_wait(executor, [nav, helper], lambda: getattr(nav, "task_id", "") == t.task_id, timeout=2.0)
    pub_pose.publish(tp)

    ok = spin_wait(executor, [nav, helper], lambda: any(s in ("AT_PICKUP", "AT_DROPOFF", "COMPLETED") for s in received), timeout=8.0)
    assert ok, f"Never saw pickup/drop/complete statuses; statuses: {received}"

    teardown_nodes(executor, helper, nav)


def test_navigation_reaches_completed():
    nav, helper, executor = make_nav_and_helper({
        "step_duration": 0.01,
        "pickup_dwell": 0.02,
        "status_publish_period": 0.05
    })

    received = []
    lock = threading.Lock()
    helper.create_subscription(NavData, "/nav_data", lambda m: (received.append((str(m.status), int(getattr(m, "current_node", -1))))) , 10)

    pub_task = helper.create_publisher(TaskQueue, "/task_queue", 10)
    pub_pose = helper.create_publisher(TargetPose, "/target_pose", 10)

    t = TaskQueue(); t.task_id = "P2D3"; t.pickup_node = 2; t.drop_node = 3
    tp = TargetPose()
    tp.path_to_pickup = [1, 2]
    tp.path_to_drop = [2, 3]
    tp.pickup_left_nodes = []; tp.pickup_right_nodes = []
    tp.drop_left_nodes = []; tp.drop_right_nodes = []

    pub_task.publish(t)
    assert spin_wait(executor, [nav, helper], lambda: getattr(nav, "task_id", "") == t.task_id, timeout=2.0)
    pub_pose.publish(tp)

    def cond():
        return any(s == "COMPLETED" for s, _ in received)

    ok = spin_wait(executor, [nav, helper], cond, timeout=12.0)

    teardown_nodes(executor, helper, nav)
    assert ok, f"NavigationNode did not publish COMPLETED within timeout, statuses: {received}"
