from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import join, expanduser


def generate_launch_description():

    nav_config = join(
        expanduser(
            "~"), "ros2_ws", "src", "delhivery_assignment", "config", "robot_config_params.yaml"
    )

    ld = LaunchDescription()

    task_manager_node = Node(
        package="task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen"
    )

    nav_node = Node(
        package="robot_navigator",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[nav_config]
    )

    status_node = Node(
        package="robot_navigator",
        executable="status_node",
        name="status_node",
        output="screen"
    )

    central_logger_node = Node(
        package="central_logger",
        executable="central_logging_node",
        name="central_logging_node",
        output="screen"
    )

    ld.add_action(task_manager_node)
    ld.add_action(nav_node)
    ld.add_action(status_node)
    ld.add_action(central_logger_node)

    return ld
