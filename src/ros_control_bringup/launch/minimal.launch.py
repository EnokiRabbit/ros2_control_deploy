from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("ros_control_description"))
    bringup_share = Path(get_package_share_directory("ros_control_bringup"))

    robot_description = (description_share / "urdf" / "minimal.urdf").read_text()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[str(bringup_share / "config" / "controllers.yaml")],
                output="screen",
            ),
        ]
    )
