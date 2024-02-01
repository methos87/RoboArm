from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(robot_name="robo_arm", package_name="robo_arm").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="robo_arm",
        executable="hello_moveit",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])