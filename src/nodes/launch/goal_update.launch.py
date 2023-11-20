from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detector = Node(
        package="nodes",
        executable="detector",
    )
    goal_generator = Node(
        package="nodes",
        executable="moving_target",
    )

    return LaunchDescription(
        [
            goal_generator,
            detector,
        ]
    )
