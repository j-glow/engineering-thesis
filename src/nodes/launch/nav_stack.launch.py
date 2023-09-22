from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    navigator = Node(
        package="nodes",
        executable="carrot_follower",
    )
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
            navigator,
            goal_generator,
            detector,
        ]
    )
