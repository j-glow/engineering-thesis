import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("jetbot_ros")
    gazebo_params_file = os.path.join(package_dir, "config", "gazebo_params.yaml")
    twist_mux_params_file = os.path.join(package_dir, "config", "twist_mux.yaml")
    gazebo_dir = get_package_share_directory("gazebo")
    world = LaunchConfiguration("world")

    world_launch_arg = DeclareLaunchArgument(
        "world",
        default_value="",
        description="Specify world file name",
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_dir, "launch", "rsp.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Gazebo server launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gzserver.launch.py",
                )
            ]
        ),
        launch_arguments={
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
            "world": PathJoinSubstitution([gazebo_dir, world]),
        }.items(),
    )

    # Gazebo client launch
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gzclient.launch.py",
                )
            ]
        ),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    # twist_mux = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_params_file],
    #     output="screen"
    # )

    return LaunchDescription(
        [
            world_launch_arg,
            rsp,
            gzserver,
            gzclient,
            spawn_entity,
            # twist_mux,
        ]
    )
