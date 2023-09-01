import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ## Packages
    jetbot_ros = get_package_share_directory("jetbot_ros")
    gazebo_util = get_package_share_directory("gazebo_util")

    ## Launch time argument variables
    sim_world = LaunchConfiguration("sim_world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params")

    ## Declare launch time arguments
    declare_sim_world_cmd = DeclareLaunchArgument(
        "sim_world",
        default_value="simple_room.world",
        description="Specify world file name",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_slam_params_cmd = DeclareLaunchArgument(
        "slam_params",
        default_value="mapper_params_online_async.yaml",
        description="Choose custom slam_toolbox params file",
    )

    ## Launches
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    jetbot_ros,
                    "launch",
                    "simulation.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": sim_world,
        }.items(),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    jetbot_ros,
                    "launch",
                    "nav.launch.py",
                )
            ]
        ),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        ),
        launch_arguments={
            "params_file": PathJoinSubstitution([jetbot_ros, "config", slam_params]),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ## Declare launch descriptor and add all actions
    ld = LaunchDescription()

    ld.add_action(declare_sim_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_cmd)

    ld.add_action(simulation)
    ld.add_action(slam)
    ld.add_action(nav)  ## Nav has to be last, otherway the launch fails for unknown reasons

    return ld
