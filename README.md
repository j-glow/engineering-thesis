# Engineering thesis

ROS 2 based mobile robot navigation system for pedestrain following.

## Dependencies

In case of not being able to access ROS packages, follow official [ROS Install Guide](https://docs.ros.org/en/humble/Installation.html)

```bash
sudo apt install \
    python3-colcon-common-extensions \
    gazebo \
    ros-humble-desktop \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-twist-mux
python3 -m pip install \
    cmake \
    ultralytics \
    pyquaternion
```

## Folder structure

* `src/gazebo`:
  * resource files and worlds for gazebo simulation.
* `src/jetbot`:
  * .xacro files containing robot specification,
  * launch files for vaious scenarios (nav launch, only robot tranforms, simulation etc.),
  * configs for RViz2 visualization and nodes' parameters for launches.
* `src/nodes`:
  * ROS2 nodes for inference, navigation, sensor infomartion analysis etc,

## Makefile shortcuts

* `make install_gazebo_res`       - build and install gazebo plugins and resources
* `make control`                  - open hand-steering node for robot simulation
* `make clean`                    - clean all built files

## Build

While in main repository folder.
Build and install enviroment variables:

```bash
colcon build
source install/setup.bash
```

## Launches

Robot state publisher:

```bash
ros2 launch jetbot rsp.launch.py
```

Simulation launch:

```bash
ros2 launch jetbot simulation.launch.py
```

Optionally append command with optional world argument using "world:={world_name}" syntax providing name of world from src/gazebo/worlds folder, eg.:

```bash
ros2 launch jetbot simulation.launch.py world:=custom.world
ros2 launch jetbot simulation.launch.py world:=simple_room.world
```

Simulation with navigation stack launch:

```bash
ros2 launch jetbot sim_with_nav.launch.py
```

## RViz2 configs

Launch RViz2 using predefined configs.

View robot model (needs [Robot state publisher](#launches)):

```bash
rviz2 -d src/jetbot/config/view_robot.rviz
```

View robot movement (needs [Simulation](#launches)):

```bash
rviz2 -d src/jetbot/config/robot_odom.rviz
```

Visualize slam (needs [Simulation](#launches)):

```bash
rviz2 -d src/jetbot/config/slam_viz.rviz
```

Visualize navigation stack (needs [Simulation with navigation stack](#launches)):

```bash
rviz2 -d src/jetbot/config/nav_viz.rviz
```

WIP commands:
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_footprint
ros2 launch slam_toolbox online_async_launch.py params_file:=src/jetbot/config/mapper_params_online_async.yaml use_sim_time:=true
ros2 launch nodes goal_update.launch.py 
```

## External sources used/inspired by

* <https://github.com/dusty-nv/jetbot_ros>
* <https://github.com/noshluk2/ROS2-Ultimate-learners-Repository>
* <https://github.com/joshnewans/articubot_one>
* <https://classic.gazebosim.org/tutorials>
* <https://docs.ros.org/en/humble/>
* <https://navigation.ros.org/>
* <https://github.com/JiangweiNEU/actor_collisions>

## Important posts
* https://answers.ros.org/question/302037/ros2-how-to-call-a-service-from-the-callback-function-of-a-subscriber/
* https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
