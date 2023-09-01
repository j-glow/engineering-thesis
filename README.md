# Engineering thesis
ROS 2 based mobile robot navigation system for pedestrain following.

##  Dependencies 
```bash
sudo apt install \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    gazebo \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-twist-mux
```

## Installl Gazebo custom models

```bash
source src/gazebo/install_gazebo_models.bash
```

# Build
While in main repository folder.
Build and install enviroment variables:
```bash
colcon build
source install/setup.bash
```

Installl Gazebo custom models:
```bash
source src/gazebo/install_gazebo_models.bash
```

# Launches

Robot state publisher:
```bash
ros2 launch jetbot_ros rsp.launch.py
```

Simulation launch:
```bash
ros2 launch jetbot_ros simulation.launch.py
```
Optionally append command with optional world argument using "world:={world_name}" syntax providing name of world from src/gazebo/worlds folder, eg.:
```bash
ros2 launch jetbot_ros simulation.launch.py world:=custom.world
ros2 launch jetbot_ros simulation.launch.py world:=simple_room.world
```

Simulation with navigation stack launch:
```bash
ros2 launch jetbot_ros sim_with_nav.launch.py
```

# Hand-steering the robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard -ros-args --remap cmd_vel:=key_vel
```

# RViz2 configs
Launch RViz2 using predefined configs.

View robot model (needs [Robot state publisher](#launches)):
```bash
rviz2 -d src/jetbot_ros/view_robot.rviz
```

View robot movement (needs [Simulation](#launches)):
```bash
rviz2 -d src/jetbot_ros/robot_odom.rviz
```

Visualize slam (needs [Simulation](#launches)):
```bash
rviz2 -d src/jetbot_ros/slam_viz.rviz
```

Visualize navigation stack (needs [Simulation with navigation stack](#launches)):
```bash
rviz2 -d src/jetbot_ros/nav_viz.rviz
```

# WIP

Teleop_twist_keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=key_vel
```
Simulation:
```bash
ros2 launch jetbot_ros simulation.launch.py world:=simple_room.world
```
Slam_toolbox:
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/jetbot_ros/config/mapper_params_online_async.yaml use_sim_time:=true
```
Nav2:
```bash
ros2 launch nav2_bringup navigation.launch.py use_sim_time:=true
```

# External sources used/inspired by:
 - https://github.com/dusty-nv/jetbot_ros
 - https://github.com/noshluk2/ROS2-Ultimate-learners-Repository
 - https://github.com/joshnewans/articubot_one
 - https://classic.gazebosim.org/tutorials
 - https://docs.ros.org/en/humble/
 - https://navigation.ros.org/