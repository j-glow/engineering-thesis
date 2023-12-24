install_gazebo_res:
	@cd src/gazebo/plugins/actor_collisions ; mkdir -p build
	@cd src/gazebo/plugins/actor_collisions/build ; cmake ..
	@cd src/gazebo/plugins/actor_collisions/build ; make

control:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard -ros-args --remap cmd_vel:=key_vel

sim:
	ros2 launch jetbot simulation.launch.py world:=actor_small_walk.world

slam:
	ros2 launch slam_toolbox online_async_launch.py params_file:=src/jetbot/config/mapper_params_online_async.yaml use_sim_time:=true

clean:
	rm -rf build install log
	rm -rf src/gazebo/plugins/actor_collisions/build

	unset GAZEBO_MODEL_PATH
	unset GAZEBO_PLUGIN_PATH
