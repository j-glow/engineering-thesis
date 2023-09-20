#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

GAZEBO_MODEL_PATH=$SCRIPT_DIR/models:$GAZEBO_MODEL_PATH
GAZEBO_PLUGIN_PATH=$SCRIPT_DIR/plugins/actor_collisions/build:$GAZEBO_PLUGIN_PATH

export GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH

echo Added $SCRIPT_DIR/models/ to Gazebo models search directories
echo Added $SCRIPT_DIR/plugins/actor_collisions/build/ to Gazebo plugin search directories