#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

GAZEBO_RESOURCE_PATH=$SCRIPT_DIR/worlds:$GAZEBO_RESOURCE_PATH
GAZEBO_MODEL_PATH=$SCRIPT_DIR/models:$GAZEBO_MODEL_PATH

echo Added $SCRIPT_DIR/worlds to Gazebo models search directories
echo Added $SCRIPT_DIR/models to Gazebo worlds search directories