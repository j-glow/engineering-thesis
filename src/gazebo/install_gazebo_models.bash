#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

GAZEBO_MODEL_PATH=$SCRIPT_DIR/models:$GAZEBO_MODEL_PATH

echo Added $SCRIPT_DIR/models/ to Gazebo models search directories
