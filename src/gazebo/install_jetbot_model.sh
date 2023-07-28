#!/bin/bash

# get the directory of the script
SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo "Adding models and worlds to Gazebo path from:   " $SOURCE_DIR
export IGN_GAZEBO_RESOURCE_PATH=$SOURCE_DIR/models:$SOURCE_DIR/worlds:$IGN_GAZEBO_RESOURCE_PATH
