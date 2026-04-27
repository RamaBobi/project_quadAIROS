#!/bin/bash

# Define the Project Root
PROJ_ROOT="$HOME/project_quadAIROS"


# ROS2
if [ -f "$PROJ_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$PROJ_ROOT/ros2_ws/install/setup.bash"
    echo "ROS 2 Workspace Sourced."
else
    echo "Warning: Run 'colcon build' in ros2_ws first."
fi

# ArduPilot
export PATH="$PATH:$PROJ_ROOT/external/ardupilot/Tools/autotest"
if [ -f "$PROJ_ROOT/external/ardupilot/Tools/completion/completion.bash" ]; then
    source "$PROJ_ROOT/external/ardupilot/Tools/completion/completion.bash"
fi

# ArduPilot-Gazebo Plugin
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PROJ_ROOT/external/ardupilot_gazebo/models:$PROJ_ROOT/external/ardupilot_gazebo/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_SIM_SYSTEM_PLUGIN_PATH:$PROJ_ROOT/external/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PROJ_ROOT/external/ardupilot_gazebo/models"


# MAVProxy
export PATH=$PATH:$HOME/.local/bin
export DISPLAY=:0

echo "--- project Environment Active ---"
