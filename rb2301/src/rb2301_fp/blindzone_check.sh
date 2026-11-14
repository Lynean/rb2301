#!/bin/bash
# Measure the LiDAR blind zone (interactive). Run from the workspace root, like fp.sh.
# Needs the LiDAR running (/scan topic), e.g. the robot's bringup or Gazebo.
source install/setup.bash

# blindzone_check is a new executable: build the package once if it isn't installed yet
if ! ros2 pkg executables rb2301_fp 2>/dev/null | grep -q blindzone_check; then
    echo "blindzone_check not installed yet, building rb2301_fp..."
    colcon build --packages-select rb2301_fp || exit 1
    source install/setup.bash
fi

if ! ros2 topic list 2>/dev/null | grep -qx "/scan"; then
    echo "Warning: no /scan topic found. Start the LiDAR first (it will wait for scans)."
fi

ros2 run rb2301_fp blindzone_check "$@"
