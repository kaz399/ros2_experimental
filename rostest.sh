#!/usr/bin/env bash

colcon build
source install/setup.bash
ros2 launch example1 navigator_bringup.launch.py
