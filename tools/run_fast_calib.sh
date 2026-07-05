#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source "$HOME/fastlivo2_platform/tools/FAST-Calib-ROS2/install/setup.bash"

export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

exec ros2 launch fast_calib calib.launch.py
