#!/usr/bin/env bash
set -euo pipefail
# Ensure ROS 2 env is sourced before running this script
# Example: source /opt/ros/jazzy/setup.bash && source ~/wheebot_ws/install/setup.bash

python3 backend/run_backend.py