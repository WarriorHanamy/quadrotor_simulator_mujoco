#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

ros2 run quadrotor_sim_ros_adapter quadrotor_sim_ros_adapter "$@"
