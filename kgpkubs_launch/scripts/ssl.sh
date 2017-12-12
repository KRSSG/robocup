#!/bin/bash
# Start SSL functionality in screen sessions

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/functions.sh

launcher    "core"         "roscore"

start_ros_node "grSim" "grSim" "project"
start_ros_node "grsim_debug" "grsim_comm" "debug_test"

start_ros_launch_file "vision"    "vision.launch"
start_ros_launch_file "belief_state"      "belief_state.launch"
start_ros_launch_file "grsim_comm"    "grsim_comm.launch"
start_ros_launch_file "robot"         "robot.launch"
start_ros_launch_file "bot_comm"         "bot_comm.launch"
