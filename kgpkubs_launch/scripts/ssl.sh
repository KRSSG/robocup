#!/bin/bash
# Start SSL functionality in screen sessions
# to kill all sessions: killall -15 screen
killall screen


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
function launcher {
    if screen -ls | grep -q $1; then
        echo "WARNING! $1 is already running!"
    else
        screen -S $1 -d -m bash
        screen -S $1 -p 0 -X stuff "source $DIR/../../../devel/setup.bash; $2$(printf \\r)"
    fi
}

function pr_launcher {
    launcher $1 "roslaunch kgpkubs_launch $2 --wait"
}

ps cax | grep project > /dev/null
if [ $? -eq 0 ]; then
    echo "grSim already running."
else
    echo "launching grSim."
    launcher "grSim" "rosrun grSim project"
fi

launcher    "core"         "roscore"
pr_launcher "vision"    "vision.launch"
pr_launcher "belief_state"      "belief_state.launch"
pr_launcher "grsim_comm"    "grsim_comm.launch"
pr_launcher "path_planner_ompl" "ompl_planner.launch"

screen -S "GUI" -dm python run_gui.py
screen -S "bs_memcache" -dm python bs.py

# python vision_comm/src/svg_filter.py & >/dev/null
# pr_launcher "play_node" "play.launch"
