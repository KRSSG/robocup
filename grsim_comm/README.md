# GRSIM - COMMUNICATION
ROS package to communicate with GrSim.
This package subscribes to /grsim_data and converts it into GRSIm Protobuf format and then sends it to GrSim to be able to be visualized.

master: 
[![Build Status](https://travis-ci.org/krssg-ssl/grsim_comm.svg?branch=master)](https://travis-ci.org/krssg-ssl/grsim_comm)

======

**Dependencies:** GrSim

1. Clone the repository [grSim](https://github.com/KRSSG/grSim) and follow the instructions in it's [Install](https://github.com/mani-monaj/grSim/blob/master/INSTALL.md) file.
2. Clone the repository [krssg_ssl_msgs](https://github.com/KRSSG/krssg_ssl_msgs) in the catkin workspace.
2. Clone this repository in your catkin workspace.
3. Run ```catkin_make``` in workspace root.
4. Run 
```
roscore
rosrun grsim_comm grsim_node
```
This runs the subscriber that subscribes to /grsim_data and executes them on grsim.


##Optional:
This subscriber can be tested using the following command which sends hard coded data to revolve robot ID 1 in a circle of fixed radius.
``` rosrun grsim_comm test_ssl ```




