# vision_comm
Node for getting vision data from ssl-vision/grSim
This package publishes krssg_ssl_msgs/BeliefState.msg on the topic '/vision'

## Dependencies:
krssg_ssl_msgs

## Running

```
roscore
rosrun vision_comm vision_node <vision_port> <team_id>
```

If using grSim, set `vision_port` to 0 else set `vision_port` to 1 for ssl-vision data
If our team is blue, set `team_id` to 0 else to 1.

eg:

If connected to ssl-vision port and our team is yellow,

```
roscore
rosrun vision_comm vision_node 1 1
```


### Note
- By default `vision_node` connects to grSim port and our team is Team Yellow

- To make the changes on what to launch, refer `kgpkubs_launch/launch/vision.launch` file