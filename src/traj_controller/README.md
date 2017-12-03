# traj_controller
Node for the trajectory controller.
This package publishes `krssg_ssl_msgs/SSL_DetectionFrame.msg` on the topic `/grsim_data`.
This package subscribes vision data from the topic `/vision`.

### Dependencies
`krssg_ssl_msgs`
`grsim_comm`
`vision_comm`

### Runing
Open up four terminal windows.
**Terminal 1**
```
roscore
```
**Terminal 2**
```
rosrun vision_comm vision_node 1
```
**Terminal 3**
```
rosrun grsim_comm grsim_node
```
**Terminal 4**
```
rosrun traj_controller traj_node
```
***Note: Currently a hard-coded circular trajectory is implemented for testing purposes.***