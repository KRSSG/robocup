# vision_comm
Node for getting vision data from ssl-vision/grSim
This package publishes krssg_ssl_msgs/SSL_DetectionFrame.msg on the topic '/vision'

======

**Dependencies:**
krssg_ssl_msgs

**Runing**
```
roscore
rosrun vision_comm vision_node 1
```
This runs the vision publisher that connects to a grSim instance's vision port, and publishes data on '/vision'
if instead you run as
```
rosrun vision_comm vision_node 0
```
it will connect to ssl-vision's port.





