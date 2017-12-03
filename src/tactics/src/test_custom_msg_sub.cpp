#include <stdio.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "test_msg.h"

void chatterCallback(const boost::shared_ptr<MyStruct const>& msg)
{
  ROS_INFO("I heard: [%d, %lf]", msg->a, msg->b);
}

int main(int argc, char  *argv[])
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}