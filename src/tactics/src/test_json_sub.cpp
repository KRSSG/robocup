#include <stdio.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tactics/TacticData.h>

using tactics::TacticData;
void chatterCallback(const boost::shared_ptr<TacticData const>& msg)
{
  printf("I heard: [%d, %s]\n", msg->tID, msg->tParamJSON.c_str());
}
int main(int argc, char  *argv[])
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}