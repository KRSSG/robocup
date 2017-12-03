#include <stdio.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tactics/TacticData.h>

using namespace rapidjson; 
int main(int argc, char *argv[])
{
  using tactics::TacticData;
  // 1. Parse a JSON string into DOM.
  const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
  Document d;
  d.Parse(json);

  // 2. Modify it by DOM.
  Value& s = d["stars"];
  s.SetInt(s.GetInt() + 1);

  // 3. Stringify the DOM
  StringBuffer buffer;
  Writer<StringBuffer> writer(buffer);
  d.Accept(writer);

  // Output {"project":"rapidjson","stars":11}
  std::cout << buffer.GetString() << std::endl;

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<tactics::TacticData>("chatter", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    TacticData t;
    t.tID = 1;
    t.tParamJSON = buffer.GetString();

    // ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(t);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
    return 0;
  return 0;
}