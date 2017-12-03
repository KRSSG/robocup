#include <stdio.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "test_msg.h"
int main(int argc, char *argv[])
{
  using namespace ros::message_traits;
  /* code */
  printf("md5: \"%s\"\n", MD5Sum<geometry_msgs::Vector3>::value());
  geometry_msgs::Vector3 msg;
  msg.x = 1; msg.y = 0; msg.z = 0;
  printf("md5 of(1,0,0) = %s\n", MD5Sum<geometry_msgs::Vector3>::value(msg));

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<MyStruct>("chatter", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    MyStruct ms;

    ms.a =1; ms.b = 2;

    // ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(ms);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
