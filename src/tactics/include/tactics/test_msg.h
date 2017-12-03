#pragma once
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/ros.h>
union MyStruct {
  int a;
  float b;
};

// ROS_STATIC_ASSERT(sizeof(MyStruct) == 12);

namespace ros {
namespace message_traits {

template<>
struct MD5Sum<MyStruct>
{
  static const char* value()
  {
    // Ensure that if the definition of geometry_msgs/Vector3 changes we have a compile error here.
    // ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Vector3>::static_value1 == 0x4a842b65f413084dULL);
    // ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Vector3>::static_value2 == 0xc2b10fb484ea7f17ULL);
    // return MD5Sum<geometry_msgs::Vector3>::value();
    return "ba369a2b053469b429e377b570b47cff";
  }

  static const char* value(const MyStruct& m)
  {
    // return MD5Sum<geometry_msgs::Vector3>::value(m);
    return value();
  }
};

template<>
struct DataType<MyStruct>
{
  static const char* value()
  {
    return "tactics/TParam";
  }

  static const char* value(const MyStruct& m)
  {
    return value();
  }
};

template<>
struct Definition<MyStruct>
{
  static const char* value()
  {
    return "sdfdsf";
  }

  static const char* value(const MyStruct& m)
  {
    return value();
  }
};
}

const int _mystructlen = sizeof(MyStruct);
namespace serialization
{
template<>
struct Serializer<MyStruct>
{
  template<typename Stream>
  inline static void write(Stream& stream, const MyStruct& t)
  {
    memcpy(stream.advance(_mystructlen), &t, _mystructlen);
  }

  template<typename Stream>
  inline static void read(Stream& stream, MyStruct& t)
  {
    t = *(MyStruct*)stream.advance(_mystructlen);
  }

  inline static uint32_t serializedLength(const MyStruct& t)
  {
    return _mystructlen;
  }
};
} // namespace serialization
}