cmake_minimum_required(VERSION 2.8.3)
project(vision_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include"
)

add_executable(belief_node src/node.cpp src/kalman.cpp)
target_link_libraries(belief_node ${catkin_LIBRARIES})

