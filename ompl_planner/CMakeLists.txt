# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(ompl_planner)

ADD_DEFINITIONS(
  -std=c++11
  )

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ssl_common
  ssl_robot
  krssg_ssl_msgs
  tactics
  skills
  robojackets
  genmsg
)
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(krssg_ssl_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

include_directories(include
                    include/listener
                    ${catkin_INCLUDE_DIRS}
                    )

#set(src 
 #   src/listener/listener.cpp
  #  src/listener/MotionPlanner.cpp
   # )

#add_library(listener ${src})

#install(TARGETS listener
 #       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   #     RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener_ompl src/listener.cpp src/MotionPlanner.cpp)
#add_executable(listener_ompl src/MotionPlanner.cpp)

target_link_libraries(listener_ompl ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(listener_ompl listener_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# %EndTag(FULLTEXT)%
