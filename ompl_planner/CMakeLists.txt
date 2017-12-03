# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(ompl_planner)

ADD_DEFINITIONS(
  -std=c++11
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  krssg_ssl_msgs
)
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${krssg_ssl_msgs_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

include_directories(include
                    include/listener
                    ${catkin_INCLUDE_DIRS}
                    )

add_executable(listener_ompl src/listener.cpp src/MotionPlanner.cpp)
target_link_libraries(listener_ompl ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(listener_ompl listener_generate_messages_cpp)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
