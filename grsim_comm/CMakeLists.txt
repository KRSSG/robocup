cmake_minimum_required(VERSION 2.8.3)
project(grsim_comm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  krssg_ssl_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
##add_message_files(
##   FILES
##   gr_Commands.msg
##   gr_Packet.msg
##   gr_Replacement.msg
##   gr_RobotReplacement.msg
##   gr_Robot_Command.msg
##   gr_BallReplacement.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 # generate_messages(
 #   DEPENDENCIES
 #   std_msgs
 # )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ssl_communication
  CATKIN_DEPENDS roscpp rospy std_msgs krssg_ssl_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ssl_communication
#   src/${PROJECT_NAME}/ssl_communication.cpp
# )

## Declare a cpp executable
# add_executable(ssl_communication_node src/ssl_communication_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ssl_communication_node ssl_communication_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ssl_communication_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ssl_communication ssl_communication_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ssl_communication.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(libs)
set(QT_USE_QTNETWORK TRUE)
find_package(Qt REQUIRED)
include(${QT_USE_FILE})
list(APPEND libs ${QT_LIBRARIES})

#target_link_libraries(listener_ssl ${QT_LIBRARIES})

################################
find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_CPP PROTO_H
    include/proto/messages_robocup_ssl_detection.proto
    include/proto/messages_robocup_ssl_geometry.proto
    include/proto/messages_robocup_ssl_wrapper.proto
    include/proto/messages_robocup_ssl_refbox_log.proto
    include/proto/grSim_Replacement.proto
    include/proto/grSim_Commands.proto
    include/proto/grSim_Packet.proto
    include/proto/sslDebug_Data.proto
)

#Including proto files
include_directories(include ${catkin_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_BINARY_DIR}/${PROJECT_NAME})

#To check available protobuf library__message prints message on the screen
#message("${PROTOBUF_LIBRARIES} <><><><><><><")
#message("${PROTOBUF_PROTOC_EXECUTABLE} <><><><")

add_executable(test_ssl src/test_ssl.cpp ${PROTO_CPP} ${PROTO_H})
target_link_libraries(test_ssl ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(test_ssl grsim_comm_generate_messages_cpp)


add_executable(grsim_xyw_node src/grsim_xyw_node.cpp ${PROTO_CPP} ${PROTO_H})
target_link_libraries(grsim_xyw_node ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(grsim_xyw_node grsim_comm_generate_messages_cpp)


add_executable(grsim_wheelspeed_node src/grsim_wheelspeed_node.cpp ${PROTO_CPP} ${PROTO_H})
target_link_libraries(grsim_wheelspeed_node ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(grsim_wheelspeed_node grsim_comm_generate_messages_cpp)

add_executable(dt src/dt.cpp ${PROTO_CPP} ${PROTO_H})
target_link_libraries(dt ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(dt grsim_comm_generate_messages_cpp)


add_executable(debug_test src/debug_test.cpp ${PROTO_CPP} ${PROTO_H})
target_link_libraries(debug_test ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(debug_test grsim_comm_generate_messages_cpp)
###############################	For client server services #########################################

#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server beginner_tutorials_generate_messages_cpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client beginner_tutorials_generate_messages_cpp)



#####################################################################################################

