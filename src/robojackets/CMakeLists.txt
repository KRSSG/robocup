cmake_minimum_required(VERSION 3.0.0)

# Qt
set(CMAKE_AUTOMOC ON)

project(robojackets)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/robojackets/cmake")

message("cmake find dir:::;;;;;;::" ${CMAKE_MODULE_PATH})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
# find_package(Eigen3 REQUIRED COMPONENTS system)
# find_package(SDL REQUIRED COMPONENTS system)
find_package(Qt5Widgets REQUIRED COMPONENTS system)

execute_process (
    COMMAND bash -c "lsb_release -sr"
    OUTPUT_VARIABLE UBUNTU_VERSION
)
if ("${UBUNTU_VERSION}" EQUAL "16.04")
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
else("${UBUNTU_VERSION}" EQUAL "14.04")
  set(CMAKE_CXX_COMPILER /usr/bin/clang)
endif("${UBUNTU_VERSION}" EQUAL "14.04")

set(CMAKE_C_COMPILER /usr/bin/gcc)

message("compiler = " "${CMAKE_CXX_COMPILER}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
 INCLUDE_DIRS include /usr/include/eigen3
 LIBRARIES robojackets
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS system_lib Boost Eigen3 QT5Widgets
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
# add_library(robojackets
#   src/${PROJECT_NAME}/robojackets.cpp
# )

## Declare a cpp executable
# add_executable(robojackets_node src/robojackets_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robojackets_node robojackets_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(robojackets_node
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
# install(TARGETS robojackets robojackets_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robojackets.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

set(geometrySrc
  src/Geometry2d/Arc.cpp
  src/Geometry2d/Circle.cpp
  src/Geometry2d/CompositeShape.cpp
  src/Geometry2d/Line.cpp
  src/Geometry2d/Polygon.cpp
  src/Geometry2d/Rect.cpp
  src/Geometry2d/Segment.cpp
  src/Geometry2d/TransformMatrix.cpp
  )

set(planningSrc
  src/planning/InterpolatedPath.cpp
  src/planning/Path.cpp
  src/planning/RRTPlanner.cpp
  src/planning/SingleRobotPathPlanner.cpp
  src/planning/SingleRobotBezierPathPlanner.cpp
  src/planning/Tree.cpp
  src/planning/Util.cpp
  src/planning/RotationConstraints.cpp
  src/planning/MotionConstraints.cpp
  src/planning/dijkstra.cpp
  src/planning/visibility.cpp
  src/planning/recursivePlanner.cpp
  )

set(src
  include/robojackets/Configuration.cpp
  ${geometrySrc}
  ${planningSrc}
  src/Field_Dimensions.cpp
  # src/MotionControl.cpp
  src/Pid.cpp
  src/TrapezoidalMotion.cpp
  src/Utils.cpp
  src/MotionControl.cpp
  src/RobotConfig.cpp
  )

include_directories(.
                    include
                    include/robojackets
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    /usr/include/eigen3
                    )

set(${CMAKE_INCLUDE_CURRENT_DIR} ON)

# include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
# include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(libs
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  # ${SDL_LIBRARY}
  )

add_library(robojackets ${src})
target_link_libraries(robojackets ${libs})
qt5_use_modules(robojackets Core Network Widgets Xml OpenGL Svg)


# for installilng the library so it can be used by other packages
install(TARGETS robojackets
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# for installing the header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# for config
# install(DIRECTORY config/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
add_executable(tester src/test.cpp)
target_link_libraries(tester robojackets)

file(COPY "config/" DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
# message("bin desitination: " ${CATKIN_PACKAGE_BIN_DESTINATION})
