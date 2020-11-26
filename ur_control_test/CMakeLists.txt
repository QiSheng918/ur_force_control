cmake_minimum_required(VERSION 3.0.2)
project(ur_control_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roslib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(orocos_kdl REQUIRED)

find_package(yaml-cpp REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gravity_con
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_LIBRARIES}
  ${orocos_kdl_INCLUDE_DIRS}
)


# add_executable(test_param src/test_param.cpp)
# target_link_libraries(test_param
#   ${catkin_LIBRARIES}
# )




add_executable(kdl_intercept_test_node src/kdl_intercept_test.cpp)
target_link_libraries(kdl_intercept_test_node
  ${catkin_LIBRARIES}
  ${EIGEN3_CPP_LIBRARIES}
  ${orocos_kdl_CPP_LIBRARIES}
  
)

add_executable(intercept_test_node src/intercept_test.cpp)
target_link_libraries(intercept_test_node
  ${catkin_LIBRARIES}
  ${EIGEN3_CPP_LIBRARIES}
  ${orocos_kdl_CPP_LIBRARIES}
  
)




