cmake_minimum_required(VERSION 3.0.2)
project(force_control)


add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Eigen3 REQUIRED)
find_package(Eigen3 REQUIRED)


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
#   geometry_msgs#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES force_control
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf
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
)

add_executable(hybrid_postion_force_node src/hybrid_postion_force.cpp)
target_link_libraries(hybrid_postion_force_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(fixed_point_admittance_control_node src/fixed_point_admittance_control.cpp)
target_link_libraries(fixed_point_admittance_control_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(movement_admittance_control_node src/movement_admittance_control.cpp)
target_link_libraries(movement_admittance_control_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(variable_admittance_control_node src/variable_admittance_control.cpp)
target_link_libraries(variable_admittance_control_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(variable_admittance_control_speedl_node src/variable_admittance_control_speedl.cpp)
target_link_libraries(variable_admittance_control_speedl_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)


add_executable(admittance_control_node src/admittance_control.cpp)
target_link_libraries(admittance_control_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(admittance_control_mass_node src/admittance_control_mass.cpp)
target_link_libraries(admittance_control_mass_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)


add_executable(oritentation_admittance_control_node src/oritentation_admittance_control.cpp)
target_link_libraries(oritentation_admittance_control_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(hybrid_postion_force_tool_node src/hybrid_postion_force_tool.cpp)
target_link_libraries(hybrid_postion_force_tool_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)


add_executable(admittance_control_of_ur_node src/admittance_control_of_ur.cpp)
target_link_libraries(admittance_control_of_ur_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

