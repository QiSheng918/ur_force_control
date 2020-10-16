cmake_minimum_required(VERSION 3.0.2)
project(gravity_compensate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf
  roslib
)

find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)




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
#   sensor_msgs
# )



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
#  LIBRARIES gravity_compensate
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs moveit_core moveit_ros_planning_interface 
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
)


add_executable(gravity_identify_node src/gravity_identify.cpp)
target_link_libraries(gravity_identify_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(gravity_compensate_node src/gravity_compensate.cpp)
target_link_libraries(gravity_compensate_node
  ${catkin_LIBRARIES}
)

add_executable(move2init_pose_node src/move2init_pose.cpp)
target_link_libraries(move2init_pose_node
  ${catkin_LIBRARIES}
)

add_executable(contact_frame_wrench_node src/contact_frame_wrench.cpp)
target_link_libraries(contact_frame_wrench_node
  ${catkin_LIBRARIES}
)






