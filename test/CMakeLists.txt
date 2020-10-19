cmake_minimum_required(VERSION 3.0.2)
project(test)

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
)



## Specify libraries to link a library or executable target against
# add_executable(gravity_identify src/gravity_identify.cpp)
# target_link_libraries(gravity_identify
#   ${catkin_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
# )

# add_executable(test_param src/test_param.cpp)
# target_link_libraries(test_param
#   ${catkin_LIBRARIES}
# )

# add_executable(gravity_compensate src/gravity_compensate.cpp)
# target_link_libraries(gravity_compensate
#   ${catkin_LIBRARIES}
# )

add_executable(tf_test src/tf_test.cpp)
target_link_libraries(tf_test
  ${catkin_LIBRARIES}
)

add_executable(mean_filter src/mean_filter.cpp)
target_link_libraries(mean_filter
  ${catkin_LIBRARIES}
)

add_executable(movel_test src/movel_test.cpp)
target_link_libraries(movel_test
  ${catkin_LIBRARIES}
)

add_executable(speedl_test src/speedl_test.cpp)
target_link_libraries(speedl_test
  ${catkin_LIBRARIES}
)

add_executable(hybird_test src/hybird_test.cpp)
target_link_libraries(hybird_test
  ${catkin_LIBRARIES}
)

add_executable(admittance_control src/admittance_control.cpp)
target_link_libraries(admittance_control
  ${catkin_LIBRARIES}
)

add_executable(compensate_filter src/compensate_filter.cpp)
target_link_libraries(compensate_filter
  ${catkin_LIBRARIES}
)

add_executable(fixed_point_admittance_control src/fixed_point_admittance_control.cpp)
target_link_libraries(fixed_point_admittance_control
  ${catkin_LIBRARIES}
)

add_executable(homogeneous_transformation_node src/homogeneous_transformation.cpp)
target_link_libraries(homogeneous_transformation_node
  ${catkin_LIBRARIES}
)


add_executable(yaml_test src/yaml_test.cpp)
target_link_libraries(yaml_test
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(tool_speedl_test_node src/tool_speedl_test.cpp)
target_link_libraries(tool_speedl_test_node
  ${catkin_LIBRARIES}
  ${EIGEN3_CPP_LIBRARIES}

)

add_executable(space2tool_velocity_control_node src/space2tool_velocity_control.cpp)
target_link_libraries(space2tool_velocity_control_node
  ${catkin_LIBRARIES}
  ${EIGEN3_CPP_LIBRARIES}

)

add_executable(tf_get2point_node src/tf_get2point.cpp)
target_link_libraries(tf_get2point_node
  ${catkin_LIBRARIES}
  ${EIGEN3_CPP_LIBRARIES}

)




# add_executable(wrench2tool src/wrench2tool.cpp)
# target_link_libraries(wrench2tool
#   ${catkin_LIBRARIES}
# )



