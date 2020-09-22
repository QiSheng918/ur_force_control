cmake_minimum_required(VERSION 3.0.2)
project(wrench_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wrench_filter
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wrench_filter.cpp
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(mean_filter_node src/mean_filter.cpp)
target_link_libraries(mean_filter_node
  ${catkin_LIBRARIES}
)

add_executable(compensate_wrench_tool_filter_node src/compensate_wrench_tool_filter.cpp)
target_link_libraries(compensate_wrench_tool_filter_node
  ${catkin_LIBRARIES}
)

add_executable(compensate_wrench_base_filter_node src/compensate_wrench_base_filter.cpp)
target_link_libraries(compensate_wrench_base_filter_node
  ${catkin_LIBRARIES}
)

add_executable(wrench2tool_node src/wrench2tool.cpp)
target_link_libraries(wrench2tool_node
  ${catkin_LIBRARIES}
)







