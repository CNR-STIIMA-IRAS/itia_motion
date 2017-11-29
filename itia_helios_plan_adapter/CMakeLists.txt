cmake_minimum_required(VERSION 2.8.3)
project(itia_helios_plan_adapter)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  itia_helios
  roscpp
  moveit_ros_planning trajectory_msgs industrial_trajectory_filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_helios_plan_adpater
  CATKIN_DEPENDS itia_helios moveit_ros_planning trajectory_msgs industrial_trajectory_filters roscpp
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(itia_helios_plan_adpater
#   src/${PROJECT_NAME}/itia_helios_plan_adpater.cpp
# )
# add_dependencies(itia_helios_plan_adpater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(itia_helios_plan_adpater_node src/itia_helios_plan_adpater_node.cpp)
# add_dependencies(itia_helios_plan_adpater_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(itia_helios_plan_adpater_node
#   ${catkin_LIBRARIES}
# )

