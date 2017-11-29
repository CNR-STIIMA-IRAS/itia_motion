cmake_minimum_required(VERSION 2.8.3)
project(itia_helios)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  itia_futils
  itia_gutils
  itia_mutils
  itia_rutils
  roscpp
  nodelet
  std_msgs
  tf
  eigen_conversions
  itia_msgs
  itia_dynamics_core
  trajectory_msgs
  actionlib
  control_msgs
  itia_motion_msgs
  itia_fir_planner
)

find_package(Eigen3 REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_helios
  CATKIN_DEPENDS cmake_modules itia_futils itia_gutils 
                 itia_mutils roscpp std_msgs control_models 
                 itia_rutils tf itia_msgs geometry_msgs eigen_conversions 
                 itia_dynamics_core trajectory_msgs control_msgs actionlib nodelet
                 itia_motion_msgs itia_fir_planner
  DEPENDS  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(itia_helios
  src/${PROJECT_NAME}/mpc_planner.cpp
  src/${PROJECT_NAME}/helios_nodelet.cpp
)
add_dependencies(itia_helios  ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_helios
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(mpc_test src/test_mpc.cpp)
add_dependencies(mpc_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_test
  itia_helios
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
#  ${orocos_kdl_LIBRARIES}
)

