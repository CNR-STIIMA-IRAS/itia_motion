cmake_minimum_required(VERSION 2.8.3)
project(itia_fir_planner)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O4")


find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  eigen_conversions
  geometry_msgs
  itia_dynamics_core
  itia_futils
  itia_gutils
  itia_motion_msgs
  itia_msgs
  itia_mutils
  itia_rutils
  nodelet
  roscpp
  std_msgs
  tf
  trajectory_msgs
  diagnostic_msgs
)

find_package(Eigen3 REQUIRED )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES itia_fir_planner
 CATKIN_DEPENDS actionlib control_msgs eigen_conversions geometry_msgs itia_dynamics_core itia_futils itia_gutils itia_motion_msgs itia_msgs itia_mutils itia_rutils nodelet roscpp std_msgs tf trajectory_msgs diagnostic_msgs
 DEPENDS  
)
 

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(itia_fir_planner
  src/${PROJECT_NAME}/itia_fir_planner.cpp
  src/${PROJECT_NAME}/velocity_profile.cpp
)
add_dependencies(itia_fir_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_fir_planner
  ${catkin_LIBRARIES}
)

add_executable(itia_fir_planner_test src/itia_fir_planner_test.cpp)
add_dependencies(itia_fir_planner_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_fir_planner_test
  itia_fir_planner
  ${catkin_LIBRARIES}
)


add_executable(itia_fir_interpolate_server  src/itia_fir_interpolate_server.cpp
                                            src/itia_fir_planner/itia_fir_interpolate_service.cpp)
add_dependencies(itia_fir_interpolate_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_fir_interpolate_server
  itia_fir_planner
  ${catkin_LIBRARIES}
)

add_executable(itia_fir_interpolate_client  src/itia_fir_interpolate_client.cpp)
add_dependencies(itia_fir_interpolate_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_fir_interpolate_client
  itia_fir_planner
  ${catkin_LIBRARIES}
)