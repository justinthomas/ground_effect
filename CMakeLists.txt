cmake_minimum_required(VERSION 2.8.3)
project(ground_effect)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs geometry_msgs quadrotor_msgs controllers_manager velocity_tracker line_tracker null_tracker std_msgs kr_common cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs quadrotor_msgs controllers_manager velocity_tracker line_tracker null_tracker std_msgs kr_common
  DEPENDS Eigen)

add_executable(state_control src/state_control.cpp src/trajectory.cpp)
add_dependencies(state_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_control ${catkin_LIBRARIES})
