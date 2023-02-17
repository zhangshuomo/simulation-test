cmake_minimum_required(VERSION 3.0.2)
project(to_quadruped)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

list(APPEND CMAKE_MODULE_PATH /usr/local/include/casadi/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
pinocchio REQUIRED
# RBDL COMPONENTS URDFREADER REQUIRED
  roscpp
  rospy
  std_msgs
  unitree_legged_msgs
)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
catkin_package( CATKIN_DEPENDS 
unitree_legged_msgs )

include_directories(
  ${PINOCCHIO_INCLUDE_DIRS}
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(quadruped_sim_test src/quadruped_sim_test.cpp)
target_link_libraries(quadruped_sim_test ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

add_executable(single_leg_sim src/single_leg_sim_test.cpp)
target_link_libraries(single_leg_sim ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})