cmake_minimum_required(VERSION 2.8.3)
project(demo_grasping)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hiqp_ros
  gazebo_msgs
  roscpp
  rosbag
  yumi_hw
  grasp_planner
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
     CATKIN_DEPENDS roscpp rosbag hiqp_ros controller_manager_msgs gazebo_msgs yumi_hw grasp_planner
     INCLUDE_DIRS include
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(demo_grasping
  src/demo_grasping.cpp
  src/look_what_i_found.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(demo_grasping ${catkin_LIBRARIES})
