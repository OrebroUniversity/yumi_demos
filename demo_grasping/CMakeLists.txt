cmake_minimum_required(VERSION 2.8.3)
project(demo_grasping)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hqp_controllers_msgs
  gazebo_msgs
  roscpp
  rosbag
  yumi_hw
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
     CATKIN_DEPENDS roscpp rosbag hqp_controllers_msgs controller_manager_msgs gazebo_msgs yumi_hw
     INCLUDE_DIRS include
)

include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(demo_grasping src/demo_grasping.cpp
                                src/look_what_i_found.cpp
              )


## Specify libraries to link a library or executable target against
target_link_libraries(demo_grasping ${catkin_LIBRARIES})
