cmake_minimum_required(VERSION 2.8.3)
project(yumi_demo_launch)

catkin_package()

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
