cmake_minimum_required(VERSION 3.0.2)
project(hw5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

catkin_install_python(PROGRAMS src/mapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
