cmake_minimum_required(VERSION 3.0.2)
project(hw3)

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

add_executable(hw3 src/goto.cpp src/utils.cpp src/move.cpp)
target_link_libraries(hw3 ${catkin_LIBRARIES})
