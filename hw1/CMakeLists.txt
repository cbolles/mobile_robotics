cmake_minimum_required(VERSION 3.0.2)
project(hw1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hw1 src/hw1.cpp)
target_link_libraries(hw1 ${catkin_LIBRARIES})
