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

add_executable(hw5 src/mapper.cpp src/maputils.cpp)
target_link_libraries(hw5 ${catkin_LIBRARIES} GL GLU glut)


