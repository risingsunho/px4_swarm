cmake_minimum_required(VERSION 2.8.3)
project(px4_swarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)




add_executable(Manual_Control src/Manual_Control.cpp)
target_link_libraries(Manual_Control ${catkin_LIBRARIES})

