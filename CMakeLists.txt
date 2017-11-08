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

add_library(Client_library
   src/Client/TCPClient.h
   src/Client/TCPClient.cpp
)


add_executable(Manual_Control src/Manual_Control.cpp)
target_link_libraries(Manual_Control ${catkin_LIBRARIES})

add_executable(Client_test src/Client/Client_test.cpp)
target_link_libraries(Client_test ${catkin_LIBRARIES} Client_library)
