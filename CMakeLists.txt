cmake_minimum_required(VERSION 2.8.3)
project(robot_positioning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)
#find_package(OpenCV)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
)

add_executable(position
  src/position.cpp
)
target_link_libraries(position
  ${catkin_LIBRARIES}
  #${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
)

