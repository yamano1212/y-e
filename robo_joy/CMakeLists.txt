cmake_minimum_required(VERSION 2.8.3)
project(robo_joy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy
  sensor_msgs
  # text_con
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joystick
#  CATKIN_DEPENDS roscpp std_msgs sensor_msgs serial
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(joy_listen_node src/joy_listen_node.cpp)
 target_link_libraries(joy_listen_node
   ${catkin_LIBRARIES}
 )
# add_executable(text_listen_node src/text_listen_node.cpp)
# target_link_libraries(text_listen_node
#   ${catkin_LIBRARIES}
# )

