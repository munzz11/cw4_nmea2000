cmake_minimum_required(VERSION 2.8.3)
project(cw4_nmea2000)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES radar_obstacle_detection
#  CATKIN_DEPENDS marine_msgs rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

