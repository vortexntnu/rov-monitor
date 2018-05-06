cmake_minimum_required(VERSION 2.8.3)
project(monitor-server)
find_package(catkin REQUIRED
  rospy
  roslint
)

roslint_python()

catkin_package(
  CATKIN_DEPENDS
  rospy
)
