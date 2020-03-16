cmake_minimum_required(VERSION 2.8.3)
project(pygazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  mavros
  mav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  message_generation
 )

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    rospy
    mavros
    mav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/hellogazebo.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)