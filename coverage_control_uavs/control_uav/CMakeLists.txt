cmake_minimum_required(VERSION 2.8.3)
project(control_uav)

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
  nav_msgs
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
    nav_msgs
)

catkin_install_python(PROGRAMS
  scripts/control_uav.py
  scripts/try_markers.py
  scripts/rviz_visualization.py
  scripts/control_script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)