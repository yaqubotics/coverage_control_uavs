#source /home/yaqub/simulation/ros_catkin_ws/devel/setup.bash
cd $1
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_typhoon_sitl.launch