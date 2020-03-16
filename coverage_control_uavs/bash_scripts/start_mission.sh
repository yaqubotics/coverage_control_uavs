#! /bin/bash
ros_wd="/home/$USER/yaqub_ws"
px4_wd="/home/$USER/Firmware_clone"
wd="$ros_wd/src/coverage_control_uavs/bash_scripts"
gnome-terminal --working-directory="${wd}" -x bash -c "source environment_start.sh ${px4_wd}; exec bash"
sleep 3
gnome-terminal --working-directory="${wd}" -x bash -c "source multi_agent_start.sh ${px4_wd} ; exec bash"
sleep 8
source $ros_wd/devel/setup.bash
gnome-terminal --working-directory="${wd}" -x bash -c "source rviz_visualization.sh ${ros_wd} ; exec bash"
sleep 3
gnome-terminal --working-directory="${wd}" -x bash -c "rosrun control_uav control_script.py ${ros_wd}; exec bash"
