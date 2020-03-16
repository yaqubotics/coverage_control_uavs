# coverage_control_uavs

To play this project, do these following commands:
1. Put this "coverage_control_uavs" folder to your ros "catkin_ws" folder then do "catkin_make"
2. Pull the PX4 Firmware project from "https://github.com/yaqubotics/Firmware/tree/cc_uavs" outside your ros "catkin_ws" folder
3. Open "coverage_control_uavs/bash_scripts/start_mission.sh" and edit the following variables:
    - ros_wd for ROS working directory
    - px4_wd for PX4 Firmware working directory
4. Run "start_mission.sh" in bash_scripts folder by
```
source start_mission.sh
```

