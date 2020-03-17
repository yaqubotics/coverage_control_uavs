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
The program should look like this:

[![Watch the video](https://img.youtube.com/vi/PpdYl5iVp4I/maxresdefault.jpg)](https://youtu.be/PpdYl5iVp4I)

References:

[1] Todescato, Marco, Andrea Carron, Ruggero Carli, Gianluigi Pillonetto, and Luca Schenato. "Multi-robots Gaussian estimation and coverage control: From client–server to peer-to-peer architectures." Automatica 80 (2017): 284-294.

[2] Snape, Jamie, Jur Van Den Berg, Stephen J. Guy, and Dinesh Manocha. "The hybrid reciprocal velocity obstacle." IEEE Transactions on Robotics 27, no. 4 (2011): 696-706.

Please cite:

Prabowo, Y.A. and Trilaksono, B.R., 2019. Collision-Free Coverage Control of Swarm Robotics Based on Gaussian Process Regression to Estimate Sensory Function in non-Convex Environment. International Journal on Electrical Engineering & Informatics, 11(1).
