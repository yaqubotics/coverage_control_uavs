#!/usr/bin/env python
import subprocess
import os
import sys
import json
print "start"
#subprocess.call("./run_control_uav.sh")
print "end"
dir_path = os.path.dirname(os.path.realpath(__file__))
print dir_path
with open(dir_path+'/config.json','r') as f:
	config = json.load(f)
num_of_agent = len(config["UAV"])
print num_of_agent
uav_id = []
uav_init_pos = []
uav_dest_pos = []
uav_color = []
for i in range(num_of_agent):
	uav_id.append(config["UAV"][repr(i+1)])
	uav_init_pos.append(config["UAV"][repr(i+1)]["INIT_POS"])
	uav_dest_pos.append(config["UAV"][repr(i+1)]["DEST_POS"])
	uav_color.append(config["UAV"][repr(i+1)]["COLOR"])
#print uav_dest_pos
print str(sys.argv)
ros_wd=str(sys.argv[1])
print ros_wd
for i in range(num_of_agent):
	# --working-directory=/home/yaqub/simulation/ros_catkin_ws/src/thesis/control_uav/scripts/ 
	sp = "gnome-terminal --title=\"Terminal UAV-"+repr(i+1)+"\" --working-directory="+ros_wd+"/src/coverage_control_uavs/control_uav/scripts/ -x bash -c \"source run_control_uav.sh -u "+repr(i+1)+" -x "+repr(uav_dest_pos[i][0])+" -y "+repr(uav_dest_pos[i][1])+" -z "+repr(uav_dest_pos[i][2])+" -rwd "+ros_wd+"; exec bash\" "
	print sp
	subprocess.call(sp,shell=True)
	#subprocess.call("gnome-terminal --working-directory=/home/yaqub/simulation/ros_catkin_ws/src/thesis/control_uav/scripts/ -x bash -c ""source run_control_uav.sh ; exec bash"" ")

# gnome-terminal --working-directory=/home/yaqub/simulation/ -x bash -c "source control_multi_agent.sh -u 1 -x 0.0 -y 0.0 -z 5.0 ; exec bash"
