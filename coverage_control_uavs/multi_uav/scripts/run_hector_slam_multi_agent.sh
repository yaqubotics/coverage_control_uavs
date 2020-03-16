#! /bin/bash

gnome-terminal --working-directory=/home/yaqub/ -x bash -c "roslaunch px4_simulation_stack multi_mapping_hexa.launch ID:=$1; exec bash"
