#!/bin/bash

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -u|--uav)
    UAV_ID="$2"
    shift # past argument
    shift # past value
    ;;
    -x)
    X_VAL="$2"
    shift # past argument
    shift # past value
    ;;
    -y)
    Y_VAL="$2"
    shift # past argument
    shift # past value
    ;;
    -z)
    Z_VAL="$2"
    shift # past argument
    shift # past value
    ;;
    -rwd)
    ROS_WD="$2"
    shift # past argument
    shift # past value
    ;;
    --default)
    DEFAULT=YES
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

echo UAV ID  = "${UAV_ID}"
echo SET POINT     = "${X_VAL} ${Y_VAL} ${Z_VAL}"
echo DEFAULT         = "${DEFAULT}"

source ${ROS_WD}/devel/setup.bash
rosrun control_uav control_uav.py -uav ${UAV_ID} -x ${X_VAL} -y ${Y_VAL} -z ${Z_VAL} -rwd ${ROS_WD}
#rosrun ros_iris_takeoff_land ros_iris_takeoff_land -uav ${UAV_ID} -x ${X_VAL} -y ${Y_VAL} -z ${Z_VAL}