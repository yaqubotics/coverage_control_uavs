#!/usr/bin/env python

import rospy
from dynamic_reconfigure import client as drc
from dynamic_reconfigure import DynamicReconfigureParameterException

if __name__ == '__main__':
    rospy.init_node('loop_on_kp')
    DYNAMIC_RECONFIGURE_SERVER = '/gazebo_test'

    # Get new client, internally it connects and monitors the services/topics related
    # to the dynamic reconfigure server, keep the client, don't create one every
    # time you need to do a call
    try:
        dyn_rec_client = drc.Client(DYNAMIC_RECONFIGURE_SERVER, timeout=10.0)
    except rospy.exceptions.ROSException as e:
        rospy.logerr('Error while creating dynamic reconfigure client: ' + str(e))


    # Spawn the ramp
    try:
        dyn_rec_client.update_configuration({'click_to_spawn_ramp': True})
    except DynamicReconfigureParameterException as e:
        rospy.logerr('There was an exception trying to set up click_to_spawn_ramp new value: ' + str(e))

    rospy.sleep(3.0)
    # Also, on the shell, this would be:
    # rosrun dynamic_reconfigure dynparam set /gazebo_test click_to_spawn_ramp true


    # Loop on KP parameter to record a video and see the influence of the different values
    rospy.loginfo("Going to loop increasing KP param of the object multiplying by 10...")
    _ = raw_input("Press any key + Enter to start")
    kp_param = 100.0
    kp_step = 10.0
    sleep_time = 5.0
    for i in range(100):
        rospy.loginfo("\n\nKP = " + str(kp_param))
        try:
            dyn_rec_client.update_configuration({'KP_': kp_param,
                'click_to_spawn_model': True})
        except DynamicReconfigureParameterException as e:
            rospy.logerr('There was an exception trying to set up click_to_spawn_ramp new value: ' + str(e))
        kp_param *= kp_step
        rospy.sleep(sleep_time)
    rospy.loginfo("Finished experiment.")
