#!/usr/bin/env python
#from __future__ import print_function
# test_gitlab
import argparse
from threading import Thread
from time import sleep
import datetime
import time
import roslib
import tf
import sys
import rospy
import numpy as np
import copy
import mavros
import json
import os
import glob
import math
from mavros.utils import *
from mavros_msgs.msg import State, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, PolygonStamped, Polygon, Point32, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float32, Int8MultiArray, Int32, Header
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from VelocityController import VelocityController

#from visualization_msgs.msg 
#========================================= IMPORT COVERAGE =============================================
import compute_coverage as com
import load_parameter as lp
import load_estimation_structure as les
import local_planning as locplan
from global_planning import *
from global_planning_function import *
#========================================= END OF IMPORT COVERAGE =============================================

AGENT_GOES_TO_INIT = 0
AGENT_HOVER_YAWING = 1
AGENT_GOES_TO_GOAL = 2
AGENT_ARRIVE_AT_GOAL = 3
mission_state = AGENT_GOES_TO_INIT
current_state = State()
#current_velocity = GlobalPositionTarget()
#current_velocity = PositionTarget()
current_velocity = TwistStamped()

list_current_position = []
list_current_velocity = []
prev_position = (0,0,0)
current_position = (0,0,0)
list_reach_goal = []
list_do_coverage = []
list_measurement_sensor = []
list_measurement_pos_x = []
list_measurement_pos_y = []
wait_measurement_sensor_flag = []
wait_measurement_pos_x_flag = []
wait_measurement_pos_y_flag = []
list_velocity_angle = dict()
list_distance_obs = dict()
#def velocity_cb(msg_var, data_var):
#  print msg_var
#  print data_var
PID_CONTROL = 1
rviz_visualization_start = False
init_coverage_start = False
control_type = PID_CONTROL

cur_pose = PoseStamped()
list_velocity_angle = dict()
list_distance_obs = dict()
'''
pathname=os.path.abspath(+'/src/coverage_control_uavs/control_uav/scripts/')
print pathname
listfolder = []
for foldername in os.listdir(pathname+'/img_res'):
  listfolder.append(int(foldername))
  #if fname.endswith('.mp4'):
  # listvideo.append(fname) 

if(len(listfolder) == 0):
  curr_folder = 0
else:
  curr_folder = np.max(listfolder)
print 'current folder:',curr_folder

folderstring = pathname+'/img_res/%04d/'%curr_folder
'''
def rad2deg(inp):
  return inp*180.0/np.pi
def deg2rad(inp):
  return inp/180.0*np.pi

def error_pub_function(arg1,arg2):
  global list_current_position, poses
  idx = arg2

  error_pub=rospy.Publisher("/uav"+str(idx+1)+"/pose_error",PoseStamped,queue_size=10)
  while(True):
    goal_pose = (poses.pose.position.x,poses.pose.position.y)
    current_agent_pose = (list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y)

    error = PoseStamped()
    error.pose.position.x = poses.pose.position.x - list_current_position[idx].pose.position.x
    error.pose.position.y = poses.pose.position.y - list_current_position[idx].pose.position.y

    error_pub.publish(error)
    sleep(0.1)

def velocity_pub_function(arg1,arg2,arg3):
  global RVO_pub_list,cur_pose,poses,list_current_position,list_current_velocity,local_vel_pub,local_pos_pub,init_coverage_start,mission_state, prev_position
  par = arg1
  idx = arg2
  vController = arg3
  count = 0
  RVO_publish_var = Float32MultiArray()
  RVO_publish_var.layout.dim.append(MultiArrayDimension())
  RVO_publish_var.layout.dim[0].label = "length"
  RVO_publish_var.layout.dim[0].size = 6
  RVO_publish_var.layout.dim[0].stride = 6
  RVO_publish_var.layout.data_offset = 0
  RVO_publish_var.data = [0,0,0,0,0,0]
  while(True):
    if(init_coverage_start):
      if(control_type == PID_CONTROL):
        header = Header()
        header.stamp = rospy.Time.now()
        #header.frame_id = "world"

        des_vel = vController.update(cur_pose)
        current_agent_pose = (list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z)
        current_agent_vel = (list_current_velocity[idx].twist.linear.x,list_current_velocity[idx].twist.linear.y)
        current_neighbor_pose = []
        for i in range(par.NumberOfAgents):
          if(i != idx):
            current_neighbor_pose.append((list_current_position[i].pose.position.x,list_current_position[i].pose.position.y))
        current_neighbor_vel = []
        for i in range(par.NumberOfAgents):
          if(i != idx):
            current_neighbor_vel.append((list_current_velocity[i].twist.linear.x,list_current_velocity[i].twist.linear.y))
        V_des = (des_vel.twist.linear.x,des_vel.twist.linear.y)
        quat_arr = np.array([list_current_position[idx].pose.orientation.x,list_current_position[idx].pose.orientation.y,list_current_position[idx].pose.orientation.z,list_current_position[idx].pose.orientation.w])
        att = euler_from_quaternion(quat_arr,'sxyz')
        #V = locplan.RVO_update_single(current_agent_pose, current_neighbor_pose, current_agent_vel, current_neighbor_vel, V_des, par,list_velocity_angle,list_distance_obs,att[2])
        #print current_agent_pose, current_neighbor_pose, par.ws_model['circular_obstacles']
        V,RVO_BA_all = locplan.RVO_update_single_static(current_agent_pose, current_neighbor_pose, current_agent_vel, current_neighbor_vel, V_des, par)
        rvo_idx = 0
        for RVO in RVO_BA_all:
          if(RVO[5] == 'obstacle'):
            RVO_publish_var.data = [RVO[0][0],RVO[0][1],RVO[1][0],RVO[1][1],RVO[2][0],RVO[2][1]]
            RVO_pub_list[rvo_idx].publish(RVO_publish_var)
            rvo_idx=rvo_idx+1
        V_msg = des_vel
        #print V_msg.twist.linear.x - V[0], V_msg.twist.linear.y - V[1]
        V_msg.header.stamp = rospy.Time.now() 
        V_msg.twist.linear.x = V[0]
        V_msg.twist.linear.y = V[1]
        local_vel_pub.publish(V_msg)
        #des_vel = vController.update_prop(cur_pose,0.4)
        #local_vel_pub.publish(des_vel)
        
      elif(mission_state == AGENT_GOES_TO_GOAL): 
        current_agent_pose = (list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z)
        goal_pose = (poses.pose.position.x,poses.pose.position.y,poses.pose.position.z)
        current_agent_vel = (list_current_velocity[idx].twist.linear.x,list_current_velocity[idx].twist.linear.y)
        #if(True): #distance2D(current_agent_pose,goal_pose) > 2):
        V_des = locplan.compute_V_des_single(current_agent_pose,goal_pose,par.V_MAX)
        #V_des = locplan.compute_V_des_single_PD(current_agent_pose,goal_pose,current_agent_vel,par.V_MAX)
        quat_arr = np.array([list_current_position[idx].pose.orientation.x,list_current_position[idx].pose.orientation.y,list_current_position[idx].pose.orientation.z,list_current_position[idx].pose.orientation.w])
        att = euler_from_quaternion(quat_arr,'sxyz')
        theta = math.atan2(V_des[1],V_des[0]) #-att[2]
        k=0.3
        #a=[-np.sin(theta),np.cos(theta)]
        #b=[np.cos(theta),np.sin(theta)]
        #pdiff=[current_agent_pose[0]-goal_pose[0],current_agent_pose[1]-goal_pose[1]]
        #w=2*k*np.arctan(np.dot(a,pdiff)/np.dot(b,pdiff))
        #v=-k*np.dot(b,pdiff)
        #V_des = [v*np.cos(att[2]),v*np.sin(att[2])]
        w = k*(theta-att[2])
        current_neighbor_pose = []
        for i in range(par.NumberOfAgents):
          if(i != idx):
            current_neighbor_pose.append((list_current_position[i].pose.position.x,list_current_position[i].pose.position.y))
        current_neighbor_vel = []
        for i in range(par.NumberOfAgents):
          if(i != idx):
            current_neighbor_vel.append((list_current_velocity[i].twist.linear.x,list_current_velocity[i].twist.linear.y))
        Vz = k*(par.StaticAltitude-list_current_position[idx].pose.position.z)
        V = locplan.RVO_update_single(current_agent_pose, current_neighbor_pose, current_agent_vel, current_neighbor_vel, V_des, par,list_velocity_angle,list_distance_obs)
        k = 0.01
        vx = k*(goal_pose[0]-current_agent_pose[0])
        vy = k*(goal_pose[1]-current_agent_pose[1])
        vz = k*(goal_pose[2]-current_agent_pose[2])
        V_msg = TwistStamped()
        V_msg.header.stamp = rospy.Time.now() 
        V_msg.twist.linear.x = vx #V_des[0] # 0 #V[0] #
        V_msg.twist.linear.y = vy #V_des[1] # 0 #V[1] # 
        V_msg.twist.linear.z = vz #Vz
        V_msg.twist.angular.x = 0.0
        V_msg.twist.angular.y = 0.0
        V_msg.twist.angular.z = 0.0
        local_vel_pub.publish(V_msg)
        '''
        else:
          pose_msg = PoseStamped()
          pose_msg.header.stamp = rospy.Time.now()
          pose_msg.pose.position.x = poses.pose.position.x
          pose_msg.pose.position.y = poses.pose.position.y
          pose_msg.pose.position.z = par.StaticAltitude
          local_pos_pub.publish(pose_msg)
        '''
      elif(mission_state == AGENT_ARRIVE_AT_GOAL):
        #pose_msg = PoseStamped()
        #pose_msg.header.stamp = rospy.Time.now()
        #pose_msg.pose.position.x = poses.pose.position.x
        #pose_msg.pose.position.y = poses.pose.position.y
        #pose_msg.pose.position.z = poses.pose.position.z
        #local_pos_pub.publish(pose_msg)
        V_msg = TwistStamped()
        V_msg.header.stamp = rospy.Time.now() 
        V_msg.twist.linear.x =0.0 #  0 # V[0] #0 #
        V_msg.twist.linear.y =0.0 #  0 # V[1] #0 #
        V_msg.twist.linear.z = 0.0
        V_msg.twist.angular.x = 0.0
        V_msg.twist.angular.y = 0.0
        V_msg.twist.angular.z = 0.0 #w
        local_vel_pub.publish(V_msg)
        #else:
        #  mission_state = AGENT_GOES_TO_GOAL
        #  publish_mission_state(mission_state)
    #sleep(0.01)

def publish_mission_state(mission_state):
  global mission_state_pub
  mission_state_msg = Int32()
  mission_state_msg.data = mission_state
  mission_state_pub.publish(mission_state_msg)

def pose_pub_function():
  #for i in range(arg):
  global uav_ID,uav_idx,br,poses,list_current_position,local_pos_pub,poses,rviz_visualization_start,uav_marker,uav_marker_publisher,uav_goal_marker,uav_goal_marker_publisher
  last_request_rviz = rospy.Time.now()
  while(True):
    if(rviz_visualization_start and (rospy.Time.now()-last_request_rviz > rospy.Duration(0.2))):
      publish_uav_position_rviz(br,list_current_position[uav_idx].pose.position.x,list_current_position[uav_idx].pose.position.y,list_current_position[uav_idx].pose.position.z,uav_ID)
      uav_marker=update_uav_marker(uav_marker,(list_current_position[uav_idx].pose.position.x,list_current_position[uav_idx].pose.position.y,list_current_position[uav_idx].pose.position.z,1.0))
      uav_marker_publisher.publish(uav_marker)
      uav_goal_marker=update_uav_marker(uav_goal_marker,(poses.pose.position.x,poses.pose.position.y,poses.pose.position.z,1.0))
      uav_goal_marker_publisher.publish(uav_goal_marker)
      last_request_rviz = rospy.Time.now()
    if( not init_coverage_start):
      local_pos_pub.publish(poses)
    #print poses
    #sleep(0.5)

def publish_uav_position_rviz(br,x,y,z,uav_idx):
  br.sendTransform((x,y,z),
                  (0.0, 0.0, 0.0, 1.0),
                  rospy.Time.now(),
                  "uav_tf_"+uav_idx,
                  "world")

def velocity_cb(msg_var,data_var): #, data_var):
  global list_current_velocity
  idx = data_var
  list_current_velocity[idx].twist.linear.x = msg_var.twist.linear.x
  list_current_velocity[idx].twist.linear.y = msg_var.twist.linear.y
  list_current_velocity[idx].twist.linear.z = msg_var.twist.linear.z
  #print msg_var.twist.linear.x,msg_var.twist.linear.y,msg_var.twist.linear.z
  #print msg_var.twist.angular.x,msg_var.twist.angular.y,msg_var.twist.angular.z

def position_cb(msg_var,data_var):
  global current_position, list_current_position,cur_pose
  #print data_var, msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z
  idx = data_var[0]
  uav_idx = data_var[1]
  offset = data_var[2]
  list_current_position[idx].pose.position.x = msg_var.pose.position.x+offset[idx][0]
  list_current_position[idx].pose.position.y = msg_var.pose.position.y+offset[idx][1]
  list_current_position[idx].pose.position.z = msg_var.pose.position.z 
  list_current_position[idx].pose.orientation.x = msg_var.pose.orientation.x
  list_current_position[idx].pose.orientation.y = msg_var.pose.orientation.y
  list_current_position[idx].pose.orientation.z = msg_var.pose.orientation.z
  list_current_position[idx].pose.orientation.w = msg_var.pose.orientation.w
  if(idx == uav_idx):
    cur_pose = list_current_position[idx]
    current_position = (msg_var.pose.position.x+offset[idx][0],msg_var.pose.position.y+offset[idx][1],msg_var.pose.position.z)

def reach_goal_cb(msg_var,data_var):
  idx = data_var
  list_reach_goal[idx].data = msg_var.data

def do_coverage_cb(msg_var,data_var):
  idx = data_var
  list_do_coverage[idx].data = msg_var.data

def measurement_sensor_cb(msg_var,data_var):
  global wait_measurement_sensor_flag
  idx = data_var
  list_measurement_sensor[idx] = msg_var.data[1]
  wait_measurement_sensor_flag[idx] = msg_var.data[0]
  #print "wait_meas_sensor_cb", wait_measurement_sensor_flag
  #print "meas_sensor_cb ["+repr(idx+1)+"] : ",msg_var

def measurement_pos_x_cb(msg_var,data_var):
  global wait_measurement_pos_x_flag
  idx = data_var
  list_measurement_pos_x[idx] = msg_var.data[1]
  wait_measurement_pos_x_flag[idx] = msg_var.data[0]
  #print "wait_meas_pos_x_cb", wait_measurement_pos_x_flag
  #print "meas_pos_x_cb ["+repr(idx+1)+"] : ",msg_var


def measurement_pos_y_cb(msg_var,data_var):
  global wait_measurement_pos_y_flag
  idx = data_var
  list_measurement_pos_y[idx] = msg_var.data[1]
  wait_measurement_pos_y_flag[idx] = msg_var.data[0]
  #print "wait_meas_pos_y_cb", wait_measurement_pos_y_flag
  #print "meas_pos_y_cb ["+repr(idx+1)+"] : ",msg_var


def state_cb(msg_var):
  global current_state
  if(current_state.armed != msg_var.armed):
    rospy.loginfo("armed state changed from {0} to {1}".format(
                current_state.armed, msg_var.armed))

  if(current_state.connected != msg_var.connected):
    rospy.loginfo("connected changed from {0} to {1}".format(
                current_state.connected, msg_var.connected))

  if(current_state.mode != msg_var.mode):
    rospy.loginfo("mode changed from {0} to {1}".format(
                current_state.mode, msg_var.mode))

  current_state = msg_var
  #print msg_var

def scan_lidar_cb(msg_var,data_var):
  global list_velocity_angle,list_distance_obs
  if(msg_var.header.frame_id == "uav"+data_var+"/sonar2_link"):
    obs_detected = False
    list_obs = []
    list_angle = []
    for i in range(len(msg_var.ranges)):
      if(msg_var.ranges[i] != np.inf):
        if(not obs_detected):
          list_sub_obs = []
          list_sub_angle = []
          obs_detected = True
        list_sub_obs.append(msg_var.ranges[i])
        list_sub_angle.append(msg_var.angle_min+i*msg_var.angle_increment)
      else:
        if(obs_detected):
          list_obs.append(list_sub_obs)
          list_angle.append(list_sub_angle)
          obs_detected = False
    for i in range(len(list_angle)):
      #print "angle["+repr(i+1)+"]:",list_angle[i][0],list_angle[i][len(list_angle[i])-1]
      list_velocity_angle[i] = [list_angle[i][0],list_angle[i][len(list_angle[i])-1]]
      list_distance_obs[i] = np.min(list_obs[i])
    if(len(list_velocity_angle) < len(list_angle)):
      for i in range(len(list_velocity_angle)-len(list_angle)):
        del list_velocity_angle[len(list_angle)+i]
        del list_distance_obs[len(list_angle)+i]

def get_number_of_agent():
  rostopic_list_all = rospy.get_published_topics()
  number_of_agent = 0
  uav_ID_list = []
  for i in rostopic_list_all:
    for j in i:
      if "uav" in j:
        if("uav"+j[4]) not in uav_ID_list:
          uav_ID_list.append("uav"+j[4])
          number_of_agent += 1

  uav_ID_list.sort()
  return uav_ID_list,number_of_agent-1

def update_uav_marker(marker,pos):
  temp_marker = marker
  temp_marker.pose.orientation.w = pos[3]
  temp_marker.pose.position.x = pos[0]
  temp_marker.pose.position.y = pos[1]
  temp_marker.pose.position.z = pos[2]

  return temp_marker

def distance2D(a,b):
  #return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)
  a = (a.position.x,a.position.y)
  b = (b.position.x,b.position.y)
  return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def distance3D(a,b):
  return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)

def publish_polygon_voronoi(data,scale):
  global polygon_voronoi_publisher
  polygon_msg = Polygon()
  polygon_msg.points = []
  for i in range(len(data)):
    point_temp = Point32()
    point_temp.x = data[i][0]*scale
    point_temp.y = data[i][1]*scale
    point_temp.z = 0
    polygon_msg.points.append(point_temp)
  polygon_voronoi_publisher.publish(polygon_msg)

def main(args):
  global wait_measurement_sensor_flag
  global wait_measurement_pos_x_flag
  global wait_measurement_pos_y_flag
  global current_position
  global current_state
  global polygon_voronoi_publisher
  global list_current_position, poses
  global list_current_velocity
  global local_vel_pub
  global local_pos_pub
  global init_coverage_start
  global mission_state
  global prev_position
  global mission_state_pub, RVO_pub_list, rviz_visualization_start
  global uav_marker,uav_marker_publisher,uav_goal_marker,uav_goal_marker_publisher,br,uav_idx,uav_ID
  #thread.join()
  par = lp.Parameter
  re = les.RecursiveEstimation
  true_function = lp.TrueFunction(par)
  est_function = re.DensityFunction
  ne = les.NoEstimation
  par.CurrentIteration = 1
  #print "Program Started"
  dir_path = os.path.dirname(os.path.realpath(__file__))
  print dir_path
  with open(dir_path+'/config.json','r') as f:
    config = json.load(f)

  uav_ID = "1"
  x_val = 0
  y_val = 0
  z_val = 0

  for i in range(len(args)):
    if(args[i] == "-uav"):
      uav_ID = args[i+1]
    elif(args[i] == "-x"):
      x_val = float(args[i+1])
    elif(args[i] == "-y"):
      y_val = float(args[i+1])
    elif(args[i] == "-z"):
      z_val = float(args[i+1])
    elif(args[i] == "-rwd"):
      rwd_val = args[i+1]
  init_pos = config["UAV"][uav_ID]["INIT_POS"]
  uav_color = config["UAV"][uav_ID]["COLOR"]
  print "init pos: ",init_pos
  rospy.init_node("control_uav_"+uav_ID)
  uav_idx = int(uav_ID)-1
  br = tf.TransformBroadcaster()
  publish_uav_position_rviz(br,init_pos[0],init_pos[1],init_pos[2],uav_ID)

  pathname=os.path.abspath(rwd_val+'/src/coverage_control_uavs/control_uav/scripts/')
  print pathname
  listfolder = []
  for foldername in os.listdir(pathname+'/img_res'):
    listfolder.append(int(foldername))
    #if fname.endswith('.mp4'):
    # listvideo.append(fname) 

  if(len(listfolder) == 0):
    curr_folder = 0
  else:
    curr_folder = np.max(listfolder)
  print 'current folder:',curr_folder

  folderstring = pathname+'/img_res/%04d/'%curr_folder

  if(uav_ID == '1'):
    os.system("cp -i "+dir_path+'/config.json '+folderstring)
    print "cp -i "+dir_path+'/config.json '+folderstring
  try:
    rospy.wait_for_service("/uav"+uav_ID+"/mavros/cmd/arming")
    rospy.wait_for_service("/uav"+uav_ID+"/mavros/set_mode")
  except rospy.ROSException:
    fail("failed to connect to service")
  state_sub=rospy.Subscriber("/uav"+uav_ID+"/mavros/state", State,queue_size=10,callback=state_cb)
  #velocity_sub=rospy.Subscriber("/uav"+uav_ID+"/mavros/local_position/velocity",)
  #list_UAV_ID,num_agent = get_number_of_agent()
  #num_agent = len(config["UAV"])
  list_UAV_ID = []
  all_init_pos = []
  for i in range(par.NumberOfAgents):
    list_UAV_ID.append("uav"+repr(i+1))
    all_init_pos.append(config["UAV"][repr(i+1)]["INIT_POS"])
  print "NUM OF AGENT = ",par.NumberOfAgents
  print "LIST UAV = ",list_UAV_ID
  do_coverage_flag = Bool()
  do_coverage_flag.data = False
  reach_goal_flag = Bool()
  reach_goal_flag.data = True
  var_float = Float32()
  var_float.data = 0.0
  for i in range(par.NumberOfAgents):
    list_current_position.append(PoseStamped())
    list_current_velocity.append(TwistStamped())
    list_reach_goal.append(reach_goal_flag)
    list_do_coverage.append(do_coverage_flag)
    #list_measurement_sensor.append(var_float)
    list_measurement_sensor.append(0.0)
    wait_measurement_sensor_flag.append(0.0)
    #list_measurement_pos_x.append(var_float)
    list_measurement_pos_x.append(0.0)
    wait_measurement_pos_x_flag.append(0.0)
    #list_measurement_pos_y.append(var_float)
    list_measurement_pos_y.append(0.0)
    wait_measurement_pos_y_flag.append(0.0)

  V_max = []
  for i in range(par.NumberOfAgents):
    velocity_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mavros/local_position/velocity",TwistStamped,queue_size = 10,callback=velocity_cb, callback_args=i)
    position_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mavros/local_position/pose",PoseStamped,queue_size = 10,callback=position_cb,callback_args=(i,uav_idx,all_init_pos))
    reach_goal_sub = rospy.Subscriber("/uav"+repr(i+1)+"/reach_goal",Bool,queue_size=10,callback=reach_goal_cb,callback_args=i)
    do_coverage_sub = rospy.Subscriber("/uav"+repr(i+1)+"/do_coverage",Bool,queue_size=10,callback=do_coverage_cb,callback_args=i)
    measurement_sensor_sub = rospy.Subscriber("/uav"+repr(i+1)+"/measurement_sensor",Float32MultiArray,queue_size=10,callback=measurement_sensor_cb,callback_args=i)
    measurement_pos_x_sub = rospy.Subscriber("/uav"+repr(i+1)+"/measurement_pos_x",Float32MultiArray,queue_size=10,callback=measurement_pos_x_cb,callback_args=i)
    measurement_pos_y_sub = rospy.Subscriber("/uav"+repr(i+1)+"/measurement_pos_y",Float32MultiArray,queue_size=10,callback=measurement_pos_y_cb,callback_args=i)
    V_max.append(par.V_MAX)

  RVO_pub_list = []
  for i in range(len(par.ws_model['circular_obstacles'])):
    RVO_pub_list.append(rospy.Publisher("/uav"+uav_ID+"/RVO"+repr(i),Float32MultiArray,queue_size=10))
  scan_lidar_sub = rospy.Subscriber("/scan",LaserScan,queue_size=10,callback=scan_lidar_cb,callback_args=uav_ID)
  #velocity_sub = rospy.Subscriber("/uav1/mavros/setpoint_raw/global",GlobalPositionTarget,queue_size = 10,callback=velocity_cb)
  #velocity_sub = rospy.Subscriber("/uav1/mavros/setpoint_raw/global",PositionTarget,queue_size = 10,callback=velocity_cb)
  #velocity_sub = rospy.Subscriber("/uav1/mavros/local_position/velocity",TwistStamped,queue_size = 10,callback=velocity_cb)
  #position_sub = rospy.Subscriber("/uav1/mavros/local_position/pose",PoseStamped,queue_size = 10,callback=position_cb)   
  global local_pos_pub, poses
  local_pos_pub=rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_position/local",PoseStamped,queue_size=10)
  local_vel_pub=rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
  goal_pos_pub=rospy.Publisher("/uav"+uav_ID+"/goal_position",PoseStamped,queue_size=10)
  sub_goal_pos_pub=rospy.Publisher("/uav"+uav_ID+"/sub_goal_position",PoseStamped,queue_size=10)
  local_reach_goal_flag_pub=rospy.Publisher("/uav"+uav_ID+"/reach_goal",Bool,queue_size=10)
  local_do_coverage_flag_pub=rospy.Publisher("/uav"+uav_ID+"/do_coverage",Bool,queue_size=10)
  mission_state_pub=rospy.Publisher("/uav"+uav_ID+"/mission_state",Int32,queue_size=10)
  #local_measurement_sensor_pub=rospy.Publisher("/uav"+uav_ID+"/measurement_sensor",Float32,queue_size=10)
  #local_measurement_pos_x_pub=rospy.Publisher("/uav"+uav_ID+"/measurement_pos_x",Float32,queue_size=10)
  #local_measurement_pos_y_pub=rospy.Publisher("/uav"+uav_ID+"/measurement_pos_y",Float32,queue_size=10)

  local_measurement_sensor_pub=rospy.Publisher("/uav"+uav_ID+"/measurement_sensor",Float32MultiArray,queue_size=10)
  local_measurement_pos_x_pub=rospy.Publisher("/uav"+uav_ID+"/measurement_pos_x",Float32MultiArray,queue_size=10)
  local_measurement_pos_y_pub=rospy.Publisher("/uav"+uav_ID+"/measurement_pos_y",Float32MultiArray,queue_size=10)
  polygon_voronoi_publisher = rospy.Publisher("/uav"+uav_ID+"/polygon_voronoi",Polygon,queue_size=10)
  voronoi_grid_publisher = rospy.Publisher("/uav"+uav_ID+"/voronoi_grid",Int8MultiArray,queue_size=10)

  current_iteration_publisher = rospy.Publisher("/current_iteration",Int32,queue_size=10)

  voronoi_grid = Int8MultiArray()
  voronoi_grid.layout.dim.append(MultiArrayDimension())
  voronoi_grid.layout.dim.append(MultiArrayDimension())
  voronoi_grid.layout.dim[0].label = "height"
  voronoi_grid.layout.dim[1].label = "width"
  voronoi_grid.layout.dim[0].size = par.NumberOfPoints
  voronoi_grid.layout.dim[1].size = par.NumberOfPoints
  voronoi_grid.layout.dim[0].stride = par.NumberOfPoints*par.NumberOfPoints
  voronoi_grid.layout.dim[1].stride = par.NumberOfPoints
  voronoi_grid.layout.data_offset = 0
  voronoi_grid.data = np.zeros((par.NumberOfPoints,par.NumberOfPoints)).reshape((1,par.NumberOfPoints*par.NumberOfPoints)).tolist()[0]

  polygon_msg = Polygon()



  meas_sensor = Float32MultiArray()
  meas_sensor.layout.dim.append(MultiArrayDimension())
  meas_sensor.layout.dim[0].label = "length"
  meas_sensor.layout.dim[0].size = 2
  meas_sensor.layout.dim[0].stride = 2
  meas_sensor.layout.data_offset = 0
  meas_sensor.data = [0.0,0.0]

  meas_pos_x = Float32MultiArray()
  meas_pos_x.layout.dim.append(MultiArrayDimension())
  meas_pos_x.layout.dim[0].label = "length"
  meas_pos_x.layout.dim[0].size = 2
  meas_pos_x.layout.dim[0].stride = 2
  meas_pos_x.layout.data_offset = 0
  meas_pos_x.data = [0.0,0.0]

  meas_pos_y = Float32MultiArray()
  meas_pos_y.layout.dim.append(MultiArrayDimension())
  meas_pos_y.layout.dim[0].label = "length"
  meas_pos_y.layout.dim[0].size = 2
  meas_pos_y.layout.dim[0].stride = 2
  meas_pos_y.layout.data_offset = 0
  meas_pos_y.data = [0.0,0.0]

  est_function_pc = Float32MultiArray()
  est_function_pc.layout.dim.append(MultiArrayDimension())
  est_function_pc.layout.dim.append(MultiArrayDimension())
  est_function_pc.layout.dim[0].label = "height"
  est_function_pc.layout.dim[1].label = "width"
  est_function_pc.layout.dim[0].size = est_function.shape[0]
  est_function_pc.layout.dim[1].size = est_function.shape[1]
  est_function_pc.layout.dim[0].stride = est_function.shape[0]*est_function.shape[1]
  est_function_pc.layout.dim[1].stride = est_function.shape[1]
  est_function_pc.layout.data_offset = 0
  est_function_pc.data = est_function.reshape((1,est_function.shape[0]*est_function.shape[1])).tolist()[0]

  est_sensory_function_pub=rospy.Publisher("/uav"+uav_ID+"/est_sensory_function",Float32MultiArray,queue_size=10)

  arming_client = rospy.ServiceProxy("/uav"+uav_ID+"/mavros/cmd/arming",CommandBool)
  set_mode_client = rospy.ServiceProxy("/uav"+uav_ID+"/mavros/set_mode",SetMode)
  topic = 'uav_marker_'+uav_ID
  uav_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
  uav_marker = Marker()
  uav_marker.header.frame_id = "world"
  uav_marker.type = uav_marker.SPHERE
  uav_marker.action = uav_marker.ADD
  uav_marker.scale.x = par.ws_model['robot_radius']*3
  uav_marker.scale.y = par.ws_model['robot_radius']*3
  uav_marker.scale.z = 0.005*par.RealScale
  uav_marker.color.r = float(uav_color[0])/255
  uav_marker.color.g = float(uav_color[1])/255
  uav_marker.color.b = float(uav_color[2])/255
  uav_marker.color.a = float(uav_color[3])/255
  uav_marker.pose.orientation.w = 1.0
  uav_marker.pose.position.x = init_pos[0]
  uav_marker.pose.position.y = init_pos[1] 
  uav_marker.pose.position.z = init_pos[2] 
  uav_marker_publisher.publish(uav_marker)

  poses = PoseStamped()
  #poses.pose = Pose()
  #poses.pose.position = Point()
  poses.pose.position.x = x_val
  poses.pose.position.y = y_val
  poses.pose.position.z = z_val
  
  uav_color = [255,255,255,255]
  topic = 'uav_goal_marker_'+uav_ID
  uav_goal_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
  uav_goal_marker = Marker()
  uav_goal_marker.header.frame_id = "world"
  uav_goal_marker.type = uav_goal_marker.CUBE
  uav_goal_marker.action = uav_goal_marker.ADD
  uav_goal_marker.scale.x = par.ws_model['robot_radius']*2
  uav_goal_marker.scale.y = par.ws_model['robot_radius']*2
  uav_goal_marker.scale.z = 0.005*par.RealScale
  uav_goal_marker.color.r = float(uav_color[0])/255
  uav_goal_marker.color.g = float(uav_color[1])/255
  uav_goal_marker.color.b = float(uav_color[2])/255
  uav_goal_marker.color.a = float(uav_color[3])/255
  uav_goal_marker.pose.orientation.w = 1.0
  uav_goal_marker.pose.position.x = 0#init_pos[0]
  uav_goal_marker.pose.position.y = 0#init_pos[1] 
  uav_goal_marker.pose.position.z = 0#init_pos[2] 
  uav_goal_marker_publisher.publish(uav_goal_marker)
  uav_goal_marker=update_uav_marker(uav_goal_marker,(poses.pose.position.x,poses.pose.position.y,poses.pose.position.z,1.0))
  uav_goal_marker_publisher.publish(uav_goal_marker)

  r=rospy.Rate(10)
  vController = VelocityController()
  #print(dir(current_state.connected))
  print "TRY TO CONNECT"
  while ((not rospy.is_shutdown()) and (not current_state.connected)):
    #rospy.spinOnce()
    r.sleep()
  print "CONNECTED"
  #print(current_state.connected.__class__)
  rospy.loginfo("CURRENT STATE CONNECTED")

  init_position_coverage = (x_val,y_val,z_val)
  print poses.pose.position.x,poses.pose.position.y,poses.pose.position.z
  i = 100
  while((not rospy.is_shutdown()) and (i>0)):
    local_pos_pub.publish(poses)
    i = i-1
  rospy.loginfo("POSITION PUBLISHED")
  publish_mission_state(mission_state)
  thread1 = Thread(target = pose_pub_function)
  thread1.start()
  thread2 = Thread(target = velocity_pub_function,args=(par,int(uav_ID)-1,vController))
  thread2.start()
  thread3 = Thread(target = error_pub_function,args=(par,int(uav_ID)-1))
  thread3.start()
  last_request = rospy.Time.now()
  last_request_neighbor = rospy.Time.now()
  last_request_rviz = rospy.Time.now()
  last_request_debug = rospy.Time.now()
  last_iteration_timeout = rospy.Time.now()
  rviz_visualization_start = False
  coverage_start = False
  coverage_end = False
  averageErrorDensityFunction = []
  timePerIteration = []
  timeCoverage = []
  while(not rospy.is_shutdown()):
    #offb_set_mode = set_mode_client(0,'OFFBOARD') 
    if(rviz_visualization_start and (rospy.Time.now()-last_request_rviz > rospy.Duration(0.2))):
      #publish_uav_position_rviz(br,list_current_position[uav_idx].pose.position.x,list_current_position[uav_idx].pose.position.y,list_current_position[uav_idx].pose.position.z,uav_ID)
      #uav_marker=update_uav_marker(uav_marker,(list_current_position[uav_idx].pose.position.x,list_current_position[uav_idx].pose.position.y,list_current_position[uav_idx].pose.position.z,1.0))
      #uav_marker_publisher.publish(uav_marker)
      #uav_goal_marker=update_uav_marker(uav_goal_marker,(poses.pose.position.x,poses.pose.position.y,poses.pose.position.z,1.0))
      #uav_goal_marker_publisher.publish(uav_goal_marker)
      last_request_rviz = rospy.Time.now()
      #print "list_reach_goal",list_reach_goal
    if(rviz_visualization_start and (rospy.Time.now()-last_request_debug > rospy.Duration(1.0))):
      last_request_debug = rospy.Time.now()
      #print "+++ DEBUG +++"
      #print list_do_coverage
      #print "+++++++++++++"

    if(rospy.Time.now()-last_request_neighbor > rospy.Duration(10.0)):
      #for i in range(num_agent):
      #  print list_current_position[i].pose.position.x,list_current_position[i].pose.position.y
      last_request_neighbor = rospy.Time.now()
      
    if((not rviz_visualization_start) and (current_state.mode != 'OFFBOARD') and (rospy.Time.now()-last_request > rospy.Duration(1.0))):
      offb_set_mode = set_mode_client(0,'OFFBOARD')
      if(offb_set_mode.mode_sent):
        rospy.loginfo("OFFBOARD ENABLED")
      last_request = rospy.Time.now()
    else:
      if((not current_state.armed) and (rospy.Time.now()-last_request > rospy.Duration(1.0))):
        arm_cmd = arming_client(True)
        if(arm_cmd.success):
          rospy.loginfo("VEHICLE ARMED")
          rviz_visualization_start = True
          #coverage_start = True
          reach_goal_flag = Bool()
          reach_goal_flag.data = True
          last_request_rviz = rospy.Time.now()
        last_request=rospy.Time.now()

    if(par.CurrentIteration == 1) and not coverage_start:
      #print "current position", current_position
      #print "init pos coverage", init_position_coverage
      #if(distance3D(init_position_coverage,current_position)<3):
      if(rospy.Time.now()-last_iteration_timeout > rospy.Duration(5.0)):
        last_iteration_timeout = rospy.Time.now()
        target = Pose()
        targetStamped = PoseStamped()
        target.position.x = poses.pose.position.x
        target.position.y = poses.pose.position.y
        target.position.z = poses.pose.position.z

        diagram4 = GridWithWeights(par.NumberOfPoints,par.NumberOfPoints)
        diagram4.walls = []
        for i in range(par.NumberOfPoints): # berikan wall pada algoritma pathfinding A*
          for j in range(par.NumberOfPoints):
            if (par.ObstacleRegion[j,i]==0):
              diagram4.walls.append((i,j))
        for i in range(par.NumberOfPoints):
          for j in range(par.NumberOfPoints):
            if(par.ObstacleRegionWithClearence[j][i] == 0):
              diagram4.weights.update({(i,j): par.UniversalCost})

        goal_pos = (target.position.x/par.RealScale,target.position.y/par.RealScale)
        start_pos = (cur_pose.pose.position.x/par.RealScale,cur_pose.pose.position.y/par.RealScale)
        pathx,pathy=Astar_version1(par,start_pos,goal_pos,diagram4)
        target.position.x = pathx[0]*par.RealScale
        target.position.y = pathy[0]*par.RealScale
        vController.setHeadingTarget(deg2rad(45.0))
        vController.setTarget(target)
        vController.setPIDGainX(1,0.0,0.0)
        vController.setPIDGainY(1,0.0,0.0)
        vController.setPIDGainZ(1,0,0)
        vController.setPIDGainPHI(1,0.0,0.0)
        vController.setPIDGainTHETA(1,0.0,0.0)
        vController.setPIDGainPSI(1,0.0,0.0)
        
        init_coverage_start = True
        coverage_start = True
        V_msg = TwistStamped()
        V_msg.header.stamp = rospy.Time.now() 
        V_msg.twist.linear.x =0 #
        V_msg.twist.linear.y =0 #
        V_msg.twist.linear.z = 0
        V_msg.twist.angular.x = 0.0
        V_msg.twist.angular.y = 0.0
        V_msg.twist.angular.z = 0.0 #w
        local_vel_pub.publish(V_msg)
        mission_state = AGENT_ARRIVE_AT_GOAL
        publish_mission_state(mission_state)

        
    if(coverage_start): # and all(item.data == True for item in list_reach_goal)): # coverage
      #gaussian regression
      print "time:" , datetime.datetime.now().strftime("%H:%M:%S")
      print "do coverage iter-",par.CurrentIteration
      do_coverage_flag = Bool()
      do_coverage_flag.data = True
      x = (list_current_position[uav_idx].pose.position.x/par.RealScale,list_current_position[uav_idx].pose.position.y/par.RealScale)
      y = com.transmitNewMeasurementOneAgent(par,x)
      #measurement.data = [par.CurrentIteration,x[0],x[1],y]

      for i in range(par.NumberOfAgents):
        meas_sensor.data = [par.CurrentIteration,y]
        meas_pos_x.data = [par.CurrentIteration,x[0]]
        meas_pos_y.data = [par.CurrentIteration,x[1]]
        local_measurement_sensor_pub.publish(meas_sensor)
        local_measurement_pos_x_pub.publish(meas_pos_x)
        local_measurement_pos_y_pub.publish(meas_pos_y)
      
      if(par.UsingStrategy == 2):
        while not (wait_measurement_sensor_flag.count(par.CurrentIteration) >= par.NumberOfAgents // 2):
          pass

        while not (wait_measurement_pos_x_flag.count(par.CurrentIteration) >= par.NumberOfAgents // 2):
          pass

        while not (wait_measurement_pos_y_flag.count(par.CurrentIteration) >= par.NumberOfAgents // 2):
          pass

        for i in range(par.NumberOfAgents):
          if(wait_measurement_sensor_flag[i] == par.CurrentIteration) and (wait_measurement_pos_x_flag[i] == par.CurrentIteration) and (wait_measurement_pos_y_flag[i] == par.CurrentIteration):
            re.statusFinishedAgent[i] = 0
      
      if(par.UsingStrategy == 1):
        while not all(item == par.CurrentIteration for item in wait_measurement_sensor_flag): 
          pass
        
        while not all(item == par.CurrentIteration for item in wait_measurement_pos_x_flag): 
          pass

        while not all(item == par.CurrentIteration for item in wait_measurement_pos_y_flag): 
          pass
      
      #print "wait_meas_sensor",wait_measurement_sensor_flag
      #print "wait_meas_pos_x",wait_measurement_pos_x_flag
      #print "wait_meas_pos_y",wait_measurement_pos_y_flag
      
      #for i in range(par.NumberOfAgents):
      #  wait_measurement_sensor_flag[i] = True
      #  wait_measurement_pos_x_flag[i] = True
      #  wait_measurement_pos_y_flag[i] = True

      print "list_measurement:",list_measurement_sensor,list_measurement_pos_x,list_measurement_pos_y
      # wait all agent measure
      iteration_msg = Int32()
      iteration_msg.data = par.CurrentIteration
      if(uav_ID == '1'):
        current_iteration_publisher.publish(iteration_msg)
      if(uav_ID == '1'):
        if(par.CurrentIteration>1):
          timePerIteration.append(time.time()-timePerIterationStart)
          f= open(folderstring+"/timeiter.txt","a+")
          f.write("%f\r\n" % (timePerIteration[par.CurrentIteration-2]))
          f.close()
        timePerIterationStart = time.time()
        timeCoverageStart = time.time()
      re = com.joinNewMeasurements(par,re,list_measurement_sensor,list_measurement_pos_x,list_measurement_pos_y)
      timeCoverageStart = time.time()
      re = com.computeCentralizedEstimation(re,par)
      re = com.computeCoverage(re,par,re.APosterioriVariance,par.ws_model)
      if(uav_ID == '1'):
        timeCoverage.append(time.time()-timeCoverageStart)
        selisih = re.DensityFunction - true_function
        averageErrorDensityFunction.append(np.sum(np.sum(np.multiply(selisih,selisih)))/(par.NumberOfPoints*par.NumberOfPoints))
        re.CostFunction.append(com.computeCost(par,re.Node,re.DensityFunction,par.CostFlag))
        f= open(folderstring+"/maxvar.txt","a+")
        f.write("%f\r\n" % (re.maxAPosterioriVariance[par.CurrentIteration-1]))
        f.close()
        f= open(folderstring+"/minvar.txt","a+")
        f.write("%f\r\n" % (re.minAPosterioriVariance[par.CurrentIteration-1]))
        f.close()
        f= open(folderstring+"/avgvar.txt","a+")
        f.write("%f\r\n" % (re.averageAPosterioriVariance[par.CurrentIteration-1]))
        f.close()
        f= open(folderstring+"/avgdens.txt","a+")
        f.write("%f\r\n" % (averageErrorDensityFunction[par.CurrentIteration-1]))
        f.close()
        f= open(folderstring+"/timecoverage.txt","a+")
        f.write("%f\r\n" % (timeCoverage[par.CurrentIteration-1]))
        f.close()
        f= open(folderstring+"/cost.txt","a+")
        f.write("%f\r\n" % (re.CostFunction[par.CurrentIteration-1]))
        f.close()
        f= open(folderstring+"/energy.txt","a+")
        f.write("%f\r\n" % (re.averageEnergy[par.CurrentIteration-1]))
        f.close()
      
      est_function = re.DensityFunction
      est_function_pc.data = est_function.reshape((1,est_function.shape[0]*est_function.shape[1])).tolist()[0]
      if(uav_ID == '1'):
        est_sensory_function_pub.publish(est_function_pc)

      prev_position = (poses.pose.position.x, poses.pose.position.y, poses.pose.position.z)
      poses.pose.position.x = re.Node[uav_idx].pos[0]*par.RealScale
      poses.pose.position.y = re.Node[uav_idx].pos[1]*par.RealScale
      #print poses.pose.position.x,poses.pose.position.y
      poses.pose.position.z = par.StaticAltitude
      goal_pos_pub.publish(poses)
      mission_state = AGENT_GOES_TO_GOAL #AGENT_HOVER_YAWING
      publish_mission_state(mission_state)

      
      #
      
      if(control_type == PID_CONTROL):
        target = Pose()
        target.position.x = poses.pose.position.x
        target.position.y = poses.pose.position.y
        target.position.z = poses.pose.position.z
        vController.setHeadingTarget(deg2rad(45.0))
        vController.setTarget(target)
        vController.setPIDGainX(1,0.0,0.0)
        vController.setPIDGainY(1,0.0,0.0)
        vController.setPIDGainZ(1,0,0)
        vController.setPIDGainPHI(1,0.0,0.0)
        vController.setPIDGainTHETA(1,0.0,0.0)
        vController.setPIDGainPSI(1,0.0,0.0)
      
      goal_pos = (target.position.x/par.RealScale,target.position.y/par.RealScale)
      start_pos = (cur_pose.pose.position.x/par.RealScale,cur_pose.pose.position.y/par.RealScale)
      pathx,pathy=Astar_version1(par,start_pos,goal_pos,diagram4)
      target.position.x = pathx[0]*par.RealScale
      target.position.y = pathy[0]*par.RealScale
      vController.setHeadingTarget(deg2rad(45.0))
      vController.setTarget(target)
      targetStamped.pose = target
      sub_goal_pos_pub.publish(targetStamped)

      print "Max Variance [iter:"+repr(par.CurrentIteration)+"] : ",np.max(np.max(re.APosterioriVariance))
      
      publish_polygon_voronoi(re.Node[uav_idx].pvicini,par.RealScale)
      voronoi_grid.data = re.Node[uav_idx].VoronoiRegion.reshape((1,par.NumberOfPoints*par.NumberOfPoints)).tolist()[0]
      voronoi_grid_publisher.publish(voronoi_grid)
      #print "Input Locations: ",re.InputLocations
      #reach_goal_flag.data = False
      #coverage_end = True
      coverage_start = False
      par.CurrentIteration = par.CurrentIteration +1

    if init_coverage_start:
      if(distance2D(cur_pose.pose,target) < 3):
        if(len(pathx) > 1):
          del pathx[0]
          del pathy[0]
          target.position.x = pathx[0]*par.RealScale
          target.position.y = pathy[0]*par.RealScale
          print "change subtarget"
          vController.setTarget(target)
          targetStamped.pose = target
          sub_goal_pos_pub.publish(targetStamped)


    if (all(item.data == True for item in list_do_coverage)):
      #print "all done coverage"
      #print poses.pose.position.x,poses.pose.position.y
      #print list_current_position[uav_idx].pose.position.x,list_current_position[uav_idx].pose.position.y
      reach_goal_flag = Bool()
      reach_goal_flag.data = False

    pose_x_diff = (poses.pose.position.x-list_current_position[uav_idx].pose.position.x)**2
    pose_y_diff = (poses.pose.position.y-list_current_position[uav_idx].pose.position.y)**2
    x = (list_current_position[uav_idx].pose.position.x,list_current_position[uav_idx].pose.position.y,list_current_position[uav_idx].pose.position.z)
    goal = (poses.pose.position.x,poses.pose.position.y,poses.pose.position.z)
    timeout_flag = rospy.Time.now()-last_iteration_timeout > rospy.Duration(5.0)
    if((do_coverage_flag.data == True) and (reach_goal_flag.data == False) and (timeout_flag or ((par.UsingStrategy == 2) and (wait_measurement_sensor_flag.count(par.CurrentIteration) >= par.NumberOfAgents // 2)))) :
    #if((do_coverage_flag.data == True) and (reach_goal_flag.data == False) and ('''(distance3D(goal,x) < 2)'''timeout_flag or ((par.UsingStrategy == 2) and (wait_measurement_sensor_flag.count(par.CurrentIteration) >= par.NumberOfAgents // 2)))) :
      last_iteration_timeout = rospy.Time.now()
      mission_state = AGENT_ARRIVE_AT_GOAL
      publish_mission_state(mission_state)
      print "time:" , datetime.datetime.now().strftime("%H:%M:%S")
      print "agent "+uav_ID+" reach the goal"
      reach_goal_flag = Bool()
      reach_goal_flag.data = True
      # Node pos is current position
      for i in range(par.NumberOfAgents):
        re.Node[i].pos[0]=float(list_current_position[i].pose.position.x)/par.RealScale
        re.Node[i].pos[1]=float(list_current_position[i].pose.position.y)/par.RealScale
      #coverage_end = False
      do_coverage_flag = Bool()
      do_coverage_flag.data = False
      coverage_start = True
      for i in range(par.NumberOfAgents):
        local_reach_goal_flag_pub.publish(reach_goal_flag)

      while(not all(item.data == True for item in list_reach_goal)):
        #local_reach_goal_flag_pub.publish(reach_goal_flag)
        pass
      print "time:" , datetime.datetime.now().strftime("%H:%M:%S")
      print "list_reach_goal",list_reach_goal

    # publish everything
    local_do_coverage_flag_pub.publish(do_coverage_flag)
    local_reach_goal_flag_pub.publish(reach_goal_flag)
    #local_pos_pub.publish(poses)
    r.sleep()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
