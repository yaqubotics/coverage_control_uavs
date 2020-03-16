#!/usr/bin/env python
import argparse
from threading import Thread
from time import sleep
import datetime
import random
import roslib
import tf
import sys
import rospy
import numpy as np
from numpy import nan
import copy
import mavros
import json
import os,tty,termios
import glob
from mavros.utils import *
from mavros_msgs.msg import AttitudeTarget, State, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TransformStamped, Point, Pose, PoseStamped, TwistStamped, PolygonStamped, Polygon, Point32, Twist, Quaternion, Vector3, Twist, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float32, Float64, Int8MultiArray, Header
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
#from VelocityController import VelocityController
from nav_msgs.msg import GridCells, OccupancyGrid
from math import sqrt,cos,sin,atan2,asin

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
#import local_planning as locplan

#import compute_coverage as com
#import load_parameter as lp

from tf.transformations import euler_from_quaternion

#from global_planning import *
#from global_planning_function import *

import tf_conversions
import tf2_ros
import time
#par = lp.Parameter
listwalls = []
target = Pose()
sub_target = Pose()
sub_target_heading = 0.0

NUMNODES = 1700
LOS_SAFETY = 0.6
RRT_SAFETY = 0.6
EPSILON = 2.5
dir_path = os.path.dirname(os.path.realpath(__file__))
list_error = [[],[],[],[]]
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
xvel = 0
yvel = 0
state = 1
global_obs_detected = False
def get():
        global xvel,yvel
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                print "up"
                yvel=yvel+1
        elif k=='\x1b[B':
                print "down"
                yvel=yvel-1
        elif k=='\x1b[C':
                print "right"
                xvel=xvel+1
        elif k=='\x1b[D':
                print "left"
                xvel=xvel-1
        elif k=='s':
        		print "s"
        else:
                print "not an arrow key!"
        print "("+str(xvel)+" , "+str(yvel)+")"


def animate(i):
    global ax1,ax2,ax3,list_error
    #print "animate"
    #print i
    #ax1 = arg0
    #ax2 = arg1
    #ax3 = arg2
    
    if(len(list_error[0]) == len(list_error[1])):
    	ax1.clear()
    	ax1.plot(list_error[0], list_error[1])
    if(len(list_error[0]) == len(list_error[2])):
    	ax2.clear()
    	ax2.plot(list_error[0],list_error[2])
    if(len(list_error[0]) == len(list_error[3])):
    	ax3.clear()
    	ax3.plot(list_error[0],list_error[3])

def plot_error_live():
	global ax1,ax2,ax3
	fig = plt.figure()
	ax1 = fig.add_subplot(3,1,1)
	ax2 = fig.add_subplot(3,1,2)
	ax3 = fig.add_subplot(3,1,3)
	ani = animation.FuncAnimation(fig, animate, interval=1000)
	plt.show()

current_state = State()
current_velocity = TwistStamped()
cur_pose = PoseStamped()
current_position = [0,0,0]
list_current_position = []
list_current_velocity = []
list_velocity_angle = dict()
list_distance_obs = dict()
usingvelocitycontrol = False
usingpositioncontrol = True
start_calculate_and_publish_wall = False
list_angle_and_distance_obs = (None,[],[])
list_obstacle_vertex = (None,[],[])

header = Header()
for i in range(4):
	list_current_position.append(PoseStamped())
	list_current_velocity.append(TwistStamped())

#global variable for A star
#print par.NumberOfPoints,par.NumberOfPoints
#diagram = GridWithWeights(par.NumberOfPoints,par.NumberOfPoints)
#diagram.walls = []

class DiscTemplate:
    def __init__(self, max_r):
        self.memos = []
        for k_r in range(1, max_r + 1):
            k_r_sq = k_r ** 2
            self.memos.append([])
            for x in range(-max_r, max_r + 1):
                x_sq = x ** 2
                for y in range(-max_r, max_r + 1):
                    y_sq = y ** 2
                    if x_sq + y_sq <= k_r_sq:
                        self.memos[k_r - 1].append((x,y))

        self.max_r = max_r

    def get_disc(self, r):
        return self.memos[r - 1]

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

def map_cb(msg_var):
	global current_map,map_size,map_data,map_flag,walls,cur_pose,safety_grid_list,safety_marker,safety_marker_publisher,free_occ,map_res
	global cost_map_publisher,cost_map,cur_pose
	#current_map = msg_var.data
	#map_width = msg_var.info.width
	#map_height = msg_var.info.height
	start_map_cb = rospy.get_time()
	map_data = msg_var
	map_list = msg_var.data
	#map_array = np.array(msg_var.data)
	#current_map=map_array.reshape((msg_var.info.width,msg_var.info.height))
	map_size = msg_var.info.width*msg_var.info.resolution
	map_res = msg_var.info.resolution
	shape = msg_var.info.width
	#np.savetxt('map_txt/map_'+repr(msg_var.header.seq)+'.out',current_map)
	#print msg_var.header.seq,msg_var.header.stamp,msg_var.header.frame_id
	#print "map-"+repr(msg_var.header.seq)
	
	safety_grid = 0
	walls_temp= set()
	distance = 2
	test = DiscTemplate(distance)
	g = test.get_disc(distance)
	g2 = test.get_disc(1)
    #costmap = gridmap[:]
    #retlist = set()
	#free_occ_temp = []
	#shape_shape = current_map.shape[0]*current_map.shape[1]
	#print "shape",shape
	x0=int(np.ceil((cur_pose.pose.position.x+map_size/2)/map_res))
	y0=int(np.ceil((cur_pose.pose.position.y+map_size/2)/map_res))
	mindis = 10.0/map_res

	start_map_cb2 = rospy.get_time()
	for i in range(shape):
		for j in range(shape):
			if dist((x0,y0),(i,j)) < mindis:
				if(map_list[j*shape+i] == 100):
					gg = {(x+i,y+j) for (x,y) in g}
					#walls_temp.add((i,j))
					walls_temp = walls_temp.union(gg)
			else:
				if(map_list[j*shape+i] == 100):
					#walls_temp = walls_temp.union({(i,j)})
					#walls_temp.add((i,j))
					gg = {(x+i,y+j) for (x,y) in g2}
					#walls_temp.add((i,j))
					walls_temp = walls_temp.union(gg)

	'''
	for m in range(i-safety_grid_w,i+safety_grid_w):
		walls_temp.append((m,j-safety_grid_w))
		walls_temp.append((m,j+safety_grid_w))
	
	for n in range(j-safety_grid_w,j+safety_grid_w):
		walls_temp.append((i-safety_grid_w,n))
		walls_temp.append((i+safety_grid_w,n))
	'''
	walls = walls_temp
	#free_occ=free_occ_temp
	'''
	cx=int((cur_pose.pose.position.x+map_size/2)/map_res)		
	cy=int((cur_pose.pose.position.y+map_size/2)/map_res)
	#safety_grid_list_temp = []	
	safety_marker.points = []	
	for i in walls_temp:
		if(dist(i,(cx,cy)) < 20.0):
			if(i[0] < cx):
				x1=i[0]+safety_grid
			else:
				x1=i[0]-safety_grid
			if(i[1] < cy):
				y1=i[1]+safety_grid
			else:
				y1=i[1]-safety_grid
			walls_temp.append((x1,y1))
			p_rrt = Point()
			p_rrt.x = x1
			p_rrt.y = y1
			p_rrt.z = 0.0
			safety_marker.points.append(p_rrt)
			#safety_grid_list_temp.append((x1,y1))
			
			#for m in range(i[0]-safety_grid_w,i[0]+safety_grid_w): 
			#	for n in range(i[1]-safety_grid_w,i[1]+safety_grid_w): 
			#		walls_temp.append((m,n))
	#safety_grid_list = safety_grid_list_temp
	'''
	start_map_cb1 = rospy.get_time()
	arr = [0]*(shape*shape)
	for i in walls:
		idx = i[1]*shape+i[0]
		#if(idx < sizemap_sizemap):
		try:
			arr[idx]=100
		except:
			pass
	if(not map_flag):
		cost_map.info.resolution = map_res
		cost_map.info.width = shape
		cost_map.info.height = shape
		cost_map.info.origin.position.x = -shape/2.0*map_res
		cost_map.info.origin.position.y = -shape/2.0*map_res
	cost_map.data = arr
	cost_map_publisher.publish(cost_map)
	if(len(walls)>0):
		map_flag = True
	print "time_spent_map_cb all",rospy.get_time()-start_map_cb
	print "time_spent_map_cb core",rospy.get_time()-start_map_cb2
	print "time_spent_map_cb just pub",rospy.get_time()-start_map_cb1
	
	safety_marker_publisher.publish(safety_marker)
	#print "/home/yaqub/simulation/ros_catkin_ws/src/thesis/debug/scripts/map_txt/map"+repr(msg_var.header.seq)+".txt"
	#file1=open("/home/yaqub/simulation/ros_catkin_ws/src/thesis/debug/scripts/map_txt/map"+repr(msg_var.header.seq)+".txt","w+")
	#file1.write(current_map)
	#file1.close()
	#print 0 in current_map
	#print -np.inf in current_map
	#print np.inf in current_map
	#print 1 in current_map # 100:obstacle, 0: free, -1: unknown
	#np.savetxt("/home/yaqub/simulation/ros_catkin_ws/src/thesis/debug/scripts/map_txt/map"+repr(msg_var.header.seq)+".txt",current_map,fmt='%i')

def velocity_cb(msg_var,data_var): #, data_var):
	global list_current_velocity
	idx = data_var
	list_current_velocity[idx] = msg_var

def position_single_cb(msg_var):
	global cur_pose0
	cur_pose0 = msg_var

def goal_cb(msg_var):
	global goal,uav_goal_marker,uav_goal_marker_publisher,goal_changed_start
	goal = msg_var
	uav_goal_marker.pose.position.x = goal.pose.position.x#init_pos[0]
	uav_goal_marker.pose.position.y = goal.pose.position.y#init_pos[1] 
	uav_goal_marker_publisher.publish(uav_goal_marker)
	'''
	path_found = False
	while(not path_found):
		path_found=Replanning_RRT()
	goal_changed_start = True
	'''

def position_slam_cb(msg_var):
	global cur_pose
	cur_pose = msg_var

def position_cb(msg_var,data_var):
	global current_position, list_current_position,cur_pose
	#print data_var, msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z
	idx = data_var[0]
	uav_idx = data_var[1]
	
	if(idx == 0):
		list_current_position[idx] = msg_var
	if(idx == 1):
		list_current_position[idx].pose.position.x = msg_var.pose.position.x+28
		list_current_position[idx].pose.position.y = msg_var.pose.position.y+28
		list_current_position[idx].pose.position.z = msg_var.pose.position.z 
	if(idx == 2):
		list_current_position[idx].pose.position.x = msg_var.pose.position.x
		list_current_position[idx].pose.position.y = msg_var.pose.position.y+17
		list_current_position[idx].pose.position.z = msg_var.pose.position.z 
	if(idx == uav_idx):
		if(idx == 0):
			cur_pose = msg_var
			current_position = (msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z,msg_var.header.stamp.to_sec())
		if(idx == 1):
			cur_pose.pose.position.x = msg_var.pose.position.x+28
			cur_pose.pose.position.y = msg_var.pose.position.y+28
			cur_pose.pose.position.z = msg_var.pose.position.z
			current_position = (msg_var.pose.position.x+28,msg_var.pose.position.y+28,msg_var.pose.position.z,msg_var.header.stamp.to_sec())
		if(idx == 2):
			cur_pose.pose.position.x = msg_var.pose.position.x
			cur_pose.pose.position.y = msg_var.pose.position.y+17
			cur_pose.pose.position.z = msg_var.pose.position.z
			current_position = (msg_var.pose.position.x,msg_var.pose.position.y+17,msg_var.pose.position.z,msg_var.header.stamp.to_sec())
		
		br = tf2_ros.TransformBroadcaster()
		t = TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "/odom"
		t.child_frame_id = "/base_link"
		t.transform.translation.x = cur_pose.pose.position.x
		t.transform.translation.y = cur_pose.pose.position.y
		t.transform.translation.z = cur_pose.pose.position.z
		t.transform.rotation.x = msg_var.pose.orientation.x
		t.transform.rotation.y = msg_var.pose.orientation.y
		t.transform.rotation.z = msg_var.pose.orientation.z
		t.transform.rotation.w = msg_var.pose.orientation.w

		br.sendTransform(t)
	#print list_velocity_angle,list_distance_obs

def scan_lidar_cb(msg_var,data_var):
	global list_velocity_angle,list_distance_obs,global_obs_detected,list_angle_and_distance_obs, list_obstacle_vertex
	if(msg_var.header.frame_id == "uav"+data_var+"/sonar2_link"):
		lcp = list_current_position[int(data_var)-1].pose
		obs_detected = False
		list_obs = []
		list_angle = []
		for i in range(len(msg_var.ranges)):
			if(msg_var.intensities[i] != 0.0):
				print "intens",msg_var.intensities[i] 
			if(msg_var.ranges[i] != np.inf) and (i is not len(msg_var.ranges)-1):
				if(not obs_detected):
					list_sub_obs = []
					list_sub_angle = []
					obs_detected = True
					global_obs_detected = True
				list_sub_obs.append(msg_var.ranges[i])
				list_sub_angle.append(msg_var.angle_min+i*msg_var.angle_increment)
			else:
				if(obs_detected):
					list_obs.append(list_sub_obs)
					list_angle.append(list_sub_angle)
					obs_detected = False
					global_obs_detected = False

		#print "obs rad",list_obs
		#print "angle",list_angle 
		list_all_velocity_angle = []
		list_all_distance_obs = []
		for i in range(len(list_angle)):
			#print "angle["+repr(i+1)+"]:",list_angle[i][0],list_angle[i][len(list_angle[i])-1]
			list_velocity_angle[i] = [list_angle[i][0],list_angle[i][len(list_angle[i])-1]]
			list_distance_obs[i] = [list_obs[i][0],np.min(list_obs[i]),list_obs[i][len(list_obs[i])-1]]
			list_all_velocity_angle.append(list_angle[i])
			list_all_distance_obs.append(list_obs[i])
		if(len(list_velocity_angle) > len(list_angle)):
			for i in range(len(list_velocity_angle)-len(list_angle)):
				del list_velocity_angle[len(list_angle)+i]
				del list_distance_obs[len(list_angle)+i]
		list_angle_and_distance_obs = (lcp,list_all_velocity_angle,list_all_distance_obs)
		list_obstacle_vertex = (lcp,list_velocity_angle,list_distance_obs)

def rad2deg(inp):
	return inp*180.0/np.pi
def deg2rad(inp):
	return inp/180.0*np.pi



def publish_pose_thread():
	global local_pos_pub,poses,init_position,uav_marker,uav_marker_publisher,cur_pose,uav_goal_marker_publisher,uav_goal_marker
	init_position = False
	last_request = rospy.Time.now()
	while(not init_position):
		pass
	while(True):
		if(rospy.Time.now()-last_request > rospy.Duration(0.2)):
			#print "published"
			last_request = rospy.Time.now()
			local_pos_pub.publish(poses)
			uav_marker.pose = cur_pose.pose
			uav_marker.pose.position.z = 2
			uav_marker_publisher.publish(uav_marker)
			uav_goal_marker_publisher.publish(uav_goal_marker)
'''
def calculate_and_publish_wall(publisher,path_publisher,idx):
	global list_angle_and_distance_obs, diagram, list_velocity_angle, list_distance_obs, list_obstacle_vertex, target, sub_target, sub_target_heading, vController, list_current_position,usingvelocitycontrol
	last_request = rospy.Time.now()
	global_planning_method = None #"Theta_star"
	local_walls = []
	list_of_cells = []
	list_of_cell_path = []
	while (not usingvelocitycontrol):
		pass
	while(True):
		if(rospy.Time.now()-last_request > rospy.Duration(0.2)):
			last_request = rospy.Time.now()
			if(global_planning_method == "A_star") or (global_planning_method == "Theta_star"):
				lado = list_angle_and_distance_obs
				if(len(lado[1]) > 0):
					
					Astar_path = GridCells()
					Astar_path.header.frame_id = "world"
					Astar_path.header.stamp = rospy.Time.now()
					Astar_path.cell_width = 1.0
					Astar_path.cell_height = 1.0

					Astar_grid = GridCells()
					Astar_grid.header.frame_id = "world"
					Astar_grid.header.stamp = rospy.Time.now()
					Astar_grid.cell_width = 1.0
					Astar_grid.cell_height = 1.0
					
					quat_orient = lado[0]
					quat_arr = np.array([quat_orient.orientation.x,quat_orient.orientation.y,quat_orient.orientation.z,quat_orient.orientation.w])
					att = euler_from_quaternion(quat_arr,'sxyz')
					#print "pitch",att[1]*180.0/np.pi

					
					posex = lado[0].position.x
					posey = lado[0].position.y
					
					for i in range(len(lado[1])):
						for j in range(len(lado[1][i])):

							cell_float_x = posex+lado[2][i][j]*np.cos(att[2]+lado[1][i][j])*np.cos(att[1])
							cell_float_y = posey+lado[2][i][j]*np.sin(att[2]+lado[1][i][j])*np.cos(att[1])
							the_cells = Point()
							the_cells.x = cell_float_x#int(np.round(cell_float_x))
							the_cells.y = cell_float_y#int(np.round(cell_float_y))
							list_of_cells.append(the_cells) 
							if((the_cells.x,the_cells.y) not in local_walls):
								if(abs(att[0]) > deg2rad(3)) and (abs(att[1]) > deg2rad(3)):
									local_walls.append((the_cells.x,the_cells.y))

							#cell_float_x = posex+(lado[2][i][j]-1)*np.cos(att[2]+lado[1][i][j])*np.cos(att[1])
							#cell_float_y = posey+(lado[2][i][j]-1)*np.sin(att[2]+lado[1][i][j])*np.cos(att[1])
							#the_cells = Point()
							#the_cells.x = int(np.round(cell_float_x))
							#the_cells.y = int(np.round(cell_float_y))
							#list_of_cells.append(the_cells) 
							#if((the_cells.x,the_cells.y) not in local_walls):
							#	local_walls.append((the_cells.x,the_cells.y))

					Astar_grid.cells = list_of_cells
					diagram.walls = local_walls
					
					if(global_planning_method == "A_star"): 
						print "walls",local_walls
						start_plan = (int(posex),int(posey))
						print "start_plan", start_plan
						goal_plan = (29,29)
						came_from, cost_so_far, priority, cost, heu = a_star_search(diagram, start_plan, goal_plan)
						path=reconstruct_path(came_from, start=start_plan, goal=goal_plan)
						print "path",path
						list_of_cell_path = []
						for i in range(len(path)):
							the_cells = Point()
							the_cells.x = path[i][0]
							the_cells.y = path[i][1]
							the_cells.z = -0.8
							list_of_cell_path.append(the_cells)
						Astar_path.cells = list_of_cell_path
						path_publisher.publish(Astar_path)
						publisher.publish(Astar_grid)
					elif(global_planning_method == "Theta_star"):
						#find the vertex
						goal_plan = (29,29)
						list_of_vertex = []
						list_of_cell_path = []
						for (x,y) in local_walls:
							neighbor_of_i = [(x-1,y-1),(x-1,y),(x-1,y+1),(x,y+1),(x,y-1),(x+1,y+1),(x+1,y),(x+1,y-1)]
							num_of_neighbor = 0
							for elem in neighbor_of_i:
								if elem in local_walls:
									num_of_neighbor+=1
							if(num_of_neighbor < 5):
								list_of_vertex.append((x,y))
								the_cells = Point()
								the_cells.x = x
								the_cells.y = y
								the_cells.z = -0.8
								list_of_cell_path.append(the_cells)
						list_of_vertex.append(goal_plan)
						the_cells = Point()
						the_cells.x = goal_plan[0]
						the_cells.y = goal_plan[1]
						the_cells.z = -0.8
						list_of_cell_path.append(the_cells)
						Astar_path.cells = list_of_cell_path
						path_publisher.publish(Astar_path)
						publisher.publish(Astar_grid)
						#print "list of vertex",list_of_vertex
						vController.setHeadingTarget(deg2rad(45.0))
						vController.setTarget(target)
						#find the list of vertex with minimum distance to goal
						
				else:
					Astar_grid = GridCells()
					Astar_grid.header.frame_id = "world"
					Astar_grid.header.stamp = rospy.Time.now()
					Astar_grid.cell_width = 1.0
					Astar_grid.cell_height = 1.0
					Astar_grid.cells = list_of_cells
					diagram.walls = []
					publisher.publish(Astar_grid)

					Astar_path = GridCells()
					Astar_path.header.frame_id = "world"
					Astar_path.header.stamp = rospy.Time.now()
					Astar_path.cell_width = 1.0
					Astar_path.cell_height = 1.0
					Astar_path.cells = list_of_cell_path
					diagram.walls = []
					path_publisher.publish(Astar_path)
			elif(global_planning_method == "Simple_VO"):
				lov = list_obstacle_vertex
				if(distance2D((target.position.x,target.position.y),(list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y)) < 2.0):
					vController.setHeadingTarget(deg2rad(45.0))
					vController.setTarget(target)
				elif(len(lov[2]) > 0):
					Thetastar_path = GridCells()
					Thetastar_path.header.frame_id = "world"
					Thetastar_path.header.stamp = rospy.Time.now()
					Thetastar_path.cell_width = 1.0
					Thetastar_path.cell_height = 1.0
					clearence = 2.0
					pos_uav = lov[0].position
					heading_to_goal_target = np.arctan2(target.position.y-pos_uav.y,target.position.x-pos_uav.x)
					list_sub_goal_target = []
					is_heading_to_target_obstructed = False
					for i in range(len(lov[2])):

						r = lov[2][i][2]
						if(abs(r) > 0.1):
							clearence_angle_left = np.arccos((2*r*r-clearence*clearence)/(2*r*r))
						else:
							clearence_angle_left = deg2rad(10.0)
						r = lov[2][i][0]
						if(abs(r) > 0.1):
							clearence_angle_right = np.arccos((2*r*r-clearence*clearence)/(2*r*r))
						else:
							clearence_angle_right = deg2rad(10.0)
						quat_orient = lov[0]
						
						quat_arr = np.array([quat_orient.orientation.x,quat_orient.orientation.y,quat_orient.orientation.z,quat_orient.orientation.w])
						att = euler_from_quaternion(quat_arr,'sxyz')
						theta_ort_left = att[2]+lov[1][i][1]+clearence_angle_left
						theta_ort_right = att[2]+lov[1][i][0]-clearence_angle_right
						#print "clearence",rad2deg(clearence_angle_left),rad2deg(clearence_angle_right)
						print "compare heading", rad2deg(theta_ort_left),rad2deg(heading_to_goal_target),rad2deg(theta_ort_right)
						if locplan.in_between(theta_ort_right,heading_to_goal_target,theta_ort_left):
							is_heading_to_target_obstructed = True
						list_sub_goal_target.append((pos_uav.x+r*np.cos(theta_ort_left),pos_uav.y+r*np.sin(theta_ort_left)))
						list_sub_goal_target.append((pos_uav.x+r*np.cos(theta_ort_right),pos_uav.y+r*np.sin(theta_ort_right)))

						if(is_heading_to_target_obstructed): #direct heading to goal obstruct by obstacle
							print "create sub goal Theta*"
							list_distance_sub_goal_to_the_goal = []
							for i in range(len(list_sub_goal_target)):
								list_distance_sub_goal_to_the_goal.append(distance2D(list_sub_goal_target[i],(target.position.x,target.position.y)))
							min_idx = np.argmin(list_distance_sub_goal_to_the_goal)
							sub_target = Pose()
							sub_target.position.x = list_sub_goal_target[min_idx][0]
							sub_target.position.y = list_sub_goal_target[min_idx][1]
							sub_target.position.z = target.position.z
							print "sub_target",sub_target.position.x,sub_target.position.y
							sub_target_heading = np.arctan2(list_sub_goal_target[min_idx][1]-pos_uav.y,list_sub_goal_target[min_idx][0]-pos_uav.x)
							vController.setHeadingTarget(sub_target_heading)
							vController.setTarget(sub_target)
						else: # go directly to the target
							#print "goes to the goal target directly"
							sub_target = Pose()
							sub_target.position.x = target.position.x
							sub_target.position.y = target.position.y
							sub_target.position.z = target.position.z
							sub_target_heading = heading_to_goal_target
							vController.setHeadingTarget(sub_target_heading)
							vController.setTarget(sub_target)
						the_cells = Point()
						the_cells.x = sub_target.position.x
						the_cells.y = sub_target.position.y
						Thetastar_path.cells = [the_cells]
						path_publisher.publish(Thetastar_path)
				else:
					Thetastar_path = GridCells()
					Thetastar_path.header.frame_id = "world"
					Thetastar_path.header.stamp = rospy.Time.now()
					Thetastar_path.cell_width = 1.0
					Thetastar_path.cell_height = 1.0
					Thetastar_path.cells = []
					path_publisher.publish(Thetastar_path)
'''
def update_uav_marker(marker,pos):
  temp_marker = marker
  temp_marker.pose.orientation.w = pos[3]
  temp_marker.pose.position.x = pos[0]
  temp_marker.pose.position.y = pos[1]
  temp_marker.pose.position.z = pos[2]

  return temp_marker

def update_uav_goal_marker(marker,pos):
  temp_marker = marker
  temp_marker.pose.orientation.w = pos[3]
  temp_marker.pose.position.x = pos[0]
  temp_marker.pose.position.y = pos[1]
  temp_marker.pose.position.z = pos[2]

  return temp_marker

def key_function():
	global usingvelocitycontrol,xvel,yvel
	while(True):
		if(usingvelocitycontrol):
			get()

def distance3D(a,b):
	#return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)
	a = (a.position.x,a.position.y,a.position.z)
	b = (b.position.x,b.position.y,b.position.z)
	return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

def distance2D(a,b):
	#return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)
	a = (a.position.x,a.position.y)
	b = (b.position.x,b.position.y)
	return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def PID_update(target,state,time):
	error = target-state

def vController_update(goal,curr):
	time = curr[3]
	outpur = TwistStamped()
	linear = Vector3()
	linear.x = PID_update(goal[0],curr[0],time)
	linear.x = PID_update(goal[1],curr[1],time)
	linear.x = PID_update(goal[2],curr[2],time)
	output.twist = Twist()
	output.twist.linear = linear
	return 

def publish_uav_position_rviz(br,x,y,z,uav_idx):
	br.sendTransform((x,y,z),
	              (0.0, 0.0, 0.0, 1.0),
	              rospy.Time.now(),
	              "uav_tf_"+uav_idx,
	              "world")

def rrt_changer_thread():
	global rrt_path,replanning_rrt
	while(len(rrt_path) == 0):
		pass
	while(True):
		for i in range(0,len(rrt_path)-2):
			if freeObstacle(rrt_path[i],rrt_path[i+1]):
				print "Replanning RRT"
				replanning_rrt = True
				break

def LOS_thread():
	global rrt_path,walls,cur_pose,poses,line_los,line_los_publisher,target_init
	global line_los_left,line_los_left_publisher,line_los_right,line_los_right_publisher,global_los,set_mode_client,offb_set_mode
	TRIANGLE = 1
	RECTANGLE = 2
	los_type = RECTANGLE
	rrt_path = []
	last_request = rospy.Time.now()
	while(len(rrt_path) == 0):
		pass
	print "LOS thread passed"
	while(True):
		if(rospy.Time.now()-last_request > rospy.Duration(0.1)):
			last_request = rospy.Time.now()
			if(not LineOfSight(walls,(cur_pose.pose.position.x,cur_pose.pose.position.y),(poses.pose.position.x,poses.pose.position.y))):
				print "Current LOS is not LOS"
				if global_los:
					global_los = False
					hold_pose = cur_pose
					poses.pose.position.x = hold_pose.pose.position.x
					poses.pose.position.y = hold_pose.pose.position.y
					poses.pose.position.z = target_init.position.z
				#offb_set_mode = set_mode_client(0,'AUTO.LOITER')
				#if(offb_set_mode.mode_sent):
				#	rospy.loginfo("LOITER ENABLED")
			if(len(rrt_path)>0):
				rrt_path_reverse = rrt_path[:]
				rrt_path_reverse.reverse()
				#print "rpr",rrt_path_reverse
				is_los = False
				for i in rrt_path_reverse:
					if(LineOfSight(walls,(cur_pose.pose.position.x,cur_pose.pose.position.y),i)):
						poses.pose.position.x = i[0]
						poses.pose.position.y = i[1]
						poses.pose.position.z = target_init.position.z
						#q=quaternion_from_euler(0,0,atan2((poses.pose.position.y-cur_pose.pose.position.y),(poses.pose.position.x-cur_pose.pose.position.x)))
						poses.pose.orientation.x = 0.0#q[0]
						poses.pose.orientation.y = 0.0#q[1]
						poses.pose.orientation.z = 0.0#q[2]
						poses.pose.orientation.w = 1.0#q[3]
						is_los = True
						global_los = True
						break

					'''
					if(los_type == TRIANGLE):
						LOSWC_flag,bound_left,bound_center,bound_right,sub_goal=LineOfSightWithClearance(walls,(cur_pose.pose.position.x,cur_pose.pose.position.y),i,LOS_SAFETY)
					else:
						LOSWC_flag,bound_left1,bound_left2,bound_right1,bound_right2,sub_goal=LineOfSightWithClearanceRectangle(walls,(cur_pose.pose.position.x,cur_pose.pose.position.y),i,LOS_SAFETY)
					print LOSWC_flag
					if(LOSWC_flag):
						#print "LOOOOOOOOOOOOOS"
						poses.pose.position.x = sub_goal[0]
						poses.pose.position.y = sub_goal[1]
						poses.pose.position.z = target_init.position.z
						#q=quaternion_from_euler(0,0,atan2((poses.pose.position.y-cur_pose.pose.position.y),(poses.pose.position.x-cur_pose.pose.position.x)))
						poses.pose.orientation.x = 0.0#q[0]
						poses.pose.orientation.y = 0.0#q[1]
						poses.pose.orientation.z = 0.0#q[2]
						poses.pose.orientation.w = 1.0#q[3]
						is_los = True
						global_los = True
						break
					'''
				#print "is LOS (in thread)?",is_los
				if(not is_los):
					if global_los:
						hold_pose = cur_pose
						poses.pose.position.x = hold_pose.pose.position.x
						poses.pose.position.y = hold_pose.pose.position.y
						poses.pose.position.z = target_init.position.z
						#offb_set_mode = set_mode_client(0,'AUTO.LOITER')
						#if(offb_set_mode.mode_sent):
						#	rospy.loginfo("LOITER ENABLED")
						global_los = False
			if(los_type == TRIANGLE):
				line_los.points = [0,0]
				p_rrt = Point()
				p_rrt.x = cur_pose.pose.position.x
				p_rrt.y = cur_pose.pose.position.y
				p_rrt.z = 0.0
				line_los.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = bound_center[0]
				p_rrt.y = bound_center[1]
				p_rrt.z = 0.0
				line_los.points[1] = p_rrt
				

				#c = 2.0
				line_los_left.points = [0,0]
				p_rrt = Point()
				p_rrt.x = cur_pose.pose.position.x
				p_rrt.y = cur_pose.pose.position.y
				p_rrt.z = 0.0
				line_los_left.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = bound_left[0]
				p_rrt.y = bound_left[1]
				p_rrt.z = 0.0
				line_los_left.points[1] = p_rrt
				

				line_los_right.points = [0,0]
				p_rrt = Point()
				p_rrt.x = cur_pose.pose.position.x
				p_rrt.y = cur_pose.pose.position.y
				p_rrt.z = 0.0
				line_los_right.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = bound_right[0]
				p_rrt.y = bound_right[1]
				p_rrt.z = 0.0
				line_los_right.points[1] = p_rrt
			else:
				line_los.points = [0,0]
				p_rrt = Point()
				p_rrt.x = cur_pose.pose.position.x
				p_rrt.y = cur_pose.pose.position.y
				p_rrt.z = 0.0
				line_los.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = i[0]#sub_goal[0]
				p_rrt.y = i[1]#sub_goal[1]
				p_rrt.z = 0.0
				line_los.points[1] = p_rrt
				'''
				line_los_left.points = [0,0]
				p_rrt = Point()
				p_rrt.x = bound_left1[0]
				p_rrt.y = bound_left1[1]
				p_rrt.z = 0.0
				line_los_left.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = bound_left2[0]
				p_rrt.y = bound_left2[1]
				p_rrt.z = 0.0
				line_los_left.points[1] = p_rrt

				line_los_right.points = [0,0]
				p_rrt = Point()
				p_rrt.x = bound_right1[0]
				p_rrt.y = bound_right1[1]
				p_rrt.z = 0.0
				line_los_right.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = bound_right2[0]
				p_rrt.y = bound_right2[1]
				p_rrt.z = 0.0
				line_los_right.points[1] = p_rrt
				'''


			line_los_publisher.publish(line_los)
			line_los_left_publisher.publish(line_los_left)
			line_los_right_publisher.publish(line_los_right)

#RRT
def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def dist_point_to_line(l1,l2,p):
	return np.abs((l2[1]-l1[1])*p[0]-(l2[0]-l1[0])*p[1]+l2[0]*l1[1]-l2[1]*l1[0])/((l2[1]-l1[1])**2+(l2[0]-l1[0])**2)**0.5

def freeObstacle(nn,newnode):
	global current_map, map_size, map_res
	x1 = (nn[0]+map_size/2)/map_res
	y1 = (nn[1]+map_size/2)/map_res
	x2 = (newnode[0]+map_size/2)/map_res
	y2 = (newnode[1]+map_size/2)/map_res
	'''
	max_x = max(x1,x2)
	max_y = max(y1,y2)
	min_x = min(x1,x2)
	min_y = min(y1,y2)
	'''
	#r = jari2, (a,b) = center lingkaran
	r = 1.0	
	safety_grid = 0
	max_x = int(np.ceil(max(x1,x2)/map_res)*map_res+safety_grid)
	max_y = int(np.ceil(max(y1,y2)/map_res)*map_res+safety_grid)
	min_x = int(np.floor(min(x1,x2)/map_res)*map_res-safety_grid)
	min_y = int(np.floor(min(y1,y2)/map_res)*map_res-safety_grid)
	#print "cmap",current_map
	#print x1,y1,x2,y2
	#print min_x,max_x,min_y,max_y
	for i in range(min_x,max_x):
		for j in range(min_y,max_y):
			try:
				#if(current_map[j,i] == 100):
				#	if(dist_point_to_line((min_x,min_y),(max_x,max_y),(i,j)) < 20):
				#		return False
				if current_map[j,i] == 100:
					return False
			except:
				pass
	
	if min_x == max_x:
		for j in range(min_y,max_y):
			try:
				if current_map[j,min_x] == 100:
					return False
			except:
				pass

	if min_y == max_y:
		for i in range(min_x,max_x):
			try:
				if current_map[min_y,i] == 100:
					return False
			except:
				pass
	'''rree
	for i in range(current_map.shape[0]):
		for j in range(current_map.shape[1]):
			print i,j
			if current_map[i,j] == 100:
				#print i,j,"black"
				a = i*map_res - 51.2
				b = j*map_res - 51.2
				#print "Obstacle Free", (a,b)
				if not((b-r) > max_y) or ((a-r) > max_x) or ((b+r) < min_y) or ((a+r) < min_x):
					return False
				d=np.linalg.norm(np.cross((x1-x2,y1-y2),(x2-a,y2-b)))/np.linalg.norm((x1-x2,y1-y2))
				#print d
				if(d <= r):
					return False
	'''
	return True

def onSegment(p,q,r):
	if((q[0] <= max(p[0],r[0])) and (q[0] >= min(p[0],r[0])) and (q[1] <= max(p[1],r[1])) and (q[1] >= min(p[1],r[1]))):
		return True
	return False

def orientation(p,q,r):
	val = (q[1]-p[1])*(r[0]-q[0])-(q[0]-p[0])*(r[1]-q[1])
	if val == 0:
		return 0
	if val>0:
		return 1
	else:
		return 2 

def doIntersect(p1,q1,p2,q2):
	o1 = orientation(p1,q1,p2)
	o2 = orientation(p1,q1,q2)
	o3 = orientation(p2,q2,p1)
	o4 = orientation(p2,q2,q1)

	if ((o1 != o2) and (o3 != o4)):
		return True

	if ((o1 == 0) and onSegment(p1,p2,q1)):
		return True

	if ((o2 == 0) and onSegment(p1,q2,q1)):
		return True

	if ((o3 == 0) and onSegment(p2,p1,q2)):
		return True

	if ((o4 == 0) and onSegment(p2,q1,q2)):
		return True

	return False

def isInside(polygon,p):
    extreme = (999999,p[1])
    i = 0
    count = 0
    n = len(polygon)
    while(True):
        nxt = (i+1)%n  
        if(doIntersect(polygon[i],polygon[nxt],p,extreme)):
            if(orientation(polygon[i],p,polygon[nxt]) == 0):
                return onSegment(polygon[i],p,polygon[nxt])
            count=count+1
        i=nxt
        if(i==0):
            break

    return count&1

def freeObstacleWithClearance(nn,newnode,c):
	global current_map, map_size, map_res
	x0 = (nn[0]+map_size/2)/map_res
	y0 = (nn[1]+map_size/2)/map_res
	x1 = (newnode[0]+map_size/2)/map_res
	y1 = (newnode[1]+map_size/2)/map_res
	c = c/map_res
	theta = atan2(y1-y0,x1-x0)
	theta1 = atan2(y0-y1,x0-x1)
	d = dist((x0,y0),(x1,y1))
	dc = d*c
	x1_temp = x1
	y1_temp = y1
	x1 = x0+dc*cos(theta) 
	y1 = y0+dc*sin(theta) 
	x0 = x1_temp+dc*cos(theta1) 
	y0 = y1_temp+dc*sin(theta1) 
	#A
	x2 = x0+c*sin(theta)
	y2 = y0-c*cos(theta)
	#B
	x3 = x1+c*sin(theta)
	y3 = y1-c*cos(theta)
	#D
	x4 = x0-c*sin(theta)
	y4 = y0+c*cos(theta)
	#C
	x5 = x1-c*sin(theta)
	y5 = y1+c*cos(theta)

	max_x = int(np.ceil(max(x2,x3,x4,x5)/map_res)*map_res)
	max_y = int(np.ceil(max(y2,y3,y4,y5)/map_res)*map_res)
	min_x = int(np.floor(min(x2,x3,x4,x5)/map_res)*map_res)
	min_y = int(np.floor(min(y2,y3,y4,y5)/map_res)*map_res)

	#AB = (x3-x2,y3-y2)
	#BC = (x5-x2,y5-y2)
	#print "A",x2,y2,"B",x3,y3,"C",x5,y5
	polygon = [(x2,y2),(x3,y3),(x5,y5),(x4,y4)]
	for i in range(min_x,max_x):
		for j in range(min_y,max_y):
			try:
				#M = (j,i)
				#AM = (j-x2,i-y2)
				#BM = (j-x3,i-y3)
				#if 0 <= np.dot(AB,AM) <= np.dot(AB,AB) and 0 <= np.dot(BC,BM) <= np.dot(BC,BC):
				if(current_map[j,i] == 100):
					if (isInside(polygon,(i,j))):
						return False
			except:
				pass
	
	return True

def reconstruct_path(came_from, start, goal_local):
    current = goal_local
    path = []
    while current != start:
        path.append(current)
        #print 'c',current
        current = came_from[current]
    path.append(start) # optional
    #path.reverse() # optional
    return path

#def LineOfSightWithClearanceRectangle(nn,newnode,c):
def LineOfSightWithClearanceRectangle(walls,s1,s2,c):
	global map_res,map_size
	theta = atan2(s2[1]-s1[1],s2[0]-s1[0])
	theta1 = atan2(s1[1]-s2[1],s1[0]-s2[0])
	d = dist(s1,s2)

	if c > d:
		c = d
	theta_ort = asin(c/d)
	#s2_min = (s2[0]+c/sin(theta),s2[1])
	#s2_max = (s2[0],s2[1]+c/cos(theta))
	dc = d+c
	theta_ort_left = theta+theta_ort # sudut RVO sebelah kiri
	bound_left2 = (s1[0]+dc*cos(theta_ort_left), s1[1]+dc*sin(theta_ort_left))
	theta_ort_right = theta-theta_ort # sudut RVO sebelah kanan
	bound_right2 = (s1[0]+dc*cos(theta_ort_right), s1[1]+dc*sin(theta_ort_right))
	bound_center = (s1[0]+dc*cos(theta), s1[1]+dc*sin(theta))
	#print bound_left,bound_right
	theta_ort_left = theta1-theta_ort # sudut RVO sebelah kiri	
	bound_left1 = (s2[0]+dc*cos(theta_ort_left), s2[1]+dc*sin(theta_ort_left))
	theta_ort_right = theta1+theta_ort # sudut RVO sebelah kanan
	bound_right1 = (s2[0]+dc*cos(theta_ort_right), s2[1]+dc*sin(theta_ort_right))

	if(LineOfSight(walls,bound_left1,bound_left2) and LineOfSight(walls,s1,bound_center) and LineOfSight(walls,bound_right1,bound_right2)):
		return True,bound_left1,bound_left2,bound_right1,bound_right2,s2
	else:
		return False,bound_left1,bound_left2,bound_right1,bound_right2,s2
	'''
	global current_map, map_size, map_res
	x0 = (nn[0]+map_size/2)/map_res
	y0 = (nn[1]+map_size/2)/map_res
	x1 = (newnode[0]+map_size/2)/map_res
	y1 = (newnode[1]+map_size/2)/map_res
	c = c/map_res
	theta = atan2(y1-y0,x1-x0)
	#A
	x2 = x0+c*sin(theta)
	y2 = y0-c*cos(theta)
	#B
	x3 = x1+c*sin(theta)
	y3 = y1-c*cos(theta)
	#D
	x4 = x0-c*sin(theta)
	y4 = y0+c*cos(theta)
	#C
	x5 = x1-c*sin(theta)
	y5 = y1+c*cos(theta)

	max_x = int(np.ceil(max(x2,x3,x4,x5)/map_res)*map_res)
	max_y = int(np.ceil(max(y2,y3,y4,y5)/map_res)*map_res)
	min_x = int(np.floor(min(x2,x3,x4,x5)/map_res)*map_res)
	min_y = int(np.floor(min(y2,y3,y4,y5)/map_res)*map_res)

	#AB = (x3-x2,y3-y2)
	#BC = (x5-x2,y5-y2)
	#print "A",x2,y2,"B",x3,y3,"C",x5,y5
	polygon = [(x2,y2),(x3,y3),(x5,y5),(x4,y4)]
	for i in range(min_x,max_x):
		for j in range(min_y,max_y):
			try:
				#M = (j,i)
				#AM = (j-x2,i-y2)
				#BM = (j-x3,i-y3)
				#if 0 <= np.dot(AB,AM) <= np.dot(AB,AB) and 0 <= np.dot(BC,BM) <= np.dot(BC,BC):
				if(current_map[j,i] == 100):
					if (isInside(polygon,(i,j))):
						x2=int(np.ceil((x2+map_size/2)/map_res))
						y2=int(np.ceil((y2+map_size/2)/map_res))
						x3=int(np.ceil((x3+map_size/2)/map_res))
						y3=int(np.ceil((y3+map_size/2)/map_res))
						x4=int(np.ceil((x4+map_size/2)/map_res))
						y4=int(np.ceil((y4+map_size/2)/map_res))
						x5=int(np.ceil((x5+map_size/2)/map_res))
						y5=int(np.ceil((y5+map_size/2)/map_res))
						return False,(x2,y2),(x3,y3),(x4,y4),(x5,y5),newnode
			except:
				pass
	
	x2=int(np.ceil((x2+map_size/2)/map_res))
	y2=int(np.ceil((y2+map_size/2)/map_res))
	x3=int(np.ceil((x3+map_size/2)/map_res))
	y3=int(np.ceil((y3+map_size/2)/map_res))
	x4=int(np.ceil((x4+map_size/2)/map_res))
	y4=int(np.ceil((y4+map_size/2)/map_res))
	x5=int(np.ceil((x5+map_size/2)/map_res))
	y5=int(np.ceil((y5+map_size/2)/map_res))
	return True,(x2,y2),(x3,y3),(x4,y4),(x5,y5),newnode
	'''

def LineOfSightWithClearance(walls,s1,s2,c):
	global map_res,map_size
	theta = atan2(s2[1]-s1[1],s2[0]-s1[0])
	d = dist(s1,s2)

	if c > d:
		c = d
	theta_ort = asin(c/d)
	#s2_min = (s2[0]+c/sin(theta),s2[1])
	#s2_max = (s2[0],s2[1]+c/cos(theta))
	dc = d+c
	theta_ort_left = theta+theta_ort # sudut RVO sebelah kiri
	bound_left = (s1[0]+dc*cos(theta_ort_left), s1[1]+dc*sin(theta_ort_left))
	theta_ort_right = theta-theta_ort # sudut RVO sebelah kanan
	bound_right = (s1[0]+dc*cos(theta_ort_right), s1[1]+dc*sin(theta_ort_right))
	bound_center = (s1[0]+dc*cos(theta), s1[1]+dc*sin(theta))
	#print bound_left,bound_right
	
	if(LineOfSight(walls,s1,bound_left) and LineOfSight(walls,s1,bound_center) and LineOfSight(walls,s1,bound_right)):
		return True,bound_left,bound_center,bound_right,s2
	else:
		return False,bound_left,bound_center,bound_right,s2
	

	'''
	s1_x=int(np.ceil((s1[0]+map_size/2)/map_res))
	s1_y=int(np.ceil((s1[1]+map_size/2)/map_res))
	bound_left_x=int(np.ceil((bound_left[0]+map_size/2)/map_res))
	bound_left_y=int(np.ceil((bound_left[1]+map_size/2)/map_res))
	bound_right_x=int(np.ceil((bound_right[0]+map_size/2)/map_res))
	bound_right_y=int(np.ceil((bound_right[1]+map_size/2)/map_res))
	polygon = [(s1_x,s1_y),(bound_left_x,bound_left_y),(bound_right_x,bound_right_y)]
	
	for i in walls:
		if isInside(polygon,i):
			return False,bound_left,bound_center,bound_right,s2
	return True,bound_left,bound_center,bound_right,s2
	'''
def Replanning_RRT():
	global goal,rrt_marker,rrt_marker_publisher,map_size,sub_goal_marker,sub_goal_marker_pblisher,rrt_path
	nodes = [(goal.pose.position.x,goal.pose.position.y)]
	rrt_marker.points = []
	#for i in range(NUMNODES):
	i=0
	print "Calculate RRT Interrupt"
	nodes_and_parent = []
	came_from = {}
	came_from[nodes[0]]=None
	path_found = False
	while (i<NUMNODES) and (not path_found):	
		rand = (random.random()-0.5)*map_size, (random.random()-0.5)*map_size
		nn = nodes[0]
		for p in nodes:
			if dist(p,rand) < dist(nn,rand):
				nn =p
		newnode = step_from_to(nn,rand)

		#if(freeObstacle(nn,newnode)):
		if(freeObstacleWithClearance(nn,newnode,RRT_SAFETY)):
			nodes.append(newnode)
			nodes_and_parent.append((nn,newnode))
			came_from[newnode]=nn
			p_rrt = Point()
			p_rrt.x=nn[0]
			p_rrt.y=nn[1]
			p_rrt.z=0.0
			rrt_marker.points.append(p_rrt)
			p_rrt = Point()
			p_rrt.x=newnode[0]
			p_rrt.y=newnode[1]
			p_rrt.z=0.0
			rrt_marker.points.append(p_rrt)
			if(dist(newnode,(cur_pose.pose.position.x,cur_pose.pose.position.y))<1.0):
				rrt_path=reconstruct_path(came_from,nodes[0],nodes[-1])
				sub_goal_marker.points = []
				for i in rrt_path:
					p_rrt = Point()
					p_rrt.x = i[0]
					p_rrt.y = i[1]
					p_rrt.z = 0.0
					sub_goal_marker.points.append(p_rrt)
				sub_goal_marker_publisher.publish(sub_goal_marker)
				#rrt_marker.header.frame_id=map_data.header.frame_id
				rrt_marker_publisher.publish(rrt_marker)
				return True
				print "Path found"
			i=i+1
		else:
			i=i+1
	return False

def LineOfSight(walls,s1,s2):
    global map_size,free_occ,current_map,map_res
    #print "masuk los",walls
    x0=int(np.ceil((s1[0]+map_size/2)/map_res))
    y0=int(np.ceil((s1[1]+map_size/2)/map_res))
    x1=int(np.ceil((s2[0]+map_size/2)/map_res))
    y1=int(np.ceil((s2[1]+map_size/2)/map_res))
    #print (x1,y1)
    #print free_occ
    #if current_map[y1,x1] == -1:
    #    return False
    #print x0,y0,x1,y1
    dy=y1-y0
    dx=x1-x0
    f=0
    if(dy < 0):
        dy=-dy
        sy=-1
    else:
        sy=1
    if(dx < 0):
        dx=-dx
        sx=-1
    else:
        sx=1
    if(dx >= dy):
        while (x0 != x1):
            f=f+dy
            if(f >= dx):
                if ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                    return False
                y0 = y0+sy
                f = f-dx
            if (f != 0) and ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                return False
            if (dy == 0) and ((x0+((sx-1)/2),y0) in walls) and ((x0+((sx-1)/2),y0-1) in walls):
                return False
            x0=x0+sx
    else:
        while (y0 != y1):
            f=f+dx
            if(f >= dy):
                if ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                    return False
                x0 = x0+sx
                f = f-dy
            if (f != 0) and ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                return False
            if (dx == 0) and ((x0,y0+((sy-1)/2)) in walls) and ((x0-1,y0+((sy-1)/2)) in walls):
                return False
            y0=y0+sy
	
    return True

def main(args):
	global list_current_position, list_current_velocity, current_position, usingvelocitycontrol,usingpositioncontrol,xvel,yvel,state,cur_pose,list_error,target
	global global_obs_detected, list_velocity_angle, vController, current_state, map_size, map_data, current_map,map_flag,walls
	global local_pos_pub,poses,init_position,uav_marker,uav_marker_publisher,rrt_path,line_los,line_los_publisher,target_init,cur_pose0
	global safety_marker,safety_marker_publisher,free_occ,map_res,replanning_rrt
	global line_los_left,line_los_left_publisher,line_los_right,line_los_right_publisher
	global uav_goal_marker,uav_goal_marker_publisher,goal,goal_changed_start,global_los,set_mode_client
	global rrt_marker,rrt_marker_publisher,sub_goal_marker, sub_goal_marker_publisher
	global cost_map,cost_map_publisher
	goal_changed_start = True
	global_los = True
	goal_changed = True
	replanning_rrt = True
	free_occ = []
	cur_pose = PoseStamped()
	cur_pose0 = PoseStamped()
	map_flag = False
	rrt_mode = False
	map_data = OccupancyGrid()

	map_array = np.array(map_data.data)
	current_map=map_array.reshape((map_data.info.width,map_data.info.height))
	map_size = 51.2
	#par.CurrentIteration = 1
	if(len(args) > 1):
		uav_ID = str(args[1])
		ID = int(args[1])
	else:
		uav_ID = "0"
		ID = 0
	idx = int(uav_ID)-1
	rospy.init_node("agent_"+uav_ID)
	print "UAV"+uav_ID
	#vController = VelocityController()
	try:
		rospy.wait_for_service("uav_"+uav_ID+"/mavros/cmd/arming")
		rospy.wait_for_service("uav_"+uav_ID+"/mavros/set_mode")
	except rospy.ROSException:
		fail("failed to connect to service")
	print "test1"
	state_sub=rospy.Subscriber("uav_"+uav_ID+"/mavros/state", State,queue_size=10,callback=state_cb)
	map_sub=rospy.Subscriber("uav_"+uav_ID+"/map",OccupancyGrid,queue_size=10,callback=map_cb)
	local_vel_pub=rospy.Publisher("uav_"+uav_ID+"/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
	local_pos_pub=rospy.Publisher("uav_"+uav_ID+"/mavros/setpoint_position/local",PoseStamped,queue_size=10)
	local_pos_target=rospy.Publisher("uav_"+uav_ID+"/mavros/setpoint_raw/local",PositionTarget,queue_size=10)
	atittude_pub = rospy.Publisher("uav_"+uav_ID+"/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=10)
	thr_pub = rospy.Publisher("uav_"+uav_ID+"/mavros/setpoint_attitude/att_throttle",Float64,queue_size=10)
	#Astar_grid_pub = rospy.Publisher("/Astar_grid",GridCells,queue_size=10)
	#Astar_path_pub = rospy.Publisher("/Astar_path",GridCells,queue_size=10)
	arming_client = rospy.ServiceProxy("uav_"+uav_ID+"/mavros/cmd/arming",CommandBool)
	set_mode_client = rospy.ServiceProxy("uav_"+uav_ID+"/mavros/set_mode",SetMode)
	#for i in range(4):
		#velocity_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mavros/local_position/velocity",TwistStamped,queue_size = 10,callback=velocity_cb, callback_args=i)
		#position_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped,queue_size = 10,callback=position_cb,callback_args=(i,int(uav_ID)-1))
	position_sub = rospy.Subscriber("uav_"+uav_ID+"/mavros/local_position/pose",PoseStamped,queue_size = 10,callback=position_single_cb)

	goal = PoseStamped()
	if(ID == 0):
		goal.pose.position.x = 18
		goal.pose.position.y = -15
		goal.pose.position.z = 2
	elif(ID == 1):
		goal.pose.position.x = 18
		goal.pose.position.y = 5
		goal.pose.position.z = 2
	
	position_slam_sub = rospy.Subscriber("uav_"+uav_ID+"/slam_out_pose",PoseStamped,queue_size = 10,callback=position_slam_cb)
	
	#scan_lidar_sub = rospy.Subscriber("/scan",LaserScan,queue_size=10,callback=scan_lidar_cb,callback_args=uav_ID)
	br = tf.TransformBroadcaster()
	r=rospy.Rate(10)
	print "TRY TO CONNECT"
	rospy.loginfo("Try to connect")
	while ((not rospy.is_shutdown()) and (not current_state.connected)):
		#rospy.spinOnce()
		r.sleep()
	#print(current_state.connected.__class__)
	rospy.loginfo("CURRENT STATE CONNECTED")

	poses = PoseStamped()
	#poses.pose = Pose()
	#poses.pose.position = Point()
	target = Pose()
	if(idx == 0):
		target.position.x = 0
		target.position.y = 0
	if(idx==1):
		target.position.x = 0
		target.position.y = 0
	if(idx==2):
		target.position.x = 0
		target.position.y = 0
	if(idx==3):
		target.position.x = 4
		target.position.y = 4
	target.position.z = 10
	poses.pose.position.x = target.position.x
	poses.pose.position.y = target.position.y
	poses.pose.position.z = target.position.z
	#q=quaternion_from_euler(0,0,45*np.pi/180.0)
	#poses.pose.orientation.x = q[0]
	#poses.pose.orientation.y = q[1]
	#poses.pose.orientation.z = q[2]
	#poses.pose.orientation.w = q[3]
	i = 100
	#while((not rospy.is_shutdown()) and (i>0)):
	#	local_pos_pub.publish(poses)
	#	i = i-1
	rviz_visualization_start = False
	last_request = rospy.Time.now()
	count = 0
	if(idx==1):
		target.position.x = 28
		target.position.y = 28
	if(idx==2):
		target.position.x = 0
		target.position.y = 17
	#thread1 = Thread(target = key_function)
  	#thread1.start()

	style.use('fivethirtyeight')
  	thread2 = Thread(target = publish_pose_thread)
  	thread2.start()

	thread3 = Thread(target = LOS_thread)
	thread3.start()

	#thread4 = Thread(target = rrt_changer_thread)
	#thread4.start()
  	#thread3 = Thread(target = calculate_and_publish_wall, args = (Astar_grid_pub,Astar_path_pub,idx))
  	#thread3.start()
  	#file_error_pos_x = open(dir_path+'/txt/error_pos_x.txt','w')
  	#file_error_pos_y = open(dir_path+'/txt/error_pos_y.txt','w')
  	#file_error_pos_z = open(dir_path+'/txt/error_pos_z.txt','w')

	cost_map = OccupancyGrid()
	cost_map_publisher = rospy.Publisher("uav_"+uav_ID+"/cost_map",OccupancyGrid,queue_size=10)
	cost_map.header.frame_id = "map"
	#cost_map.info.width = sizemap
	#cost_map.info.height = sizemap
	#cost_map.info.origin.position.x = -sizemap/2*og.info.resolution
	#cost_map.info.origin.position.y = -sizemap/2*og.info.resolution
	cost_map.info.origin.position.z = 0.0
	cost_map.info.origin.orientation.x = 0.0
	cost_map.info.origin.orientation.y = 0.0
	cost_map.info.origin.orientation.z = 0.0
	cost_map.info.origin.orientation.w = 1.0

  	error_time = 0
  	print poses
  	uav_color = [255,0,0,255]
	topic = "uav_"+uav_ID+"/uav_marker"
	uav_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	uav_marker = Marker()
	uav_marker.header.frame_id = "map"
	uav_marker.type = uav_marker.SPHERE
	uav_marker.action = uav_marker.ADD
	uav_marker.scale.x = 0.5
	uav_marker.scale.y = 0.5
	uav_marker.scale.z = 0.5
	uav_marker.color.r = 0.0/255.0
	uav_marker.color.g = 0.0/255.0
	uav_marker.color.b = 255.0/255.0
	uav_marker.color.a = 255.0/255.0
	#uav_marker_publisher.publish(uav_marker)

	topic = "uav_"+uav_ID+"/safety_marker"
	safety_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	safety_marker = Marker()
	safety_marker.header.frame_id = "map"
	safety_marker.type = safety_marker.CUBE_LIST
	safety_marker.action = safety_marker.ADD
	safety_marker.scale.x = 0.1
	safety_marker.scale.y = 0.1
	safety_marker.scale.z = 0.1
	safety_marker.color.r = 255.0/255.0
	safety_marker.color.g = 69.0/255.0
	safety_marker.color.b = 255.0/255.0
	safety_marker.color.a = 255.0/255.0

	topic = "uav_"+uav_ID+"/line_los"
	line_los_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	line_los = Marker()
	line_los.header.frame_id = "map"
	line_los.type = line_los.ARROW
	line_los.action = line_los.ADD
	line_los.scale.x = 0.2
	line_los.scale.y = 0.2
	line_los.scale.z = 0.2
	line_los.color.r = 255.0/255.0
	line_los.color.g = 0.0/255.0
	line_los.color.b = 255.0/255.0
	line_los.color.a = 255.0/255.0

	topic = "uav_"+uav_ID+"/line_los_left"
	line_los_left_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	line_los_left = Marker()
	line_los_left.header.frame_id = "map"
	line_los_left.type = line_los_left.ARROW
	line_los_left.action = line_los_left.ADD
	line_los_left.scale.x = 0.2
	line_los_left.scale.y = 0.2
	line_los_left.scale.z = 0.2
	line_los_left.color.r = 0.0/255.0
	line_los_left.color.g = 255.0/255.0
	line_los_left.color.b = 0.0/255.0
	line_los_left.color.a = 255.0/255.0

	topic = "uav_"+uav_ID+"/line_los_right"
	line_los_right_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	line_los_right = Marker()
	line_los_right.header.frame_id = "map"
	line_los_right.type = line_los_right.ARROW
	line_los_right.action = line_los_right.ADD
	line_los_right.scale.x = 0.2
	line_los_right.scale.y = 0.2
	line_los_right.scale.z = 0.2
	line_los_right.color.r = 0.0/255.0
	line_los_right.color.g = 0.0/255.0
	line_los_right.color.b = 255.0/255.0
	line_los_right.color.a = 255.0/255.0

	uav_color = [255,255,255,255]
	topic = "uav_"+uav_ID+"/uav_goal_marker"
	uav_goal_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	uav_goal_marker = Marker()
	uav_goal_marker.header.frame_id = "map"
	uav_goal_marker.header.stamp=rospy.Time.now()
	uav_goal_marker.id = 1
	uav_goal_marker.type = uav_goal_marker.SPHERE
	uav_goal_marker.action = uav_goal_marker.ADD
	uav_goal_marker.scale.x = 1.0#par.ws_model['robot_radius']*2
	uav_goal_marker.scale.y = 1.0#par.ws_model['robot_radius']*2
	uav_goal_marker.scale.z = 1.0#0.005*par.RealScale
	uav_goal_marker.color.r = 0.0/255.0
	uav_goal_marker.color.g = 255.0/255.0
	uav_goal_marker.color.b = 0.0/255.0
	uav_goal_marker.color.a = 1.0
	uav_goal_marker.lifetime = rospy.Duration(0)
	uav_goal_marker.pose.orientation.w = 1.0
	uav_goal_marker.pose.position.x = goal.pose.position.x#init_pos[0]
	uav_goal_marker.pose.position.y = goal.pose.position.y#init_pos[1] 
	uav_goal_marker.pose.position.z = 0#goal.pose.position.z#init_pos[2] 
	uav_goal_marker_publisher.publish(uav_goal_marker)

	goal_sub = rospy.Subscriber("uav_"+uav_ID+"/move_base_simple/goal",PoseStamped,queue_size = 10,callback=goal_cb)

	topic = "uav_"+uav_ID+"/sub_goal_marker"
	sub_goal_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	sub_goal_marker = Marker()
	sub_goal_marker.header.frame_id = "map"
	sub_goal_marker.header.stamp=rospy.Time.now()
	sub_goal_marker.id = 1
	sub_goal_marker.type = sub_goal_marker.SPHERE_LIST
	sub_goal_marker.action = sub_goal_marker.ADD
	sub_goal_marker.scale.x = 0.5#par.ws_model['robot_radius']*2
	sub_goal_marker.scale.y = 0.5#par.ws_model['robot_radius']*2
	sub_goal_marker.scale.z = 0.5#0.005*par.RealScale
	sub_goal_marker.color.r = 255.0/255.0
	sub_goal_marker.color.g = 255.0/255.0
	sub_goal_marker.color.b = 0.0/255.0
	sub_goal_marker.color.a = 1.0
	sub_goal_marker.lifetime = rospy.Duration(0)
	#sub_goal_marker.pose.orientation.w = 1.0
	#sub_goal_marker.pose.position.x = goal.pose.position.x#init_pos[0]
	#sub_goal_marker.pose.position.y = goal.pose.position.y#init_pos[1] 
	#sub_goal_marker.pose.position.z = 0#goal.pose.position.z#init_pos[2] 
	#sub_goal_marker_publisher.publish(uav_goal_marker)
	#uav_goal_marker=update_uav_marker(uav_goal_marker,(target.position.x,target.position.y,target.position.z,1.0))
	#uav_goal_marker_publisher.publish(uav_goal_marker)
	last_request_rviz = rospy.Time.now()
	#RRT config
	nodes = [(10,10)]
	rrt_topic = "uav_"+uav_ID+"/rrt_marker"
	rrt_marker_publisher = rospy.Publisher(rrt_topic, Marker,queue_size=10)
	rrt_marker = Marker()
	rrt_marker.header.frame_id = "map"
	rrt_marker.header.stamp=rospy.Time.now()
	rrt_marker.id = 1
	rrt_marker.type = rrt_marker.LINE_LIST
	rrt_marker.action = rrt_marker.ADD
	rrt_marker.scale.x = 0.1
	rrt_marker.scale.y = 0.1
	#rrt_marker.scale.z = 0.005*par.RealScale
	rrt_marker.pose.orientation.w = 1.0
	#rrt_marker.pose.position.x = 0#init_pos[0]
	#rrt_marker.pose.position.y = 0#init_pos[1] 
	#rrt_marker.pose.position.z = 0#init_pos[2] 
	rrt_marker.color.r =255.0/255.0
	rrt_marker.color.g= 0.0/255.0
	rrt_marker.color.b =0.0/255.0
	rrt_marker.color.a =1.0
	rrt_marker.lifetime = rospy.Duration(0)
	#rrt_marker_publisher.publish(rrt_marker)
	p_rrt = Point()
	last_request_rrt = rospy.Time.now()
	last_HRVO_request=rospy.Time.now()

	target_init = Pose()
	target_init.position.x = 0
	target_init.position.y = 0
	target_init.position.z = 2
	init_position = True
	init_RRT = True
	los_method = True
	rrt_star = False
	rrt_path = []
	prev_length_rrt_path = 9999
	while(not rospy.is_shutdown()):
		if(rviz_visualization_start and (rospy.Time.now()-last_request_rviz > rospy.Duration(0.2))):
			publish_uav_position_rviz(br,list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z,uav_ID)
			#uav_marker=update_uav_marker(uav_marker,(list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z,1.0))
			#uav_marker_publisher.publish(uav_marker)
			uav_marker.pose = cur_pose.pose
			uav_marker.pose.position.z = 2
			uav_marker_publisher.publish(uav_marker)
			uav_goal_marker_publisher.publish(uav_goal_marker)
			last_request_rviz = rospy.Time.now()
		if((current_state.mode != 'OFFBOARD') and global_los and (rospy.Time.now()-last_request > rospy.Duration(1.0))):
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
				last_request=rospy.Time.now()
		'''
		if(dist((cur_pose.position.x,cur_pose.position.y),(goal.position.x,goal.position.y)) < 2.0):
			poses.position.x = goal.position.x
			poses.position.y = goal.position.y
			is_arrive_goal = True
		else:
			is_arrive_goal = False
		'''
		#RRT
		
		if((rospy.Time.now()-last_request_rrt > rospy.Duration(0.1)) or (init_RRT)) and (map_flag):
		#if(map_flag) and (replanning_rrt):
			last_request_rrt = rospy.Time.now()
			
			start_rrt_time = rospy.get_time()
			init_RRT = False
			if(dist((cur_pose.pose.position.x,cur_pose.pose.position.y),(goal.pose.position.x,goal.pose.position.y)) < 2.0):
				print 'UAV arrived at the goal location',(cur_pose.pose.position.x,cur_pose.pose.position.y),(goal.pose.position.x,goal.pose.position.y)
				rrt_path = [(goal.pose.position.x,goal.pose.position.y)]
			else:
				#print map_size
				nodes = [(goal.pose.position.x,goal.pose.position.y)]
				rrt_marker.points = []
				#for i in range(NUMNODES):
				i=0
				print "Calculate RRT"
				path_found = False
				nodes_and_parent = []
				came_from = {}
				came_from[nodes[0]]=None
				list_time_los = []
				while (i<NUMNODES) and (not path_found):	
					rand = (random.random()-0.5)*map_size, (random.random()-0.5)*map_size
					nn = nodes[0]
					for p in nodes:
						if dist(p,rand) < dist(nn,rand):
							nn = p

					newnode = step_from_to(nn,rand)
					if(not rrt_star):
						#if(freeObstacle(nn,newnode)):
						#if(freeObstacleWithClearance(nn,newnode,RRT_SAFETY)):
						start_los_time = time.time()
						is_los = LineOfSight(walls,nn,newnode)
						list_time_los.append(time.time()-start_los_time)
						if(is_los):
							nodes.append(newnode)
							nodes_and_parent.append((nn,newnode))
							came_from[newnode]=nn
							p_rrt = Point()
							p_rrt.x=nn[0]
							p_rrt.y=nn[1]
							p_rrt.z=0.0
	 						rrt_marker.points.append(p_rrt)
							p_rrt = Point()
	 						p_rrt.x=newnode[0]
							p_rrt.y=newnode[1]
							p_rrt.z=0.0
	 						rrt_marker.points.append(p_rrt)
							if(dist(newnode,(cur_pose.pose.position.x,cur_pose.pose.position.y))<2.0):
								path_found = True
								print "Path found",i
							#rrt_marker.header.frame_id=map_data.header.frame_id
							#rrt_marker.header.stamp=rospy.Time.now()
							#rrt_marker.lifetime = rospy.Duration(0)

							#pygame.draw.line(screen,white,nn,newnode)
							#pygame.display.update()
							#print nn, " ",newnode
							i=i+1
						else:
							i=i+1
					else: # RRT*
						if(freeObstacle(nn,newnode)):
							near = []
							for n in nodes:
								if(dist(n,newnode) < EPSILON*2):
									near.append(n)

							nodes.append(newnode)
							nodes_and_parent.append((nn,newnode))
							came_from[newnode]=nn
							p_rrt = Point()
							p_rrt.x=nn[0]
							p_rrt.y=nn[1]
							p_rrt.z=0.0
	 						rrt_marker.points.append(p_rrt)
							p_rrt = Point()
	 						p_rrt.x=newnode[0]
							p_rrt.y=newnode[1]
							p_rrt.z=0.0
	 						rrt_marker.points.append(p_rrt)
							if(dist(newnode,(cur_pose.pose.position.x,cur_pose.pose.position.y))<3.0):
								path_found = True
								print "Path found"
							#rrt_marker.header.frame_id=map_data.header.frame_id
							#rrt_marker.header.stamp=rospy.Time.now()
							#rrt_marker.lifetime = rospy.Duration(0)

							#pygame.draw.line(screen,white,nn,newnode)
							#pygame.display.update()
							#print nn, " ",newnode
							i=i+1
						else:
							pass
							#print i,"obs det"
					if (path_found):
						replanning_rrt = False
						break
				if(not path_found):
					print "Path not found"
				if(path_found):
					
					#print "Calculate RRT Path"
					'''
					rrt_path = [nodes_and_parent[-1][0],nodes_and_parent[-1][1]]
					path_found = False
					while(not path_found):
						for i in range(len(nodes_and_parent)-1,-1,-1):
							if(nodes_and_parent[i][0] == rrt_path[-1]):
								rrt_path.append(nodes_and_parent[i][1])
								break
						if(rrt_path[-1]==nodes[0]):
							path_found = True
					print "the rrt path: ",rrt_path
					'''
					#print "came_from",came_from
					#print "nodes", (cur_pose.pose.position.x,cur_pose.pose.position.y),nodes[-1],nodes[0]
					rrt_path_temp=reconstruct_path(came_from,nodes[0],nodes[-1])
					dr = 0
					for rr in range(0,len(rrt_path_temp)-2):
						dr=dr+dist((rrt_path_temp[rr][0],rrt_path_temp[rr][1]),(rrt_path_temp[rr+1][0],rrt_path_temp[rr+1][1]))
					#if(len(rrt_path_temp) <= prev_length_rrt_path) or (goal_changed) or (not global_los):
					if(dr < 3.0*dist((cur_pose.pose.position.x,cur_pose.pose.position.y),(goal.pose.position.x,goal.pose.position.y)) or (not global_los)): #not global_los) or (len(rrt_path_temp) <= prev_length_rrt_path+5):
						print "rrt generated cz global_los"
						prev_length_rrt_path = len(rrt_path_temp)
						rrt_path = rrt_path_temp
						sub_goal_marker.points = []
						for i in rrt_path:
							p_rrt = Point()
							p_rrt.x = i[0]
							p_rrt.y = i[1]
							p_rrt.z = 0.0
							sub_goal_marker.points.append(p_rrt)
						sub_goal_marker_publisher.publish(sub_goal_marker)
						#rrt_marker.header.frame_id=map_data.header.frame_id
						rrt_marker_publisher.publish(rrt_marker)
						#print rrt_marker.points
						#print "the rrt path: ",rrt_path	
					'''
					else:
						for p in range(0,len(rrt_path)-2):
							if not freeObstacleWithClearance(rrt_path[p],rrt_path[p+1],0.5):
								prev_length_rrt_path = len(rrt_path_temp)
								rrt_path = rrt_path_temp
								sub_goal_marker.points = []
								for i in rrt_path:
									p_rrt = Point()
									p_rrt.x = i[0]
									p_rrt.y = i[1]
									p_rrt.z = 0.0
									sub_goal_marker.points.append(p_rrt)
								sub_goal_marker_publisher.publish(sub_goal_marker)
								#rrt_marker.header.frame_id=map_data.header.frame_id
								rrt_marker_publisher.publish(rrt_marker)
								#print rrt_marker.points
								#print "the rrt path: ",rrt_path	
								print "rrt path generated cz un-los path"
								break
					'''
				print "Time spent RRT",rospy.get_time()-start_rrt_time
				print "Average Time Calculation LOS",np.average(list_time_los)
		
		#if(rrt_mode):

			#nearest_node = nodes[0]
			'''
			dist_min = 999999
			for p in nodes:
				dist_calc = dist(p,(cur_pose.pose.position.x,cur_pose.pose.position.y))
				if(dist_calc < dist_min):
					dist_min = dist_calc
					nearest_node = p
			'''
			'''
			if(los_method):
				if(len(rrt_path)>0):
					rrt_path_reverse = rrt_path[:]
					rrt_path_reverse.reverse()
					#print "rpr",rrt_path_reverse
					is_los = False
					for i in rrt_path_reverse:
						if(LineOfSight(walls,(cur_pose.pose.position.x,cur_pose.pose.position.y),i)):
							print "LOOOOOOOOOOOOOS"
							poses.pose.position.x = i[0]
							poses.pose.position.y = i[1]
							poses.pose.position.z = target_init.position.z
							#q=quaternion_from_euler(0,0,atan2((poses.pose.position.y-cur_pose.pose.position.y),(poses.pose.position.x-cur_pose.pose.position.x)))
							poses.pose.orientation.x = 0.0#q[0]
							poses.pose.orientation.y = 0.0#q[1]
							poses.pose.orientation.z = 0.0#q[2]
							poses.pose.orientation.w = 1.0#q[3]
							is_los = True
							break
					print "is LOS?",is_los
				line_los.points = [0,0]
				p_rrt = Point()
				p_rrt.x = cur_pose.pose.position.x
				p_rrt.y = cur_pose.pose.position.y
				p_rrt.z = 0.0
				line_los.points[0] = p_rrt
				p_rrt = Point()
				p_rrt.x = poses.pose.position.x
				p_rrt.y = poses.pose.position.y
				p_rrt.z = 0.0
				line_los.points[1] = p_rrt
				line_los_publisher.publish(line_los)
			else:
				if(len(rrt_path)>0):
					poses.pose.position.x = rrt_path[0][0]
					poses.pose.position.y = rrt_path[0][1]
					poses.pose.position.z = target_init.position.z
					q=quaternion_from_euler(0,0,45*tan((poses.pose.position.y-cur_pose.pose.position.y)/(poses.pose.position.x-cur_pose.pose.position.x))/180.0)
					poses.pose.orientation.x = q[0]
					poses.pose.orientation.y = q[1]
					poses.pose.orientation.z = q[2]
					poses.pose.orientation.w = q[3]
					if(dist((cur_pose.pose.position.x,cur_pose.pose.position.y),rrt_path[0]) < 0.5):
						del rrt_path[0]
			'''
		
		'''
		if(distance3D(cur_pose.pose,target) < 0.5) and (not usingvelocitycontrol):
			print "sampai"
			usingvelocitycontrol = False#True
			usingpositioncontrol = True#False
			target = Pose()
			if(idx == 0):
				target.position.x = 28
				target.position.y = 28
			if(idx == 1):
				target.position.x = 0
				target.position.y = 0
			if(idx == 2):
				target.position.x = 21
				target.position.y = 14
			if(idx == 3):
				target.position.x = 0
				target.position.y = 0
			
			target.position.z = 10
			print target
			psi_target = 45*np.pi/180.0 #np.pi/180.0 #np.arctan((self.target.position.y-self.prev_target.position.y)/(self.target.position.x-self.prev_target.position.x))#(linear.y,linear.x)
			
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
			print pathx
			print pathy
			vController.setHeadingTarget(deg2rad(90.0)) #45.0))
			target.position.x = pathx[0]*par.RealScale
			target.position.y = pathy[0]*par.RealScale
			uav_goal_marker=update_uav_marker(uav_goal_marker,(target.position.x,target.position.y,target.position.z,1.0))
			uav_goal_marker_publisher.publish(uav_goal_marker)
			vController.setTarget(target)
			
			vController.setPIDGainX(1,0.0,0.0)
			vController.setPIDGainY(1,0.0,0.0)
			vController.setPIDGainZ(1,0,0)
			vController.setPIDGainPHI(1,0.0,0.0)
			vController.setPIDGainTHETA(1,0.0,0.0)
			vController.setPIDGainPSI(1,0.0,0.0)
		'''
		if(init_position) and (distance3D(cur_pose0.pose,target_init) < 0.3):
			print "Arrived at the init position"
			init_position = False
			dir_path = os.path.dirname(os.path.realpath(__file__))
			print dir_path
			cwd = os.getcwd()
			print cwd
			os.system('./run_hector_slam_multi_agent.sh '+uav_ID)

		if(init_position):
			#print "kontrol posisi"
			poses.pose.position.x = target_init.position.x
			poses.pose.position.y = target_init.position.y
			poses.pose.position.z = target_init.position.z
			poses.pose.orientation.x = 0.0
			poses.pose.orientation.y = 0.0
			poses.pose.orientation.z = 0.0
			poses.pose.orientation.w = 1.0
		local_pos_pub.publish(poses)
		'''
		elif(usingvelocitycontrol):
			if(distance2D(cur_pose.pose,target) < 1.5):
				if(len(pathx) > 1):
					del pathx[0]
					del pathy[0]
					target.position.x = pathx[0]*par.RealScale
					target.position.y = pathy[0]*par.RealScale
					uav_goal_marker=update_uav_marker(uav_goal_marker,(target.position.x,target.position.y,target.position.z,1.0))
					uav_goal_marker_publisher.publish(uav_goal_marker)
					print target
					vController.setTarget(target)

			if(rospy.Time.now()-last_HRVO_request > rospy.Duration(0.05)):
				last_HRVO_request=rospy.Time.now()
				header.stamp = rospy.Time.now()
				
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
				V,RVO_BA_all = locplan.RVO_update_single_static(current_agent_pose, current_neighbor_pose, current_agent_vel, current_neighbor_vel, V_des, par)
	        
				V_msg = des_vel
				V_msg.twist.linear.x = V[0]
				V_msg.twist.linear.y = V[1]
				local_vel_pub.publish(V_msg)
				
				goal = (target.position.x,target.position.y,target.position.z)
				x = current_position
				list_error[0].append(error_time)
				list_error[1].append(goal[0]-x[0])
				list_error[2].append(goal[1]-x[1])
				list_error[3].append(goal[2]-x[2])
				error_time = error_time + 1
				k=0.1
				vx = k*(goal[0]-x[0])
				vy = k*(goal[1]-x[1])
				vz = k*(goal[2]-x[2])
				postar = PositionTarget()
				postar.header = header
				postar.coordinate_frame = PositionTarget().FRAME_LOCAL_NED
				p = PositionTarget().IGNORE_PX | PositionTarget().IGNORE_PY | PositionTarget().IGNORE_PZ 
				a = PositionTarget().IGNORE_AFX | PositionTarget().IGNORE_AFY | PositionTarget().IGNORE_AFZ	
				v = PositionTarget().IGNORE_VX | PositionTarget().IGNORE_VY | PositionTarget().IGNORE_VZ	
				y = PositionTarget().IGNORE_YAW | PositionTarget().IGNORE_YAW_RATE
				f = PositionTarget().FORCE
				postar.type_mask = a | y | p | f #| PositionTarget().IGNORE_YAW | PositionTarget().IGNORE_YAW_RATE | PositionTarget().FORCE
				postar.velocity.x = vx
				postar.velocity.y = vy
				postar.velocity.z = vz
				postar.position.x = goal[0]
				postar.position.y = goal[1]
				postar.position.z = goal[2]
				
				vel_msg = TwistStamped()
				vel_msg.header = header
				vel_msg.twist.linear.x = xvel
				vel_msg.twist.linear.y = yvel
				vel_msg.twist.linear.z = 0.0
				vel_msg.twist.angular.x = 0.0
				vel_msg.twist.angular.y = 0.0
				vel_msg.twist.angular.z = 0.0
				
				q=quaternion_from_euler(0,0,0.2)
				att = PoseStamped()
				att.pose.orientation.x = q[0]
				att.pose.orientation.y = q[1]
				att.pose.orientation.z = q[2]
				att.pose.orientation.w = q[3]
				att.pose.position.x = 0.0
				att.pose.position.y = 0.0
				att.pose.position.z = 2.0

				cmd_thr = Float64()
				cmd_thr.data = 0.3

				att_raw = AttitudeTarget()
				att_raw.header = Header()
				att_raw.body_rate = Vector3()

				att_raw.thrust = 0.7 #Float64()
				att_raw.header.stamp = rospy.Time.now()
				att_raw.header.frame_id = "fcu"
				att_raw.type_mask = 71
				att_raw.orientation = Quaternion(*quaternion_from_euler(0,0,0.2))
		else:
			local_pos_pub.publish(poses)
		'''
		count=count+1
		r.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)