#!/usr/bin/env python

import roslib#; roslib.load_manifest('visualization_marker_tutorials')
import warnings
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32, PolygonStamped, Polygon, PoseStamped
import std_msgs.msg 
import sensor_msgs.point_cloud2 as pcl2
import rospy
import math
import numpy as np
import load_parameter as lp
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Int32
from nav_msgs.msg import GridCells, Path
from sensor_msgs.msg import Image
import json
import os
import cv2
import matplotlib
import matplotlib.cm as cm
import matplotlib.ticker as ticker
import matplotlib.pyplot as plt
from scipy import misc
import glob
from cv_bridge import CvBridge, CvBridgeError
import Image as im
from threading import Thread
import sys
from time import sleep
from tf.transformations import euler_from_quaternion
import psutil

par = lp.Parameter
header = std_msgs.msg.Header()
est_sensory_function_current = np.zeros((len(par.Xaxis),len(par.Yaxis)))
print est_sensory_function_current
control_uav_start = False
current_iteration = 0

AGENT_GOES_TO_INIT = 0
AGENT_HOVER_YAWING = 1
AGENT_GOES_TO_GOAL = 2
AGENT_ARRIVE_AT_GOAL = 3

img_res_idx = 0

def rotated(array_2d):
    list_of_tuples = zip(*array_2d[::-1])
    return [list(elem) for elem in list_of_tuples]

def image_pub_function(folderstring):
  #for i in range(arg):
  global RVO_list,plot_image_publisher,est_sensory_function_current,list_current_position, control_uav_start, list_goal_position, list_sub_goal_position
  while(True):
  	if(control_uav_start):
  		image_msg = matrix_to_image_msg(list_voronoi_grid,est_sensory_function_current,list_current_position,list_goal_position,list_sub_goal_position,RVO_list,folderstring) #lp.TrueFunction(par))
  		if(image_msg == 0):
  			pass
  		else:
  			plot_image_publisher.publish(image_msg) 
  	#print poses
  	sleep(0.5)

list_current_position = []
list_goal_position = []
list_sub_goal_position = []
list_voronoi_grid = []
list_new_goal = []
for i in range(par.NumberOfAgents):
	list_current_position.append(PoseStamped())
	list_goal_position.append(PoseStamped())
	list_sub_goal_position.append(PoseStamped())
	list_new_goal.append(False)
	list_voronoi_grid.append(np.zeros((par.NumberOfPoints,par.NumberOfPoints)))

def current_iteration_cb(msg_var):
	global current_iteration
	current_iteration = msg_var.data

def position_cb(msg_var,data_var):
  global current_position, control_uav_start, list_current_position
  #print data_var, msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z
  idx = data_var[0]
  offset= data_var[1]
  list_current_position[idx].pose.position.x = msg_var.pose.position.x+offset[idx][0]
  list_current_position[idx].pose.position.y = msg_var.pose.position.y+offset[idx][1]
  list_current_position[idx].pose.position.z = msg_var.pose.position.z 

  list_current_position[idx].pose.orientation.x = msg_var.pose.orientation.x
  list_current_position[idx].pose.orientation.y = msg_var.pose.orientation.y
  list_current_position[idx].pose.orientation.z = msg_var.pose.orientation.z
  list_current_position[idx].pose.orientation.w = msg_var.pose.orientation.w
  current_position = (msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z)
  '''
  if(idx==0):
	  list_current_position[idx].pose.position.x = msg_var.pose.position.x
	  list_current_position[idx].pose.position.y = msg_var.pose.position.y
	  list_current_position[idx].pose.position.z = msg_var.pose.position.z 

	  list_current_position[idx].pose.orientation.x = msg_var.pose.orientation.x
	  list_current_position[idx].pose.orientation.y = msg_var.pose.orientation.y
	  list_current_position[idx].pose.orientation.z = msg_var.pose.orientation.z
	  list_current_position[idx].pose.orientation.w = msg_var.pose.orientation.w
	  current_position = (msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z)
  if(idx==1):
	  list_current_position[idx].pose.position.x = msg_var.pose.position.x+28
	  list_current_position[idx].pose.position.y = msg_var.pose.position.y+28
	  list_current_position[idx].pose.position.z = msg_var.pose.position.z 

	  list_current_position[idx].pose.orientation.x = msg_var.pose.orientation.x
	  list_current_position[idx].pose.orientation.y = msg_var.pose.orientation.y
	  list_current_position[idx].pose.orientation.z = msg_var.pose.orientation.z
	  list_current_position[idx].pose.orientation.w = msg_var.pose.orientation.w
	  current_position = (msg_var.pose.position.x+28,msg_var.pose.position.y+28,msg_var.pose.position.z)
  if(idx==2):
	  list_current_position[idx].pose.position.x = msg_var.pose.position.x
	  list_current_position[idx].pose.position.y = msg_var.pose.position.y+17
	  list_current_position[idx].pose.position.z = msg_var.pose.position.z 

	  list_current_position[idx].pose.orientation.x = msg_var.pose.orientation.x
	  list_current_position[idx].pose.orientation.y = msg_var.pose.orientation.y
	  list_current_position[idx].pose.orientation.z = msg_var.pose.orientation.z
	  list_current_position[idx].pose.orientation.w = msg_var.pose.orientation.w
	  current_position = (msg_var.pose.position.x,msg_var.pose.position.y+17,msg_var.pose.position.z)
  '''
  control_uav_start = True

def goal_position_cb(msg_var,data_var):
  global list_goal_position,list_new_goal
  #print data_var, msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z
  idx = data_var
  list_goal_position[idx].pose.position.x = msg_var.pose.position.x
  list_goal_position[idx].pose.position.y = msg_var.pose.position.y
  list_goal_position[idx].pose.position.z = msg_var.pose.position.z 
  list_new_goal[idx] = True


def sub_goal_position_cb(msg_var,data_var):
  global list_sub_goal_position
  #print data_var, msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z
  idx = data_var
  list_sub_goal_position[idx].pose.position.x = msg_var.pose.position.x
  list_sub_goal_position[idx].pose.position.y = msg_var.pose.position.y
  list_sub_goal_position[idx].pose.position.z = msg_var.pose.position.z 

def mission_state_cb(msg_var,data_var):
	if(msg_var.data == AGENT_HOVER_YAWING):
		state_str = "AGENT_HOVER_YAWING"
	elif(msg_var.data == AGENT_ARRIVE_AT_GOAL):
		state_str = "AGENT_ARRIVE_AT_GOAL"
	elif(msg_var.data == AGENT_GOES_TO_GOAL):
		state_str = "AGENT_GOES_TO_GOAL"
	elif(msg_var.data == AGENT_GOES_TO_INIT):
		state_str = "AGENT_GOES_TO_INIT"
	else:
		state_str = "UNKNOWN"
	print "UAV"+str(data_var+1)+" state = "+state_str

polygon_voronoi_publisher = []

def polygon_cb(msg_var,data_var):
	global polygon_voronoi_publisher,header
	idx = data_var
	polygon_msg = PolygonStamped()
	polygon_msg.header = header
	polygon_msg.polygon = msg_var
	#print data_var,msg_var
	polygon_voronoi_publisher[idx].publish(polygon_msg)

voronoi_grid_publisher = []

def voronoi_grid_cb(msg_var,data_var):
	global voronoi_grid_publisher,list_voronoi_grid,header
	idx = data_var
	voronoi_grid = np.asarray(msg_var.data).reshape((len(par.Xaxis),len(par.Xaxis)))
	list_voronoi_grid[idx] = voronoi_grid
	voronoi_grid = np.fliplr(voronoi_grid)
	#np.rot90(voronoi_grid,k=2)
	voronoi_grid = list(rotated(voronoi_grid))
	voronoi_grid = list(rotated(voronoi_grid))
	voronoi_grid = list(rotated(voronoi_grid))
	voronoi_grid_msg = GridCells()
	voronoi_grid_msg.header=header
	voronoi_grid_msg.cell_width = 1.0
	voronoi_grid_msg.cell_height = 1.0
	voronoi_grid_msg.cells = []
	for i in range(par.NumberOfPoints):
		for j in range(par.NumberOfPoints):
			if(voronoi_grid[i][j] == 1):
				temp_point = Point32()
				temp_point.x = i
				temp_point.y = j
				temp_point.z = 0.0
				voronoi_grid_msg.cells.append(temp_point)

	voronoi_grid_publisher[idx].publish(voronoi_grid_msg)

def matrix_to_image_msg(vor,mat,pos,goal,subgoal,rvo_list,folderstring):
	global img_res_idx
	if np.all(mat == np.zeros(mat.shape)):
		return 0

	bridge = CvBridge()
	fig1 = plt.figure()
	ax1 = plt.subplot()
	
	plotvororeg = np.zeros((par.NumberOfPoints,par.NumberOfPoints))
	for i in range(par.NumberOfAgents):
		plotvororeg = vor[i].astype(int)*((i+1)*20)+plotvororeg
	extent = 0,1,0,1
	ax1.imshow(np.flipud(plotvororeg),extent=extent,cmap='Pastel1')
	# Basic contour plot
	#print mat
	#print par.Xgrid
	#print par.Ygrid
	CS1 = ax1.contour(par.Xgrid,par.Ygrid,mat)

	for i in range(par.NumberOfAgents):
		srec = matplotlib.patches.Circle(
				(subgoal[i].pose.position.x/par.RealScale, subgoal[i].pose.position.y/par.RealScale),
				par.ws_model['robot_radius']/par.RealScale,
				facecolor= 'yellow',
				fill = True,
				alpha=1)
		ax1.add_patch(srec)

	for i in range(par.NumberOfAgents):
		srec = matplotlib.patches.Circle(
				(goal[i].pose.position.x/par.RealScale, goal[i].pose.position.y/par.RealScale),
				par.ws_model['robot_radius']/par.RealScale,
				facecolor= 'green',
				fill = True,
				alpha=1)
		ax1.add_patch(srec)

	for i in range(par.NumberOfAgents):
		srec = matplotlib.patches.Circle(
				(pos[i].pose.position.x/par.RealScale, pos[i].pose.position.y/par.RealScale),
				par.ws_model['robot_radius']/par.RealScale,
				facecolor= 'red',
				fill = True,
				alpha=1)
		ax1.add_patch(srec)	
	'''
	for i in range(par.NumberOfAgents):
		quat_arr = np.array([pos[i].pose.orientation.x,pos[i].pose.orientation.y,pos[i].pose.orientation.z,pos[i].pose.orientation.w])
		att = euler_from_quaternion(quat_arr,'sxyz')
		ax1.quiver(pos[i].pose.position.x/par.RealScale, pos[i].pose.position.y/par.RealScale,np.cos(att[2]),np.sin(att[2]))
		ax1.annotate(str(i+1), (pos[i].pose.position.x/par.RealScale+0.02, pos[i].pose.position.y/par.RealScale+0.02))
	'''
	for hole in par.ws_model['circular_obstacles']:
		srec = matplotlib.patches.Circle(
				(hole[0], hole[1]),
				hole[2],
				facecolor= 'black',
				fill = True,
				alpha=1)
		ax1.add_patch(srec)	
	'''
	for RVO in rvo_list:
	#RVO = rvo_list[0]
		srec = matplotlib.patches.Arrow(RVO[0]/par.RealScale,RVO[1]/par.RealScale,RVO[2],RVO[3],width=0.001)
		ax1.add_patch(srec)
		srec = matplotlib.patches.Arrow(RVO[0]/par.RealScale,RVO[1]/par.RealScale,RVO[4],RVO[5],width=0.001)
		ax1.add_patch(srec)
	'''
	#plt.savefig(dir_path+'/temp_img/foo.png')
	ax1.set_xlim((-0.1,1.1))
	ax1.set_ylim((-0.1,1.1))
	plt.title("Coverage iteration : "+repr(current_iteration))
	namebuf = "%04d.png" % img_res_idx
	plt.savefig(folderstring+namebuf)
	img_res_idx = img_res_idx+1
	fig1.canvas.draw()
	#
	#plt.show()

	image_temp = np.fromstring(fig1.canvas.tostring_rgb(), dtype=np.uint8, sep='')
	image_temp = image_temp.reshape(fig1.canvas.get_width_height()[::-1] + (3,))
	image_temp = cv2.cvtColor(image_temp,cv2.COLOR_RGB2BGR)
	fig1.clf()
	#plt.close()
	return bridge.cv2_to_imgmsg(image_temp, "bgr8")


def RVO_cb(msg_var,data_var):
	global RVO_list
	RVO_list[data_var] = msg_var.data

def est_sensory_function_cb(msg_var):
	global scaled_polygon_pcl_est_lvl,est_sensory_level,header,plot_image_publisher, est_sensory_function_current, list_current_position
	est_sensory_function_current = np.asarray(msg_var.data).reshape((len(par.Xaxis),len(par.Xaxis)))
	est_points_lvl1 = []
	est_points_lvl2 = []
	est_points_lvl3 = []
	est_points_lvl4 = []
	est_points_lvl5 = []
	maxSF = np.max(np.max(est_sensory_function_current))
	minSF = np.min(np.min(est_sensory_function_current))
	gapSF = (maxSF-minSF)/est_sensory_level
	est_threshold_lvl1_2 = minSF+gapSF
	est_threshold_lvl2_3 = minSF+2*gapSF
	est_threshold_lvl3_4 = minSF+3*gapSF
	est_threshold_lvl4_5 = minSF+4*gapSF
	for h in range(len(par.Xaxis)):
		for k in range(len(par.Yaxis)):
			if(est_sensory_function_current[k,h] < est_threshold_lvl1_2):
				est_points_lvl1.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			elif(est_sensory_function_current[k,h] > est_threshold_lvl1_2) and (est_sensory_function_current[k,h] < est_threshold_lvl2_3):
				est_points_lvl2.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			elif(est_sensory_function_current[k,h] > est_threshold_lvl2_3) and (est_sensory_function_current[k,h] < est_threshold_lvl3_4):
				est_points_lvl3.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			elif(est_sensory_function_current[k,h] > est_threshold_lvl3_4) and (est_sensory_function_current[k,h] < est_threshold_lvl4_5):
				est_points_lvl4.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			else:
				est_points_lvl5.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])

	scaled_polygon_pcl_est_lvl[0] = pcl2.create_cloud_xyz32(header, est_points_lvl1)
	scaled_polygon_pcl_est_lvl[1] = pcl2.create_cloud_xyz32(header, est_points_lvl2)
	scaled_polygon_pcl_est_lvl[2] = pcl2.create_cloud_xyz32(header, est_points_lvl3)
	scaled_polygon_pcl_est_lvl[3] = pcl2.create_cloud_xyz32(header, est_points_lvl4)
	scaled_polygon_pcl_est_lvl[4] = pcl2.create_cloud_xyz32(header, est_points_lvl5)

	#image_msg = matrix_to_image_msg(list_voronoi_grid,est_sensory_function_current,list_current_position,list_goal_position) #lp.TrueFunction(par))
	#plot_image_publisher.publish(image_msg) 

def main(args):
	global list_new_goal,RVO_list,header, est_sensory_function_current,plot_image_publisher,list_current_position,scaled_polygon_pcl_est_lvl,list_goal_position,list_sub_goal_position,list_voronoi_grid
	print "===================== RVIZ VISUALIZATION ======================"
	rospy.init_node('rviz_visualization')
	print args
	ros_wd = args[1]
	print ros_wd
	pathname=os.path.abspath(ros_wd+'/src/coverage_control_uavs/control_uav/scripts/')
	print pathname
	listfolder = []
	for foldername in os.listdir(pathname+'/img_res'):
		listfolder.append(int(foldername))

	if(len(listfolder) == 0):
		curr_folder = 0
	else:
		curr_folder = np.max(listfolder)+1
	print 'current folder:',curr_folder
	makedircommand = 'mkdir '+pathname+'/img_res/%04d'%curr_folder
	os.system(makedircommand)
	folderstring = pathname+'/img_res/%04d/'%curr_folder

	header.stamp = rospy.Time.now()
	header.frame_id = "world"
	dir_path = os.path.dirname(os.path.realpath(__file__))

	topic = 'visualization_marker_array'
	markerArray = MarkerArray()
	count = 0
	MARKERS_MAX = 100
	marker_publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)

	if(par.UsingObstacle):
		obstacle_pub_list = []
		obstacle_list = []
		RVO_list = []
		idx = 0
		for hole in par.ws_model['circular_obstacles']:
			obstacle_pub = rospy.Publisher('obstacle_'+repr(idx),Marker,queue_size=10)
			obstacle = Marker()
			obstacle.header.frame_id = "world"
			obstacle.type = obstacle.CYLINDER
			obstacle.action = obstacle.ADD
			obstacle.scale.x = hole[2]*par.RealScale*2
			obstacle.scale.y = hole[2]*par.RealScale*2
			obstacle.scale.z = 6
			obstacle.color.a = 1.0
			obstacle.color.r = 1.0
			obstacle.color.g = 1.0
			obstacle.color.b = 1.0
			obstacle.pose.orientation.w = 1.0
			obstacle.pose.position.x = hole[0]*par.RealScale
			obstacle.pose.position.y = hole[1]*par.RealScale 
			obstacle.pose.position.z = obstacle.scale.z/2
			obstacle_pub_list.append(obstacle_pub)
			obstacle_list.append(obstacle)
			idx = idx+1
			RVO_list.append([])
	path_publisher = []
	for i in range(par.NumberOfAgents): 
		polygon_voronoi_publisher.append(rospy.Publisher("uav"+repr(i+1)+"/polygon_voronoi_rviz",PolygonStamped,queue_size=10))
		polygon_voronoi_subscriber = rospy.Subscriber("uav"+repr(i+1)+"/polygon_voronoi",Polygon,queue_size=10,callback=polygon_cb,callback_args=i)
		voronoi_grid_publisher.append(rospy.Publisher("uav"+repr(i+1)+"/voronoi_grid_rviz",GridCells,queue_size=10))
		path_publisher.append(rospy.Publisher("uav"+repr(i+1)+"/path",Path,queue_size=10))
		voronoi_grid_subscriber = rospy.Subscriber("uav"+repr(i+1)+"/voronoi_grid",Int8MultiArray,queue_size=10,callback=voronoi_grid_cb,callback_args=i)
		position_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mavros/local_position/pose",PoseStamped,queue_size = 10,callback=position_cb,callback_args=(i,par.all_init_pos))
		goal_position_sub=rospy.Subscriber("/uav"+repr(i+1)+"/goal_position",PoseStamped,queue_size=10,callback=goal_position_cb,callback_args=i)
		sub_goal_position_sub=rospy.Subscriber("/uav"+repr(i+1)+"/sub_goal_position",PoseStamped,queue_size=10,callback=sub_goal_position_cb,callback_args=i)
		mission_state_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mission_state",Int32,queue_size=10,callback=mission_state_cb,callback_args=i)
	current_iteration_sub = rospy.Subscriber("/current_iteration",Int32,queue_size=10,callback=current_iteration_cb)
	
	for i in range(len(par.ws_model['circular_obstacles'])):
		RVO_sub=rospy.Subscriber("/uav1/RVO"+repr(i),Float32MultiArray,queue_size=10,callback=RVO_cb,callback_args=i)
  
	plot_image_publisher = rospy.Publisher('plot_coverage_image',Image,queue_size=10)
	uav_path = []
	
	for i in range(par.NumberOfAgents): 
		uav_path.append(Path())
		uav_path[i].header.frame_id = "world"
		uav_path[i].poses = []
	#first image
	bridge = CvBridge()
	fig1 = plt.figure()
	ax1 = plt.subplot()
	nan_matrix = np.empty((par.NumberOfPoints,par.NumberOfPoints)) #full([par.NumberOfPoints,par.NumberOfPoints],np.nan)# 
	nan_matrix.fill(np.nan)
	zero_matrix = np.zeros((par.NumberOfPoints,par.NumberOfPoints))
	CS1 = ax1.imshow(zero_matrix+255,cmap=plt.get_cmap('gray'),vmin=0, vmax=255)
	fig1.canvas.draw()
	image_temp = np.fromstring(fig1.canvas.tostring_rgb(), dtype=np.uint8, sep='')
	image_temp = image_temp.reshape(fig1.canvas.get_width_height()[::-1] + (3,))
	image_temp = cv2.cvtColor(image_temp,cv2.COLOR_RGB2BGR)
	fig1.clf()
	plt.close()
	plot_image_publisher.publish(bridge.cv2_to_imgmsg(image_temp, "bgr8"))

	real_sensory_function = lp.TrueFunction(par)

	global est_sensory_level
	est_sensory_level = 5.0
	est_point2_lvl1_publisher = rospy.Publisher('viz_est_point2_lvl1',PointCloud2,queue_size=10)
	est_point2_lvl2_publisher = rospy.Publisher('viz_est_point2_lvl2',PointCloud2,queue_size=10)
	est_point2_lvl3_publisher = rospy.Publisher('viz_est_point2_lvl3',PointCloud2,queue_size=10)
	est_point2_lvl4_publisher = rospy.Publisher('viz_est_point2_lvl4',PointCloud2,queue_size=10)
	est_point2_lvl5_publisher = rospy.Publisher('viz_est_point2_lvl5',PointCloud2,queue_size=10)

	real_sensory_level = 5.0
	real_point2_lvl1_publisher = rospy.Publisher('viz_real_point2_lvl1',PointCloud2,queue_size=10)
	real_point2_lvl2_publisher = rospy.Publisher('viz_real_point2_lvl2',PointCloud2,queue_size=10)
	real_point2_lvl3_publisher = rospy.Publisher('viz_real_point2_lvl3',PointCloud2,queue_size=10)
	real_point2_lvl4_publisher = rospy.Publisher('viz_real_point2_lvl4',PointCloud2,queue_size=10)
	real_point2_lvl5_publisher = rospy.Publisher('viz_real_point2_lvl5',PointCloud2,queue_size=10)

	est_points_lvl1 = []
	est_points_lvl2 = []
	est_points_lvl3 = []
	est_points_lvl4 = []
	est_points_lvl5 = []
	maxSF = np.max(np.max(est_sensory_function_current))
	minSF = np.min(np.min(est_sensory_function_current))
	gapSF = (maxSF-minSF)/est_sensory_level
	est_threshold_lvl1_2 = minSF+gapSF
	est_threshold_lvl2_3 = minSF+2*gapSF
	est_threshold_lvl3_4 = minSF+3*gapSF
	est_threshold_lvl4_5 = minSF+4*gapSF

	real_points_lvl1 = []
	real_points_lvl2 = []
	real_points_lvl3 = []
	real_points_lvl4 = []
	real_points_lvl5 = []
	maxSF = np.max(np.max(real_sensory_function))
	minSF = np.min(np.min(real_sensory_function))
	gapSF = (maxSF-minSF)/real_sensory_level
	real_threshold_lvl1_2 = minSF+gapSF
	real_threshold_lvl2_3 = minSF+2*gapSF
	real_threshold_lvl3_4 = minSF+3*gapSF
	real_threshold_lvl4_5 = minSF+4*gapSF

	for h in range(len(par.Xaxis)):
		for k in range(len(par.Yaxis)):
			if(est_sensory_function_current[k,h] < est_threshold_lvl1_2):
				est_points_lvl1.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			elif(est_sensory_function_current[k,h] > est_threshold_lvl1_2) and (est_sensory_function_current[k,h] < est_threshold_lvl2_3):
				est_points_lvl2.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			elif(est_sensory_function_current[k,h] > est_threshold_lvl2_3) and (est_sensory_function_current[k,h] < est_threshold_lvl3_4):
				est_points_lvl3.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			elif(est_sensory_function_current[k,h] > est_threshold_lvl3_4) and (est_sensory_function_current[k,h] < est_threshold_lvl4_5):
				est_points_lvl4.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])
			else:
				est_points_lvl5.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,par.StaticAltitude])

	for h in range(len(par.Xaxis)):
		for k in range(len(par.Yaxis)):
			if(real_sensory_function[k,h] < real_threshold_lvl1_2):
				real_points_lvl1.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,1])
			elif(real_sensory_function[k,h] > real_threshold_lvl1_2) and (real_sensory_function[k,h] < real_threshold_lvl2_3):
				real_points_lvl2.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,1])
			elif(real_sensory_function[k,h] > real_threshold_lvl2_3) and (real_sensory_function[k,h] < real_threshold_lvl3_4):
				real_points_lvl3.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,1])
			elif(real_sensory_function[k,h] > real_threshold_lvl3_4) and (real_sensory_function[k,h] < real_threshold_lvl4_5):
				real_points_lvl4.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,1])
			else:
				real_points_lvl5.append([par.Xaxis[h]*par.RealScale,par.Yaxis[k]*par.RealScale,1])
		#pointss.append([h*10-5,k*10-5,0])

	scaled_polygon_pcl_est_lvl = [0,0,0,0,0]
	scaled_polygon_pcl_est_lvl[0] = pcl2.create_cloud_xyz32(header, est_points_lvl1)
	scaled_polygon_pcl_est_lvl[1] = pcl2.create_cloud_xyz32(header, est_points_lvl2)
	scaled_polygon_pcl_est_lvl[2] = pcl2.create_cloud_xyz32(header, est_points_lvl3)
	scaled_polygon_pcl_est_lvl[3] = pcl2.create_cloud_xyz32(header, est_points_lvl4)
	scaled_polygon_pcl_est_lvl[4] = pcl2.create_cloud_xyz32(header, est_points_lvl5)

	scaled_polygon_pcl_real_lvl = [0,0,0,0,0]
	scaled_polygon_pcl_real_lvl[0] = pcl2.create_cloud_xyz32(header, real_points_lvl1)
	scaled_polygon_pcl_real_lvl[1] = pcl2.create_cloud_xyz32(header, real_points_lvl2)
	scaled_polygon_pcl_real_lvl[2] = pcl2.create_cloud_xyz32(header, real_points_lvl3)
	scaled_polygon_pcl_real_lvl[3] = pcl2.create_cloud_xyz32(header, real_points_lvl4)
	scaled_polygon_pcl_real_lvl[4] = pcl2.create_cloud_xyz32(header, real_points_lvl5)

	est_sensory_function_sub = rospy.Subscriber("/uav1/est_sensory_function",Float32MultiArray,queue_size=10,callback=est_sensory_function_cb)

	#thread = Thread(target = image_pub_function)
	#thread.start()

	with open(dir_path+'/config.json','r') as f:
		config = json.load(f)

	marker = Marker()
	marker.header.frame_id = "world"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0

	while not rospy.is_shutdown():
		os.system('clear')
		print "CPU(%)= "+repr(psutil.cpu_percent(percpu=True))
		print "CPU avg(%)= "+str(psutil.cpu_percent())
		print "RAM(%)= "+str(psutil.virtual_memory().percent)
		for i in range(par.NumberOfAgents): 
			pose_for_path = PoseStamped()
			pose_for_path.header.frame_id = "world"
			pose_for_path.pose.position.x = list_current_position[i].pose.position.x
			pose_for_path.pose.position.y = list_current_position[i].pose.position.y
			pose_for_path.pose.position.z = list_current_position[i].pose.position.z
			uav_path[i].poses.append(pose_for_path)
			if(list_new_goal[i] == True):
				uav_path[i].poses = []
				list_new_goal[i] = False
			path_publisher[i].publish(uav_path[i])

		marker.pose.position.x = math.cos(count / 50.0)
		marker.pose.position.y = math.cos(count / 40.0) 
		marker.pose.position.z = math.cos(count / 30.0) 

	   # We add the new marker to the MarkerArray, removing the oldest
	   # marker from it when necessary
		if(count > MARKERS_MAX):
			markerArray.markers.pop(0)

		markerArray.markers.append(marker)

		# Renumber the marker IDs
	 	id = 0
		for m in markerArray.markers:
			m.id = id
			id += 1

	   # Publish the MarkerArray
		marker_publisher.publish(markerArray)

		count += 1
		
		# Visualize Estimated Sensory Function
		est_point2_lvl1_publisher.publish(scaled_polygon_pcl_est_lvl[0])
		est_point2_lvl2_publisher.publish(scaled_polygon_pcl_est_lvl[1])
		est_point2_lvl3_publisher.publish(scaled_polygon_pcl_est_lvl[2])
		est_point2_lvl4_publisher.publish(scaled_polygon_pcl_est_lvl[3])
		est_point2_lvl5_publisher.publish(scaled_polygon_pcl_est_lvl[4])

		# Visualize Real Sensory Function
		real_point2_lvl1_publisher.publish(scaled_polygon_pcl_real_lvl[0])
		real_point2_lvl2_publisher.publish(scaled_polygon_pcl_real_lvl[1])
		real_point2_lvl3_publisher.publish(scaled_polygon_pcl_real_lvl[2])
		real_point2_lvl4_publisher.publish(scaled_polygon_pcl_real_lvl[3])
		real_point2_lvl5_publisher.publish(scaled_polygon_pcl_real_lvl[4])

		if(control_uav_start):
  			image_msg = matrix_to_image_msg(list_voronoi_grid,est_sensory_function_current,list_current_position,list_goal_position,list_sub_goal_position,RVO_list,folderstring) #lp.TrueFunction(par))
  			if(image_msg == 0):
  				pass
  			else:
  				plot_image_publisher.publish(image_msg) 

		if(config["DEFAULT"]["USING_OBSTACLE"] == True):
			scale = config["DEFAULT"]["REAL_SCALE"]
			num_obstacle = len(config["OBSTACLE"])
			obstacle_size = []
			obstacle_pose = []
			obstacle_type = []
			for i in range(num_obstacle):
				obstacle_type.append(config["OBSTACLE"][repr(i)]["TYPE"])
				obstacle_size.append(config["OBSTACLE"][repr(i)]["SIZE"])
				obstacle_pose.append(config["OBSTACLE"][repr(i)]["POSE"])
			#print obstacle_size
			#print obstacle_pose
			obstacle_pub_list = []
			obstacle_list = []

			for i in range(num_obstacle):
				obstacle_pub = rospy.Publisher('obstacle_'+repr(i),Marker,queue_size=10)
				obstacle = Marker()
				obstacle.header.frame_id = "world"
				if(obstacle_type[i] == "BOX"):
					obstacle.type = obstacle.CUBE
				elif(obstacle_type[i] == "CYLINDER"):
					obstacle.type = obstacle.CYLINDER
				obstacle.action = obstacle.ADD
				if(obstacle_type[i] == "BOX"):
					obstacle.scale.x = obstacle_size[i][0]*scale*2
					obstacle.scale.y = obstacle_size[i][1]*scale*2
					obstacle.scale.z = obstacle_size[i][2]*scale
					obstacle.pose.position.z = obstacle_pose[i][2]*scale+0.5*obstacle_size[i][2]*scale
				elif(obstacle_type[i] == "CYLINDER"):
					obstacle.scale.x = obstacle_size[i][0]*scale*2
					obstacle.scale.y = obstacle_size[i][0]*scale*2
					obstacle.scale.z = obstacle_size[i][1]*scale
					obstacle.pose.position.z = obstacle_pose[i][2]*scale+0.5*obstacle_size[i][1]*scale
					
				obstacle.color.a = 1.0
				obstacle.color.r = 1.0
				obstacle.color.g = 1.0
				obstacle.color.b = 1.0
				obstacle.pose.orientation.w = 1.0
				obstacle.pose.position.x = obstacle_pose[i][0]*scale
				obstacle.pose.position.y = obstacle_pose[i][1]*scale
				#print obstacle.scale
				#print obstacle.pose
				obstacle_pub_list.append(obstacle_pub)
				obstacle_list.append(obstacle)

			for i in range(num_obstacle):
				obstacle_pub_list[i].publish(obstacle_list[i])
		rospy.sleep(0.1)

if __name__ == '__main__':
    main(sys.argv)