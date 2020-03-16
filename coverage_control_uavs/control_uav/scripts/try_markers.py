#!/usr/bin/env python

import roslib#; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray

topic = 'visualization_marker_array'
marker_publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)
point_publisher = rospy.Publisher('viz_point',PointCloud,queue_size=10)
point2_lvl1_publisher = rospy.Publisher('viz_point2_lvl1',PointCloud2,queue_size=10)
point2_lvl2_publisher = rospy.Publisher('viz_point2_lvl2',PointCloud2,queue_size=10)
point2_lvl3_publisher = rospy.Publisher('viz_point2_lvl3',PointCloud2,queue_size=10)
point2_lvl4_publisher = rospy.Publisher('viz_point2_lvl4',PointCloud2,queue_size=10)
point2_lvl5_publisher = rospy.Publisher('viz_point2_lvl5',PointCloud2,queue_size=10)
rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

GridStart = 0      
GridEnd = 1
GridStep = 0.02

def computeSensoryFunction(x,y,GridEnd):
  #nx = 1 #len(x)
  #ny = 1 #len(y)
  #f = np.zeros((nx,ny))
  #for i in range(0,nx):
  # for j in range(0,ny):
  #f = 20*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)+5*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+5*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)
  #fun1:
  f = 20*np.exp(-((x-GridEnd*0.5)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.85)**2+(y-GridEnd*0.3)**2)/(GridEnd*0.2)**2)+12*np.exp(-((x-GridEnd*0.5)**2+(y-GridEnd*0.6)**2)/(GridEnd*0.2)**2)
  #fun2:
  #f = 20*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)+7*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+7*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)
  
  return f

Xaxis = np.arange(GridStart,GridEnd+GridStep,GridStep).reshape(1,int((GridEnd-GridStart)/GridStep+1))
Xaxis = np.transpose(Xaxis)
# values of the y axis
Yaxis = Xaxis
TF = np.zeros((len(Xaxis),len(Yaxis)))
for h in xrange(len(Xaxis)):
  for k in xrange(len(Yaxis)):
    TF[k,h] = computeSensoryFunction(Xaxis[h],Yaxis[k],GridEnd)
est_sensory_function_current = np.zeros((len(Xaxis),len(Yaxis)))
points_lvl1 = []
points_lvl2 = []
points_lvl3 = []
points_lvl4 = []
points_lvl5 = []
maxTF = np.max(np.max(est_sensory_function_current))
minTF = np.min(np.min(est_sensory_function_current))
print maxTF
print minTF
gapTF = (maxTF-minTF)/5.0
print gapTF
threshold_lvl1_2 = minTF+gapTF
threshold_lvl2_3 = minTF+2*gapTF
threshold_lvl3_4 = minTF+3*gapTF
threshold_lvl4_5 = minTF+4*gapTF

for h in range(len(Xaxis)):
  for k in range(len(Yaxis)):
    if(TF[k,h] < threshold_lvl1_2):
      points_lvl1.append([Xaxis[h]*50,Yaxis[k]*50,3])
    elif(TF[k,h] > threshold_lvl1_2) and (TF[k,h] < threshold_lvl2_3):
      points_lvl2.append([Xaxis[h]*50,Yaxis[k]*50,3])
    elif(TF[k,h] > threshold_lvl2_3) and (TF[k,h] < threshold_lvl3_4):
      points_lvl3.append([Xaxis[h]*50,Yaxis[k]*50,3])
    elif(TF[k,h] > threshold_lvl3_4) and (TF[k,h] < threshold_lvl4_5):
      points_lvl4.append([Xaxis[h]*50,Yaxis[k]*50,3])
    else:
      points_lvl5.append([Xaxis[h]*50,Yaxis[k]*50,3])
    #pointss.append([h*10-5,k*10-5,0])
header = std_msgs.msg.Header()
header.stamp = rospy.Time.now()
header.frame_id = "world"
scaled_polygon_pcl_lvl = [0,0,0,0,0]
scaled_polygon_pcl_lvl[0] = pcl2.create_cloud_xyz32(header, points_lvl1)
scaled_polygon_pcl_lvl[1] = pcl2.create_cloud_xyz32(header, points_lvl2)
scaled_polygon_pcl_lvl[2] = pcl2.create_cloud_xyz32(header, points_lvl3)
scaled_polygon_pcl_lvl[3] = pcl2.create_cloud_xyz32(header, points_lvl4)
scaled_polygon_pcl_lvl[4] = pcl2.create_cloud_xyz32(header, points_lvl5)

def est_sensory_function_cb(msg_var):
  global scaled_polygon_pcl_lvl
  est_sensory_function_current = np.asarray(msg_var.data).reshape((51,51))
  #print est_sensory_function_current
  points_lvl1 = []
  points_lvl2 = []
  points_lvl3 = []
  points_lvl4 = []
  points_lvl5 = []
  maxTF = np.max(np.max(est_sensory_function_current))
  minTF = np.min(np.min(est_sensory_function_current))
  gapTF = (maxTF-minTF)/5.0
  threshold_lvl1_2 = minTF+gapTF
  threshold_lvl2_3 = minTF+2*gapTF
  threshold_lvl3_4 = minTF+3*gapTF
  threshold_lvl4_5 = minTF+4*gapTF
  for h in range(len(Xaxis)):
    for k in range(len(Yaxis)):
      if(est_sensory_function_current[k,h] < threshold_lvl1_2):
        points_lvl1.append([Xaxis[h]*50,Yaxis[k]*50,3])
      elif(est_sensory_function_current[k,h] > threshold_lvl1_2) and (est_sensory_function_current[k,h] < threshold_lvl2_3):
        points_lvl2.append([Xaxis[h]*50,Yaxis[k]*50,3])
      elif(est_sensory_function_current[k,h] > threshold_lvl2_3) and (est_sensory_function_current[k,h] < threshold_lvl3_4):
        points_lvl3.append([Xaxis[h]*50,Yaxis[k]*50,3])
      elif(est_sensory_function_current[k,h] > threshold_lvl3_4) and (est_sensory_function_current[k,h] < threshold_lvl4_5):
        points_lvl4.append([Xaxis[h]*50,Yaxis[k]*50,3])
      else:
        points_lvl5.append([Xaxis[h]*50,Yaxis[k]*50,3])

  scaled_polygon_pcl_lvl[0] = pcl2.create_cloud_xyz32(header, points_lvl1)
  scaled_polygon_pcl_lvl[1] = pcl2.create_cloud_xyz32(header, points_lvl2)
  scaled_polygon_pcl_lvl[2] = pcl2.create_cloud_xyz32(header, points_lvl3)
  scaled_polygon_pcl_lvl[3] = pcl2.create_cloud_xyz32(header, points_lvl4)
  scaled_polygon_pcl_lvl[4] = pcl2.create_cloud_xyz32(header, points_lvl5)

est_sensory_function_sub = rospy.Subscriber("/uav4/est_sensory_function",Float32MultiArray,queue_size=10,callback=est_sensory_function_cb)

while not rospy.is_shutdown():

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
   
   #point_publisher.publish(point)
   point2_lvl1_publisher.publish(scaled_polygon_pcl_lvl[0])
   point2_lvl2_publisher.publish(scaled_polygon_pcl_lvl[1])
   point2_lvl3_publisher.publish(scaled_polygon_pcl_lvl[2])
   point2_lvl4_publisher.publish(scaled_polygon_pcl_lvl[3])
   point2_lvl5_publisher.publish(scaled_polygon_pcl_lvl[4])

   rospy.sleep(0.01)