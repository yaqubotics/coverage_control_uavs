#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import json

def spawn_cylinder(pos,size,id,scale):
  cylinder_sdf_model = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='unit_cylinder'>
    <link name='link'>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>0.145833</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.145833</iyy>
          <iyz>0</iyz>
          <izz>0.125</izz>
        </inertia>
      </inertial>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>"""+repr(size[0]*scale)+"""</radius>
            <length>"""+repr(size[1]*scale)+"""</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <shader type='pixel'/>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.01 0.01 0.01 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <cylinder>
            <radius>"""+repr(size[0]*scale)+"""</radius>
            <length>"""+repr(size[1]*scale)+"""</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>
    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
  """
  req = SpawnModelRequest()
  req.model_name = 'cylinder_'+repr(id)
  req.model_xml = cylinder_sdf_model
  req.robot_namespace = ""
  req.reference_frame = ""
  req.initial_pose = Pose()
  req.initial_pose.position.x = pos[0]*scale
  req.initial_pose.position.y = pos[1]*scale
  req.initial_pose.position.z = pos[2]*scale+0.5*size[1]*scale
  req.initial_pose.orientation.x = pos[3]*scale
  req.initial_pose.orientation.y = pos[4]*scale
  req.initial_pose.orientation.z = pos[5]*scale
  req.initial_pose.orientation.w = pos[6]*scale
  return req

def spawn_box(pos,size,id,scale):
  box_sdf_model = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='unit_box'>
    <link name='link'>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>0.166667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.166667</iyy>
          <iyz>0</iyz>
          <izz>0.166667</izz>
        </inertia>
      </inertial>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <visual name='visual'>
        <geometry>
          <box>
            <size>"""+repr(size[0]*scale)+""" """+repr(size[1]*scale)+""" """+repr(size[2]*scale)+"""</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <shader type='pixel'/>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.01 0.01 0.01 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>"""+repr(size[0]*scale)+""" """+repr(size[1]*scale)+""" """+repr(size[2]*scale)+"""</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>
    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
  """
  req = SpawnModelRequest()
  req.model_name = 'box_'+repr(id)
  req.model_xml = box_sdf_model
  req.robot_namespace = ""
  req.reference_frame = ""
  req.initial_pose = Pose()
  req.initial_pose.position.x = pos[0]*scale
  req.initial_pose.position.y = pos[1]*scale
  req.initial_pose.position.z = pos[2]*scale+0.5*size[2]*scale
  req.initial_pose.orientation.x = pos[3]*scale
  req.initial_pose.orientation.y = pos[4]*scale
  req.initial_pose.orientation.z = pos[5]*scale
  req.initial_pose.orientation.w = pos[6]*scale
  return req


def spawn_ground_plane(id,scale):
  scale = scale+2
  box_sdf_model = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='unit_box'>
    <link name='link'>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>0.166667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.166667</iyy>
          <iyz>0</iyz>
          <izz>0.166667</izz>
        </inertia>
      </inertial>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <visual name='visual'>
        <geometry>
          <box>
            <size>"""+repr(scale)+""" """+repr(scale)+""" """+repr(0.01)+"""</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <shader type='pixel'/>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.01 0.01 0.01 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>"""+repr(scale)+""" """+repr(scale)+""" """+repr(0.01)+"""</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>
    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
  """
  req = SpawnModelRequest()
  req.model_name = 'box_'+repr(id)
  req.model_xml = box_sdf_model
  req.robot_namespace = ""
  req.reference_frame = ""
  req.initial_pose = Pose()
  req.initial_pose.position.x = scale/2-1
  req.initial_pose.position.y = scale/2-1
  req.initial_pose.position.z = -0.005
  return req

if __name__ == '__main__':
    rospy.init_node('test_spawn')
    #json config
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print dir_path
    with open(dir_path+'/../../control_uav/scripts/config.json','r') as f:
      config = json.load(f)
    uav_ID = "2"
    scale = config["DEFAULT"]["REAL_SCALE"]
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_srv.wait_for_service()
    req = spawn_ground_plane(100,scale)
    rospy.loginfo("Call: " + str(req))
    resp = spawn_srv.call(req)
    rospy.loginfo("Response: " + str(resp))
    if(config["DEFAULT"]["USING_OBSTACLE"] == True):
      
      num_obstacle = len(config["OBSTACLE"])
      obstacle_size = []
      obstacle_pose = []
      obstacle_type = []
      for i in range(num_obstacle):
        obstacle_type.append(config["OBSTACLE"][repr(i)]["TYPE"])
        obstacle_size.append(config["OBSTACLE"][repr(i)]["SIZE"])
        obstacle_pose.append(config["OBSTACLE"][repr(i)]["POSE"])

      
      for i in range(num_obstacle):
        if(obstacle_type[i] == "BOX"):
          req = spawn_box(obstacle_pose[i],obstacle_size[i],i,scale)
          rospy.loginfo("Call: " + str(req))
          resp = spawn_srv.call(req)
          rospy.loginfo("Response: " + str(resp))
        elif(obstacle_type[i] == "CYLINDER"):
          req = spawn_cylinder(obstacle_pose[i],obstacle_size[i],i,scale)
          rospy.loginfo("Call: " + str(req))
          resp = spawn_srv.call(req)
          rospy.loginfo("Response: " + str(resp))
        else:
          print "Obstacle type = "+obstacle_type[i]+"for id = "+repr(i)+"is not known"

