"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Twist, TwistStamped
import math
import numpy
from std_msgs.msg import Header
from PID import PID
from tf.transformations import euler_from_quaternion
import math
import numpy as np

def deg2rad(inp):
    return inp*np.pi/180.0
        
class VelocityController:
    target = PoseStamped()
    output = TwistStamped()

    def __init__(self):
        self.X = PID()
        self.Y = PID()
        self.Z = PID()
        self.PHI = PID()
        self.THETA = PID()
        self.PSI = PID()
        self.lastTime = rospy.get_time()
        self.target = None
        self.prev_target = None
        self.heading_target = None

    
    
    def setTarget(self, target):
        self.target = target

    def setHeadingTarget(self,target):
        self.heading_target = target

    def distance(self,a,b):
        #return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)
        a = (a.position.x,a.position.y,a.position.z)
        b = (b.position.x,b.position.y,b.position.z)
        return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)



    def update(self, state):
        if (self.target is None):
            rospy.logwarn("Target position for velocity controller is none.")
            return None
        # simplify variables a bit
        time = state.header.stamp.to_sec()
        position = state.pose.position
        orientation = state.pose.orientation
        quat_arr = np.array([orientation.x,orientation.y,orientation.z,orientation.w])
        att = euler_from_quaternion(quat_arr,'sxyz')
        # create output structure
        output = TwistStamped()
        output.header = state.header
        # output velocities
        linear = Vector3()
        angular = Vector3()
        if(False): #abs(att[2]-self.heading_target) > deg2rad(5.0)):
            # Control in X vel
            linear.x = self.X.update(position.x, position.x, time)
            # Control in Y vel
            linear.y = self.Y.update(position.y, position.y, time)
            # Control in Z vel
            linear.z = self.Z.update(position.z, position.z, time)
        else:
            # Control in X vel
            linear.x = self.X.update(self.target.position.x, position.x, time)
            # Control in Y vel
            linear.y = self.Y.update(self.target.position.y, position.y, time)
            # Control in Z vel
            linear.z = self.Z.update(self.target.position.z, position.z, time)
        # Control yaw (no x, y angular)

        delta = 0.3
        #if((abs(linear.y) > delta) and (abs(linear.x) > delta)) or (self.psi_target == None):
        output.twist = Twist()
        #if(self.distance(self.target,state.pose)>delta):
        phi_target = 0.0
        theta_target = 0.0
        psi_target = self.heading_target #np.arctan((self.target.position.y-self.prev_target.position.y)/(self.target.position.x-self.prev_target.position.x))#(linear.y,linear.x)
        #psi_target = np.arctan((self.target.position.y-self.prev_target.position.y)/(self.target.position.x-self.prev_target.position.x))#(linear.y,linear.x)
        #psi_target = math.atan2(linear.y,linear.x)
        
        #angular.x = self.PHI.update(phi_target, att[0],time)
        #angular.y = self.THETA.update(theta_target, att[1],time)
        angular.z = self.PSI.update(psi_target, att[2],time)
        output.twist.angular = angular
       
        # TODO
        
        output.twist.linear = linear
        if output == None:
            print "output",output

        return output
    def update_prop(self,state,gain):
        output = TwistStamped()
        output.header = state.header
        position = state.pose.position
        
        linear = Vector3()
        angular = Vector3()
        linear.x = gain*(self.target.position.x-position.x)
        # Control in Y vel
        linear.y = gain*(self.target.position.y-position.y)
        # Control in Z vel
        linear.z = gain*(self.target.position.z-position.z)
        # output velocities

        output.twist = Twist()
        output.twist.linear = linear
        return output
    def stop(self):
        setTarget(self.current)
        update(self.current)

    def setPIDGainX(self,p,i,d):
        self.X.setKp(p)
        self.X.setKi(i)
        self.X.setKd(d)

    def setPIDGainY(self,p,i,d):
        self.Y.setKp(p)
        self.Y.setKi(i)
        self.Y.setKd(d)

    def setPIDGainZ(self,p,i,d):
        self.Z.setKp(p)
        self.Z.setKi(i)
        self.Z.setKd(d)

    def setPIDGainPHI(self,p,i,d):
        self.PHI.setKp(p)
        self.PHI.setKi(i)
        self.PHI.setKd(d)

    def setPIDGainTHETA(self,p,i,d):
        self.THETA.setKp(p)
        self.THETA.setKi(i)
        self.THETA.setKd(d)

    def setPIDGainPSI(self,p,i,d):
        self.PSI.setKp(p)
        self.PSI.setKi(i)
        self.PSI.setKd(d)
