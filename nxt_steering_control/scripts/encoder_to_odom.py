#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_steering_control')  
import rospy
import math
import thread
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState
from math import sin, cos

global my_lock
my_lock = thread.allocate_lock()

class Converter:
  def __init__(self):
        self.sub = rospy.Subscriber('joint_state', JointState, self.sub_cb)
        self.pub = rospy.Publisher('odom', Odometry)
        self.lastEncoderBackWheel = 0
        self.steeringAngle =0
        self.ratioGearSteering = 0.2
        self.ratioGearBackWheel = -1.652
        self.radiusWheel = 0.021
        self.wheelBaseLenght = 0.3
        self.wheelBaseWidht = 0.12
        self.Angle = 0
        self.X = 0
        self.Y = 0

  def sub_cb(self, msg):
	if (msg.name[0] == 'steering_motor'):
	  self.steeringAngle = msg.position[0]*self.ratioGearSteering
	if (msg.name[0] == 'power_motor'):
	  encoderBackWheel = msg.position[0]
	  deltaEncoderBackWheel = encoderBackWheel - self.lastEncoderBackWheel
	  self.lastEncoderBackWheel = encoderBackWheel;
	  deltaTravel = self.radiusWheel*deltaEncoderBackWheel*self.ratioGearBackWheel
	  
	  if (self.steeringAngle == 0):
	    deltaAngle = 0
	    radiusCurve = 'nan'
	  else:
	    radiusCurve = self.wheelBaseWidht/2 + self.wheelBaseLenght/math.sin(self.steeringAngle)
	    deltaAngle = deltaTravel/radiusCurve;
	  self.Angle = self.Angle + deltaAngle;
	  self.X = self.X + deltaTravel*math.cos(self.Angle)
	  self.Y = self.Y + deltaTravel*math.sin(self.Angle)
	  #rospy.loginfo('Radius curve='+str(radiusCurve)+' Angle='+str(self.Angle)+' X='+str(self.X)+' Y='+str(self.Y))
  
  def publish_odom(self):
	  odom = Odometry()
	  odom.header.stamp = rospy.Time.now()
	  odom.pose.pose.position.x = self.X;
	  odom.pose.pose.position.y = self.Y;
	  odom.pose.pose.position.z = 0.0;
	  odom.pose.pose.orientation.x = 0.0;
	  odom.pose.pose.orientation.y = 0.0;
	  odom.pose.pose.orientation.z = self.Angle;
	  odom.pose.pose.orientation.w = 1.0;
	  odom.pose.covariance = [0.001, 0, 0, 0, 0, 0,
				  0, 0.001, 0, 0, 0, 0, 
				  0, 0, 0.001, 0, 0, 0,
				  0, 0, 0, 0.001, 0, 0,
				  0, 0, 0, 0, 0.001, 0,
				  0, 0, 0, 0, 0, 0.001] 
	  self.pub.publish(odom)

def main():
    rospy.init_node('encoder_to_odom')
    rospy.loginfo('Init odometry')
    converter = Converter()
    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()
    while not rospy.is_shutdown():
        my_lock.acquire()
        converter.publish_odom()
        my_lock.release()
        now = rospy.Time.now()
        if (now - last_callback_handle).to_sec() > 1.0/callback_handle_frequency:
            last_callback_handle = now
            rospy.sleep(0.01)




if __name__ == '__main__':
    main()
