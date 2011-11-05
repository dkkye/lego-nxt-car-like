#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_steering_control')  
import rospy
import math
import thread
from art_msgs.msg import CarDriveStamped
from nxt_rosjava_msgs.msg import JointCommand
from math import sin, cos

global my_lock
my_lock = thread.allocate_lock()

class NXT_steering:
  def __init__(self):
        self.sub = rospy.Subscriber('pilot/drive', CarDriveStamped, self.sub_cb)
        self.pub = rospy.Publisher('joint_command', JointCommand)
	self.last_speed = 0;
	self.last_steering_angle= 0;
	self.ratioGearSteering = 10;

  def sub_cb(self, msg):
	if (msg.control.speed <> self.last_speed):
	  JC = JointCommand();
	  JC.name = 'power_motor';
	  JC.type = 'speed';
	  JC.speed = -msg.control.speed*100; #convert m/s to deg/s for Lego NXT_steering
	  self.last_speed = msg.control.speed;
	  self.pub.publish(JC)
	if (msg.control.steering_angle <> self.last_steering_angle):
	  JC = JointCommand();
	  JC.name = 'steering_motor';
	  JC.type = 'to_angle';
	  JC.angle = msg.control.steering_angle*360/(2*math.pi)*self.ratioGearSteering;
	  JC.speed = 400;
	  self.last_steering_angle = msg.control.steering_angle;
	  self.pub.publish(JC)	  
	  

def main():
    rospy.init_node('nxt_steering_control')
    nxt_steering = NXT_steering()
    rospy.spin()

if __name__ == '__main__':
    main()
