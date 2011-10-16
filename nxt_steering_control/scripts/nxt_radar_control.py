#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_steering_control')  
import rospy
import math
import thread
from art_msgs.msg import CarDriveStamped
from nxt_rosjava_msgs.msg import JointCommand, Range
from sensor_msgs.msg import JointState, PointCloud
from geometry_msgs.msg import Point32
from math import sin, cos

global my_lock
my_lock = thread.allocate_lock()

class NXT_radar:
  def __init__(self):
        self.sub_js = rospy.Subscriber('joint_state', JointState, self.sub_cb_js)
        self.sub_us = rospy.Subscriber('ultrasonic_sensor', Range, self.sub_cb_us)
        self.pub_PC = rospy.Publisher('cloud_topic', PointCloud)
        self.pub_JC = rospy.Publisher('joint_command', JointCommand)
        self.init_radar_motor = False
	self.last_angle_cmd = 450;
	self.last_range = 0;


  def sub_cb_js(self, msg):
	if (msg.name[0] == 'radar_motor'):
	    #angle = msg.position[0]*math.pi/950;
	    angle = msg.position[0]/5;
	    rospy.loginfo("Angle /%f/ Range /%f/",angle, self.last_range)
	    pnt = PointCloud()
	    pnt.header.frame_id = "base_link"
	    pnt.header.stamp = rospy.Time.now()
	    pnt.points.append(Point32(self.last_range*cos(angle), self.last_range*sin(angle), 0))
	    self.pub_PC.publish(pnt)
	    #control radar
	    if ((msg.position[0]*180/math.pi > self.last_angle_cmd - 5 and self.last_angle_cmd>0) or (msg.position[0]*180/math.pi < self.last_angle_cmd + 5 and self.last_angle_cmd<0) or (self.init_radar_motor == False)):
	      self.init_radar_motor = True;
	      JC = JointCommand();
	      JC.name = 'radar_motor';
	      JC.type = 'to_angle'; 
	      JC.speed = 400;
	      self.last_angle_cmd = -self.last_angle_cmd;
	      JC.angle = self.last_angle_cmd;
	      self.pub_JC.publish(JC);	 	      

	 
  def sub_cb_us(self, msg):
      if (msg.range <> self.last_range):
	self.last_range = msg.range/100;

    
	  

def main():
    rospy.init_node('nxt_radar_control')
    nxt_radar = NXT_radar()
    rospy.spin()

if __name__ == '__main__':
    main()
