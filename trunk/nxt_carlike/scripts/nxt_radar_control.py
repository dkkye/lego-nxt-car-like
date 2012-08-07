#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_carlike')  
import rospy
import math
import thread
from art_msgs.msg import CarDriveStamped
from nxt_lejos_ros_msgs.msg import JointPosition
from sensor_msgs.msg import JointState, PointCloud, LaserScan, Range
from geometry_msgs.msg import Point32
from math import sin, cos

global my_lock
my_lock = thread.allocate_lock()

class NXT_radar:
  def __init__(self):
        self.pub_JC = rospy.Publisher('joint_position', JointPosition)
        self.pub_PC = rospy.Publisher('cloud_topic', PointCloud)
        self.pub_LS = rospy.Publisher('base_scan', LaserScan)
        rospy.sleep(2.);
        self.sub_us = rospy.Subscriber('ultrasonic_sensor', Range, self.sub_cb_us)
        self.sub_js = rospy.Subscriber('radar_motor', JointState, self.sub_cb_js)
        self.init_radar_motor = False
	self.last_angle_cmd = 450;
	self.last_range = 0;
	self.count_ray = 0;


  def sub_cb_js(self, msg):
	if (msg.name[0] == 'radar_motor'):
	    #control radar
	    if ((msg.position[0]*180/math.pi > self.last_angle_cmd - 5 and self.last_angle_cmd>0) or (msg.position[0]*180/math.pi < self.last_angle_cmd + 5 and self.last_angle_cmd<0) or (self.init_radar_motor == False)):
	      rospy.loginfo('Init command radar');
	      if (self.init_radar_motor == True):
		#publish LaserScan message
		self.LS_msg.angle_max = -msg.position[0]/5;
		self.LS_msg.angle_increment = (self.LS_msg.angle_max - self.LS_msg.angle_min)/self.count_ray;
		self.LS_msg.time_increment = 0.01;
		self.pub_LS.publish(self.LS_msg);
	      self.init_radar_motor = True;
	      #new LaserScan message
	      self.count_ray = 0;
	      self.LS_msg = LaserScan();
	      self.LS_msg.header.frame_id = "base_link";
	      self.LS_msg.header.stamp = rospy.Time.now();
	      self.LS_msg.angle_min = -msg.position[0]/5;
	      self.LS_msg.range_min = 0.04;
	      self.LS_msg.range_max = 2.55;
	      
	      
	      #command for run motor radar
	      JC = JointPosition();
	      JC.name = 'radar_motor';
	      self.last_angle_cmd = -self.last_angle_cmd;
	      JC.angle = self.last_angle_cmd;
	      self.pub_JC.publish(JC);
	    self.count_ray = self.count_ray + 1;
	    angle = msg.position[0]/5;
	    #add ray to laserscan message
	    self.LS_msg.ranges.append(self.last_range);
	    
	    #publish PointCloud message
	    pnt = PointCloud()
	    pnt.header.frame_id = "base_link"
	    pnt.header.stamp = rospy.Time.now()
	    #rospy.loginfo('Range:'+str(self.last_range)+' Angle: '+str(angle)+' X: '+str(self.last_range*cos(angle))+' Y: '+str(self.last_range*sin(angle)));
	    pnt.points.append(Point32(self.last_range*cos(angle), -self.last_range*sin(angle), 0))
	    self.pub_PC.publish(pnt)

	 
  def sub_cb_us(self, msg):
      if (msg.range <> self.last_range):
	self.last_range = msg.range/100;

    
	  

def main():
    rospy.init_node('nxt_radar_control')
    nxt_radar = NXT_radar()
    rospy.spin()

if __name__ == '__main__':
    main()
