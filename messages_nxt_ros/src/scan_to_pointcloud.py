#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_ros')  
import rospy
import math
import thread
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from math import sin, cos

class Converter:
    def __init__(self):
        self.sub = rospy.Subscriber('mess_reader', String, self.sub_cb)
        self.pub = rospy.Publisher('cloud_topic', PointCloud)

    def sub_cb(self, msg):
	if (msg.data[:4]=="scan"): 
	        pnt = PointCloud()
        	pnt.header.frame_id = "base_link"
	        pnt.header.stamp = rospy.Time.now()
		tmp = msg.data[5:]
		i_tz = tmp.find(";") 
		while (i_tz>0):
			rospy.loginfo("tmp /%s/",tmp)
			str_point = tmp[:i_tz]
			tmp = tmp[i_tz+1:]
			i_tz = tmp.find(";") 
			i_space = str_point.find(" ")
			str_angle = str_point[:i_space]
			str_range = str_point[i_space+1:]
			rospy.loginfo("Str /%s/ Angle /%s/ Range /%s/",str_point,str_angle,str_range)
			angle =float(str_angle)*math.pi/950;
			rng = float(str_range)/100;
			rospy.loginfo("Angle /%f/ Range /%f/",angle, rng)
	       	        pnt.points.append(Point32(rng*cos(angle), rng*sin(angle), 0))
			rospy.loginfo("Add point x=/%f/ y=/%f/", rng*cos(angle), rng*sin(angle) )
        	self.pub.publish(pnt)

def main():
    rospy.init_node('scan_to_pointcloud')
    converter = Converter()
    rospy.spin()



if __name__ == '__main__':
    main()
