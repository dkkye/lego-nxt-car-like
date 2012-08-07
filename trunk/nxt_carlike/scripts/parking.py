#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_carlike')  
import rospy
import math
import thread
from art_msgs.msg import CarDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from math import *

class Converter:
    def __init__(self):
        self.sub = rospy.Subscriber('parking', String, self.cmd_vel_cb)
        self.sub_us = rospy.Subscriber('ultrasonic_sensor', Range, self.sub_cb_us)
        self.sub_ir = rospy.Subscriber('IR_sensor', Range, self.sub_cb_ir)
        #self.sub_odom = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.odom_cb)     
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb)    
        self.pub = rospy.Publisher('pilot/drive', CarDriveStamped)
        self.pub_park = rospy.Publisher('parking', String)
        self.xt =0
        self.yt =0
        self.state =0 #States 0 - idle, 1 - find park place, 2 - parking
        self.n_manevr = 0
        self.xc = 0 #current x coordinate from odometry
        self.yc = 0 #current y coordinate from odometry
        self.thetac=0 #current angle from odometry
        self.min_length_park_place = 0.4
        self.length_park_place = 0.1
        self.depth_park_place = 0.3
        self.LengthFreePlace =0
        self.oldX=0
        self.oldY=0
        self.last_range = 0
        self.safety_dist = 0.08
        self.isFree= 0
        self.wheelBaseLenght = 0.23
        self.freeDistanceBehind = 0.2
        self.lastDiffRange = 0
	self.LastTime = 0
    
    def sub_cb_us(self, msg):
	if (msg.range <> self.last_range):
	    self.last_range = msg.range;
	    #rospy.loginfo('Range ='+str(self.last_range))
	    if self.state==1:
		if self.last_range < self.depth_park_place+self.safety_dist:
		    self.LengthFreePlace =0.1
		    self.isFree= 0
		    rospy.loginfo('NOT free')
		else:
		    self.isFree= 1
		    rospy.loginfo('Free! LengthFreePlace='+str(self.LengthFreePlace))
	if self.state==1:
	  if msg.range < 2*self.safety_dist:
	      dRangeDt = 0
	      differenceRange = msg.range - 1.2*self.safety_dist
	      if self.lastDiffRange<> 0:
		dRangeDt = (differenceRange - self.lastDiffRange)/(rospy.get_time() - self.lastTime)
	      self.lastTime = rospy.get_time()
	      CD = CarDriveStamped();
	      CD.control.steering_angle = 3 * differenceRange + 1*dRangeDt 
	      CD.control.speed = 2
	      rospy.loginfo('Control direction Angle='+str(CD.control.steering_angle)+' DiffRange='+str(differenceRange) + ' Range='+str(msg.range) + ' DRangeDt='+str(dRangeDt))
	      self.pub.publish(CD)
	      self.lastDiffRange = differenceRange;
	  else:
	      CD = CarDriveStamped();
	      CD.control.steering_angle = 0.0
	      CD.control.speed = 2
	      rospy.loginfo('Control direction off')
	      self.pub.publish(CD)	      
	      
	      
	  

    def sub_cb_ir(self, msg):
	if (msg.range < self.freeDistanceBehind):
	  if self.state == 2:
	    self.pub_park.publish("stop")
	    #rospy.loginfo('Stop! Free dist behind: '+str(msg.range))
	    
    def odom_cb(self, msg):
        self.xc = msg.pose.pose.position.x
        self.yc = msg.pose.pose.position.y
        self.thetac=msg.pose.pose.orientation.z
	#rospy.loginfo('xc-xt='+ str(abs(self.xc-self.xt))+ 'yc-yt=' + str(abs(self.yc-self.yt)));
        if abs(self.xc-self.xt)<0.04  and self.state==2 and self.n_manevr ==1: #and abs(self.yc-self.yt)<0.1
            rospy.loginfo('It is target x='+ str(self.xc)+ 'y=' + str(self.yc)+ 'theta='+ str(self.thetac));
	    self.state =0
	    self.pub_park.publish("target")
	if abs(self.xc-self.xt)<0.04 and abs(self.yc-self.yt)<0.1 and self.state==2 and self.n_manevr ==4:
            rospy.loginfo('It is target x='+ str(self.xc)+ 'y=' + str(self.yc)+ 'theta='+ str(self.thetac));
	    self.state =0
	    self.pub_park.publish("end")
	if abs(self.xc-self.xt)<0.04 and self.state==2 and self.n_manevr ==2:
            rospy.loginfo('It is target x='+ str(self.xc)+ 'y=' + str(self.yc)+ 'theta='+ str(self.thetac));
	    self.state =0
	    self.pub_park.publish("target")
	if self.thetac>0.1 and self.thetac<0.15  and self.state==2 and self.n_manevr ==3:
            rospy.loginfo('It is target x='+ str(self.xc)+ 'y=' + str(self.yc)+ 'theta='+ str(self.thetac));
	    self.state =0
	    self.pub_park.publish("target")
	if self.state == 1 and self.isFree==1:
	    self.LengthFreePlace += sqrt((self.xc-self.oldX)*(self.xc-self.oldX) + (self.yc-self.oldY)*(self.yc-self.oldY))
	    #rospy.loginfo('LengthFreePlace='+str(self.LengthFreePlace))
	    if self.LengthFreePlace >= self.min_length_park_place:
		self.length_park_place = self.LengthFreePlace
		self.state=0
		self.pub_park.publish("found")
	self.oldX = self.xc
	self.oldY = self.yc  
	  
	
	
    def cmd_vel_cb(self, msg):
        cmd=msg.data[0:6]
        rospy.loginfo(cmd);
        if cmd=='startP':
	    self.state = 1
            rospy.loginfo('Find');
            CD = CarDriveStamped();
            CD.control.steering_angle = 0.;
            CD.control.speed = 2;
            self.pub.publish(CD);
        if cmd=='found':     
            self.n_manevr =1
            rospy.loginfo('Manevr 1');
	    self.xt = self.xc + 0.4
            self.yt = self.yc
            self.state=2
            CD = CarDriveStamped();
            CD.control.steering_angle = 0;
            CD.control.speed = 1.5;
            self.pub.publish(CD);
        if cmd=='target':        
            if self.n_manevr ==1:
		#stop before park
		self.state = 0
		CD = CarDriveStamped();
		CD.control.steering_angle = 0;
		CD.control.speed = 0;
		self.pub.publish(CD);
		#calculate steer angles for park
		xk=self.xc-0.5
		yk=self.yc+0.5*self.depth_park_place+self.safety_dist
		[self.alpha1, self.alpha2, self.x1, self.y1, self.x2, self.y2]=self.algorithm_park(self.xc, self.yc, 0., xk, yk, 0.)
		rospy.loginfo('xc='+ str(self.xc)+ 'yc=' + str(self.yc)+ 'xk='+ str(xk)+ 'yk='+ str(yk));
		rospy.loginfo('x1='+ str(self.x1)+ 'y1=' + str(self.y1)+ 'x2='+ str(self.x2)+ 'y2='+ str(self.y2));
		rospy.loginfo('alpha1='+ str(self.alpha1)+ 'alpha2=' + str(self.alpha2));
		self.xt = self.x1
		self.yt = self.y1
		self.n_manevr =2
		self.state = 2
		rospy.loginfo('Manevr 2');
		#rotate steer wheels
		CD = CarDriveStamped();
		CD.control.steering_angle = radians(self.alpha1)
		CD.control.speed = 0
		self.pub.publish(CD)
		rospy.sleep(0.1)
		#switch on power motor
		CD = CarDriveStamped()
		CD.control.steering_angle = radians(self.alpha1)
		CD.control.speed = -1.5
		self.pub.publish(CD)
            elif self.n_manevr ==2:
                self.xt = self.x2
                self.yt = self.y2
                self.n_manevr=3
                rospy.loginfo('Manevr 3');
		self.state = 2
                CD = CarDriveStamped();
                CD.control.steering_angle = radians(self.alpha2);
                CD.control.speed = -1.5;
		self.pub.publish(CD);	
	    else:
		CD = CarDriveStamped()
		CD.control.steering_angle = 0.0
		CD.control.speed = 0.0
		self.pub.publish(CD)
	if cmd=='stop':
	    rospy.loginfo('Manevr 4');
	    self.state = 2
	    self.n_manevr=4
	    self.xt = self.xc + 0.08
            self.yt = self.yc
            CD = CarDriveStamped();
            CD.control.steering_angle = radians(15);
            CD.control.speed = 1.5;
	    self.pub.publish(CD);
	if cmd=='end':
	    CD = CarDriveStamped()
	    CD.control.steering_angle = 0.0
	    CD.control.speed = 0.0
	    self.pub.publish(CD)
	    
		 		  
	  
    def algorithm_park(self,x0, y0, theta0, xk, yk, thetak):
        dt_ackerman=5;
        dt_lin=-0.02;
        k_x=1.0;
        k_y=1.0;
        k_theta=0.5;
        score_min=99999;
        for alpha1 in range (10,40, dt_ackerman):
            beta = tan(radians(alpha1))*dt_lin/self.wheelBaseLenght
            for shag in range (15,30,1):
                theta1=theta0;
                x1=x0;
                y1=y0;
		for i in range(shag):
		    if beta<0.001:
			x1 = x1 + dt_lin*cos(theta1)
			y1 = y1 + dt_lin*sin(theta1)
			theta1=theta1+beta;
		    else:
			r = dt_lin/beta
			center_x = x1 - sin(theta1)*r
			center_y = y1 + cos(theta1)*r
			theta1 = (theta1 + beta)
			x1 = center_x + r*sin(theta1)
			y1 = center_y - r*cos(theta1)

		for alpha2 in range (-10,-40, -dt_ackerman):
			x2=x1;
			y2=y1
			theta2=theta1; 
			beta2 = tan(radians(alpha2))*dt_lin/self.wheelBaseLenght
			i2=0
			while abs(x2-xk)>0.02 and abs(y2-yk)>0.02 and i2<25:
				if beta2<0.001:
					x2 = x2 + dt_lin*cos(theta2)
					y2 = y2 + dt_lin*sin(theta2)
					theta2=theta2+beta2
				else:
					r2 = dt_lin/beta2
					center_x = x2 - sin(theta2)*r2
					center_y = y2 + cos(theta2)*r2
					theta2 = (theta2 + beta2)
					x2 = center_x + r2*sin(theta2)
					y2 = center_y - r2*cos(theta2)
				i2=i2+1
			score=abs(x2-xk)*k_x+abs(y2-yk)*k_y+abs(theta2-thetak)*k_theta
			if score<score_min:
				alpha1_best=alpha1
				alpha2_best=alpha2
				x1_best=x1
				y1_best=y1
				x2_best=x2
				y2_best=y2
				theta_best=theta2
				score_min=score
        return alpha1_best, alpha2_best, x1_best, y1_best, x2_best, y2_best

def main():
    rospy.init_node('parking')
    converter = Converter()
    rospy.spin()

if __name__ == '__main__':
    main()
