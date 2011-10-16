/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.nxt_rosjava;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import lejos.nxt.*;
import lejos.nxt.remote.RemoteMotor;

import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import java.util.*;


abstract class Device {
	
	double desired_period;
	org.ros.message.Time last_run;
	boolean initialized;
	Node node_owner;
	String name;
	String frame_id;

	Device(double frequency, Node node, String name_dev, String _frame_id){
		desired_period = 1 / frequency;
		initialized = false;
		node_owner = node;
		name = name_dev;
		frame_id = _frame_id;
	}
	
	boolean needs_trigger(Log log_2){
        if (!initialized){
            initialized = true;
            last_run = node_owner.getCurrentTime();
            return false;
        }
        
        org.ros.message.Time curTime = node_owner.getCurrentTime();
        double period = (curTime.secs + curTime.nsecs/1000000000.0) - (last_run.secs + last_run.nsecs/1000000000.0);
        //log_2.info("Device: "+name+" Period: " + period +" Desired period: "+desired_period);
        return period > desired_period;
	}
	
	abstract void do_trigger();
	
}

class UltraSonicSensorNXT extends Device {
	
	UltrasonicSensor sonic;
	Publisher<org.ros.message.nxt_rosjava_msgs.Range> publisher;
	
	UltraSonicSensorNXT(int port, double frequency, Node node, String name_dev, String _frame_id) {
		super(frequency, node, name_dev, _frame_id);
		switch (port){
			case 1: sonic = new UltrasonicSensor(SensorPort.S1); break;
			case 2: sonic = new UltrasonicSensor(SensorPort.S2); break;
			case 3: sonic = new UltrasonicSensor(SensorPort.S3); break;
			case 4: sonic = new UltrasonicSensor(SensorPort.S4); break;
		}
		
	    publisher = node.newPublisher("ultrasonic_sensor", "nxt_rosjava_msgs/Range");
	}
	
	void do_trigger(){
		last_run = node_owner.getCurrentTime();
        org.ros.message.nxt_rosjava_msgs.Range msg = new org.ros.message.nxt_rosjava_msgs.Range();
        msg.header.stamp = last_run;
        msg.header.frame_id = frame_id;
        msg.range = sonic.getDistance();
        publisher.publish(msg);		
	}
	
}

class TouchSensorNXT extends Device {
	
	TouchSensor touch;
	Publisher<org.ros.message.nxt_rosjava_msgs.Contact> publisher;
	
	TouchSensorNXT(int port, double frequency, Node node, String name_dev, String _frame_id) {
		super(frequency, node, name_dev, _frame_id);
		switch (port){
			case 1: touch = new TouchSensor(SensorPort.S1); break;
			case 2: touch = new TouchSensor(SensorPort.S2); break;
			case 3: touch = new TouchSensor(SensorPort.S3); break;
			case 4: touch = new TouchSensor(SensorPort.S4); break;
		}
		
	    publisher = node.newPublisher("touch", "nxt_rosjava_msgs/Contact");
	}
	
	void do_trigger(){
		last_run = node_owner.getCurrentTime();
        org.ros.message.nxt_rosjava_msgs.Contact msg = new org.ros.message.nxt_rosjava_msgs.Contact();
        msg.header.stamp = last_run;
        msg.header.frame_id = frame_id;
        msg.contact = touch.isPressed();
        publisher.publish(msg);		
	}
	
}

class MotorNXT extends Device {
	
	RemoteMotor motor;
	Publisher<org.ros.message.sensor_msgs.JointState> publisher;
	double POWER_TO_NM;
	double POWER_MAX;
	
	MotorNXT(int port, double frequency, Node node, String name_dev) {
		super(frequency, node, name_dev, "");
		POWER_TO_NM = 0.01;
		POWER_MAX = 100;
		switch (port){
			case 1: motor = new Motor().A; break;
			case 2: motor = new Motor().B; break;
			case 3: motor = new Motor().C; break;
		}
		
	    publisher = node.newPublisher("joint_state", "sensor_msgs/JointState");
	    node.newSubscriber("joint_command", "nxt_rosjava_msgs/JointCommand",
	              new MessageListener<org.ros.message.nxt_rosjava_msgs.JointCommand>() {
	                @Override
	                public void onNewMessage(org.ros.message.nxt_rosjava_msgs.JointCommand message) {
	                	if (message.name.contains(name)){
	                		if (message.type.contains("effort")){
			                	double power = message.effort / POWER_TO_NM;
			                    if (power > POWER_MAX)
			                    	power = POWER_MAX;
			                    else if (power < -POWER_MAX)
			                    	power = -POWER_MAX;
			                	power = power>0 ? power : -power; 
			                	motor.setPower((int)power);
			                	if (message.effort>0){
			                		motor.forward();
			                	}else{
			                		motor.backward();
			                	}
		                	}
		                	else if (message.type.contains("speed")){
			                	double speed = message.speed>0 ? message.speed : -message.speed; 
			                	motor.setSpeed((int)speed);
			                	if (message.speed>0){
			                		motor.forward();
			                	}else{
			                		motor.backward();
			                	}
		                	}
		                	else if (message.type.contains("on_angle")){
		                		motor.rotate((int) message.angle, true); 
		                	}
		                	else if (message.type.contains("to_angle")){
		                		motor.setSpeed((int)message.speed);
		                		motor.rotateTo((int) message.angle, true); 
		                	}	
	                	}
	                	
	                }
	              });
	}
	
	
	
	
	void do_trigger(){
		
		last_run = node_owner.getCurrentTime();
		org.ros.message.sensor_msgs.JointState msg = new org.ros.message.sensor_msgs.JointState();
		msg.header.stamp = node_owner.getCurrentTime();
		msg.name.add(name);
		msg.position = new double[1];
		msg.effort = new double[1];
		msg.velocity = new double[1];
		msg.position[0] = (double) motor.getTachoCount()* Math.PI / 180.0;
		msg.effort[0] = (double) motor.getPower()* POWER_TO_NM;
		msg.velocity[0] = (double) motor.getSpeed();
        publisher.publish(msg);
        		
	}
	
}

public class Listener implements NodeMain {

  private Node node;
  ArrayList<Device> lst_devices;

  @Override
  public void main(NodeConfiguration configuration) {
    try {
      node = new DefaultNodeFactory().newNode("listener", configuration);
      final Log log = node.getLog();
      ParameterTree param = node.newParameterTree();
      GraphName paramNamespace = new GraphName(param.getString("parameter_namespace"));
      NameResolver resolver = node.getResolver().createResolver(paramNamespace);
      Map setttings_map = param.getMap(resolver.resolve("setttings"));
      Object[] list = param.getList(resolver.resolve("list")).toArray();
      lst_devices = new ArrayList<Device>();
      for (int i = 0; i < list.length; i++) { 
    	  String type = (String) ((Map) setttings_map.get(list[i])).get("type");
    	  String name_dev = (String) ((Map) setttings_map.get(list[i])).get("name");
    	  String frame_id = (String) ((Map) setttings_map.get(list[i])).get("frame_id");
    	  double tmp_port = (Double) ((Map) setttings_map.get(list[i])).get("port");
    	  int port = (int) tmp_port;
    	  double desired_frequency = (Double) ((Map) setttings_map.get(list[i])).get("desired_frequency");
    	  log.info("Device: " + list[i] + " type: " + type + " frequency: "+desired_frequency);
    	  if (type.contains("ultrasonic")){
    		  UltraSonicSensorNXT dev = new UltraSonicSensorNXT(port, desired_frequency, node, name_dev, frame_id);
    		  lst_devices.add(dev);  
    	  }
    	  if (type.contains("motor")){
    		  MotorNXT dev = new MotorNXT(port, (int)desired_frequency, node, name_dev);
    		  lst_devices.add(dev);  
    	  }
    	  if (type.contains("touch")){
    		  TouchSensorNXT dev = new TouchSensorNXT(port, (int)desired_frequency, node, name_dev, frame_id);
    		  lst_devices.add(dev);  
    	  }
      }
      while (true) {
          for(int i = 0; i<lst_devices.size(); i++) {
        	  Device curDevice = lst_devices.get(i);
        	  //log.info("Check device:"+curDevice.name);
        	  if (curDevice.needs_trigger(log)){
        		  //log.info("Trigger device:"+curDevice.name);
        		  curDevice.do_trigger();
        	  }
          }
          
          Thread.sleep(10);
        }
    } catch (Exception e) {
      if (node != null) {
        node.getLog().fatal(e);
      } else {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void shutdown() {
    node.shutdown();
  }

}
