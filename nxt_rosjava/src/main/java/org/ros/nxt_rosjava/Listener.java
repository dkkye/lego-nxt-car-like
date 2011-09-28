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
//import org.ros.node.topic.Subscriber;

public class Listener implements NodeMain {

  private Node node;

  @Override
  public void main(NodeConfiguration configuration) {
    try {
      node = new DefaultNodeFactory().newNode("listener", configuration);
	  final Motor motor = new Motor();
	  //motor.A.setSpeed(300);
      final Log log = node.getLog();
      node.newSubscriber("cmd_vel", "geometry_msgs/Twist",
          new MessageListener<org.ros.message.geometry_msgs.Twist>() {
            @Override
            public void onNewMessage(org.ros.message.geometry_msgs.Twist message) {
              log.info("Linear: \"" + message.linear.x + "\"");
              if (message.linear.x == 1.0)
            	  motor.A.forward();
              if (message.linear.x == -1.0)
            	  motor.A.backward();
            }
          });
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
