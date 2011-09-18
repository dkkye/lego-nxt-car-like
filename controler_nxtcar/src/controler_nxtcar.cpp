
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <costmap_2d/costmap_2d_ros.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class ControlerCarNxt
{
public:
  ControlerCarNxt();
  void keyLoop();
  void watchdog();

private:

  
  ros::NodeHandle nh_,ph_;
  ros::Time last_publish_;
  ros::Publisher msg_pub_;
  boost::mutex publish_mutex_;

};

ControlerCarNxt::ControlerCarNxt()
{
  msg_pub_ = nh_.advertise<std_msgs::String>("mess_writer", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "controler_nxtcar");
  ControlerCarNxt controler_carnxt;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&ControlerCarNxt::keyLoop, &controler_carnxt));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&ControlerCarNxt::watchdog, &controler_carnxt));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


void ControlerCarNxt::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
}

void ControlerCarNxt::keyLoop()
{
  char c;
  std_msgs::String msg;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the carnxt.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
	msg.data = "steering_rotate 60 ";
        msg_pub_.publish(msg);
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
	msg.data = "steering_rotate -60 ";
        msg_pub_.publish(msg);
        break;
      case 115:
        ROS_DEBUG("Scan");
	msg.data = "scan 360 ";
        msg_pub_.publish(msg);
        break;
      case KEYCODE_D:
        ROS_DEBUG("Move");
	msg.data = "move 360 ";
        msg_pub_.publish(msg);
        break;
      case KEYCODE_U:
        ROS_DEBUG("Move");
	msg.data = "move -360 ";
        msg_pub_.publish(msg);
        break;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    last_publish_ = ros::Time::now();
    
  }

  return;
}





