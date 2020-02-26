#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#include "Robot.hpp"
#include <signal.h>
#include "curses.h"
#include <locale.h>

void shutdown(int sig)
{
  ROS_INFO("MTracker ROS driver shutdown");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker");
  ros::NodeHandle n;
  Robot robot;
  ros::Publisher  pos_pub = n.advertise<geometry_msgs::Pose2D>("/pos", 100);
  ros::Publisher  vel_pub = n.advertise<geometry_msgs::Twist>("/vel", 100);
  ros::Subscriber ctrl_sub = n.subscribe("/cmd_vel", 10, &Robot::controlsCallback, &robot);

  ros::ServiceServer trig_srv = n.advertiseService("/trigger_motors", &Robot::triggerCallback, &robot);

 

  geometry_msgs::Pose2D odom;
  odom.x = 1;
  odom.y = 1;
  odom.theta = 2;
  
  ROS_INFO("MTracker ROS driver start");

  robot.openPort();
  robot.stopWheels();
  robot.setOdometry(0.0f, 0.0f, 0.0f);

  signal(SIGINT, shutdown);

  ros::Rate rate(20.0);

  while (ros::ok())
  {
    ros::spinOnce();
    if (robot.readFrame())
    pos_pub.publish(odom);
      //ROS_INFO("I've got data.");
      
      robot.pos_odom = odom;
      robot.vel_odom = robot.getVelocity();
      
      robot.publishPose(pos_pub);
      robot.publishVelocity(vel_pub);

    if (robot.motors_on)
    {
        robot.setVelocity(robot.w_l, robot.w_r);
    }
    else
      robot.switchOffMotors();
      

    rate.sleep();
  }

  return 0;
}
