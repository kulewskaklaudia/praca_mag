#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "curses.h"
#include <locale.h>

class Controller {
public:
  float x, y, theta;
  float t;
  geometry_msgs::Twist controls;

  ros::Publisher  ctrl_pub;
  ros::Subscriber pos_sub;

  void posCallback(const geometry_msgs::Pose2D::ConstPtr& pos_msg)
  {
    this->x = pos_msg->x;
    this->y = pos_msg->y;
    this->theta = pos_msg->theta;
  }

  void computeControls()
  {
      printf("***************\n");
      int c;
      c = getch();
      printf("%d\n", c);
        switch (c) {
        case 259:
          printf("Up\n");
          controls.linear.x = 0.1;
           break;
        case 339:
          printf("UpRight\n");
          controls.linear.x = 0.1;
          controls.angular.z = 0.3;
           break;
        case 261:  
          printf("Right\n");
          controls.linear.x = 0.05;
          controls.angular.z = 0.5;
          break;
        case 338:  
          printf("DownRight\n");
          controls.linear.x = -0.1;
          controls.angular.z = -0.3;
          break;
        case 258:   
          printf("Down\n"); 
          controls.linear.x = -0.1;
          break;
        case 360:   
          printf("DownLeft\n"); 
          controls.linear.x = -0.1;
          controls.angular.z = 0.3;
          break;
        case 260:   
          printf("Left\n");
          controls.linear.x = 0.05;
          controls.angular.z = 0.5;
          break;
        case 262:   
          printf("UpLeft\n");
          controls.linear.x = 0.05;
          controls.angular.z = -0.5;
          break;
        case 350:
          controls.linear.x = 0.0;
          controls.angular.z = 0.0;
          break;
        case 263:
        return 0;
        default:         printf("Code 0x%x\n\r", c);
        }
 
        refresh();
      
    // HERE PUT THE CODE

    //controls.linear.x = 0.0;
    //controls.angular.z = 0.0;
  }
};


int main(int argc, char **argv)
{
  Controller c;

  ros::init(argc, argv, "mtracker_controller");
  ros::NodeHandle n;

  c.ctrl_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  c.pos_sub = n.subscribe("/pos", 10, &Controller::posCallback, &c);

  ROS_INFO("MTracker Controller");

  ros::Rate rate(20.0);
 
    /* Initialize locale, for wide-character support. */
    setlocale(LC_ALL, "");
 
    /* Start curses mode. */
    initscr();
 
    /* No line buffering. */
    raw();
 
    /* No echoing keypresses to the screen. */
    //noecho();
 
    /* We want F1 et cetera. */
    keypad(stdscr, TRUE);
 
    /* We "print" stuff to the screen,
     * moving the cursor to the next line '\n',
     * and to the beginning of that line '\r'. */
    printw("Press Q to quit.\n\r");
 
    /* We do need to tell the curses library
     * to update the changes made thus far,
     * i.e. that all changes are visible on screen. */
    refresh();

  while (ros::ok())
  {
    ros::spinOnce();

    c.computeControls();
    c.ctrl_pub.publish(c.controls);

    rate.sleep();
  }

  return 0;
}
