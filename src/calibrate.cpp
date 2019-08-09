#include <ros/ros.h>
#include "sensor_msgs/JointState.h"


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <articulated_client/calibrateAction.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_TAB 0x09


//Add publisher (consider msg types)4
ros::Publisher cal_pub;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "calibrate");
  ros::NodeHandle nh_;
  cal_pub = nh_.advertise<sensor_msgs::JointState>("articulated/calibration", 1000);
  actionlib::SimpleActionClient<articulated_client::calibrateAction> ac("calibration_action", true);
  
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  if (ac.waitForServer()) //will wait for infinite time, return true if connected
  { 

    ROS_INFO("Calibration server started");
    //Send empty goal to start
    articulated_client::calibrateGoal goal;
    ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}
