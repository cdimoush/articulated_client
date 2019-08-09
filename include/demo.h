#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <articulated_client/ikAction.h>
#include "gripper_client.h"


class DemoClient
{
public:
	DemoClient(std::string ik_server_name);

private:
	//FUNCTIONS
	bool grabBlock();
	bool depositBlock();
	bool checkObstacle();
	bool moveRobot();

	//OBJECTS
	ros::NodeHandle nh_;
	GripperClient gc_; //Custom client w/ premade methods

	actionlib::SimpleActionClient<articulated_client::ikAction> ik_client_;

	
	//VARS
	bool gripper_state_;

};