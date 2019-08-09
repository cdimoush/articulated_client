#include "demo.h"

DemoClient::DemoClient(std::string ik_server_name) :
	ik_client_(ik_server_name, true)
{

	bool gripper_bool;
	nh_.getParam("/articulated/gripper/bool", gripper_bool);
	nh_.getParam("/articulated/gripper/state", gripper_state_);
	if (!gripper_bool)
	{
		ROS_ERROR_STREAM("ROBOT DOES NOT HAVE GRIPPER (check parameters");
		ROS_ERROR_STREAM("demo_client: shutdown");
		ros::shutdown();
		exit(0);
	}
	if (gripper_state_) //Gripper is closed
	{
		//Open the gripper
		gc_.toggleGripper(gripper_state_);
	}

	if (grabBlock())
	{
		ROS_INFO("DEMO: Blocked has been grabbed");
	}

	while (ros::ok())
	{
		ros::spinOnce();
		ros::Duration(1).sleep();	
	}
}

bool DemoClient::grabBlock()
{
	ROS_INFO("DEMO: grab block called");
	//STEP 1: POSITION GRIPPER JUST ABOVE BLOCK
	//STEP 1A: DETERMINE XYZ TO ACCOMPLISH STEP 1
	float x, y, z;
	float b_dimension;
	nh_.getParam("/demo/block/x", x);
	nh_.getParam("/demo/block/y", y);
	nh_.getParam("/demo/block/z", z);
	nh_.getParam("/demo/block/dimension", b_dimension);
	y = y + 2*b_dimension;
	//CHECK IF XYZ IS INSIDE CONFIGURATION SPACE

	//CHECK OBSTICALE
	if (checkObstacle())
	{
		//Generate path (give up for now)
		return false;
	}


	articulated_client::ikGoal goal;
	ROS_ERROR_STREAM("X: " << x << ", Y: " << y << ", Z: " <<x);
	goal.x = x;
	goal.y = y;
	goal.z = z;
	ik_client_.waitForServer();
	ik_client_.sendGoal(goal);

	bool finished_before_timeout = ik_client_.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ik_client_.getState();
		return true;
	}
	return false;
}

bool DemoClient::checkObstacle()
{
	return false; //assume no obstacle for now....
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "demo_client");
	DemoClient dc("ik_action");
	//actionlib::SimpleActionClient<articulated_client::ikAction> ik_client_("ik_action", true);

	return 0;

}