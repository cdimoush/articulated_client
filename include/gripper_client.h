#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <articulated_client/gripperAction.h>

class GripperClient
{
public:
	GripperClient();
	bool toggleGripper(bool gripper_sate);
private:
};

GripperClient::GripperClient()
{
}

bool GripperClient::toggleGripper(bool gripper_state)
{
	actionlib::SimpleActionClient<articulated_client::gripperAction>gc("gripper_action", true);

	ROS_INFO("Looking for Gripper Action Server");
	// wait for the action server to start
	gc.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server Found");
	articulated_client::gripperGoal goal;
	
	if (gripper_state) //Gripper is currently CLOSED
	{
		ROS_INFO("Goal: Open Gripper");
		goal.close = false;
		goal.open = true;
	}
	else //Gripper is currently OPEN
	{
		ROS_INFO("Goal: Close Gripper");
		goal.close = true;
		goal.open = false;
	}

	ROS_INFO("Goal: Sending");
	gc.sendGoal(goal);

	bool finished_before_timeout = gc.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = gc.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		return true;
	}	
	

	return false;
}