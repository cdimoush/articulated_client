#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <articulated_client/gripperAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_client");
	ros::NodeHandle nh_;
	bool gripper_bool;
	bool gripper_state;
	nh_.getParam("/articulated/gripper/bool", gripper_bool);
	nh_.getParam("/articulated/gripper/state", gripper_state);

	if (gripper_bool)
	{
		actionlib::SimpleActionClient<articulated_client::gripperAction>gc("gripper_action", true);

		ROS_INFO("Waiting for action server to start.");
		// wait for the action server to start
		gc.waitForServer(); //will wait for infinite time
		ROS_INFO("Action server started");
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
		}	
	}
	else
	{
		ROS_ERROR_STREAM("ROBOT DOES NOT HAVE GRIPPER (check parameters)");
	}

	return 0;
}