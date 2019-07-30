#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <articulated_client/ikAction.h>


int main (int argc, char **argv)
{
	ros::init(argc, argv, "articulated_client");
	actionlib::SimpleActionClient<articulated_client::ikAction> ac("ik_action", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started");
	ROS_INFO("sending goal.");

	float x_goal[4] = {0.195, 0.165, 0.15, 0.165};
	float y_goal[4] = {0.0, 0.035, 0.085, 0.035};
	float z_goal[4] = {0.0, 0.5, 0.5, 0.0};
	while (ros::ok())
	{
		for (int i = 0; i < 4; i ++)
		{
		// send a goal to the action
			articulated_client::ikGoal goal;
			ROS_ERROR_STREAM("X: " << x_goal[i] << ", Y: " << y_goal[i]);
			goal.x = x_goal[i];
			goal.y = y_goal[i];
			goal.z = z_goal[i];
			ac.sendGoal(goal);

			bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

			if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Action finished: %s",state.toString().c_str());
			}
			else
				ROS_INFO("Action did not finish before the time out.");


			ros::Rate r(3);
			r.sleep();
		}
	}

	//exit
	return 0;

}