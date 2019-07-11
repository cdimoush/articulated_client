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

	float x_goal[3] = {0.185, 0.165, 0.165};
	float y_goal[3] = {0.05, 0.05, 0.075};
	float z_goal[3] = {0.0, 0.0, 0.0};
	while (ros::ok())
	{
		for (int i = 0; i < 10; i ++)
		{
		// send a goal to the action
			articulated_client::ikGoal goal;
			goal.x = 0.165;
			goal.y = 0.005*i + 0.025;
			goal.z = 0;
			ac.sendGoal(goal);

			bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

			if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Action finished: %s",state.toString().c_str());
			}
			else
				ROS_INFO("Action did not finish before the time out.");


			ros::Rate r(1);
			r.sleep();
		}
	}

	//exit
	return 0;

}