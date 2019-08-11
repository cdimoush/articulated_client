#include "demo.h"

DemoClient::DemoClient(std::string ik_server_name) :
	ik_client_(ik_server_name, true)
{
	//ENSURE ROBOT HAS GRIPPER
	bool gripper_bool;
	nh_.getParam("/articulated/gripper/bool", gripper_bool);
	if (!gripper_bool)
	{
		ROS_ERROR_STREAM("ROBOT DOES NOT HAVE GRIPPER (check parameters");
		ROS_ERROR_STREAM("demo_client: shutdown");
		ros::shutdown();
		exit(0);
	}

	destroy_pub_ = nh_.advertise<std_msgs::Empty>("demo/destroy", 1000);

	//SET UP TRANSFORMS
	//-----------------
	//get world to robot
	transform_listener_.waitForTransform("/world", "/joint1", ros::Time(0), ros::Duration(3.0));
	transform_listener_.lookupTransform("/world", "/joint1", ros::Time(0), world_robot_transform_);
	tf::StampedTransform block_transform;
	ros::Time block_detection_time = ros::Time::now();
	while (ros::ok())
	{	try
		{
			transform_listener_.lookupTransform("/world", "/block", ros::Time(0), block_transform);
			if (block_detection_time < block_transform.stamp_)
			{
				block_detection_time = block_transform.stamp_;

				ROS_INFO("DEMO: New Block Detected");
				if (grabBlock(block_transform))
				{
					ROS_INFO("DEMO: Blocked has been grabbed");
				}
				std_msgs::Empty destroy_block;
				destroy_pub_.publish(destroy_block);
				ros::Duration(3).sleep();
				block_detection_time = ros::Time::now();
			}
		}
		catch (tf::TransformException ex)
		{
      		ros::Duration(1.0).sleep();
    	}
		
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}	
}

bool DemoClient::grabBlock(tf::StampedTransform b_tf)
{
	ROS_INFO("DEMO: grab block called");

	//SETUP GRIPPER FOR DEMO
	//----------------------
	nh_.getParam("/articulated/gripper/state", gripper_state_);
	if (gripper_state_) //Gripper is closed
	{
		//Open the gripper
		gc_.toggleGripper(gripper_state_);
	}

	//STEP 1: POSITION GRIPPER JUST ABOVE BLOCK
	//STEP 1A: DETERMINE XYZ TO ACCOMPLISH STEP 1
	float x_block, y_block, z_block, d_block;
	x_block = b_tf.getOrigin().x();
	y_block = b_tf.getOrigin().y();
	z_block = b_tf.getOrigin().z();
	y_block = y_block - world_robot_transform_.getOrigin().y(); //Put in robot coordinate system
	nh_.getParam("/demo/block/dimension", d_block);
	//CHECK IF XYZ IS INSIDE CONFIGURATION SPACE
	//CHECK OBSTICALE
	if (checkObstacle())
	{
		//Generate path (give up for now)
		return false;
	}

	articulated_client::ikGoal goal;
	goal.x = x_block;
	goal.y = y_block + 1.5 * d_block;
	goal.z = z_block;
	if (!moveRobot(goal))
	{
		return false;
	}

	goal.y = y_block;
	if (!moveRobot(goal))
	{
		return false;
	}

	nh_.getParam("/articulated/gripper/state", gripper_state_);
	gc_.toggleGripper(gripper_state_);
	goal.y = y_block + 2 * d_block;
	if (!moveRobot(goal))
	{
		return false;
	}

	goal.z= 0;
	if (!moveRobot(goal))
	{
		return false;
	}

	return true;
}

bool DemoClient::checkObstacle()
{
	return false; //assume no obstacle for now....
}

bool DemoClient::moveRobot(articulated_client::ikGoal goal)
{
	ik_client_.waitForServer();
	ROS_ERROR_STREAM("Move robot with goal...");
	ROS_ERROR_STREAM("X: " << goal.x << ", Y: " << goal.y << ", Z: " << goal.z);
	if (!broadFilter(goal))
	{
		ROS_ERROR_STREAM("ERROR: Goal failed in broad filter");
		return false;
	}

	ik_client_.sendGoal(goal);
	bool finished_before_timeout = ik_client_.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ik_client_.getState();
		ROS_ERROR_STREAM(state.toString().c_str());
		if (state.toString() == "SUCCEEDED")
		{
			return true;
		}
		return false;
	}

	ROS_ERROR_STREAM("moveRobot failed");
	return false;
}

bool DemoClient::broadFilter(articulated_client::ikGoal goal)
{
	float x, y, z;
	x = goal.x;
	y = goal.y;
	z = goal.z;
	float l1, l2, g;
	nh_.getParam("/articulated/joint/1/transform", l1);
	nh_.getParam("/articulated/joint/2/transform", l2);
	nh_.getParam("/articulated/gripper/transform", g);
	l2 = l2+g;
	
	if (0 <= z || z <= 0.5)
	{
		//ROS_ERROR_STREAM("z range is good");
		if (0 < x || x < l1+l2)
		{
			//ROS_ERROR_STREAM("x range is good");
			if (pow(x,2)+pow(y,2) > pow((l1-l2),2))
			{
				//ROS_ERROR_STREAM("Out side small");
				if (pow(x,2)+pow(y,2) < pow((l2+l2),2))
				{
					//ROS_ERROR_STREAM("inside big");
					return true;
				}
			}
		}
	}

	return false;
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "demo_client");
	DemoClient dc("ik_action");
	//actionlib::SimpleActionClient<articulated_client::ikAction> ik_client_("ik_action", true);

	return 0;

}