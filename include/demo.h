#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <articulated_client/ikAction.h>
#include "gripper_client.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"


class DemoClient
{
public:
	DemoClient(std::string ik_server_name);

private:
	//FUNCTIONS
	bool grabBlock();
	bool depositBlock();
	bool checkObstacle();
	bool moveRobot(articulated_client::ikGoal goal);
	bool broadFilter(articulated_client::ikGoal goal);

	//OBJECTS
	ros::NodeHandle nh_;
	
	GripperClient gc_; //Custom client w/ premade methods

	actionlib::SimpleActionClient<articulated_client::ikAction> ik_client_;
	tf::TransformListener transform_listener_;
	
	//VARS
	bool gripper_state_;
	tf::StampedTransform world_robot_transform_; //Used to put goals in robot perspective

};