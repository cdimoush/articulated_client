#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Empty.h"

#include <tf/tf.h>

class DemoVisualizer
{
public:
	DemoVisualizer();

private:
	//FUNCTIONS
	void spawnCB(geometry_msgs::Pose b_pose);
	void destroyCB(std_msgs::Empty e);
	void buildMarker();

	//OBJECTS
	ros::NodeHandle nh_;
	visualization_msgs::Marker block_marker_;
	ros::Publisher marker_pub_;
	ros::Subscriber spawn_sub_;
	ros::Subscriber destroy_sub_;
	tf::TransformBroadcaster br_;
	
	//VARS
	int b_count_;
	geometry_msgs::Pose b_pose_;
	tf::Transform b_tf_;
};

DemoVisualizer::DemoVisualizer()
{
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	spawn_sub_ = nh_.subscribe("/demo/spawn", 1000, &DemoVisualizer::spawnCB, this);
	destroy_sub_ = nh_.subscribe("/demo/destroy", 1000, &DemoVisualizer::destroyCB, this);

	b_count_ = 0;
  	buildMarker();

	while (ros::ok())
	{
		if (b_count_ == 1)
		{
			br_.sendTransform(tf::StampedTransform(b_tf_, ros::Time::now(), "world", "block"));
			marker_pub_.publish(block_marker_);
		}

		ros::spinOnce();
		ros::Duration(0.01).sleep();	
	}
}

void DemoVisualizer::buildMarker()
{
	float d_block;
	nh_.getParam("demo/block/dimension", d_block);
	//Create Marker
	block_marker_.id = 0;
	block_marker_.type = visualization_msgs::Marker::CUBE;
	block_marker_.action = visualization_msgs::Marker::ADD;
	block_marker_.header.frame_id = "block";
	block_marker_.scale.x = d_block;
	block_marker_.scale.y = d_block;
	block_marker_.scale.z = d_block;
	block_marker_.lifetime = ros::Duration(0.2);
	block_marker_.pose.orientation.x = 0;
	block_marker_.pose.orientation.y = 0;
	block_marker_.pose.orientation.z = 0;
	block_marker_.pose.orientation.w = 1.0;
	block_marker_.color.r = 0.5f;
	block_marker_.color.g = 1.0f;
	block_marker_.color.b = 1.0f;
	block_marker_.color.a = 1.0;
}

void DemoVisualizer::spawnCB(geometry_msgs::Pose b_pose)
{
	ROS_INFO("BLOCK SPAWNING");
	ROS_INFO("--------------");
	if (b_count_ != 0)
	{
		ROS_ERROR_STREAM("WARNING: AT BLOCK LIMIT");
	}
	else
	{
		float x, y, z;
		x = b_pose.position.x; 
		y = b_pose.position.y; 
		z = b_pose.position.z; 
		b_tf_.setOrigin(tf::Vector3(x, y, z));
	  	tf::Quaternion q;
	  	q.setRPY(0, 0, 0);
	  	b_tf_.setRotation(q);

	  	b_count_ ++;
	}
}

void DemoVisualizer::destroyCB(std_msgs::Empty e)
{
	b_count_ = 0;
}


int main (int argc, char **argv)
{
	ROS_ERROR_STREAM("DEMO VIZ");
	ros::init(argc, argv, "demo_visualizer");
	DemoVisualizer dv;

	return 0;
}