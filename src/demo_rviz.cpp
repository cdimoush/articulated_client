#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

class DemoVizualizer
{
public:
	DemoVizualizer();

private:
	visualization_msgs::Marker block_marker_;
	ros::Publisher marker_pub_;
	tf::TransformBroadcaster br_;

	ros::NodeHandle nh_;
};

DemoVizualizer::DemoVizualizer()
{
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	float x_block, y_block, z_block, d_block;
	nh_.getParam("/demo/block/x", x_block);
	nh_.getParam("/demo/block/y", y_block);
	nh_.getParam("/demo/block/z", z_block);
	nh_.getParam("/demo/block/dimension", d_block);

	//Create TF
	tf::Transform tf_block;
  	tf_block.setOrigin(tf::Vector3(x_block, y_block, z_block));
  	tf::Quaternion q;
  	q.setRPY(0, 0, 0);
  	tf_block.setRotation(q);
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

	while (ros::ok())
	{
		br_.sendTransform(tf::StampedTransform(tf_block, ros::Time::now(), "world", "block"));
		marker_pub_.publish(block_marker_);
		ros::spinOnce();
		ros::Duration(0.01).sleep();	
	}
}

int main (int argc, char **argv)
{
	ROS_ERROR_STREAM("DEMO VIZ");
	ros::init(argc, argv, "demo_visualizer");
	DemoVizualizer dv;

	return 0;
}