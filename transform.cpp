#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

ros::Publisher pcl_pub;
void pclCb(const sensor_msgs::PointCloud2 &msg)
{
	Eigen::Matrix4f transform;
	sensor_msgs::PointCloud2 out;
	transform << 0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1; //Rotate camera optical to NEU i.e. Z -pi/2 X -pi/2
	
	tf::Transform camera_to_world;
	camera_to_world.setBasis(tf::Matrix3x3(0,0,1,-1,0,0,0,-1,0));

	pcl_ros::transformPointCloud(transform.matrix(), msg, out);

	tf::TransformListener listener;
	tf::StampedTransform transformed_frame;
	try 
	{
	    listener.waitForTransform("world", "camera", ros::Time(0), ros::Duration(10.0) );
	    listener.lookupTransform("world", "camera", ros::Time(0), transformed_frame);
	} 
	catch (tf::TransformException ex) 
	{
	    ROS_ERROR("%s",ex.what());
	}

	static tf::TransformBroadcaster br;
	tf::Transform camera_to_ros = camera_to_world * transformed_frame * camera_to_world.inverse();
	if (~std::isnan(camera_to_ros.getOrigin().x()) || ~std::isnan(camera_to_ros.getOrigin().y()) || ~std::isnan(camera_to_ros.getOrigin().z()))
	{
		br.sendTransform(tf::StampedTransform(camera_to_ros, ros::Time::now(), "world", "camera_transformed"));
		out.header.frame_id = "camera_transformed";
		pcl_pub.publish(out);
	}
	else
		ROS_ERROR("is NAN");
	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_transform");
	ros::NodeHandle n;

	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pcl_out", 100);
	ros::Subscriber pcl_sub = n.subscribe("/orb_slam2_rgbd/map_points", 100, pclCb);

	ros::spin();

	return 0;
}