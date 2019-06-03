#include <iostream>
#include "ros/ros.h"
#include <tf/tf.h>
#include "geometry_msgs/PoseStamped.h"
#include "turtlesim/Pose.h"

geometry_msgs::PoseStamped buffer;	// Contains the pose of the turtle

void pose_callback(const turtlesim::Pose::ConstPtr& msg){
	tf::Quaternion q;

	// Get current position
	buffer.pose.position.x = msg->x - 5.544444561; // Subtract the starting point coordinates 
	buffer.pose.position.y = msg->y - 5.544444561; 
	buffer.pose.position.z = 0;

	// Get current orientation
	q.setRPY(0,0,msg->theta);
	buffer.pose.orientation.x = q.getX();
	buffer.pose.orientation.y = q.getY();
	buffer.pose.orientation.z = q.getZ();
	buffer.pose.orientation.w = q.getW();

	// Set the reference frame and current time
	buffer.header.frame_id = "/map";
	buffer.header.stamp = ros::Time::now();
}



int main(int argc, char** argv){
	ros::init(argc, argv, "rviz_display");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, pose_callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("rviz_pose",1);
	ros::Rate rate(10);	

	while(ros::ok()){
		
		ros::spinOnce();
		pub.publish(buffer);
		rate.sleep();

	}

	return 0;
}
