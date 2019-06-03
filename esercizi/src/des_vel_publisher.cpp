#include <string>
#include "ros/ros.h"
#include "esercizi/vel_command.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv){

	esercizi::vel_command cmd;

	ros::init(argc, argv, "des_vel_publisher");
	ros::NodeHandle nh("~");
	ros::Publisher pub = nh.advertise<esercizi::vel_command>("/vel_requested", 10);
	ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/aaa", 10);
	ros::Rate rate(1);

	cmd.lin_vel = std::stof(argv[1]);
	cmd.ang_vel = std::stof(argv[2]);



	while (ros::ok()){
		ros::spinOnce();
		cmd.header.stamp = ros::Time::now();
		pub.publish(cmd);
		
	}

	return 0;

}