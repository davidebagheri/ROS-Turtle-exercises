#include <cmath>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

#define DISTANCE	0.7		// range of turtles' sensors
#define FLOOR		5		// Y coordinate of the bounding floor
#define CEILING		11		// Y coordinate of the bounding ceiling
#define WALL_RIGHT	11		// X coordinate of the bounding right wall
#define WALL_LEFT	0		// X coordinate of the bounding left wall
#define SAT_P		3		// angular velocity positive saturation
#define SAT_N		-3		// angular velocity negative saturation
#define FORWARD_VEL	1		// forward velocity

class wall_avoidance{
	public:
		explicit wall_avoidance(ros::NodeHandle m_nh) : nh(m_nh){
			sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1, &wall_avoidance::Callback, this);
			pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
		}

	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;

		void Callback(const turtlesim::Pose::ConstPtr& msg);
};

void wall_avoidance::Callback(const turtlesim::Pose::ConstPtr& msg){
	// Retreive current position and angle 
	float x = msg -> x;
	float y = msg -> y;
	float theta = msg -> theta;

	// Potential forces
	float fy = 0;	
	float fx = 0;

	float k = 3; // Potential constant
	geometry_msgs::Twist cmd; // Velocity command

	// Feedforward control used if the turtle falls into a local equilibrium (maps angles)
	if (((y >= (CEILING - DISTANCE)) || (y <= FLOOR + DISTANCE)) && ((x <= WALL_LEFT + DISTANCE) || (x >= WALL_RIGHT - DISTANCE))){
		cmd.linear.x = -FORWARD_VEL;
		pub.publish(cmd);
		sleep(1);
		cmd.angular.z = SAT_P/2;
		cmd.linear.x = 0;
		pub.publish(cmd);
		sleep(1);
		return;
	}

	// if the turtle gets near the upper or lower bound
	else if ((y >= (CEILING - DISTANCE)) || (y <= FLOOR + DISTANCE)){
		 cmd.angular.z = k * (atan2(copysign(1.0, (FLOOR+CEILING)/2-y), 0) - theta);	// theta_dot = k*(angle_ref - theta), angle_ref = pi/2 (floor) or 3/2pi (ceiling)
		 ROS_INFO("NEAR Y BOUND!!!");
	}

	// if the turtle gets near the right or left bound
	else if ((x <= WALL_LEFT + DISTANCE) || (x >= WALL_RIGHT - DISTANCE)){
		cmd.angular.z = k * (atan2(0, copysign(1.0, WALL_RIGHT/2-x)) - theta);	// theta_dot = k*(angle_ref - theta), angle_ref = 0 (left wall) or pi (right wall)
		 ROS_INFO("NEAR X BOUND!!!");
	}

	// Apply positive and negative saturations
	cmd.angular.z = (cmd.angular.z > SAT_P) ? SAT_P : cmd.angular.z;	
	cmd.angular.z = (cmd.angular.z < SAT_N) ? SAT_N : cmd.angular.z;

	// Constant forward velocity
	cmd.linear.x = FORWARD_VEL;

	pub.publish(cmd);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "wall_avoidance");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	wall_avoidance wall_avoidance_controller(nh);	

	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
