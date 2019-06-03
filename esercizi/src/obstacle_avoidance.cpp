#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

#define RADIUS		1.5	// Radius of the circle centered in the obstacle turtle, in which the repulsive force acts
#define V_SAT 		2	// Linear velocity control saturation
#define W_SAT 		2   // Angular velocity control saturation

class obstacles_avoidance{
	private:
		float x1, y1, theta1;
		float x2, y2;
		float x3, y3;

		ros::NodeHandle nh;
		ros::Subscriber sub1;
		ros::Subscriber sub2;
		ros::Subscriber sub3;
		ros::Publisher pub;
		void Callback1(const turtlesim::Pose::ConstPtr& msg);
		void Callback2(const turtlesim::Pose::ConstPtr& msg);
		void Callback3(const turtlesim::Pose::ConstPtr& msg);

	public:
		explicit obstacles_avoidance(ros::NodeHandle m_nh) : nh(m_nh){
			sub1 = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1, &obstacles_avoidance::Callback1, this);
			sub2 = nh.subscribe<turtlesim::Pose>("/turtle2/pose",1, &obstacles_avoidance::Callback2, this);
			sub3 = nh.subscribe<turtlesim::Pose>("/turtle3/pose",1, &obstacles_avoidance::Callback3, this);
			pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
		}

		void apply_control(float xg, float yg){

			geometry_msgs::Twist cmd;
			float fx, fy;	// Artifitial force components
			float d2, d3;	// Distance of turtle1 from turtle2 and turtle3 respectively
			float Ka = 1;	// Attraction control constant
			float Kr = 1;	// Repulsion control constant
			float Kth = 2;	// Angle control constant

			// Compute distances 
			d2 = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
			d3 = sqrt(pow(x1-x3,2) + pow(y1-y3,2));

			// Add attraction to (xg,yg)
			fx = Ka * (xg - x1);
			fy = Ka * (yg - y1);

			// Add repulsion from turtle2 if the distance from it is lower than RADIUS
			if (d2 < RADIUS){
				fx += Kr / pow(d2,4) * (1 - d2/RADIUS) * (x1 - x2);
				fy += Kr / pow(d2,4) * (1 - d2/RADIUS) * (y1 - y2);
			}

			// Add repulsion from turtle3 if the distance from it is lower than RADIUS
			if (d3 < RADIUS){
				fx += Kr / pow(d3,4) * (1 - d3/RADIUS) * (x1 - x3);
				fy += Kr / pow(d3,4) * (1 - d3/RADIUS) * (y1 - y3);
			}

			// Calculate velocity control 
			cmd.linear.x = fx * cos(theta1) + fy * sin(theta1);
			cmd.angular.z = Kth * (atan2(fy,fx) - theta1);

			// Saturate control
			cmd.linear.x = (cmd.linear.x > V_SAT) ? V_SAT : cmd.linear.x;
			cmd.angular.z = (cmd.angular.z > W_SAT) ? W_SAT : cmd.angular.z;

			pub.publish(cmd);

		}

};

// Get coordinates from turtle1
void obstacles_avoidance::Callback1(const turtlesim::Pose::ConstPtr& msg){
	x1 = msg->x;
	y1 = msg->y;
	theta1 = msg->theta;
	ROS_INFO("Turtle 1 coordinates: x1: %f y1: %f", x1, y1);
}

// Get coordinates from turtle2
void obstacles_avoidance::Callback2(const turtlesim::Pose::ConstPtr& msg){
	x2 = msg -> x;
	y2 = msg -> y;
}

// Get coordinates from turtle3
void obstacles_avoidance::Callback3(const turtlesim::Pose::ConstPtr& msg){
	x3 = msg -> x;
	y3 = msg -> y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacles_avoidance");
	ros::NodeHandle nh("~");
	ros::Rate rate(50);

	// Variable containing the goal point coordinates
	std::string xg, yg;		// used to retreive from parameters
	float x_goal, y_goal;	// used to compute control actions

	// Get goal point coordinates from parameters
	if (nh.getParam("x_goal", xg)){
		// Erase the first and the last item of the string (it should be a "'") and set float saturations
		xg.erase(xg.begin(), xg.begin() + 1);
		xg.erase(xg.end()-1,xg.end());
		x_goal = std::stof(xg);
	}

	if (nh.getParam("y_goal", yg)){
		// Erase the first and the last item of the string (it should be a "'") and set float saturations
		yg.erase(yg.begin(), yg.begin() + 1);
		yg.erase(yg.end()-1,yg.end());
		y_goal = std::stof(yg);
	}

	obstacles_avoidance ob_av(nh);

	sleep(2);

	while(ros::ok()){
		ros::spinOnce();
		ob_av.apply_control(x_goal, y_goal);
		rate.sleep();
	}
	return 0;
}