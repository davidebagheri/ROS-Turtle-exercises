#include <string>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "esercizi/TurtleStatus.h"
#include "esercizi/TurtleArrived.h"
#include "turtlesim/Pose.h"
#include "std_msgs/String.h"

#define RESOLUTION		0.1		// Distance error from target point accepted

struct status{
	bool is_ready = false;
	float x;
	float y;
};


// Subscribers
ros::Subscriber self_info_sub;
ros::Subscriber team_status_sub;
ros::Subscriber go_next_sub;

//Publishers
ros::Publisher team_status_pub;
ros::Publisher vel_pub;
ros::Publisher go_next_pub;

//Centroid coordinates
float x_avg = 0;
float y_avg = 0;

// Current coordinates
float x_cur;
float y_cur;
float theta_cur;

// Useful global variables
std::map<std::string,status>  robots_state;
bool team_ready = false;
std::string robot_name;								// Synchronization map
const int robot_number = 4; 						// Number of robots
std::map<std::string, bool> previous_robot_state;	// Map of arrived robots
bool has_arrived = false;

// Get turtle coordinates
void SelfInfoCallback(const turtlesim::Pose::ConstPtr& msg){
	x_cur = msg->x;
	y_cur = msg->y;
	theta_cur = msg->theta;
}

// Publish to the synchronization map if the turtle is ready or not and the coordinates
void publishReadyStatus(){
	esercizi::TurtleStatus status_msg;
	status_msg.header.stamp = ros::Time::now();
	status_msg.robot_id = robot_name;
	status_msg.is_ready = true;
	status_msg.x = x_cur;
	status_msg.y = y_cur;

	// Wait for the publisher to connect to subscribers
	sleep(1.0);
	team_status_pub.publish(status_msg);
	ROS_INFO_STREAM("Robot published ready status "<< robot_name);
}

void waitForTeam(){
	ros::Rate loopRate(5);

	//Wait until all robots are ready
	while (!team_ready){
		ROS_INFO_STREAM("Robot wait for team "<<robot_name);
		publishReadyStatus();
		ros::spinOnce();
		loopRate.sleep();
	}
}

void team_StatusCallBack(const esercizi::TurtleStatus::ConstPtr& status_msg){
	//Check if message came from monitor
	if (team_ready) return;

	robots_state[status_msg->robot_id].is_ready = status_msg->is_ready;
	robots_state[status_msg->robot_id].x = status_msg->x;
	robots_state[status_msg->robot_id].y = status_msg->y;

	int ready_counter = 0;

	for (auto robot : robots_state){

		if (robot.second.is_ready) ready_counter++;
	}
	if (ready_counter==robot_number){
		ROS_INFO_STREAM("Robot: Team is ready!");

		// Compute centroid coordinates
		for (auto robot : robots_state){
			x_avg += robot.second.x;
			y_avg += robot.second.y;
		}
		x_avg = x_avg / robot_number;
		y_avg = y_avg / robot_number;

		team_ready = true;
	}
}

void move_forward(){
	ros::Rate rate(10);	// Hz

	geometry_msgs::Twist cmd;
	esercizi::TurtleArrived msg;

	float fx, fy;	// Potential forces
	float Ka = 1;	// Artifitial potential constant
	float dist;		// Distance to the goal point

	msg.robot_id = robot_name;

	// Get the previous turtle name
	int previous_id = atoi(&robot_name.at(6)) - 1;	// 6th position of turtleX name is the turtle id number
	std::string previous_name = "turtle"+std::to_string(previous_id);

	while (ros::ok()){
		dist = sqrt(pow(x_cur-x_avg,2) + pow(y_cur-y_avg,2));

		if (dist <= RESOLUTION) has_arrived=true;

		if ((robot_name == "turtle1") && (has_arrived == false)){
			fx = Ka * (x_avg - x_cur);
			fy = Ka * (y_avg - y_cur);
			cmd.linear.x = fx * cos(theta_cur) + fy * sin(theta_cur);
			cmd.angular.z = Ka * (atan2(fy,fx) - theta_cur);
			vel_pub.publish(cmd);
		}

		if ((robot_name != "turtle1") && (has_arrived == false)){
			for (auto robot : previous_robot_state){
				if ((robot.first == previous_name) && (robot.second == true)){// devo prendere quello prima di me
					fx = Ka * (x_avg - x_cur);
					fy = Ka * (y_avg - y_cur);
					cmd.linear.x = fx * cos(theta_cur) + fy * sin(theta_cur);
					cmd.angular.z = Ka * (atan2(fy,fx) - theta_cur);
					vel_pub.publish(cmd);
				}
			}
		}
		msg.has_arrived = has_arrived;
		go_next_pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}
}

// Add arrived-or-not info in the map
void PreviousCallback(const esercizi::TurtleArrived::ConstPtr& msg){ 
	if (has_arrived) return;
	
	previous_robot_state[msg->robot_id] = msg->has_arrived;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "one_point_gathering");
	ros::NodeHandle nh("~");

	// Get params	
	if (nh.getParam("robot_name", robot_name))
		ROS_INFO("Found parameter:setting robot name: %s", robot_name.c_str());
	else
	{
		robot_name = "turtle1";
		ROS_INFO("Parameter not found: setting default robot name: %s", robot_name.c_str());
	}

	team_status_pub = nh.advertise<esercizi::TurtleStatus>("/team_status", 10);
	team_status_sub = nh.subscribe<esercizi::TurtleStatus>("/team_status", 20, &team_StatusCallBack);
	self_info_sub = nh.subscribe<turtlesim::Pose>("/" + robot_name + "/pose", 1, SelfInfoCallback);
	go_next_sub = nh.subscribe<esercizi::TurtleArrived>("/previous_status", 10, PreviousCallback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/" + robot_name + "/cmd_vel",10);
	go_next_pub = nh.advertise<esercizi::TurtleArrived>("/previous_status", 10);

	std_msgs::String msg;
	msg.data = robot_name.c_str();

	// Wait subscriber to get initial position coordinate
	sleep(2);
	ros::spinOnce();
	sleep(1);
	publishReadyStatus();
	waitForTeam();
	move_forward();

	return 0;
}