#include <string>
#include <cmath>
#include "ros/ros.h"
#include "esercizi/vel_command.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist cmd;
geometry_msgs::Twist res;

// Callback retreiving the input velocity request
void vel_req_callback(esercizi::vel_command::ConstPtr msg){
	cmd.linear.x = msg -> lin_vel;
	cmd.angular.z = msg -> ang_vel;
}


geometry_msgs::Twist PI_controller(geometry_msgs::Twist vel_cmd, float sample_time, float lin_saturation = -1, float ang_saturation = -1){
	geometry_msgs::Twist error;				// error between the requested velocity and control
	static geometry_msgs::Twist control;	// output of the controller
	static geometry_msgs::Twist int_error;	// integral of the error

	float Kp = 0.1;	// Proportional gain
	float Ki = 0.1;	// Integral gain

	// Compute errors and its itegrals
	error.linear.x = vel_cmd.linear.x - control.linear.x;
	error.angular.z = vel_cmd.angular.z - control.angular.z;
	int_error.linear.x += error.linear.x * sample_time;
	int_error.angular.z += error.angular.z * sample_time;

	//Compute PI controls
	control.linear.x = Kp * error.linear.x + Ki * int_error.linear.x;
	control.angular.z = Kp * error.angular.z + Ki * int_error.angular.z;

	// Apply a saturation on the control
	if ((lin_saturation > 0) && (fabs(control.linear.x) > lin_saturation)){
		control.linear.x = copysign(lin_saturation, control.linear.x);
	}

	if ((ang_saturation > 0) && (fabs(control.angular.z) > ang_saturation)){
		control.angular.z = copysign(ang_saturation, control.angular.z);
	}

	return control;
}


int main(int argc, char** argv){
	float freq = 10;

	ros::init(argc, argv, "vel_controller");
	ros::NodeHandle nh("~");
	ros::Subscriber sub = nh.subscribe<esercizi::vel_command>("/vel_requested", 10, vel_req_callback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ros::Rate rate(freq);

	// String containing the saturation values passed as parameters
	std::string linear_sat, angular_sat;

	// Float numbers of saturation
	float lin_sat = -1;	 // < 0 -> no saturation
	float ang_sat = -1;

	// Get velocities saturations from parameters
	if (nh.getParam("linear_sat", linear_sat)){
		// Erase the first and the last item of the string (it should be a "'") and set float saturations
		linear_sat.erase(linear_sat.begin(), linear_sat.begin() + 1);
		linear_sat.erase(linear_sat.end()-1,linear_sat.end());
		ROS_INFO("Found linear saturation parameter: %s", linear_sat.c_str());
		lin_sat = std::stof(linear_sat);
	}

	if (nh.getParam("angular_sat", angular_sat)){
		// Erase the first and the last item of the string (it should be a "'") and set float saturations
		angular_sat.erase(angular_sat.begin(), angular_sat.begin() + 1);
		angular_sat.erase(angular_sat.end()-1,angular_sat.end());
		ROS_INFO("Found linear saturation parameter: %s", linear_sat.c_str());
		ang_sat = std::stof(angular_sat);
	}

	while (ros::ok()){
		ros::spinOnce();
		res = PI_controller(cmd, 1/freq, lin_sat, ang_sat);
		pub.publish(res);	
	}

	return 0;

}