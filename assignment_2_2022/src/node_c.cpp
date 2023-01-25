#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
#include <math.h>
#include "assignment_2_2022/Info.h"

//Variables
double p_x, p_y, v_x, v_y;
double vel = 0.0;
int count = 1;

//Compute current distance between the robot and the goal
double distance(double p_x, double p_y){

	double t_x, t_y, dist;
	
	//Get goal position from ROS parameter
	ros::param::get("des_pos_x",t_x);
	ros::param::get("des_pos_y",t_y);
	
	//Calculate distance
	dist = sqrt(pow((t_x - p_x), 2.0) + pow((t_y - p_y), 2.0));
	
	return dist;
}

//Compute average speed of the robot
double average_speed(double v_x, double v_y){

	double ave_vel;
	
	//Calculate average speed
	vel += sqrt(pow(v_x,2.0) + pow(v_y, 2.0));
	ave_vel = vel/count;
	
	return ave_vel;
}

void data_Callback(const assignment_2_2022::Info::ConstPtr& data){

	p_x = data->x;
	p_y = data->y;
	v_x = data->vel_x;
	v_y = data->vel_y;
	
	//Display current robot information
	ROS_INFO("Distance to goal: %4.2f", distance(p_x, p_y));
	ROS_INFO("Average speed: %4.2f", average_speed(v_x, v_y));
	
	count += 1;
}


int main(int argc, char **argv){
	
	//Initialize the robot_info node(c)
	ros::init(argc, argv, "robot_info");
	ros::NodeHandle n;
	
	//Subscribe robot position and velocity from publisher node(a2)
	ros::Subscriber sub = n.subscribe("robot_data", 10, data_Callback);
	
	ros::spin();
	return 0;
}
