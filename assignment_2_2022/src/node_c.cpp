/**
 *\file node_c.cpp
 *\brief Node to display the distance and average speed of the robot.
 *\author Kazuto Muto
 *\version 0.1
 *\date 04/05/2023
 *
 *\details
 *
 *Subscriber to: <BR>
 * /robot_data
 *
 *Descriptions:
 *
 *This node works as the subscriber which receives the topic /robot_data published by node_a2.
 *
*/

#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
#include <math.h>
#include "assignment_2_2022/Info.h"

//Variables
double p_x; ///<Variables for the recieved x position.
double p_y; ///<Variables for the recieved y position.
double v_x; ///<Variables for the recieved x velocity.
double v_y; ///<Variables for the recieved y velocity.
double vel = 0.0; ///<Variables to compute average velocity.
int count = 1; ///<Variables to count loops.

/**
*\brief Function to compute the distance between the robot and the current goal.
*\param p_x defines the x position of the robot
*
*\param p_y defines the y position of the robot
*
*\return dist the computed distance
*
*This function 'distance' computes the distance to the goal, which is executed in data_Callback function. 
*This function retrieves the goal's position from ROS parameters.
*/
double distance(double p_x, double p_y){

	double t_x, t_y, dist;
	
	//Get goal position from ROS parameter
	ros::param::get("des_pos_x",t_x);
	ros::param::get("des_pos_y",t_y);
	
	//Calculate distance
	dist = sqrt(pow((t_x - p_x), 2.0) + pow((t_y - p_y), 2.0));
	
	return dist;
}

/**
*\brief Function to compute the robot's average speed.
*\param v_x defines the x velocity of the robot
*
*\param v_y defines the y velocity of the robot
*
*\return ave_vel the average speed of the robot
*
*This function 'average_speed' computes the average speed. The variable 'vel' is the sum of the speed from the beginning and the variable 'count' is the number of subscribing. 
*The average speed 'ave_vel' is computed by deviding vel by 'count'.
*/
double average_speed(double v_x, double v_y){

	double ave_vel;
	
	//Calculate average speed
	vel += sqrt(pow(v_x,2.0) + pow(v_y, 2.0));
	ave_vel = vel/count;
	
	return ave_vel;
}

/**
*\brief data_Callback function for subscriber of /robot_data.
*\param data defines subscribed Info data
*
*Based on the current position and velocity of the robot, the distance of the robot from the goal and the average speed are computed and displayed.
*/
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


/**
*\brief Main function to the subscriber to /robot_data.
*
*\param argc
*\param argv
*
*\return always 0
*
*This node works as the subscriber which receives the topic /robot_data published by node_a2.
*
*/
int main(int argc, char **argv){
	
	//Initialize the robot_info node(c)
	ros::init(argc, argv, "robot_info");
	ros::NodeHandle n;
	
	//Subscribe robot position and velocity from publisher node(a2)
	ros::Subscriber sub = n.subscribe("robot_data", 10, data_Callback);
	
	ros::spin();
	return 0;
}
