/**
 *\file node_a2.cpp
 *\brief Node for publisher
 *\author Kazuto Muto
 *\version 0.1
 *\date 01/05/2023
 *
 *\details
 *
 *Subscribe to: <BR>
 * /odom
 *
 *Publish to: <BR>
 * /robot_data
 *
 *Descriptions:
 *
 *This node publishes the topic /robot_data which contains the position and velocity of the   
 *robot.
 *
*/

#include "ros/ros.h"
#include <unistd.h>
#include "nav_msgs/Odometry.h"
#include "assignment_2_2022/Info.h"

//Variables for the corrent position and velocity of the robot
double x; ///<Variables to pass x position from /odom to /robot_data.
double y; ///<Variables to pass y position from /odom to /robot_data.
double vel_x; ///<Variables to pass x velocity from /odom to /robot_data.
double vel_y; ///<Variables to pass y velocity from /odom to /robot_data.
double freq; ///<Variables to set frequency to publish /robot_data.

/**
*\brief Callback function for subscriber of /odom.
*\param data defines subscribed Odometry data
*
*In this callback function subscribed data are substituted to double variables to publish /robot_data. 
*/

void data_Callback(const nav_msgs::Odometry::ConstPtr& data){

	//substitute the info from topic /odom
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;
	vel_x = data->twist.twist.linear.x;
	vel_y = data->twist.twist.linear.y;
	
}

/**
*\brief Main function to implement publisher.
*\param argc
*\param argv
*\return always 0
*
*This function works as the publisher. I made a custom messages Info composed of the position x, y, and the velocity vel_x, vel_y. The name of the topic 
*is /robot_data.
*/

int main(int argc, char **argv){
	
	//Initialize the publisher node(a2)
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;

	//Subscribe from /odom and publish robot_data
	ros::Subscriber sub = n.subscribe("/odom", 1, data_Callback);
	ros::Publisher pub = n.advertise<assignment_2_2022::Info>("robot_data",1);
	
	//rate for publisher
	ros::param::get("freq", freq);
	ros::Rate rate(freq);

	while(ros::ok()){
	
		//publish the robot info(position and velocity)
		assignment_2_2022::Info data;
		data.x = x;
		data.y = y;
		data.vel_x = vel_x;
		data.vel_y = vel_y;
		pub.publish(data);
		
		rate.sleep(); //Sleep function
		ros::spinOnce();
	}
return 0;
}
