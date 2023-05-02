/**
 *\file node_b.cpp
 *\brief Node for display the number of reached/cancelled goals when called.
 *\author Kazuto Muto
 *\version 0.1
 *\date 03/05/2023
 *
 *\details
 *
 *Subscriber to: <BR>
 * /reaching_goal/result
 *
 *Service server: <BR>
 * /goal_info
 *
 *Descriptions:
 *
 *This node is to display the number of reached/cancelled goals when called by using service server and subscriber.
 *
*/

#include "ros/ros.h"
#include <unistd.h>
#include <math.h>
#include "assignment_2_2022/Goal.h"
#include <assignment_2_2022/PlanningAction.h>

//Goal counters
int reached_goals = 0; ///<Variables to count reached goal.
int cancelled_goals = 0; ///<Variables to count cancelled goal.

/**
*\brief Callback function for subscriber of /reaching_goal/result.
*\param msg defines subscribed PlanningActionResult data
*
*Every time the node subscribes /reaching_goal/result, it checks if the status is SUCCEEDED(3), 
*which means the goal was reached. And it displays the goal's number and the variable reached_goals 
*is incremented by one when the current goal was reached.
*/

void status_Callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg){
	
	//When the status is SUCCEEDED(3)
	if(msg->status.status == 3){
	
		ROS_INFO("The robot has reached the goal");
		reached_goals += 1; //One goal was reached
		ROS_INFO("Reached goals:%d, Cancelled goals:%d", reached_goals, cancelled_goals);
	
	}	
}

/**
*\brief Function to display the number of reached/cancelled goals. 
*\param req recieved number from the service client.
*\param res response to the service client.
*\return always true
*
*When the service client (node_a2) sends request, the variable cancelled_goals is incremented by one and the server responds.
*/

bool goal_number(assignment_2_2022::Goal::Request &req, assignment_2_2022::Goal::Response &res){
	
	cancelled_goals += 1; //One goal was cancelled
	res.total_r = reached_goals;
	res.total_c =cancelled_goals;
	ROS_INFO("Reached goals:%d, Cancelled goals:%d", res.total_r, res.total_c);
	
	return true;
}

/**
*\brief Main function to the service server which publishes the number of reached/cancelled goals.
*
*\param argc
*\param argv
*
*\return always 0
*
*This node works as the service server which publishes the number of reached/cancelled goals. 
*I made a custom service definded as Goal to recieve the notification of cancelling as request 
*and publish the goal's number as response.
*
*/

int main (int argc, char **argv){
	
	//Initialize the goal_info node(b)
	ros:: init(argc, argv, "goal_info");
	ros::NodeHandle n;
	
	//Servic server that sends the number of reached/cancelled goals 
	ros::ServiceServer service = n.advertiseService("/goal_info", goal_number);
	
	//Subscribe status from action server
	ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, status_Callback);
	
	ros::spin();
	
	return 0;
}
