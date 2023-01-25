#include "ros/ros.h"
#include <unistd.h>
#include <math.h>
#include "assignment_2_2022/Goal.h"
#include <assignment_2_2022/PlanningAction.h>

//Goal counters
int reached_goals = 0, cancelled_goals = 0;

void status_Callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg){
	
	//When the status is SUCCEEDED(3)
	if(msg->status.status == 3){
	
		ROS_INFO("The robot has reached the goal");
		reached_goals += 1; //One goal was reached
		ROS_INFO("Reached goals:%d, Cancelled goals:%d", reached_goals, cancelled_goals);
	
	}	
}

bool goal_number(assignment_2_2022::Goal::Request &req, assignment_2_2022::Goal::Response &res){
	
	cancelled_goals += 1; //One goal was cancelled
	res.total_r = reached_goals;
	res.total_c =cancelled_goals;
	ROS_INFO("Reached goals:%d, Cancelled goals:%d", res.total_r, res.total_c);
	
	return true;
}

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
