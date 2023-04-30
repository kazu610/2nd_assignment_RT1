/**
 *\file node_a1.cpp
 *\brief Node for interface
 *\author Kazuto Muto
 *\version 0.1
 *\date 30/04/2023
 *
 *\details
 *
 *Service client: <BR>
 * /goal_info
 *
 *Action client: <BR>
 * /reaching_goal
 *
 *Descriptions:
 *
 *This node works as the interface that enables the user to set new goal and cancel 
 *the current goal.
 *
*/

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "assignment_2_2022/Goal.h"
#include <assignment_2_2022/PlanningAction.h>

//Variables for setting new goal
double x_new, y_new; ///<Variables for setting new goal.

/**
*\brief Main function
*/

int main(int argc, char **argv){
	
	//Initialize the interface node(a1)
	ros::init(argc, argv, "interface");
	ros::NodeHandle n;
	
	//Service client corresponding to the service server in goal_info node(b)
	ros::ServiceClient client = n.serviceClient<assignment_2_2022::Goal>("/goal_info");
	assignment_2_2022::Goal goal; //Declare

	
	//Input from terminal
	std::string input;
	
	//Actionlib
	actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
	ROS_INFO("Waiting for action server to start");
	ac.waitForServer(); //waiting for action server start
	ROS_INFO("Confirmed the server started");
	assignment_2_2022::PlanningGoal target; //Declare
	
	//Interface to set new goal or to cancel current goal
	while(ros::ok()){
		
		//Display message and the user can choose 'g' or 'c'
		std::cout << "Please type 'g' or 'c'\n'g' setting new goal, 'c' cancelling  the current goal: " << std::endl;
		std::cin >> input;
		
		//Set new goal
		if (input == "g"){
			
			//Set x and y position
			std::cout << "Please enter new goal position (x, y): " << std::endl;
			std::cin >> x_new >> y_new;
			
			//Send new goal to the server
			target.target_pose.pose.position.x = x_new;
			target.target_pose.pose.position.y = y_new;
			ac.sendGoal(target);
			ROS_INFO("You set new goal");
			
			//Update rosparam
			ros::param::set("des_pos_x", x_new);
			ros::param::set("des_pos_y", y_new);
			
		}
		else if (input == "c"){
	
			//Cancel the current goal
			ac.cancelGoal();
			ROS_INFO("The goal was cancelled");

			//Service client
			goal.request.count_c = 1; //One goal was cancelled
			client.call(goal);
			
		}
		else{
			
			//Invalid input
			std::cout << "Please type valid keys, 'g' or 'c': " << std::endl;
		
		}
	ros::spinOnce();
	}
	return 0;
}
